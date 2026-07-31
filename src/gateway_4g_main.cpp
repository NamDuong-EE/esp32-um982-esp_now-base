#include <Arduino.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <cstring>

#include "Prog_Config.h"
#include "UartMqttBridgeProtocol.h"

namespace {
HardwareSerial modemSerial(2);
HardwareSerial bridgeSerial(1);
TinyGsm modem(modemSerial);
TinyGsmClient transport(modem);
PubSubClient mqtt(transport);

struct QueuedPublish {
    char topic[UART_MQTT_MAX_TOPIC_LENGTH + 1] = {};
    char payload[UART_MQTT_MAX_PAYLOAD_LENGTH + 1] = {};
    uint32_t sequence = 0;
    bool retained = false;
};

inline constexpr size_t QUEUE_LENGTH = 8;
QueuedPublish publishQueue[QUEUE_LENGTH]{};
size_t queueHead = 0;
size_t queueCount = 0;
inline constexpr size_t RESULT_QUEUE_LENGTH = 4;
QueuedPublish resultQueue[RESULT_QUEUE_LENGTH]{};
size_t resultQueueHead = 0;
size_t resultQueueCount = 0;

struct PendingCommand {
    char topic[UART_MQTT_MAX_TOPIC_LENGTH + 1] = {};
    uint8_t payload[UART_MQTT_MAX_PAYLOAD_LENGTH + 1] = {};
    uint32_t sequence = 0;
    uint32_t lastSentAtMs = 0;
    uint16_t payloadLength = 0;
    uint8_t attempts = 0;
};

inline constexpr size_t COMMAND_QUEUE_LENGTH = 4;
inline constexpr uint8_t COMMAND_MAX_ATTEMPTS = 3;
inline constexpr uint32_t COMMAND_RETRY_INTERVAL_MS = 1000;
PendingCommand commandQueue[COMMAND_QUEUE_LENGTH]{};
size_t commandHead = 0;
size_t commandCount = 0;
uint32_t nextCommandSequence = 1;

uint8_t receiveBuffer[sizeof(UartMqttFrameHeader) +
                      UART_MQTT_MAX_TOPIC_LENGTH +
                      UART_MQTT_MAX_PAYLOAD_LENGTH] = {};
size_t receiveLength = 0;
size_t expectedLength = 0;

bool modemInitialized = false;
bool modemPowerSequenced = false;
uint32_t lastNetworkAttemptAtMs = 0;
uint32_t lastMqttAttemptAtMs = 0;
bool baseIdentityReady = false;
char attachedBaseMac[13] = {};
char mqttTopicStatus[48] = {};
char mqttTopicCommand[48] = {};
char mqttTopicCommandResult[56] = {};

bool isHexCharacter(char value)
{
    return (value >= '0' && value <= '9') ||
           (value >= 'A' && value <= 'F') ||
           (value >= 'a' && value <= 'f');
}

bool learnBaseIdentity(const char* topic)
{
    if (topic == nullptr) return false;
    char prefix[16] = {};
    snprintf(prefix, sizeof(prefix), "%s/", MQTT_TOPIC_NAMESPACE);
    const size_t prefixLength = std::strlen(prefix);
    if (std::strncmp(topic, prefix, prefixLength) != 0) return false;
    const char* macText = topic + prefixLength;
    for (size_t index = 0; index < 12; ++index) {
        if (!isHexCharacter(macText[index])) return false;
    }
    if (std::strncmp(macText + 12, "/base/", 6) != 0) return false;

    char discoveredMac[13] = {};
    for (size_t index = 0; index < 12; ++index) {
        const char value = macText[index];
        discoveredMac[index] = value >= 'a' && value <= 'f'
            ? value - ('a' - 'A') : value;
    }
    if (baseIdentityReady) {
        return std::strcmp(attachedBaseMac, discoveredMac) == 0;
    }
    std::memcpy(attachedBaseMac, discoveredMac, sizeof(attachedBaseMac));
    snprintf(mqttTopicStatus, sizeof(mqttTopicStatus), "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE, attachedBaseMac, MQTT_TOPIC_STATUS_SUFFIX);
    snprintf(mqttTopicCommand, sizeof(mqttTopicCommand), "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE, attachedBaseMac, MQTT_TOPIC_COMMAND_SUFFIX);
    snprintf(mqttTopicCommandResult, sizeof(mqttTopicCommandResult),
             "%s/%s/base/%s", MQTT_TOPIC_NAMESPACE, attachedBaseMac,
             MQTT_TOPIC_COMMAND_RESULT_SUFFIX);
    baseIdentityReady = true;
    Serial.printf("[4G-GW][UART] Attached Base MAC=%s topic_root=%s/%s/base\n",
                  attachedBaseMac, MQTT_TOPIC_NAMESPACE, attachedBaseMac);
    return true;
}

void removeHeadCommand()
{
    if (commandCount == 0) {
        return;
    }
    commandQueue[commandHead] = {};
    commandHead = (commandHead + 1) % COMMAND_QUEUE_LENGTH;
    --commandCount;
}

void sendAck(uint32_t sequence, UartMqttAckCode code)
{
    UartMqttFrameHeader header{};
    header.magic = UART_MQTT_MAGIC;
    header.version = UART_MQTT_VERSION;
    header.type = UART_MQTT_FRAME_ACK;
    header.flags = static_cast<uint8_t>(code);
    header.sequence = sequence;
    header.crc32 = uartMqttFrameCrc(header, nullptr, 0);
    bridgeSerial.write(reinterpret_cast<const uint8_t*>(&header),
                       sizeof(header));
}

void processCommandAck(const UartMqttFrameHeader& header)
{
    if (commandCount == 0 ||
        commandQueue[commandHead].sequence != header.sequence) {
        Serial.printf("[4G-GW][UART][WARN] Unexpected command ACK seq=%lu status=%u\n",
                      static_cast<unsigned long>(header.sequence),
                      static_cast<unsigned>(header.flags));
        return;
    }
    if (header.flags == UART_MQTT_ACK_QUEUED) {
        Serial.printf("[4G-GW][UART] Command delivered seq=%lu attempts=%u\n",
                      static_cast<unsigned long>(header.sequence),
                      static_cast<unsigned>(
                          commandQueue[commandHead].attempts));
        removeHeadCommand();
    } else {
        Serial.printf("[4G-GW][UART][WARN] Command NACK seq=%lu status=%u\n",
                      static_cast<unsigned long>(header.sequence),
                      static_cast<unsigned>(header.flags));
    }
}

bool enqueuePublish(QueuedPublish* queue,
                    size_t capacity,
                    size_t& head,
                    size_t& count,
                    const QueuedPublish& incoming,
                    bool replaceSameTopic)
{
    if (replaceSameTopic) {
        for (size_t offset = 0; offset < count; ++offset) {
            const size_t index = (head + offset) % capacity;
            if (std::strcmp(queue[index].topic, incoming.topic) == 0) {
                queue[index] = incoming;
                return true;
            }
        }
    }
    if (count >= capacity) {
        return false;
    }
    const size_t tail = (head + count) % capacity;
    queue[tail] = incoming;
    ++count;
    return true;
}

void processFrame()
{
    const auto* header =
        reinterpret_cast<const UartMqttFrameHeader*>(receiveBuffer);
    const size_t bodyLength =
        static_cast<size_t>(header->topicLength) + header->payloadLength;
    const uint8_t* body = receiveBuffer + sizeof(UartMqttFrameHeader);

    if (uartMqttFrameCrc(*header, body, bodyLength) != header->crc32) {
        if (header->type == UART_MQTT_FRAME_PUBLISH) {
            sendAck(header->sequence, UART_MQTT_ACK_INVALID);
        }
        Serial.printf("[4G-GW][UART][WARN] Invalid frame seq=%lu\n",
                      static_cast<unsigned long>(header->sequence));
        return;
    }
    if (header->type == UART_MQTT_FRAME_ACK &&
        header->topicLength == 0 &&
        header->payloadLength == 0) {
        processCommandAck(*header);
        return;
    }
    if (header->type != UART_MQTT_FRAME_PUBLISH ||
        header->topicLength == 0) {
        Serial.printf("[4G-GW][UART][WARN] Unsupported frame type=%u seq=%lu\n",
                      static_cast<unsigned>(header->type),
                      static_cast<unsigned long>(header->sequence));
        return;
    }
    QueuedPublish incoming{};
    std::memcpy(incoming.topic, body, header->topicLength);
    incoming.topic[header->topicLength] = '\0';
    std::memcpy(incoming.payload,
                body + header->topicLength,
                header->payloadLength);
    incoming.payload[header->payloadLength] = '\0';
    incoming.sequence = header->sequence;
    incoming.retained = (header->flags & UART_MQTT_FLAG_RETAIN) != 0;

    if (!learnBaseIdentity(incoming.topic)) {
        sendAck(header->sequence, UART_MQTT_ACK_INVALID);
        Serial.printf("[4G-GW][UART][WARN] Topic outside attached Base namespace: %s\n",
                      incoming.topic);
        return;
    }

    const bool commandResult =
        std::strcmp(incoming.topic, mqttTopicCommandResult) == 0;
    const bool queued = commandResult
        ? enqueuePublish(resultQueue,
                         RESULT_QUEUE_LENGTH,
                         resultQueueHead,
                         resultQueueCount,
                         incoming,
                         false)
        : enqueuePublish(publishQueue,
                         QUEUE_LENGTH,
                         queueHead,
                         queueCount,
                         incoming,
                         true);
    if (!queued) {
        sendAck(header->sequence, UART_MQTT_ACK_QUEUE_FULL);
        Serial.printf("[4G-GW][UART][WARN] %s queue full seq=%lu\n",
                      commandResult ? "result" : "telemetry",
                      static_cast<unsigned long>(header->sequence));
        return;
    }

    sendAck(incoming.sequence, UART_MQTT_ACK_QUEUED);
    Serial.printf("[4G-GW][UART] Queued seq=%lu topic=%s bytes=%u depth=%u class=%s\n",
                  static_cast<unsigned long>(incoming.sequence),
                  incoming.topic,
                  static_cast<unsigned>(header->payloadLength),
                  static_cast<unsigned>(
                      commandResult ? resultQueueCount : queueCount),
                  commandResult ? "result" : "telemetry");
}

void readBridge()
{
    while (bridgeSerial.available() > 0) {
        const uint8_t value = static_cast<uint8_t>(bridgeSerial.read());
        if (receiveLength == 0 &&
            value != static_cast<uint8_t>(UART_MQTT_MAGIC & 0xFFU)) {
            continue;
        }
        if (receiveLength == 1 &&
            value != static_cast<uint8_t>(UART_MQTT_MAGIC >> 8U)) {
            receiveLength =
                value == static_cast<uint8_t>(UART_MQTT_MAGIC & 0xFFU) ? 1 : 0;
            continue;
        }
        receiveBuffer[receiveLength++] = value;
        if (receiveLength == sizeof(UartMqttFrameHeader)) {
            const auto* header =
                reinterpret_cast<const UartMqttFrameHeader*>(receiveBuffer);
            if (header->magic != UART_MQTT_MAGIC ||
                header->version != UART_MQTT_VERSION ||
                header->topicLength > UART_MQTT_MAX_TOPIC_LENGTH ||
                header->payloadLength > UART_MQTT_MAX_PAYLOAD_LENGTH) {
                receiveLength = 0;
                expectedLength = 0;
                continue;
            }
            expectedLength = sizeof(UartMqttFrameHeader) +
                             header->topicLength +
                             header->payloadLength;
        }
        if (expectedLength != 0 && receiveLength == expectedLength) {
            processFrame();
            receiveLength = 0;
            expectedLength = 0;
        }
        if (receiveLength >= sizeof(receiveBuffer)) {
            receiveLength = 0;
            expectedLength = 0;
        }
    }
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length)
{
    if (topic == nullptr || !baseIdentityReady ||
        std::strcmp(topic, mqttTopicCommand) != 0) {
        return;
    }
    const size_t topicLength = std::strlen(topic);
    if (topicLength == 0 ||
        topicLength > UART_MQTT_MAX_TOPIC_LENGTH ||
        length > UART_MQTT_MAX_PAYLOAD_LENGTH) {
        Serial.printf("[4G-GW][MQTT][CMD][WARN] Invalid size topic=%u payload=%u\n",
                      static_cast<unsigned>(topicLength),
                      length);
        return;
    }
    if (commandCount >= COMMAND_QUEUE_LENGTH) {
        Serial.printf("[4G-GW][MQTT][CMD][WARN] Command queue full bytes=%u\n",
                      length);
        return;
    }

    const size_t tail =
        (commandHead + commandCount) % COMMAND_QUEUE_LENGTH;
    PendingCommand& command = commandQueue[tail];
    std::memcpy(command.topic, topic, topicLength);
    command.topic[topicLength] = '\0';
    std::memcpy(command.payload, payload, length);
    command.payload[length] = '\0';
    command.payloadLength = static_cast<uint16_t>(length);
    command.sequence = nextCommandSequence++;
    if (nextCommandSequence == 0) {
        nextCommandSequence = 1;
    }
    ++commandCount;
    Serial.printf("[4G-GW][MQTT][CMD] Queued seq=%lu bytes=%u depth=%u\n",
                  static_cast<unsigned long>(command.sequence),
                  length,
                  static_cast<unsigned>(commandCount));
}

void sendPendingCommand(uint32_t now)
{
    if (commandCount == 0) {
        return;
    }
    PendingCommand& command = commandQueue[commandHead];
    if (command.attempts != 0 &&
        now - command.lastSentAtMs < COMMAND_RETRY_INTERVAL_MS) {
        return;
    }
    if (command.attempts >= COMMAND_MAX_ATTEMPTS) {
        Serial.printf("[4G-GW][UART][CMD][WARN] Delivery failed seq=%lu attempts=%u\n",
                      static_cast<unsigned long>(command.sequence),
                      static_cast<unsigned>(command.attempts));
        removeHeadCommand();
        return;
    }

    const size_t topicLength = std::strlen(command.topic);
    uint8_t body[UART_MQTT_MAX_TOPIC_LENGTH +
                 UART_MQTT_MAX_PAYLOAD_LENGTH] = {};
    std::memcpy(body, command.topic, topicLength);
    std::memcpy(body + topicLength,
                command.payload,
                command.payloadLength);

    UartMqttFrameHeader header{};
    header.magic = UART_MQTT_MAGIC;
    header.version = UART_MQTT_VERSION;
    header.type = UART_MQTT_FRAME_MESSAGE;
    header.sequence = command.sequence;
    header.topicLength = static_cast<uint16_t>(topicLength);
    header.payloadLength = command.payloadLength;
    header.crc32 = uartMqttFrameCrc(
        header, body, topicLength + command.payloadLength);

    const size_t headerWritten = bridgeSerial.write(
        reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    const size_t bodyWritten = bridgeSerial.write(
        body, topicLength + command.payloadLength);
    ++command.attempts;
    command.lastSentAtMs = now;
    Serial.printf("[4G-GW][UART][CMD] TX seq=%lu attempt=%u bytes=%u result=%s\n",
                  static_cast<unsigned long>(command.sequence),
                  static_cast<unsigned>(command.attempts),
                  static_cast<unsigned>(command.payloadLength),
                  headerWritten == sizeof(header) &&
                          bodyWritten == topicLength + command.payloadLength
                      ? "ok"
                      : "partial");
}

void powerOnModem()
{
    if (modemPowerSequenced) {
        return;
    }
    modemPowerSequenced = true;
    if (!MODEM_POWER_CONTROL_ENABLED) {
        return;
    }
    const uint8_t active =
        MODEM_POWER_CONTROL_ACTIVE_HIGH ? HIGH : LOW;
    const uint8_t inactive =
        MODEM_POWER_CONTROL_ACTIVE_HIGH ? LOW : HIGH;
    pinMode(MODEM_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(MODEM_POWER_CONTROL_PIN, inactive);
    delay(50);
    digitalWrite(MODEM_POWER_CONTROL_PIN, active);
    delay(MODEM_POWER_PULSE_MS);
    digitalWrite(MODEM_POWER_CONTROL_PIN, inactive);
    delay(MODEM_BOOT_WAIT_MS);
}

bool ensureMobileNetwork()
{
    if (!modemInitialized) {
        powerOnModem();
        modemInitialized = modem.init();
        if (!modemInitialized) {
            Serial.println("[4G-GW][WARN] Modem init failed");
            return false;
        }
        Serial.println("[4G-GW] Modem=" + modem.getModemInfo());
    }
    if (!modem.isNetworkConnected() &&
        !modem.waitForNetwork(MODEM_NETWORK_TIMEOUT_MS, true)) {
        Serial.println("[4G-GW][WARN] Network registration failed");
        return false;
    }
    if (!modem.isGprsConnected() &&
        !modem.gprsConnect(MODEM_APN,
                           MODEM_GPRS_USER,
                           MODEM_GPRS_PASSWORD)) {
        Serial.println("[4G-GW][WARN] APN connection failed");
        return false;
    }
    return true;
}

void ensureMqtt()
{
    if (mqtt.connected() || MQTT_HOST[0] == '\0' || !baseIdentityReady) {
        return;
    }
    const uint32_t now = millis();
    if (now - lastMqttAttemptAtMs < MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }
    lastMqttAttemptAtMs = now;
    String clientId = "4g-gateway-" + String(attachedBaseMac);
    const char* user = MQTT_USER[0] == '\0' ? nullptr : MQTT_USER;
    const char* password =
        MQTT_PASSWORD[0] == '\0' ? nullptr : MQTT_PASSWORD;
    if (mqtt.connect(clientId.c_str(),
                     user,
                     password,
                      mqttTopicStatus,
                     0,
                     true,
                     "offline")) {
        const bool statusPublished =
            mqtt.publish(mqttTopicStatus, "online", true);
        const bool commandSubscribed =
            mqtt.subscribe(mqttTopicCommand, 1);
        Serial.printf("[4G-GW][MQTT] Connected broker=%s:%u\n",
                      MQTT_HOST,
                      MQTT_PORT);
        Serial.printf("[4G-GW][MQTT] Status topic=%s online=%s retained=yes\n",
                      mqttTopicStatus,
                      statusPublished ? "published" : "publish_failed");
        Serial.printf("[4G-GW][MQTT] Command topic=%s subscribe=%s qos=1\n",
                       mqttTopicCommand,
                      commandSubscribed ? "ok" : "failed");
    } else {
        Serial.printf("[4G-GW][MQTT][WARN] Connect failed state=%d\n",
                      mqtt.state());
    }
}

bool publishQueueHead(QueuedPublish* queue,
                      size_t capacity,
                      size_t& head,
                      size_t& count,
                      const char* queueClass)
{
    if (!mqtt.connected() || count == 0) {
        return false;
    }
    QueuedPublish& item = queue[head];
    if (!mqtt.publish(item.topic, item.payload, item.retained)) {
        return false;
    }
    Serial.printf("[4G-GW][MQTT] Published seq=%lu topic=%s depth=%u class=%s\n",
                  static_cast<unsigned long>(item.sequence),
                  item.topic,
                  static_cast<unsigned>(count - 1),
                  queueClass);
    sendAck(item.sequence, UART_MQTT_ACK_PUBLISHED);
    head = (head + 1) % capacity;
    --count;
    return true;
}

void publishQueued()
{
    if (resultQueueCount != 0) {
        publishQueueHead(resultQueue,
                         RESULT_QUEUE_LENGTH,
                         resultQueueHead,
                         resultQueueCount,
                         "result");
        return;
    }
    publishQueueHead(publishQueue,
                     QUEUE_LENGTH,
                     queueHead,
                     queueCount,
                     "telemetry");
}
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    do {
        nextCommandSequence = esp_random();
    } while (nextCommandSequence == 0);
    Serial.println("\n=========================================");
    Serial.println("       ESP32 4G UART MQTT GATEWAY");
    Serial.println("=========================================");
    bridgeSerial.setRxBufferSize(2048);
    bridgeSerial.begin(UART_GATEWAY_BAUD,
                       SERIAL_8N1,
                       UART_GATEWAY_MODEM_BOARD_RX_PIN,
                       UART_GATEWAY_MODEM_BOARD_TX_PIN);
    modemSerial.setRxBufferSize(2048);
    modemSerial.begin(MODEM_BAUD,
                      SERIAL_8N1,
                      MODEM_RX_PIN,
                      MODEM_TX_PIN);
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_SECONDS);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SECONDS);
    mqtt.setBufferSize(MQTT_BUFFER_SIZE);
    Serial.printf("[4G-GW][UART] Client link RX=%d TX=%d baud=%lu\n",
                  UART_GATEWAY_MODEM_BOARD_RX_PIN,
                  UART_GATEWAY_MODEM_BOARD_TX_PIN,
                  static_cast<unsigned long>(UART_GATEWAY_BAUD));
    Serial.printf("[4G-GW][MODEM] RX=%d TX=%d baud=%lu\n",
                  MODEM_RX_PIN,
                  MODEM_TX_PIN,
                  static_cast<unsigned long>(MODEM_BAUD));
    Serial.printf("[4G-GW][UART][CMD] Initial sequence=%lu\n",
                  static_cast<unsigned long>(nextCommandSequence));
    Serial.println("[4G-GW][UART] Waiting for Base MAC namespace announcement");
}

void loop()
{
    readBridge();

    const uint32_t now = millis();
    if ((!modemInitialized ||
         !modem.isNetworkConnected() ||
         !modem.isGprsConnected()) &&
        (lastNetworkAttemptAtMs == 0 ||
         now - lastNetworkAttemptAtMs >= NETWORK_RECONNECT_INTERVAL_MS)) {
        lastNetworkAttemptAtMs = now;
        ensureMobileNetwork();
    }
    if (modemInitialized && modem.isGprsConnected()) {
        ensureMqtt();
    }
    if (mqtt.connected()) {
        mqtt.loop();
        publishQueued();
    }
    sendPendingCommand(now);
    delay(10);
}
