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

uint8_t receiveBuffer[sizeof(UartMqttFrameHeader) +
                      UART_MQTT_MAX_TOPIC_LENGTH +
                      UART_MQTT_MAX_PAYLOAD_LENGTH] = {};
size_t receiveLength = 0;
size_t expectedLength = 0;

bool modemInitialized = false;
bool modemPowerSequenced = false;
uint32_t lastNetworkAttemptAtMs = 0;
uint32_t lastMqttAttemptAtMs = 0;

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

void processFrame()
{
    const auto* header =
        reinterpret_cast<const UartMqttFrameHeader*>(receiveBuffer);
    const size_t bodyLength =
        static_cast<size_t>(header->topicLength) + header->payloadLength;
    const uint8_t* body = receiveBuffer + sizeof(UartMqttFrameHeader);

    if (header->type != UART_MQTT_FRAME_PUBLISH ||
        header->topicLength == 0 ||
        header->topicLength > UART_MQTT_MAX_TOPIC_LENGTH ||
        header->payloadLength > UART_MQTT_MAX_PAYLOAD_LENGTH ||
        uartMqttFrameCrc(*header, body, bodyLength) != header->crc32) {
        sendAck(header->sequence, UART_MQTT_ACK_INVALID);
        Serial.printf("[4G-GW][UART][WARN] Invalid frame seq=%lu\n",
                      static_cast<unsigned long>(header->sequence));
        return;
    }
    if (queueCount >= QUEUE_LENGTH) {
        sendAck(header->sequence, UART_MQTT_ACK_QUEUE_FULL);
        Serial.printf("[4G-GW][UART][WARN] Queue full seq=%lu\n",
                      static_cast<unsigned long>(header->sequence));
        return;
    }

    const size_t tail = (queueHead + queueCount) % QUEUE_LENGTH;
    QueuedPublish& item = publishQueue[tail];
    std::memcpy(item.topic, body, header->topicLength);
    item.topic[header->topicLength] = '\0';
    std::memcpy(item.payload,
                body + header->topicLength,
                header->payloadLength);
    item.payload[header->payloadLength] = '\0';
    item.sequence = header->sequence;
    item.retained = (header->flags & UART_MQTT_FLAG_RETAIN) != 0;
    ++queueCount;

    sendAck(item.sequence, UART_MQTT_ACK_QUEUED);
    Serial.printf("[4G-GW][UART] Queued seq=%lu topic=%s bytes=%u depth=%u\n",
                  static_cast<unsigned long>(item.sequence),
                  item.topic,
                  static_cast<unsigned>(header->payloadLength),
                  static_cast<unsigned>(queueCount));
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
    if (mqtt.connected() || MQTT_HOST[0] == '\0') {
        return;
    }
    const uint32_t now = millis();
    if (now - lastMqttAttemptAtMs < MQTT_RECONNECT_INTERVAL_MS) {
        return;
    }
    lastMqttAttemptAtMs = now;
    String clientId = "4g-gateway-" +
                      String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
    const char* user = MQTT_USER[0] == '\0' ? nullptr : MQTT_USER;
    const char* password =
        MQTT_PASSWORD[0] == '\0' ? nullptr : MQTT_PASSWORD;
    if (mqtt.connect(clientId.c_str(),
                     user,
                     password,
                     MQTT_TOPIC_STATUS,
                     0,
                     true,
                     "offline")) {
        const bool statusPublished =
            mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
        Serial.printf("[4G-GW][MQTT] Connected broker=%s:%u\n",
                      MQTT_HOST,
                      MQTT_PORT);
        Serial.printf("[4G-GW][MQTT] Status topic=%s online=%s retained=yes\n",
                      MQTT_TOPIC_STATUS,
                      statusPublished ? "published" : "publish_failed");
    } else {
        Serial.printf("[4G-GW][MQTT][WARN] Connect failed state=%d\n",
                      mqtt.state());
    }
}

void publishQueued()
{
    if (!mqtt.connected() || queueCount == 0) {
        return;
    }
    QueuedPublish& item = publishQueue[queueHead];
    if (!mqtt.publish(item.topic, item.payload, item.retained)) {
        return;
    }
    Serial.printf("[4G-GW][MQTT] Published seq=%lu topic=%s depth=%u\n",
                  static_cast<unsigned long>(item.sequence),
                  item.topic,
                  static_cast<unsigned>(queueCount - 1));
    sendAck(item.sequence, UART_MQTT_ACK_PUBLISHED);
    queueHead = (queueHead + 1) % QUEUE_LENGTH;
    --queueCount;
}
}

void setup()
{
    Serial.begin(115200);
    delay(200);
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
    delay(10);
}
