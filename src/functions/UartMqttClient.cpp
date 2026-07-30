#include "functions/UartMqttClient.h"

#include <cstring>

#include "Prog_Config.h"
#include "UartMqttBridgeProtocol.h"

namespace {
HardwareSerial gatewaySerial(2);
}

void UartMqttClient::begin()
{
    if (started_) {
        return;
    }
    gatewaySerial.setRxBufferSize(1024);
    gatewaySerial.begin(UART_GATEWAY_BAUD,
                        SERIAL_8N1,
                        UART_GATEWAY_CLIENT_RX_PIN,
                        UART_GATEWAY_CLIENT_TX_PIN);
    started_ = true;
    Serial.printf("[BASE][UART-GW] Ready baud=%lu RX=%d TX=%d\n",
                  static_cast<unsigned long>(UART_GATEWAY_BAUD),
                  UART_GATEWAY_CLIENT_RX_PIN,
                  UART_GATEWAY_CLIENT_TX_PIN);
}

bool UartMqttClient::connect(const char*,
                             const char*,
                             const char*,
                             const char*,
                             uint8_t,
                             bool,
                             const char*)
{
    begin();
    return true;
}

bool UartMqttClient::publish(const char* topic,
                             const char* payload,
                             bool retained)
{
    begin();
    if (topic == nullptr || payload == nullptr) {
        return false;
    }

    const size_t topicLength = std::strlen(topic);
    const size_t payloadLength = std::strlen(payload);
    if (topicLength == 0 ||
        topicLength > UART_MQTT_MAX_TOPIC_LENGTH ||
        payloadLength > UART_MQTT_MAX_PAYLOAD_LENGTH) {
        Serial.printf("[BASE][UART-GW][WARN] Frame too large topic=%u payload=%u\n",
                      static_cast<unsigned>(topicLength),
                      static_cast<unsigned>(payloadLength));
        return false;
    }

    uint8_t body[UART_MQTT_MAX_TOPIC_LENGTH +
                 UART_MQTT_MAX_PAYLOAD_LENGTH] = {};
    std::memcpy(body, topic, topicLength);
    std::memcpy(body + topicLength, payload, payloadLength);

    UartMqttFrameHeader header{};
    header.magic = UART_MQTT_MAGIC;
    header.version = UART_MQTT_VERSION;
    header.type = UART_MQTT_FRAME_PUBLISH;
    header.flags = retained ? UART_MQTT_FLAG_RETAIN : 0;
    header.sequence = nextSequence_++;
    header.topicLength = static_cast<uint16_t>(topicLength);
    header.payloadLength = static_cast<uint16_t>(payloadLength);
    header.crc32 = uartMqttFrameCrc(header,
                                   body,
                                   topicLength + payloadLength);

    const size_t headerWritten = gatewaySerial.write(
        reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    const size_t bodyWritten = gatewaySerial.write(
        body, topicLength + payloadLength);
    gatewaySerial.flush();
    if (headerWritten != sizeof(header) ||
        bodyWritten != topicLength + payloadLength) {
        Serial.printf("[BASE][UART-GW][WARN] TX incomplete seq=%lu\n",
                      static_cast<unsigned long>(header.sequence));
        return false;
    }

    Serial.printf("[BASE][UART-GW] TX publish seq=%lu topic=%s bytes=%u\n",
                  static_cast<unsigned long>(header.sequence),
                  topic,
                  static_cast<unsigned>(payloadLength));
    return true;
}

void UartMqttClient::processIncoming()
{
    while (gatewaySerial.available() > 0) {
        const uint8_t value = static_cast<uint8_t>(gatewaySerial.read());
        if (receiveLength_ == 0 &&
            value != static_cast<uint8_t>(UART_MQTT_MAGIC & 0xFFU)) {
            continue;
        }
        if (receiveLength_ == 1 &&
            value != static_cast<uint8_t>(UART_MQTT_MAGIC >> 8U)) {
            receiveLength_ =
                value == static_cast<uint8_t>(UART_MQTT_MAGIC & 0xFFU) ? 1 : 0;
            continue;
        }
        receiveBuffer_[receiveLength_++] = value;
        if (receiveLength_ == sizeof(UartMqttFrameHeader)) {
            const auto* header =
                reinterpret_cast<const UartMqttFrameHeader*>(receiveBuffer_);
            if (header->magic != UART_MQTT_MAGIC ||
                header->version != UART_MQTT_VERSION ||
                header->type != UART_MQTT_FRAME_ACK ||
                header->topicLength != 0 ||
                header->payloadLength != 0 ||
                uartMqttFrameCrc(*header, nullptr, 0) != header->crc32) {
                receiveLength_ = 0;
                expectedLength_ = 0;
                continue;
            }
            expectedLength_ = sizeof(UartMqttFrameHeader);
        }
        if (expectedLength_ != 0 && receiveLength_ == expectedLength_) {
            const auto* header =
                reinterpret_cast<const UartMqttFrameHeader*>(receiveBuffer_);
            Serial.printf("[BASE][UART-GW] ACK seq=%lu status=%u\n",
                          static_cast<unsigned long>(header->sequence),
                          static_cast<unsigned>(header->flags));
            receiveLength_ = 0;
            expectedLength_ = 0;
        }
        if (receiveLength_ >= sizeof(receiveBuffer_)) {
            receiveLength_ = 0;
            expectedLength_ = 0;
        }
    }
}

bool UartMqttClient::loop()
{
    processIncoming();
    return true;
}
