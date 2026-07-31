#ifndef UART_MQTT_BRIDGE_PROTOCOL_H
#define UART_MQTT_BRIDGE_PROTOCOL_H

#include <cstddef>
#include <cstdint>

inline constexpr uint16_t UART_MQTT_MAGIC = 0xA74D;
inline constexpr uint8_t UART_MQTT_VERSION = 1;
inline constexpr size_t UART_MQTT_MAX_TOPIC_LENGTH = 96;
inline constexpr size_t UART_MQTT_MAX_PAYLOAD_LENGTH = 768;

enum UartMqttFrameType : uint8_t {
    UART_MQTT_FRAME_PUBLISH = 1,
    UART_MQTT_FRAME_ACK = 2,
    UART_MQTT_FRAME_MESSAGE = 3,
};

enum UartMqttAckCode : uint8_t {
    UART_MQTT_ACK_QUEUED = 1,
    UART_MQTT_ACK_PUBLISHED = 2,
    UART_MQTT_ACK_QUEUE_FULL = 3,
    UART_MQTT_ACK_INVALID = 4,
};

inline constexpr uint8_t UART_MQTT_FLAG_RETAIN = 0x01;

#pragma pack(push, 1)
struct UartMqttFrameHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t flags;
    uint8_t reserved;
    uint32_t sequence;
    uint16_t topicLength;
    uint16_t payloadLength;
    uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(UartMqttFrameHeader) == 18,
              "Unexpected UART MQTT frame header size");

inline uint32_t uartMqttCrc32Update(uint32_t crc,
                                    const uint8_t* data,
                                    size_t length)
{
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1U) ^
                  (0xEDB88320UL & (0U - (crc & 1U)));
        }
    }
    return crc;
}

inline uint32_t uartMqttFrameCrc(const UartMqttFrameHeader& header,
                                 const uint8_t* body,
                                 size_t bodyLength)
{
    uint32_t crc = 0xFFFFFFFFUL;
    crc = uartMqttCrc32Update(
        crc,
        reinterpret_cast<const uint8_t*>(&header),
        offsetof(UartMqttFrameHeader, crc32));
    if (body != nullptr && bodyLength != 0) {
        crc = uartMqttCrc32Update(crc, body, bodyLength);
    }
    return crc ^ 0xFFFFFFFFUL;
}

#endif // UART_MQTT_BRIDGE_PROTOCOL_H
