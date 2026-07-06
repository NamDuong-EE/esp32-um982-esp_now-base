#ifndef RTCM_ESPNOW_PROTOCOL_H
#define RTCM_ESPNOW_PROTOCOL_H

#include <cstddef>
#include <cstdint>

inline constexpr uint16_t RTCM_ESPNOW_MAGIC = 0x5452; // "RT" on little-endian wire.
inline constexpr uint8_t RTCM_ESPNOW_VERSION = 1;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_RTCM_DATA = 1;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_FRAME_ACK = 2;
inline constexpr uint8_t RTCM_ESPNOW_ACK_STATUS_WRITTEN = 1;
inline constexpr size_t RTCM_ESPNOW_MAX_PACKET_SIZE = 250;
inline constexpr size_t RTCM_ESPNOW_MAX_FRAME_LENGTH = 1029;

#pragma pack(push, 1)
struct RtcmEspNowHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t packetType;
    uint16_t streamId;
    uint32_t frameSequence;
    uint16_t frameLength;
    uint8_t fragmentIndex;
    uint8_t fragmentCount;
    uint16_t payloadLength;
};

struct RtcmEspNowAck {
    uint16_t magic;
    uint8_t version;
    uint8_t packetType;
    uint16_t streamId;
    uint32_t frameSequence;
    uint8_t status;
    uint8_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(RtcmEspNowHeader) == 16, "RTCM ESP-NOW header must be 16 bytes");
static_assert(sizeof(RtcmEspNowAck) == 12, "RTCM ESP-NOW ACK must be 12 bytes");

inline constexpr size_t RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD =
    RTCM_ESPNOW_MAX_PACKET_SIZE - sizeof(RtcmEspNowHeader);
inline constexpr size_t RTCM_ESPNOW_MAX_FRAGMENT_COUNT =
    (RTCM_ESPNOW_MAX_FRAME_LENGTH + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) /
    RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;

#endif // RTCM_ESPNOW_PROTOCOL_H
