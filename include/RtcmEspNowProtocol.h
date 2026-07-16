#ifndef RTCM_ESPNOW_PROTOCOL_H
#define RTCM_ESPNOW_PROTOCOL_H

#include <cstddef>
#include <cstdint>

inline constexpr uint16_t RTCM_ESPNOW_MAGIC = 0x5452; // "RT" on little-endian wire.
inline constexpr uint8_t RTCM_ESPNOW_VERSION = 1;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_RTCM_DATA = 1;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_FRAME_ACK = 2;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_DISCOVERY = 3;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_RESPONSE = 4;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_CONFIRM = 5;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_ROVER_LLH_STATUS = 6;
inline constexpr uint8_t RTCM_ESPNOW_ROLE_BASE = 1;
inline constexpr uint8_t RTCM_ESPNOW_ROLE_ROVER = 2;
inline constexpr double RTCM_ESPNOW_LLH_COORDINATE_SCALE = 10000000.0;
inline constexpr double RTCM_ESPNOW_LLH_HEIGHT_SCALE = 1000.0;
inline constexpr uint8_t RTCM_ESPNOW_ACK_STATUS_WRITTEN = 1;
inline constexpr size_t RTCM_ESPNOW_MAX_PACKET_SIZE = 250;
inline constexpr size_t RTCM_ESPNOW_MAX_FRAME_LENGTH = 1029;

#pragma pack(push, 1)
struct RtcmEspNowCommonHeader {
    uint16_t magic;
    uint8_t version;
    uint8_t packetType;
};

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

struct RoverLlhStatusPacket {
    RtcmEspNowCommonHeader common;
    uint32_t sequence;
    int32_t latitudeE7;
    int32_t longitudeE7;
    int32_t heightMm;
};

struct RtcmEspNowPairDiscovery {
    RtcmEspNowCommonHeader common;
    uint8_t role;
    uint8_t reserved[3];
    uint32_t networkId;
    uint32_t baseDeviceId;
    uint32_t baseNonce;
    uint32_t pairingWindowMs;
    uint32_t authTag;
};

struct RtcmEspNowPairResponse {
    RtcmEspNowCommonHeader common;
    uint8_t role;
    uint8_t reserved[3];
    uint32_t networkId;
    uint32_t roverDeviceId;
    uint32_t roverNonce;
    uint32_t baseNonceEcho;
    uint32_t authTag;
};

struct RtcmEspNowPairConfirm {
    RtcmEspNowCommonHeader common;
    uint8_t role;
    uint8_t reserved[3];
    uint32_t networkId;
    uint32_t baseNonce;
    uint32_t roverNonce;
    uint32_t authTag;
};
#pragma pack(pop)

static_assert(sizeof(RtcmEspNowCommonHeader) == 4, "RTCM ESP-NOW common header must be 4 bytes");
static_assert(sizeof(RtcmEspNowHeader) == 16, "RTCM ESP-NOW header must be 16 bytes");
static_assert(sizeof(RtcmEspNowAck) == 12, "RTCM ESP-NOW ACK must be 12 bytes");
static_assert(sizeof(RoverLlhStatusPacket) == 20, "ROVER_LLH_STATUS must be 20 bytes");
static_assert(sizeof(RtcmEspNowPairDiscovery) == 28, "PAIR_DISCOVERY must be 28 bytes");
static_assert(sizeof(RtcmEspNowPairResponse) == 28, "PAIR_RESPONSE must be 28 bytes");
static_assert(sizeof(RtcmEspNowPairConfirm) == 24, "PAIR_CONFIRM must be 24 bytes");

inline constexpr size_t RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD =
    RTCM_ESPNOW_MAX_PACKET_SIZE - sizeof(RtcmEspNowHeader);
inline constexpr size_t RTCM_ESPNOW_MAX_FRAGMENT_COUNT =
    (RTCM_ESPNOW_MAX_FRAME_LENGTH + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) /
    RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;

inline uint32_t rtcmEspNowPairingAuthTag(const uint8_t* data,
                                         size_t lengthWithoutAuthTag,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    if ((data == nullptr && lengthWithoutAuthTag != 0) ||
        (pairingKey == nullptr && pairingKeyLength != 0)) {
        return 0;
    }

    uint32_t hash = 2166136261UL;
    for (size_t index = 0; index < lengthWithoutAuthTag; ++index) {
        hash ^= data[index];
        hash *= 16777619UL;
    }
    hash ^= 0x9E3779B9UL;
    for (size_t index = 0; index < pairingKeyLength; ++index) {
        hash ^= pairingKey[index];
        hash *= 16777619UL;
    }
    return hash == 0 ? 0xFFFFFFFFUL : hash;
}

inline uint32_t rtcmEspNowPairingAuthTag(const RtcmEspNowPairDiscovery& packet,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    return rtcmEspNowPairingAuthTag(reinterpret_cast<const uint8_t*>(&packet),
                                    offsetof(RtcmEspNowPairDiscovery, authTag),
                                    pairingKey,
                                    pairingKeyLength);
}

inline uint32_t rtcmEspNowPairingAuthTag(const RtcmEspNowPairResponse& packet,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    return rtcmEspNowPairingAuthTag(reinterpret_cast<const uint8_t*>(&packet),
                                    offsetof(RtcmEspNowPairResponse, authTag),
                                    pairingKey,
                                    pairingKeyLength);
}

inline uint32_t rtcmEspNowPairingAuthTag(const RtcmEspNowPairConfirm& packet,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    return rtcmEspNowPairingAuthTag(reinterpret_cast<const uint8_t*>(&packet),
                                    offsetof(RtcmEspNowPairConfirm, authTag),
                                    pairingKey,
                                    pairingKeyLength);
}

inline bool rtcmEspNowValidatePairResponse(const RtcmEspNowPairResponse& packet,
                                           size_t receivedLength,
                                           uint32_t expectedNetworkId,
                                           uint32_t expectedBaseNonce,
                                           const uint8_t* pairingKey,
                                           size_t pairingKeyLength)
{
    return receivedLength == sizeof(RtcmEspNowPairResponse) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType == RTCM_ESPNOW_PACKET_TYPE_PAIR_RESPONSE &&
           packet.role == RTCM_ESPNOW_ROLE_ROVER &&
           packet.networkId == expectedNetworkId &&
           packet.baseNonceEcho == expectedBaseNonce &&
           packet.authTag == rtcmEspNowPairingAuthTag(packet, pairingKey, pairingKeyLength);
}

inline bool rtcmEspNowValidateRoverLlhStatus(const RoverLlhStatusPacket& packet,
                                             size_t receivedLength)
{
    return receivedLength == sizeof(RoverLlhStatusPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType == RTCM_ESPNOW_PACKET_TYPE_ROVER_LLH_STATUS &&
           packet.latitudeE7 >= -900000000 && packet.latitudeE7 <= 900000000 &&
           packet.longitudeE7 >= -1800000000 && packet.longitudeE7 <= 1800000000;
}

#endif // RTCM_ESPNOW_PROTOCOL_H
