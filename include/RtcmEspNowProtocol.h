#ifndef RTCM_ESPNOW_PROTOCOL_H
#define RTCM_ESPNOW_PROTOCOL_H

#include <cstddef>
#include <cstdint>

inline constexpr uint16_t RTCM_ESPNOW_MAGIC = 0x5452; // "RT" on little-endian wire.
inline constexpr uint8_t RTCM_ESPNOW_VERSION = 2;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_RTCM_DATA = 1;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_FRAME_ACK = 2;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_DISCOVERY = 3;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_RESPONSE = 4;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_PAIR_CONFIRM = 5;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_ROVER_ECEF_STATUS = 6;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_ECEF_STATUS = 7;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_REQUEST = 8;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_RESULT = 9;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_TEMP_RTCM_DATA = 10;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_TEMP_RTCM_ACK = 11;
inline constexpr uint8_t RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_RTCM_ACK_STATUS = 12;
inline constexpr uint8_t RTCM_ESPNOW_ROLE_BASE = 1;
inline constexpr uint8_t RTCM_ESPNOW_ROLE_ROVER = 2;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER = 2;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF = 3;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK = 4;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK = 5;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_PORT_COM2 = 2;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_STATUS_VERIFIED = 1;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_SEQUENCE_WRITTEN =
    RTCM_ESPNOW_GNSS_COMMAND_STATUS_VERIFIED; // Backward-compatible wire alias.
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_STATUS_REJECTED = 2;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_ERROR = 3;
inline constexpr uint8_t RTCM_ESPNOW_GNSS_COMMAND_STATUS_BUSY = 4;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_NONE = 0;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_INVALID_REQUEST = 1;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_QUEUE_FULL = 2;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_UART_WRITE = 3;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_RESPONSE_TIMEOUT = 4;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_RESPONSE_REJECTED = 5;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_MODE_VERIFY = 6;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_GGA_VERIFY = 7;
inline constexpr uint16_t RTCM_ESPNOW_GNSS_COMMAND_DETAIL_PERSISTENCE_VERIFY = 8;
inline constexpr double RTCM_ESPNOW_ECEF_SCALE = 10000.0;
inline constexpr int64_t RTCM_ESPNOW_ECEF_SCALED_LIMIT = 70000000000LL;
inline constexpr uint32_t RTCM_ESPNOW_GNSS_MILLISECONDS_PER_DAY = 86400000UL;
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

struct RoverEcefStatusPacket {
    RtcmEspNowCommonHeader common;
    uint32_t sequence;
    uint32_t gnssTimeMsOfDay;
    uint16_t correctionStreamId;
    uint8_t fixQuality;
    uint8_t reserved;
    int64_t ecefXScaled;
    int64_t ecefYScaled;
    int64_t ecefZScaled;
};

struct RelayedRoverEcefStatusPacket {
    RtcmEspNowCommonHeader common;
    uint32_t sequence;
    uint8_t roverMac[6];
    uint32_t gnssTimeMsOfDay;
    uint16_t correctionStreamId;
    uint8_t fixQuality;
    uint8_t reserved[3];
    int64_t ecefXScaled;
    int64_t ecefYScaled;
    int64_t ecefZScaled;
};

struct RelayedRoverRtcmAckStatusPacket {
    RtcmEspNowCommonHeader common;
    uint8_t roverMac[6];
    uint16_t streamId;
    uint32_t frameSequence;
    uint32_t ackAgeMs;
};

struct GnssCommandRequestPacket {
    RtcmEspNowCommonHeader common;
    uint32_t networkId;
    uint32_t transactionId;
    int64_t ecefXScaled;
    int64_t ecefYScaled;
    int64_t ecefZScaled;
    uint8_t commandId;
    uint8_t targetPort;
    uint16_t reserved;
    uint32_t authTag;
};

struct GnssCommandResultPacket {
    RtcmEspNowCommonHeader common;
    uint32_t networkId;
    uint32_t transactionId;
    uint8_t commandId;
    uint8_t status;
    uint8_t completedStep;
    uint8_t totalSteps;
    uint16_t detailCode;
    uint16_t reserved;
    uint32_t authTag;
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
static_assert(sizeof(RoverEcefStatusPacket) == 40,
              "ROVER_ECEF_STATUS must be 40 bytes");
static_assert(sizeof(RelayedRoverEcefStatusPacket) == 48,
              "RELAYED_ROVER_ECEF_STATUS must be 48 bytes");
static_assert(sizeof(RelayedRoverRtcmAckStatusPacket) == 20,
              "RELAYED_ROVER_RTCM_ACK_STATUS must be 20 bytes");
static_assert(sizeof(GnssCommandRequestPacket) == 44,
              "GNSS_COMMAND_REQUEST must be 44 bytes");
static_assert(sizeof(GnssCommandResultPacket) == 24,
              "GNSS_COMMAND_RESULT must be 24 bytes");
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

inline uint32_t rtcmEspNowPairingAuthTag(const GnssCommandRequestPacket& packet,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    return rtcmEspNowPairingAuthTag(reinterpret_cast<const uint8_t*>(&packet),
                                    offsetof(GnssCommandRequestPacket, authTag),
                                    pairingKey,
                                    pairingKeyLength);
}

inline uint32_t rtcmEspNowPairingAuthTag(const GnssCommandResultPacket& packet,
                                         const uint8_t* pairingKey,
                                         size_t pairingKeyLength)
{
    return rtcmEspNowPairingAuthTag(reinterpret_cast<const uint8_t*>(&packet),
                                    offsetof(GnssCommandResultPacket, authTag),
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

inline bool rtcmEspNowValidEcef(int64_t x, int64_t y, int64_t z)
{
    return x >= -RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           x <= RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           y >= -RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           y <= RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           z >= -RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           z <= RTCM_ESPNOW_ECEF_SCALED_LIMIT &&
           (x < -900000 || x > 900000);
}

inline bool rtcmEspNowValidateRoverEcefStatus(
    const RoverEcefStatusPacket& packet,
    size_t receivedLength)
{
    return receivedLength == sizeof(RoverEcefStatusPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType ==
               RTCM_ESPNOW_PACKET_TYPE_ROVER_ECEF_STATUS &&
           packet.gnssTimeMsOfDay <
               RTCM_ESPNOW_GNSS_MILLISECONDS_PER_DAY &&
           rtcmEspNowValidEcef(packet.ecefXScaled,
                               packet.ecefYScaled,
                               packet.ecefZScaled) &&
           packet.fixQuality <= 8;
}

inline bool rtcmEspNowValidateRelayedRoverEcefStatus(
    const RelayedRoverEcefStatusPacket& packet,
    size_t receivedLength)
{
    bool macConfigured = false;
    bool macBroadcast = true;
    for (uint8_t octet : packet.roverMac) {
        macConfigured = macConfigured || octet != 0;
        macBroadcast = macBroadcast && octet == 0xFF;
    }
    return receivedLength == sizeof(RelayedRoverEcefStatusPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType ==
               RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_ECEF_STATUS &&
           macConfigured && !macBroadcast && (packet.roverMac[0] & 0x01U) == 0 &&
           packet.gnssTimeMsOfDay <
               RTCM_ESPNOW_GNSS_MILLISECONDS_PER_DAY &&
           rtcmEspNowValidEcef(packet.ecefXScaled,
                               packet.ecefYScaled,
                               packet.ecefZScaled) &&
           packet.fixQuality <= 8;
}

inline bool rtcmEspNowValidateRelayedRoverRtcmAckStatus(
    const RelayedRoverRtcmAckStatusPacket& packet,
    size_t receivedLength)
{
    bool macConfigured = false;
    bool macBroadcast = true;
    for (uint8_t octet : packet.roverMac) {
        macConfigured = macConfigured || octet != 0;
        macBroadcast = macBroadcast && octet == 0xFF;
    }
    return receivedLength == sizeof(RelayedRoverRtcmAckStatusPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType ==
               RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_RTCM_ACK_STATUS &&
           macConfigured && !macBroadcast && (packet.roverMac[0] & 0x01U) == 0;
}

inline bool rtcmEspNowValidateGnssCommandRequest(
    const GnssCommandRequestPacket& packet,
    size_t receivedLength,
    uint32_t expectedNetworkId,
    const uint8_t* pairingKey,
    size_t pairingKeyLength)
{
    const bool validEcef =
        rtcmEspNowValidEcef(packet.ecefXScaled,
                            packet.ecefYScaled,
                            packet.ecefZScaled);
    const bool validCommandParameters =
        (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER &&
         packet.ecefXScaled == 0 &&
         packet.ecefYScaled == 0 &&
         packet.ecefZScaled == 0) ||
        (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK &&
         packet.ecefXScaled == 0 &&
         packet.ecefYScaled == 0 &&
         packet.ecefZScaled == 0) ||
        (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK &&
         packet.ecefXScaled == 0 &&
         packet.ecefYScaled == 0 &&
         packet.ecefZScaled == 0) ||
        (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF &&
         validEcef);
    return receivedLength == sizeof(GnssCommandRequestPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType == RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_REQUEST &&
           packet.networkId == expectedNetworkId &&
           packet.transactionId != 0 &&
           validCommandParameters &&
           packet.targetPort == RTCM_ESPNOW_GNSS_PORT_COM2 &&
           packet.authTag == rtcmEspNowPairingAuthTag(packet,
                                                      pairingKey,
                                                      pairingKeyLength);
}

inline bool rtcmEspNowValidateGnssCommandResult(
    const GnssCommandResultPacket& packet,
    size_t receivedLength,
    uint32_t expectedNetworkId,
    const uint8_t* pairingKey,
    size_t pairingKeyLength)
{
    return receivedLength == sizeof(GnssCommandResultPacket) &&
           packet.common.magic == RTCM_ESPNOW_MAGIC &&
           packet.common.version == RTCM_ESPNOW_VERSION &&
           packet.common.packetType == RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_RESULT &&
           packet.networkId == expectedNetworkId &&
           packet.transactionId != 0 &&
           (packet.commandId ==
                RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF ||
            packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER ||
            packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK ||
            packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK) &&
           packet.status >= RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_SEQUENCE_WRITTEN &&
           packet.status <= RTCM_ESPNOW_GNSS_COMMAND_STATUS_BUSY &&
           packet.authTag == rtcmEspNowPairingAuthTag(packet,
                                                      pairingKey,
                                                      pairingKeyLength);
}

#endif // RTCM_ESPNOW_PROTOCOL_H
