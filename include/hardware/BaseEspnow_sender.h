#ifndef BASE_ESPNOW_SENDER_H
#define BASE_ESPNOW_SENDER_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

enum class BaseRtcmSourceState : uint8_t {
    LocalActive = 0,
    TempPreparing,
    TempResetting,
    TempActive,
    LocalFallback,
};

struct BaseRoverEcefStatus {
    uint8_t mac[6] = {};
    uint8_t relayMac[6] = {};
    bool viaRelay = false;
    uint32_t sequence = 0;
    uint32_t gnssTimeMsOfDay = 0;
    uint16_t correctionStreamId = 0;
    int64_t ecefXScaled = 0;
    int64_t ecefYScaled = 0;
    int64_t ecefZScaled = 0;
    uint8_t fixQuality = 0;
    bool usesTempBase = false;
    bool hasSelectedTempBase = false;
    uint8_t tempBaseMac[6] = {};
    BaseRtcmSourceState rtcmSourceState =
        BaseRtcmSourceState::LocalActive;
    uint32_t rtcmSourceEpoch = 0;
    uint32_t receivedAtMs = 0;
    bool valid = false;
};

// Snapshot for a direct or relayed Rover that has recently application-ACKed
// an RTCM frame. GNSS telemetry is optional and joined independently.
struct BaseOnlineRoverStatus {
    uint8_t mac[6] = {};
    uint8_t relayMac[6] = {};
    bool viaRelay = false;
    uint16_t lastAckStreamId = 0;
    uint32_t lastAckFrameSequence = 0;
    uint32_t lastRtcmAckAtMs = 0;
    uint16_t correctionStreamId = 0;
    uint8_t fixQuality = 0;
    uint32_t fixQualityReceivedAtMs = 0;
    bool hasFixQuality = false;
};

struct BaseRtcmSourceSnapshot {
    BaseRtcmSourceState state = BaseRtcmSourceState::LocalActive;
    uint8_t tempBaseMac[6] = {};
    uint32_t epoch = 0;
    uint32_t lastTempFrameAtMs = 0;
    uint8_t readyCycles = 0;
};

struct BaseTempRtcmFrame {
    uint16_t length = 0;
    uint16_t messageId = 0;
    uint32_t receivedAtMs = 0;
    uint8_t sourceMac[6] = {};
    uint16_t upstreamStreamId = 0;
    uint32_t upstreamSequence = 0;
    uint8_t data[1029] = {};
};

enum class BaseGnssCommandQueueResult : uint8_t {
    Queued = 0,
    InvalidArgument,
    NotReady,
    NoPairedRover,
    TargetNotPaired,
    QueueFull,
};

struct BaseGnssCommandResultEvent {
    uint8_t roverMac[6] = {};
    uint32_t transactionId = 0;
    uint8_t commandId = 0;
    uint8_t status = 0;
    uint8_t completedStep = 0;
    uint8_t totalSteps = 0;
    uint16_t detailCode = 0;
    int64_t ecefXScaled = 0;
    int64_t ecefYScaled = 0;
    int64_t ecefZScaled = 0;
    uint32_t receivedAtMs = 0;
    bool responseTimedOut = false;
};

struct BaseEspnowStats {
    uint32_t framesSent = 0;
    uint32_t framesDropped = 0;
    uint32_t fragmentsSent = 0;
    uint32_t sendFailures = 0;
    uint32_t sendTimeouts = 0;
    uint32_t frameRetries = 0;
    uint32_t frameAckTimeouts = 0;
    uint32_t ackPacketsReceived = 0;
    uint32_t ackPacketsInvalid = 0;
    uint32_t frameDeadlineDrops = 0;
    uint32_t lastFrameSendMs = 0;
    uint32_t maxFrameSendMs = 0;
    uint32_t activeRoverCount = 0;
    uint32_t storedRoverCount = 0;
    uint32_t rtcmEnabledRoverCount = 0;
    uint32_t pairResponsesReceived = 0;
    uint32_t pairConfirmsSent = 0;
    uint32_t pairAuthFailures = 0;
    uint32_t llhStatusReceived = 0;
    uint32_t llhStatusRelayedReceived = 0;
    uint32_t llhStatusInvalid = 0;
    uint32_t llhStatusUnknownSource = 0;
    uint32_t llhStatusCapacityDrops = 0;
    uint32_t relayedAckStatusReceived = 0;
    uint32_t relayedAckStatusInvalid = 0;
    uint32_t relayedAckStatusCapacityDrops = 0;
    uint32_t gnssCommandQueued = 0;
    uint32_t gnssCommandSent = 0;
    uint32_t gnssCommandResults = 0;
    uint32_t gnssCommandTimeouts = 0;
    uint32_t gnssCommandInvalidResults = 0;
    uint32_t tempFragmentsReceived = 0;
    uint32_t tempFramesValid = 0;
    uint32_t tempFramesInvalid = 0;
    uint32_t tempQueueDrops = 0;
    uint32_t tempAcksSent = 0;
    uint32_t tempAckFailures = 0;
    uint32_t tempAcksDeferred = 0;
    uint32_t tempDeferredDuplicates = 0;
    uint32_t tempForwardCompleted = 0;
    uint32_t tempForwardFailed = 0;
    uint32_t sourceSwitches = 0;
    uint32_t sourceFallbacks = 0;
    uint32_t peerCooldownEvents = 0;
    uint32_t peerCooldownSkips = 0;
    uint32_t peerRecoveries = 0;
    bool pairingActive = false;
};

bool setupEspNowBase();
void baseEspNowLoop();
void baseEspNowMarkLocalBaseReady();
bool baseEspNowSendRtcmFrame(const uint8_t* frame, size_t length);
bool baseEspNowShouldForwardLocalRtcm();
bool baseEspNowPopTempRtcmFrame(BaseTempRtcmFrame& frame, TickType_t waitTicks);
void baseEspNowCompleteTempRtcmForward(const BaseTempRtcmFrame& frame,
                                       bool downstreamDelivered);
BaseRtcmSourceSnapshot getBaseRtcmSourceSnapshot();
bool baseEspNowGnssCommandPending();
const char* baseRtcmSourceStateToString(BaseRtcmSourceState state);
BaseEspnowStats getBaseEspnowStats();
uint16_t getBaseEspNowStreamId();
size_t baseEspNowCopyLatestRoverEcef(BaseRoverEcefStatus* destination,
                                     size_t capacity);
size_t baseEspNowCopyOnlineRovers(
    BaseOnlineRoverStatus* destination,
    size_t capacity,
    uint32_t now,
    uint32_t onlineWindowMs);
BaseGnssCommandQueueResult baseEspNowQueueFirstRoverMode(
    uint32_t transactionId,
    uint8_t targetMac[6]);
BaseGnssCommandQueueResult baseEspNowQueueRoverMode(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    uint8_t selectedTargetMac[6]);
BaseGnssCommandQueueResult baseEspNowQueueRoverBaseFixedEcef(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    int64_t ecefXScaled,
    int64_t ecefYScaled,
    int64_t ecefZScaled,
    uint8_t selectedTargetMac[6]);
BaseGnssCommandQueueResult baseEspNowQueueRoverRtkReset(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    uint8_t selectedTargetMac[6]);
bool baseEspNowIsDirectRoverPaired(const uint8_t targetMac[6]);
bool baseEspNowPopGnssCommandResult(BaseGnssCommandResultEvent& result);
const char* baseGnssCommandQueueResultToString(BaseGnssCommandQueueResult result);

#endif // BASE_ESPNOW_SENDER_H
