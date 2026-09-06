#include "hardware/BaseEspnow_sender.h"

#include <Preferences.h>
#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "functions/Rtcm_Frame_Reader.h"

void beginResetCohort();
void observeResetCohortStatus(const uint8_t* roverMac,
                              uint8_t fixQuality,
                              uint32_t receivedAtMs);
void markResetBranchAcked(const uint8_t* branchMac);
void progressResetCohort(uint32_t now);

namespace {
struct RoverPeer {
    uint8_t mac[6];
    bool stored;
    bool rtcmEnabled;
    uint8_t consecutiveFrameFailures;
    uint32_t cooldownUntilMs;
    uint16_t lastAckStreamId;
    uint32_t lastAckFrameSequence;
    uint32_t lastRtcmAckAtMs;
};

struct RelayedRoverAckState {
    uint8_t roverMac[6];
    uint8_t relayMac[6];
    uint16_t streamId;
    uint32_t frameSequence;
    uint32_t lastRtcmAckAtMs;
    bool valid;
};

struct QueuedGnssCommand {
    uint8_t roverMac[6];
    uint32_t transactionId;
    int64_t ecefXScaled;
    int64_t ecefYScaled;
    int64_t ecefZScaled;
    uint8_t commandId;
};

struct TempRtcmRxPacket {
    uint8_t sourceMac[6];
    uint16_t length;
    uint8_t data[RTCM_ESPNOW_MAX_PACKET_SIZE];
};

struct TempRtcmReassemblyState {
    bool active;
    uint16_t streamId;
    uint32_t frameSequence;
    uint16_t frameLength;
    uint8_t fragmentCount;
    uint8_t receivedMask;
    uint32_t startedAtMs;
    uint8_t frame[RTCM_ESPNOW_MAX_FRAME_LENGTH];
};

struct ResetCohortEntry {
    uint8_t roverMac[6];
    uint8_t branchMac[6];
    uint32_t baselineReceivedAtMs;
    uint32_t nonRtkObservedAtMs;
};

struct ResetCohortBranch {
    uint8_t mac[6];
    uint32_t transactionId;
    uint32_t ackedAtMs;
    bool commandQueued;
    bool commandAcked;
};

struct DeferredTempAckState {
    bool pending;
    bool completed;
    uint8_t sourceMac[6];
    uint16_t streamId;
    uint32_t sequence;
};

BaseEspnowStats stats;
RoverPeer roverPeers[ESPNOW_MAX_PAIRED_ROVERS] = {};
BaseRoverEcefStatus latestRoverEcef[ESPNOW_MAX_ECEF_SOURCES] = {};
RelayedRoverAckState relayedRoverAcks[ESPNOW_MAX_ECEF_SOURCES] = {};
size_t roverPeerCount = 0;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE peerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ecefMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE relayedAckMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE pairingMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE sourceMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE resetMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE tempAckMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t streamId = 0;
uint32_t frameSequence = 0;
volatile bool lastSendSucceeded = false;
volatile bool waitingForFrameAck = false;
volatile uint16_t expectedAckStreamId = 0;
volatile uint32_t expectedAckSequence = 0;
uint8_t expectedAckMac[6] = {};
SemaphoreHandle_t sendCallbackSemaphore = nullptr;
SemaphoreHandle_t frameAckSemaphore = nullptr;
SemaphoreHandle_t espnowSendMutex = nullptr;
SemaphoreHandle_t gnssCommandResultSemaphore = nullptr;
QueueHandle_t gnssCommandQueue = nullptr;
QueueHandle_t gnssCommandResultQueue = nullptr;
QueueHandle_t tempRtcmRxQueue = nullptr;
QueueHandle_t tempRtcmFrameQueue = nullptr;
portMUX_TYPE gnssCommandMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool waitingForGnssCommandResult = false;
uint8_t expectedGnssCommandMac[6] = {};
uint32_t expectedGnssCommandTransactionId = 0;
uint8_t expectedGnssCommandId = 0;
int64_t expectedGnssCommandEcefXScaled = 0;
int64_t expectedGnssCommandEcefYScaled = 0;
int64_t expectedGnssCommandEcefZScaled = 0;
uint32_t lastNoRtcmPeerWarningAtMs = 0;
size_t nextPeerStartIndex = 0;

BaseRtcmSourceSnapshot rtcmSource{};
uint32_t tempFixedReadyAtMs = 0;
uint32_t tempPreparingStartedAtMs = 0;
uint8_t tempCycleMsmMask = 0;
bool tempCycleStarted = false;
bool resetCohortRequested = false;
ResetCohortEntry resetCohort[ESPNOW_MAX_ECEF_SOURCES] = {};
ResetCohortBranch resetBranches[ESPNOW_MAX_PAIRED_ROVERS] = {};
BaseRoverEcefStatus resetCohortSnapshots[ESPNOW_MAX_ECEF_SOURCES] = {};
size_t resetCohortCount = 0;
size_t resetBranchCount = 0;
uint32_t resetGateStartedAtMs = 0;
bool resetWaitingForFreshTempLogged = false;
DeferredTempAckState deferredTempAck{};

constexpr uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool pairingButtonReady = false;
bool pairingActive = false;
bool pendingPairResponse = false;
uint32_t pairingEndsAtMs = 0;
uint32_t pairingBaseNonce = 0;
uint32_t lastDiscoverySentAtMs = 0;
uint8_t pendingPairResponseMac[6] = {};
RtcmEspNowPairResponse pendingPairResponsePacket{};

void incrementStat(uint32_t BaseEspnowStats::*member, uint32_t amount = 1)
{
    portENTER_CRITICAL(&statsMux);
    stats.*member += amount;
    portEXIT_CRITICAL(&statsMux);
}

void setPairingActiveStat(bool active)
{
    portENTER_CRITICAL(&statsMux);
    stats.pairingActive = active;
    portEXIT_CRITICAL(&statsMux);
}

void updatePeerStats()
{
    portENTER_CRITICAL(&peerMux);
    const size_t count = roverPeerCount;
    uint32_t storedCount = 0;
    uint32_t rtcmEnabledCount = 0;
    for (size_t index = 0; index < roverPeerCount; ++index) {
        if (roverPeers[index].stored) {
            ++storedCount;
        }
        if (roverPeers[index].rtcmEnabled) {
            ++rtcmEnabledCount;
        }
    }
    portEXIT_CRITICAL(&peerMux);

    portENTER_CRITICAL(&statsMux);
    stats.activeRoverCount = static_cast<uint32_t>(count);
    stats.storedRoverCount = storedCount;
    stats.rtcmEnabledRoverCount = rtcmEnabledCount;
    portEXIT_CRITICAL(&statsMux);
}

void recordFrameSendDuration(uint32_t durationMs)
{
    portENTER_CRITICAL(&statsMux);
    stats.lastFrameSendMs = durationMs;
    if (durationMs > stats.maxFrameSendMs) {
        stats.maxFrameSendMs = durationMs;
    }
    portEXIT_CRITICAL(&statsMux);
}

bool macIsConfigured(const uint8_t* mac)
{
    return mac != nullptr &&
           (mac[0] != 0 || mac[1] != 0 || mac[2] != 0 ||
            mac[3] != 0 || mac[4] != 0 || mac[5] != 0);
}

bool macEquals(const uint8_t* a, const uint8_t* b)
{
    return a != nullptr && b != nullptr && std::memcmp(a, b, 6) == 0;
}

enum class DeferredTempAckMatch : uint8_t {
    None = 0,
    Pending,
    Completed,
};

void clearDeferredTempAck()
{
    portENTER_CRITICAL(&tempAckMux);
    deferredTempAck = {};
    portEXIT_CRITICAL(&tempAckMux);
}

DeferredTempAckMatch matchDeferredTempAck(const uint8_t* sourceMac,
                                          uint16_t streamId,
                                          uint32_t sequence)
{
    DeferredTempAckMatch match = DeferredTempAckMatch::None;
    portENTER_CRITICAL(&tempAckMux);
    if (macEquals(deferredTempAck.sourceMac, sourceMac) &&
        deferredTempAck.streamId == streamId &&
        deferredTempAck.sequence == sequence) {
        if (deferredTempAck.pending) {
            match = DeferredTempAckMatch::Pending;
        } else if (deferredTempAck.completed) {
            match = DeferredTempAckMatch::Completed;
        }
    }
    portEXIT_CRITICAL(&tempAckMux);
    return match;
}

bool stageDeferredTempAck(const uint8_t* sourceMac,
                          uint16_t streamId,
                          uint32_t sequence)
{
    bool staged = false;
    portENTER_CRITICAL(&tempAckMux);
    if (!deferredTempAck.pending) {
        deferredTempAck = {};
        deferredTempAck.pending = true;
        std::memcpy(deferredTempAck.sourceMac, sourceMac,
                    sizeof(deferredTempAck.sourceMac));
        deferredTempAck.streamId = streamId;
        deferredTempAck.sequence = sequence;
        staged = true;
    }
    portEXIT_CRITICAL(&tempAckMux);
    return staged;
}

void cancelDeferredTempAck(const uint8_t* sourceMac,
                           uint16_t streamId,
                           uint32_t sequence)
{
    portENTER_CRITICAL(&tempAckMux);
    if (deferredTempAck.pending &&
        macEquals(deferredTempAck.sourceMac, sourceMac) &&
        deferredTempAck.streamId == streamId &&
        deferredTempAck.sequence == sequence) {
        deferredTempAck = {};
    }
    portEXIT_CRITICAL(&tempAckMux);
}

String macToString(const uint8_t* mac)
{
    if (mac == nullptr) {
        return String("<null>");
    }
    char buffer[18];
    snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buffer);
}

uint16_t newStreamId()
{
    uint16_t value = 0;
    while (value == 0) {
        value = static_cast<uint16_t>(esp_random() & 0xFFFFU);
    }
    return value;
}

uint16_t rtcmMessageId(const uint8_t* frame, size_t length)
{
    if (frame == nullptr || length < 5) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(frame[3]) << 4) |
                                 (static_cast<uint16_t>(frame[4]) >> 4));
}

bool validateCompleteRtcmFrame(const uint8_t* frame, size_t length)
{
    if (frame == nullptr || length < 6 || length > RTCM_ESPNOW_MAX_FRAME_LENGTH ||
        frame[0] != 0xD3 || (frame[1] & 0xFCU) != 0) {
        return false;
    }
    const size_t payloadLength =
        (static_cast<size_t>(frame[1] & 0x03U) << 8) | frame[2];
    if (length != payloadLength + 6) {
        return false;
    }
    const uint32_t expected =
        (static_cast<uint32_t>(frame[length - 3]) << 16) |
        (static_cast<uint32_t>(frame[length - 2]) << 8) |
        static_cast<uint32_t>(frame[length - 1]);
    return rtcmCrc24q(frame, length - 3) == expected;
}

void rotateDownstreamStreamLocked()
{
    streamId = newStreamId();
    frameSequence = 0;
}

void beginTempPreparing(const uint8_t* mac)
{
    const uint32_t now = millis();
    portENTER_CRITICAL(&sourceMux);
    rtcmSource.state = BaseRtcmSourceState::TempPreparing;
    std::memcpy(rtcmSource.tempBaseMac, mac, sizeof(rtcmSource.tempBaseMac));
    rtcmSource.lastTempFrameAtMs = 0;
    rtcmSource.readyCycles = 0;
    tempPreparingStartedAtMs = now;
    tempFixedReadyAtMs = now + TEMP_RTCM_FIXED_GUARD_MS;
    tempCycleMsmMask = 0;
    tempCycleStarted = false;
    resetCohortRequested = false;
    portEXIT_CRITICAL(&sourceMux);
    clearDeferredTempAck();
    if (tempRtcmFrameQueue != nullptr) {
        xQueueReset(tempRtcmFrameQueue);
    }
    if (tempRtcmRxQueue != nullptr) {
        xQueueReset(tempRtcmRxQueue);
    }
    Serial.printf("[BASE][RTCM_SOURCE] state=TEMP_PREPARING temp=%s mode=fixed_ecef\n",
                  macToString(mac).c_str());
}

void switchToLocalFallback(const char* reason)
{
    bool fallback = false;
    bool preparationCanceled = false;
    uint32_t epoch = 0;
    uint16_t activeStreamId = 0;
    portENTER_CRITICAL(&sourceMux);
    fallback = rtcmSource.state == BaseRtcmSourceState::TempActive;
    preparationCanceled =
        rtcmSource.state == BaseRtcmSourceState::TempPreparing ||
        rtcmSource.state == BaseRtcmSourceState::TempResetting;
    if (fallback) {
        rtcmSource.state = BaseRtcmSourceState::LocalFallback;
        ++rtcmSource.epoch;
        rtcmSource.lastTempFrameAtMs = 0;
        rtcmSource.readyCycles = 0;
        tempPreparingStartedAtMs = 0;
        resetCohortRequested = false;
        rotateDownstreamStreamLocked();
    } else if (preparationCanceled) {
        rtcmSource.state = BaseRtcmSourceState::LocalActive;
        std::memset(rtcmSource.tempBaseMac, 0, sizeof(rtcmSource.tempBaseMac));
        rtcmSource.lastTempFrameAtMs = 0;
        rtcmSource.readyCycles = 0;
        tempPreparingStartedAtMs = 0;
        resetCohortRequested = false;
    }
    epoch = rtcmSource.epoch;
    activeStreamId = streamId;
    portEXIT_CRITICAL(&sourceMux);
    if (!fallback && !preparationCanceled) {
        return;
    }
    clearDeferredTempAck();
    if (tempRtcmFrameQueue != nullptr) {
        xQueueReset(tempRtcmFrameQueue);
    }
    if (tempRtcmRxQueue != nullptr) {
        xQueueReset(tempRtcmRxQueue);
    }
    if (fallback) {
        incrementStat(&BaseEspnowStats::sourceFallbacks);
        Serial.printf("[BASE][RTCM_SOURCE] state=LOCAL_FALLBACK epoch=%lu streamId=%u reason=%s\n",
                      static_cast<unsigned long>(epoch),
                      activeStreamId,
                      reason == nullptr ? "unknown" : reason);
    } else {
        Serial.printf("[BASE][RTCM_SOURCE] state=LOCAL_ACTIVE preparation_canceled reason=%s\n",
                      reason == nullptr ? "unknown" : reason);
    }
}

void activateTempSource()
{
    uint8_t mac[6] = {};
    uint32_t epoch = 0;
    uint16_t newId = 0;
    portENTER_CRITICAL(&sourceMux);
    if (rtcmSource.state != BaseRtcmSourceState::TempResetting) {
        portEXIT_CRITICAL(&sourceMux);
        return;
    }
    rtcmSource.state = BaseRtcmSourceState::TempActive;
    ++rtcmSource.epoch;
    tempPreparingStartedAtMs = 0;
    resetCohortRequested = false;
    rotateDownstreamStreamLocked();
    std::memcpy(mac, rtcmSource.tempBaseMac, sizeof(mac));
    epoch = rtcmSource.epoch;
    newId = streamId;
    portEXIT_CRITICAL(&sourceMux);
    clearDeferredTempAck();
    incrementStat(&BaseEspnowStats::sourceSwitches);
    Serial.printf("[BASE][RTCM_SOURCE] state=TEMP_ACTIVE temp=%s epoch=%lu streamId=%u\n",
                  macToString(mac).c_str(),
                  static_cast<unsigned long>(epoch),
                  newId);
}

uint8_t msmBitForMessage(uint16_t messageId)
{
    switch (messageId) {
    case 1074: return 0x01;
    case 1084: return 0x02;
    case 1094: return 0x04;
    case 1124: return 0x08;
    default: return 0;
    }
}

void updateTempReadiness(uint16_t messageId, uint32_t now)
{
    portENTER_CRITICAL(&sourceMux);
    if (rtcmSource.state == BaseRtcmSourceState::TempPreparing &&
        static_cast<int32_t>(now - tempFixedReadyAtMs) >= 0) {
        if (messageId == 1006) {
            if (tempCycleStarted && tempCycleMsmMask == 0x0F) {
                if (rtcmSource.readyCycles < UINT8_MAX) {
                    ++rtcmSource.readyCycles;
                }
            } else if (tempCycleStarted) {
                rtcmSource.readyCycles = 0;
            }
            tempCycleStarted = true;
            tempCycleMsmMask = 0;
        } else if (tempCycleStarted) {
            tempCycleMsmMask |= msmBitForMessage(messageId);
        }
        if (rtcmSource.readyCycles >= TEMP_RTCM_READY_CYCLES) {
            resetCohortRequested = true;
        }
    }
    portEXIT_CRITICAL(&sourceMux);
}

void makeRoverMacKey(size_t index, char* buffer, size_t bufferSize)
{
    snprintf(buffer, bufferSize, "%s%u",
             ESPNOW_NVS_ROVER_MAC_PREFIX,
             static_cast<unsigned>(index));
}

uint32_t localDeviceId()
{
    uint8_t mac[6] = {};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) {
        return 0;
    }
    return (static_cast<uint32_t>(mac[2]) << 24) |
           (static_cast<uint32_t>(mac[3]) << 16) |
           (static_cast<uint32_t>(mac[4]) << 8) |
           static_cast<uint32_t>(mac[5]);
}

bool addEspNowPeer(const uint8_t* mac, bool encrypted)
{
    if (!macIsConfigured(mac)) {
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    std::memcpy(peerInfo.peer_addr, mac, sizeof(peerInfo.peer_addr));
    peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = encrypted;
    if (peerInfo.encrypt) {
        std::memcpy(peerInfo.lmk, ESPNOW_LMK, sizeof(peerInfo.lmk));
    }

    const esp_err_t result = esp_now_is_peer_exist(mac)
                                 ? esp_now_mod_peer(&peerInfo)
                                 : esp_now_add_peer(&peerInfo);
    if (result != ESP_OK) {
        Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_add/mod_peer %s failed: %d\n",
                      macToString(mac).c_str(),
                      result);
        return false;
    }
    return true;
}

bool addRoverRuntimePeer(const uint8_t* mac, bool stored)
{
    if (!macIsConfigured(mac)) {
        return false;
    }

    portENTER_CRITICAL(&peerMux);
    for (size_t index = 0; index < roverPeerCount; ++index) {
        if (macEquals(roverPeers[index].mac, mac)) {
            roverPeers[index].stored = roverPeers[index].stored || stored;
            roverPeers[index].rtcmEnabled = true;
            roverPeers[index].consecutiveFrameFailures = 0;
            roverPeers[index].cooldownUntilMs = 0;
            portEXIT_CRITICAL(&peerMux);
            updatePeerStats();
            // Pairing temporarily configures this peer without encryption so
            // PAIR_CONFIRM can be delivered. Always restore the runtime LMK.
            return addEspNowPeer(mac, ESPNOW_ENCRYPTION_ENABLED);
        }
    }

    if (roverPeerCount >= ESPNOW_MAX_PAIRED_ROVERS) {
        portEXIT_CRITICAL(&peerMux);
        Serial.printf("[BASE][PAIR][WARN] Rover list full, ignore %s\n",
                      macToString(mac).c_str());
        return false;
    }

    std::memcpy(roverPeers[roverPeerCount].mac, mac, 6);
    roverPeers[roverPeerCount].stored = stored;
    roverPeers[roverPeerCount].rtcmEnabled = true;
    roverPeers[roverPeerCount].consecutiveFrameFailures = 0;
    roverPeers[roverPeerCount].cooldownUntilMs = 0;
    ++roverPeerCount;
    portEXIT_CRITICAL(&peerMux);

    updatePeerStats();
    return addEspNowPeer(mac, ESPNOW_ENCRYPTION_ENABLED);
}

bool findRoverPeerIndex(const uint8_t* mac, size_t& selectedIndex)
{
    bool found = false;
    portENTER_CRITICAL(&peerMux);
    for (size_t index = 0; index < roverPeerCount; ++index) {
        if (macEquals(roverPeers[index].mac, mac)) {
            selectedIndex = index;
            found = true;
            break;
        }
    }
    portEXIT_CRITICAL(&peerMux);
    return found;
}

bool storeLatestRoverEcef(const uint8_t* roverMac,
                          const uint8_t* relayMac,
                          bool viaRelay,
                          uint32_t sequence,
                          uint32_t gnssTimeMsOfDay,
                          uint16_t correctionStreamId,
                          int64_t ecefXScaled,
                          int64_t ecefYScaled,
                          int64_t ecefZScaled,
                          uint8_t fixQuality)
{
    const uint32_t receivedAtMs = millis();
    const BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
    size_t selectedIndex = ESPNOW_MAX_ECEF_SOURCES;
    size_t freeIndex = ESPNOW_MAX_ECEF_SOURCES;
    portENTER_CRITICAL(&ecefMux);
    for (size_t index = 0; index < ESPNOW_MAX_ECEF_SOURCES; ++index) {
        if (latestRoverEcef[index].valid &&
            macEquals(latestRoverEcef[index].mac, roverMac)) {
            selectedIndex = index;
            break;
        }
        if (!latestRoverEcef[index].valid &&
            freeIndex == ESPNOW_MAX_ECEF_SOURCES) {
            freeIndex = index;
        }
    }
    if (selectedIndex == ESPNOW_MAX_ECEF_SOURCES) {
        selectedIndex = freeIndex;
    }
    if (selectedIndex == ESPNOW_MAX_ECEF_SOURCES) {
        portEXIT_CRITICAL(&ecefMux);
        return false;
    }

    BaseRoverEcefStatus& latest = latestRoverEcef[selectedIndex];
    std::memcpy(latest.mac, roverMac, sizeof(latest.mac));
    if (viaRelay && relayMac != nullptr) {
        std::memcpy(latest.relayMac, relayMac, sizeof(latest.relayMac));
    } else {
        std::memset(latest.relayMac, 0, sizeof(latest.relayMac));
    }
    latest.viaRelay = viaRelay;
    latest.sequence = sequence;
    latest.gnssTimeMsOfDay = gnssTimeMsOfDay;
    latest.correctionStreamId = correctionStreamId;
    latest.ecefXScaled = ecefXScaled;
    latest.ecefYScaled = ecefYScaled;
    latest.ecefZScaled = ecefZScaled;
    latest.fixQuality = fixQuality;
    latest.rtcmSourceState = source.state;
    latest.usesTempBase = source.state == BaseRtcmSourceState::TempActive;
    latest.hasSelectedTempBase =
        (source.state == BaseRtcmSourceState::TempPreparing ||
         source.state == BaseRtcmSourceState::TempResetting ||
         source.state == BaseRtcmSourceState::TempActive ||
         source.state == BaseRtcmSourceState::LocalFallback) &&
        macIsConfigured(source.tempBaseMac);
    if (latest.hasSelectedTempBase) {
        std::memcpy(latest.tempBaseMac,
                    source.tempBaseMac,
                    sizeof(latest.tempBaseMac));
    } else {
        std::memset(latest.tempBaseMac, 0, sizeof(latest.tempBaseMac));
    }
    latest.rtcmSourceEpoch = source.epoch;
    latest.receivedAtMs = receivedAtMs;
    latest.valid = true;
    portEXIT_CRITICAL(&ecefMux);
    observeResetCohortStatus(roverMac, fixQuality, receivedAtMs);
    return true;
}

bool storeRelayedRoverAck(const uint8_t* roverMac,
                          const uint8_t* relayMac,
                          uint16_t streamIdValue,
                          uint32_t frameSequenceValue,
                          uint32_t ackAgeMs)
{
    size_t selectedIndex = ESPNOW_MAX_ECEF_SOURCES;
    size_t freeIndex = ESPNOW_MAX_ECEF_SOURCES;
    portENTER_CRITICAL(&relayedAckMux);
    for (size_t index = 0; index < ESPNOW_MAX_ECEF_SOURCES; ++index) {
        if (relayedRoverAcks[index].valid &&
            macEquals(relayedRoverAcks[index].roverMac, roverMac)) {
            selectedIndex = index;
            break;
        }
        if (!relayedRoverAcks[index].valid &&
            freeIndex == ESPNOW_MAX_ECEF_SOURCES) {
            freeIndex = index;
        }
    }
    if (selectedIndex == ESPNOW_MAX_ECEF_SOURCES) {
        selectedIndex = freeIndex;
    }
    if (selectedIndex == ESPNOW_MAX_ECEF_SOURCES) {
        portEXIT_CRITICAL(&relayedAckMux);
        return false;
    }

    RelayedRoverAckState& state = relayedRoverAcks[selectedIndex];
    std::memcpy(state.roverMac, roverMac, sizeof(state.roverMac));
    std::memcpy(state.relayMac, relayMac, sizeof(state.relayMac));
    state.streamId = streamIdValue;
    state.frameSequence = frameSequenceValue;
    state.lastRtcmAckAtMs = millis() - ackAgeMs;
    state.valid = true;
    portEXIT_CRITICAL(&relayedAckMux);
    return true;
}

size_t copyRoverPeers(RoverPeer* destination, size_t capacity)
{
    portENTER_CRITICAL(&peerMux);
    const size_t count = roverPeerCount < capacity ? roverPeerCount : capacity;
    for (size_t index = 0; index < count; ++index) {
        destination[index] = roverPeers[index];
    }
    portEXIT_CRITICAL(&peerMux);
    return count;
}

bool saveRoverToNvs(const uint8_t* mac)
{
    if (!macIsConfigured(mac)) {
        return false;
    }

    Preferences preferences;
    if (!preferences.begin(ESPNOW_NVS_NAMESPACE, false)) {
        Serial.println("[BASE][PAIR][ERROR] Khong mo duoc NVS de luu Rover MAC");
        return false;
    }

    uint8_t count = preferences.getUChar(ESPNOW_NVS_ROVER_COUNT_KEY, 0);
    for (uint8_t index = 0; index < count && index < ESPNOW_MAX_PAIRED_ROVERS; ++index) {
        char key[12] = {};
        uint8_t storedMac[6] = {};
        makeRoverMacKey(index, key, sizeof(key));
        if (preferences.getBytesLength(key) == 6 &&
            preferences.getBytes(key, storedMac, sizeof(storedMac)) == 6 &&
            macEquals(storedMac, mac)) {
            preferences.end();
            addRoverRuntimePeer(mac, true);
            return true;
        }
    }

    if (count >= ESPNOW_MAX_PAIRED_ROVERS) {
        preferences.end();
        Serial.println("[BASE][PAIR][ERROR] NVS rover list full");
        return false;
    }

    char key[12] = {};
    makeRoverMacKey(count, key, sizeof(key));
    const bool ok = preferences.putBytes(key, mac, 6) == 6 &&
                    preferences.putUChar(ESPNOW_NVS_ROVER_COUNT_KEY, count + 1) == 1;
    preferences.end();

    if (ok) {
        addRoverRuntimePeer(mac, true);
    }
    return ok;
}

void loadStoredRovers()
{
    Preferences preferences;
    if (!preferences.begin(ESPNOW_NVS_NAMESPACE, true)) {
        Serial.println("[BASE][PAIR][WARN] Khong mo duoc NVS de doc Rover MAC");
        return;
    }

    const uint8_t count = preferences.getUChar(ESPNOW_NVS_ROVER_COUNT_KEY, 0);
    for (uint8_t index = 0; index < count && index < ESPNOW_MAX_PAIRED_ROVERS; ++index) {
        char key[12] = {};
        uint8_t mac[6] = {};
        makeRoverMacKey(index, key, sizeof(key));
        if (preferences.getBytesLength(key) == 6 &&
            preferences.getBytes(key, mac, sizeof(mac)) == 6 &&
            macIsConfigured(mac)) {
            addRoverRuntimePeer(mac, true);
            Serial.println("[BASE][PAIR] Loaded Rover MAC from NVS: " + macToString(mac));
        }
    }
    preferences.end();
}

bool configureMaxTxPower()
{
    if (!WiFi.setTxPower(WIFI_POWER_19_5dBm)) {
        Serial.println("[BASE][WIFI][ERROR] Khong set duoc TX power 19.5 dBm");
        return false;
    }

    int8_t actualPower = 0;
    const esp_err_t result = esp_wifi_get_max_tx_power(&actualPower);
    if (result == ESP_OK) {
        Serial.printf("[BASE][WIFI] TX power fixed raw=%d dBm=%.2f\n",
                      actualPower,
                      actualPower / 4.0f);
    } else {
        Serial.printf("[BASE][WIFI][WARN] Khong doc duoc TX power: %d\n", result);
    }
    return true;
}

void drainSendCallbackSemaphore()
{
    while (xSemaphoreTake(sendCallbackSemaphore, 0) == pdTRUE) {
    }
}

bool sendControlPacket(const uint8_t* mac, const uint8_t* packet, size_t packetLength)
{
    if (espnowSendMutex == nullptr ||
        xSemaphoreTake(espnowSendMutex, pdMS_TO_TICKS(ESPNOW_SEND_TIMEOUT_MS)) != pdTRUE) {
        return false;
    }

    drainSendCallbackSemaphore();
    lastSendSucceeded = false;
    const esp_err_t sendResult = esp_now_send(mac, packet, packetLength);
    if (sendResult != ESP_OK) {
        xSemaphoreGive(espnowSendMutex);
        Serial.printf("[BASE][PAIR][ERROR] esp_now_send control failed: %d\n", sendResult);
        return false;
    }

    const BaseType_t callbackReceived =
        xSemaphoreTake(sendCallbackSemaphore, pdMS_TO_TICKS(ESPNOW_SEND_TIMEOUT_MS));
    const bool succeeded = callbackReceived == pdTRUE && lastSendSucceeded;
    drainSendCallbackSemaphore();
    xSemaphoreGive(espnowSendMutex);
    return succeeded;
}

bool sendTempRtcmAck(const uint8_t* mac, uint16_t upstreamStreamId, uint32_t sequence)
{
    RtcmEspNowAck ack{};
    ack.magic = RTCM_ESPNOW_MAGIC;
    ack.version = RTCM_ESPNOW_VERSION;
    ack.packetType = RTCM_ESPNOW_PACKET_TYPE_TEMP_RTCM_ACK;
    ack.streamId = upstreamStreamId;
    ack.frameSequence = sequence;
    ack.status = RTCM_ESPNOW_ACK_STATUS_WRITTEN;
    if (!sendControlPacket(mac, reinterpret_cast<const uint8_t*>(&ack), sizeof(ack))) {
        incrementStat(&BaseEspnowStats::tempAckFailures);
        return false;
    }
    incrementStat(&BaseEspnowStats::tempAcksSent);
    return true;
}

bool validateTempFragmentHeader(const RtcmEspNowHeader& header, size_t packetLength)
{
    if (header.magic != RTCM_ESPNOW_MAGIC ||
        header.version != RTCM_ESPNOW_VERSION ||
        header.packetType != RTCM_ESPNOW_PACKET_TYPE_TEMP_RTCM_DATA ||
        header.frameLength < 6 || header.frameLength > RTCM_ESPNOW_MAX_FRAME_LENGTH ||
        header.fragmentCount == 0 ||
        header.fragmentCount > RTCM_ESPNOW_MAX_FRAGMENT_COUNT ||
        header.fragmentIndex >= header.fragmentCount) {
        return false;
    }
    const uint8_t expectedCount = static_cast<uint8_t>(
        (header.frameLength + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) /
        RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD);
    const size_t offset = static_cast<size_t>(header.fragmentIndex) *
                          RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;
    const size_t expectedPayload = min(RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD,
                                       static_cast<size_t>(header.frameLength) - offset);
    return header.fragmentCount == expectedCount &&
           header.payloadLength == expectedPayload &&
           packetLength == sizeof(RtcmEspNowHeader) + expectedPayload;
}

[[noreturn]] void tempRtcmReceiveTask(void*)
{
    TempRtcmReassemblyState reassembly{};
    bool haveCompleted = false;
    uint16_t completedStreamId = 0;
    uint32_t completedSequence = 0;

    while (true) {
        TempRtcmRxPacket packet{};
        if (tempRtcmRxQueue == nullptr ||
            xQueueReceive(tempRtcmRxQueue, &packet, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (reassembly.active &&
                millis() - reassembly.startedAtMs > TEMP_RTCM_REASSEMBLY_TIMEOUT_MS) {
                reassembly = {};
                incrementStat(&BaseEspnowStats::tempFramesInvalid);
            }
            continue;
        }

        bool expectedSource = false;
        BaseRtcmSourceState currentSourceState =
            BaseRtcmSourceState::LocalActive;
        portENTER_CRITICAL(&sourceMux);
        currentSourceState = rtcmSource.state;
        expectedSource = (rtcmSource.state == BaseRtcmSourceState::TempPreparing ||
                          rtcmSource.state == BaseRtcmSourceState::TempResetting ||
                          rtcmSource.state == BaseRtcmSourceState::TempActive) &&
                         macEquals(packet.sourceMac, rtcmSource.tempBaseMac);
        portEXIT_CRITICAL(&sourceMux);
        if (!expectedSource) {
            reassembly = {};
            continue;
        }

        RtcmEspNowHeader header{};
        std::memcpy(&header, packet.data, sizeof(header));
        if (!validateTempFragmentHeader(header, packet.length)) {
            incrementStat(&BaseEspnowStats::tempFramesInvalid);
            continue;
        }

        if (currentSourceState == BaseRtcmSourceState::TempActive) {
            const DeferredTempAckMatch ackMatch =
                matchDeferredTempAck(packet.sourceMac,
                                     header.streamId,
                                     header.frameSequence);
            if (ackMatch != DeferredTempAckMatch::None) {
                portENTER_CRITICAL(&sourceMux);
                rtcmSource.lastTempFrameAtMs = millis();
                portEXIT_CRITICAL(&sourceMux);
                incrementStat(&BaseEspnowStats::tempDeferredDuplicates);
                if (ackMatch == DeferredTempAckMatch::Completed) {
                    sendTempRtcmAck(packet.sourceMac,
                                    header.streamId,
                                    header.frameSequence);
                }
                continue;
            }
        } else if (haveCompleted &&
                   header.streamId == completedStreamId &&
                   header.frameSequence == completedSequence) {
            sendTempRtcmAck(packet.sourceMac, header.streamId, header.frameSequence);
            continue;
        }

        const bool sameFrame = reassembly.active &&
                               reassembly.streamId == header.streamId &&
                               reassembly.frameSequence == header.frameSequence;
        if (!sameFrame) {
            reassembly = {};
            reassembly.active = true;
            reassembly.streamId = header.streamId;
            reassembly.frameSequence = header.frameSequence;
            reassembly.frameLength = header.frameLength;
            reassembly.fragmentCount = header.fragmentCount;
            reassembly.startedAtMs = millis();
        } else if (reassembly.frameLength != header.frameLength ||
                   reassembly.fragmentCount != header.fragmentCount) {
            reassembly = {};
            incrementStat(&BaseEspnowStats::tempFramesInvalid);
            continue;
        }

        const uint8_t fragmentBit = static_cast<uint8_t>(1U << header.fragmentIndex);
        if ((reassembly.receivedMask & fragmentBit) == 0) {
            const size_t offset = static_cast<size_t>(header.fragmentIndex) *
                                  RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;
            std::memcpy(reassembly.frame + offset,
                        packet.data + sizeof(RtcmEspNowHeader),
                        header.payloadLength);
            reassembly.receivedMask |= fragmentBit;
        }

        const uint8_t expectedMask = static_cast<uint8_t>(
            (1U << reassembly.fragmentCount) - 1U);
        if (reassembly.receivedMask != expectedMask) {
            continue;
        }

        if (!validateCompleteRtcmFrame(reassembly.frame, reassembly.frameLength)) {
            reassembly = {};
            incrementStat(&BaseEspnowStats::tempFramesInvalid);
            continue;
        }

        const uint32_t now = millis();
        const uint16_t messageId = rtcmMessageId(reassembly.frame, reassembly.frameLength);
        portENTER_CRITICAL(&sourceMux);
        rtcmSource.lastTempFrameAtMs = now;
        portEXIT_CRITICAL(&sourceMux);
        updateTempReadiness(messageId, now);

        BaseRtcmSourceSnapshot sourceSnapshot{};
        portENTER_CRITICAL(&sourceMux);
        sourceSnapshot = rtcmSource;
        portEXIT_CRITICAL(&sourceMux);

        bool accepted =
            sourceSnapshot.state == BaseRtcmSourceState::TempPreparing ||
            sourceSnapshot.state == BaseRtcmSourceState::TempResetting;
        if (sourceSnapshot.state == BaseRtcmSourceState::TempActive) {
            const bool staged =
                stageDeferredTempAck(packet.sourceMac,
                                     reassembly.streamId,
                                     reassembly.frameSequence);
            if (staged) {
                BaseTempRtcmFrame output{};
                output.length = reassembly.frameLength;
                output.messageId = messageId;
                output.receivedAtMs = now;
                std::memcpy(output.sourceMac, packet.sourceMac,
                            sizeof(output.sourceMac));
                output.upstreamStreamId = reassembly.streamId;
                output.upstreamSequence = reassembly.frameSequence;
                std::memcpy(output.data, reassembly.frame, output.length);
                accepted = tempRtcmFrameQueue != nullptr &&
                           xQueueSend(tempRtcmFrameQueue, &output, 0) == pdTRUE;
            } else {
                accepted = false;
                incrementStat(&BaseEspnowStats::tempFramesInvalid);
            }
            if (!accepted && staged) {
                cancelDeferredTempAck(packet.sourceMac,
                                      reassembly.streamId,
                                      reassembly.frameSequence);
                incrementStat(&BaseEspnowStats::tempQueueDrops);
            }
        }

        if (accepted) {
            incrementStat(&BaseEspnowStats::tempFramesValid);
            if (sourceSnapshot.state == BaseRtcmSourceState::TempActive) {
                incrementStat(&BaseEspnowStats::tempAcksDeferred);
            } else {
                haveCompleted = true;
                completedStreamId = reassembly.streamId;
                completedSequence = reassembly.frameSequence;
                sendTempRtcmAck(packet.sourceMac,
                                reassembly.streamId,
                                reassembly.frameSequence);
            }
        }
        reassembly = {};
    }
}

const char* gnssCommandName(uint8_t commandId)
{
    switch (commandId) {
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF:
        return "switch_to_base_fixed_ecef";
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER:
        return "switch_to_rover";
    case RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK:
        return "rtk_reset";
    case RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK:
        return "rtk_resume";
    default:
        return "unknown";
    }
}

void setRoverRtcmEnabled(const uint8_t* mac, bool enabled)
{
    bool changed = false;
    portENTER_CRITICAL(&peerMux);
    for (size_t index = 0; index < roverPeerCount; ++index) {
        if (macEquals(roverPeers[index].mac, mac)) {
            changed = roverPeers[index].rtcmEnabled != enabled;
            roverPeers[index].rtcmEnabled = enabled;
            if (changed) {
                // Do not reuse an ACK from a previous RTCM delivery state.
                // A peer becomes online again only after a new RTCM frame is
                // delivered and application-ACKed.
                roverPeers[index].lastAckStreamId = 0;
                roverPeers[index].lastAckFrameSequence = 0;
                roverPeers[index].lastRtcmAckAtMs = 0;
            }
            if (enabled) {
                roverPeers[index].consecutiveFrameFailures = 0;
                roverPeers[index].cooldownUntilMs = 0;
            }
            break;
        }
    }
    portEXIT_CRITICAL(&peerMux);
    if (changed) {
        updatePeerStats();
    }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t*, esp_now_send_status_t status)
#else
void onDataSent(const uint8_t*, esp_now_send_status_t status)
#endif
{
    lastSendSucceeded = (status == ESP_NOW_SEND_SUCCESS);
    if (sendCallbackSemaphore != nullptr) {
        xSemaphoreGive(sendCallbackSemaphore);
    }
}

void queuePairResponse(const uint8_t* sourceMac, const uint8_t* data, int length)
{
    if (!ESPNOW_PAIRING_ENABLED || sourceMac == nullptr || data == nullptr ||
        length != static_cast<int>(sizeof(RtcmEspNowPairResponse))) {
        return;
    }

    bool active = false;
    portENTER_CRITICAL(&pairingMux);
    active = pairingActive;
    if (active) {
        std::memcpy(&pendingPairResponsePacket, data, sizeof(pendingPairResponsePacket));
        std::memcpy(pendingPairResponseMac, sourceMac, sizeof(pendingPairResponseMac));
        pendingPairResponse = true;
    }
    portEXIT_CRITICAL(&pairingMux);
}

bool peerCooldownActive(const RoverPeer& peer, uint32_t now)
{
    return peer.cooldownUntilMs != 0 &&
           static_cast<int32_t>(now - peer.cooldownUntilMs) < 0;
}

void recordPeerDeliveryResult(const uint8_t* mac,
                              bool delivered,
                              uint16_t acknowledgedStreamId,
                              uint32_t acknowledgedFrameSequence)
{
    bool enteredCooldown = false;
    bool recovered = false;
    uint8_t failureCount = 0;
    uint32_t cooldownMs = 0;
    const uint32_t now = millis();

    portENTER_CRITICAL(&peerMux);
    for (size_t index = 0; index < roverPeerCount; ++index) {
        RoverPeer& peer = roverPeers[index];
        if (!macEquals(peer.mac, mac)) {
            continue;
        }
        if (delivered) {
            recovered = peer.consecutiveFrameFailures != 0 ||
                        peer.cooldownUntilMs != 0;
            peer.consecutiveFrameFailures = 0;
            peer.cooldownUntilMs = 0;
            peer.lastAckStreamId = acknowledgedStreamId;
            peer.lastAckFrameSequence = acknowledgedFrameSequence;
            peer.lastRtcmAckAtMs = now;
        } else {
            if (peer.consecutiveFrameFailures < UINT8_MAX) {
                ++peer.consecutiveFrameFailures;
            }
            failureCount = peer.consecutiveFrameFailures;
            if (peer.consecutiveFrameFailures >=
                ESPNOW_PEER_FAILURES_BEFORE_COOLDOWN) {
                peer.cooldownUntilMs = now + ESPNOW_PEER_FAILURE_COOLDOWN_MS;
                cooldownMs = ESPNOW_PEER_FAILURE_COOLDOWN_MS;
                enteredCooldown = true;
            }
        }
        break;
    }
    portEXIT_CRITICAL(&peerMux);

    if (enteredCooldown) {
        incrementStat(&BaseEspnowStats::peerCooldownEvents);
        Serial.printf("[BASE][ESP-NOW][PEER] mac=%s state=cooldown failures=%u duration_ms=%lu\n",
                      macToString(mac).c_str(),
                      static_cast<unsigned>(failureCount),
                      static_cast<unsigned long>(cooldownMs));
    } else if (recovered) {
        incrementStat(&BaseEspnowStats::peerRecoveries);
        Serial.printf("[BASE][ESP-NOW][PEER] mac=%s state=recovered\n",
                      macToString(mac).c_str());
    }
}

void queueTempRtcmFragment(const uint8_t* sourceMac, const uint8_t* data, int length)
{
    bool expectedSource = false;
    portENTER_CRITICAL(&sourceMux);
    expectedSource = (rtcmSource.state == BaseRtcmSourceState::TempPreparing ||
                      rtcmSource.state == BaseRtcmSourceState::TempResetting ||
                      rtcmSource.state == BaseRtcmSourceState::TempActive) &&
                     macEquals(sourceMac, rtcmSource.tempBaseMac);
    portEXIT_CRITICAL(&sourceMux);
    if (!expectedSource || length < static_cast<int>(sizeof(RtcmEspNowHeader)) ||
        length > static_cast<int>(RTCM_ESPNOW_MAX_PACKET_SIZE)) {
        incrementStat(&BaseEspnowStats::tempFramesInvalid);
        return;
    }

    TempRtcmRxPacket packet{};
    std::memcpy(packet.sourceMac, sourceMac, sizeof(packet.sourceMac));
    packet.length = static_cast<uint16_t>(length);
    std::memcpy(packet.data, data, static_cast<size_t>(length));
    if (tempRtcmRxQueue == nullptr || xQueueSend(tempRtcmRxQueue, &packet, 0) != pdTRUE) {
        incrementStat(&BaseEspnowStats::tempQueueDrops);
        return;
    }
    incrementStat(&BaseEspnowStats::tempFragmentsReceived);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int length)
{
    const uint8_t* sourceMac = info == nullptr ? nullptr : info->src_addr;
#else
void onDataReceived(const uint8_t* sourceMac, const uint8_t* data, int length)
#endif
{
    if (sourceMac == nullptr || data == nullptr ||
        length < static_cast<int>(sizeof(RtcmEspNowCommonHeader))) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    RtcmEspNowCommonHeader common{};
    std::memcpy(&common, data, sizeof(common));
    if (common.magic != RTCM_ESPNOW_MAGIC || common.version != RTCM_ESPNOW_VERSION) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    if (common.packetType == RTCM_ESPNOW_PACKET_TYPE_PAIR_RESPONSE) {
        queuePairResponse(sourceMac, data, length);
        return;
    }

    if (common.packetType == RTCM_ESPNOW_PACKET_TYPE_TEMP_RTCM_DATA) {
        queueTempRtcmFragment(sourceMac, data, length);
        return;
    }

    if (common.packetType == RTCM_ESPNOW_PACKET_TYPE_ROVER_ECEF_STATUS) {
        if (length != static_cast<int>(sizeof(RoverEcefStatusPacket))) {
            incrementStat(&BaseEspnowStats::llhStatusInvalid);
            return;
        }
        RoverEcefStatusPacket packet{};
        std::memcpy(&packet, data, sizeof(packet));
        if (!rtcmEspNowValidateRoverEcefStatus(packet, sizeof(packet))) {
            incrementStat(&BaseEspnowStats::llhStatusInvalid);
            return;
        }

        size_t ignoredIndex = 0;
        if (!findRoverPeerIndex(sourceMac, ignoredIndex)) {
            incrementStat(&BaseEspnowStats::llhStatusUnknownSource);
            return;
        }
        if (!storeLatestRoverEcef(sourceMac, nullptr, false,
                                  packet.sequence,
                                  packet.gnssTimeMsOfDay,
                                  packet.correctionStreamId,
                                  packet.ecefXScaled,
                                  packet.ecefYScaled,
                                  packet.ecefZScaled,
                                  packet.fixQuality)) {
            incrementStat(&BaseEspnowStats::llhStatusCapacityDrops);
            return;
        }
        incrementStat(&BaseEspnowStats::llhStatusReceived);
        return;
    }

    if (common.packetType ==
        RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_ECEF_STATUS) {
        if (length != static_cast<int>(sizeof(RelayedRoverEcefStatusPacket))) {
            incrementStat(&BaseEspnowStats::llhStatusInvalid);
            return;
        }
        RelayedRoverEcefStatusPacket packet{};
        std::memcpy(&packet, data, sizeof(packet));
        if (!rtcmEspNowValidateRelayedRoverEcefStatus(packet,
                                                      sizeof(packet))) {
            incrementStat(&BaseEspnowStats::llhStatusInvalid);
            return;
        }
        size_t ignoredIndex = 0;
        if (!findRoverPeerIndex(sourceMac, ignoredIndex)) {
            incrementStat(&BaseEspnowStats::llhStatusUnknownSource);
            return;
        }
        if (!storeLatestRoverEcef(packet.roverMac, sourceMac, true,
                                  packet.sequence,
                                  packet.gnssTimeMsOfDay,
                                  packet.correctionStreamId,
                                  packet.ecefXScaled,
                                  packet.ecefYScaled,
                                  packet.ecefZScaled,
                                  packet.fixQuality)) {
            incrementStat(&BaseEspnowStats::llhStatusCapacityDrops);
            return;
        }
        incrementStat(&BaseEspnowStats::llhStatusReceived);
        incrementStat(&BaseEspnowStats::llhStatusRelayedReceived);
        return;
    }

    if (common.packetType ==
        RTCM_ESPNOW_PACKET_TYPE_RELAYED_ROVER_RTCM_ACK_STATUS) {
        if (length !=
            static_cast<int>(sizeof(RelayedRoverRtcmAckStatusPacket))) {
            incrementStat(&BaseEspnowStats::relayedAckStatusInvalid);
            return;
        }
        RelayedRoverRtcmAckStatusPacket packet{};
        std::memcpy(&packet, data, sizeof(packet));
        size_t ignoredIndex = 0;
        if (!findRoverPeerIndex(sourceMac, ignoredIndex) ||
            !rtcmEspNowValidateRelayedRoverRtcmAckStatus(packet,
                                                         sizeof(packet))) {
            incrementStat(&BaseEspnowStats::relayedAckStatusInvalid);
            return;
        }
        if (!storeRelayedRoverAck(packet.roverMac,
                                  sourceMac,
                                  packet.streamId,
                                  packet.frameSequence,
                                  packet.ackAgeMs)) {
            incrementStat(&BaseEspnowStats::relayedAckStatusCapacityDrops);
            return;
        }
        incrementStat(&BaseEspnowStats::relayedAckStatusReceived);
        return;
    }

    if (common.packetType == RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_RESULT) {
        if (length != static_cast<int>(sizeof(GnssCommandResultPacket))) {
            incrementStat(&BaseEspnowStats::gnssCommandInvalidResults);
            return;
        }
        GnssCommandResultPacket packet{};
        std::memcpy(&packet, data, sizeof(packet));
        size_t ignoredIndex = 0;
        if (!findRoverPeerIndex(sourceMac, ignoredIndex) ||
            !rtcmEspNowValidateGnssCommandResult(packet,
                                                 sizeof(packet),
                                                 ESPNOW_NETWORK_ID,
                                                 ESPNOW_PAIRING_KEY,
                                                 sizeof(ESPNOW_PAIRING_KEY))) {
            incrementStat(&BaseEspnowStats::gnssCommandInvalidResults);
            return;
        }

        bool matches = false;
        portENTER_CRITICAL(&gnssCommandMux);
        matches = waitingForGnssCommandResult &&
                  macEquals(sourceMac, expectedGnssCommandMac) &&
                  packet.transactionId == expectedGnssCommandTransactionId &&
                  packet.commandId == expectedGnssCommandId;
        portEXIT_CRITICAL(&gnssCommandMux);
        if (!matches) {
            incrementStat(&BaseEspnowStats::gnssCommandInvalidResults);
            return;
        }

        BaseGnssCommandResultEvent event{};
        std::memcpy(event.roverMac, sourceMac, sizeof(event.roverMac));
        event.transactionId = packet.transactionId;
        event.commandId = packet.commandId;
        event.status = packet.status;
        event.completedStep = packet.completedStep;
        event.totalSteps = packet.totalSteps;
        event.detailCode = packet.detailCode;
        event.ecefXScaled = expectedGnssCommandEcefXScaled;
        event.ecefYScaled = expectedGnssCommandEcefYScaled;
        event.ecefZScaled = expectedGnssCommandEcefZScaled;
        event.receivedAtMs = millis();
        if (gnssCommandResultQueue != nullptr &&
            xQueueSend(gnssCommandResultQueue, &event, 0) != pdTRUE) {
            BaseGnssCommandResultEvent discarded{};
            xQueueReceive(gnssCommandResultQueue, &discarded, 0);
            xQueueSend(gnssCommandResultQueue, &event, 0);
        }
        incrementStat(&BaseEspnowStats::gnssCommandResults);
        if (packet.status == RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_SEQUENCE_WRITTEN) {
            if (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK) {
                markResetBranchAcked(sourceMac);
            }
            const bool enableRtcm =
                packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER ||
                packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK;
            setRoverRtcmEnabled(sourceMac, enableRtcm);
            if (packet.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER) {
                switchToLocalFallback("temp_base_returned_to_rover");
            }
            Serial.printf("[BASE][GNSS_CMD] RTCM delivery %s for peer %s action=%s\n",
                          enableRtcm ? "enabled" : "disabled",
                          macToString(sourceMac).c_str(),
                          gnssCommandName(packet.commandId));
        } else if (packet.commandId ==
                   RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF) {
            switchToLocalFallback("temp_base_command_rejected");
        }
        if (gnssCommandResultSemaphore != nullptr) {
            xSemaphoreGive(gnssCommandResultSemaphore);
        }
        return;
    }

    if (length != static_cast<int>(sizeof(RtcmEspNowAck))) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    RtcmEspNowAck ack{};
    std::memcpy(&ack, data, sizeof(ack));
    if (ack.packetType != RTCM_ESPNOW_PACKET_TYPE_FRAME_ACK ||
        ack.status != RTCM_ESPNOW_ACK_STATUS_WRITTEN) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    if (!waitingForFrameAck || !macEquals(sourceMac, expectedAckMac) ||
        ack.streamId != expectedAckStreamId ||
        ack.frameSequence != expectedAckSequence) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    incrementStat(&BaseEspnowStats::ackPacketsReceived);
    if (frameAckSemaphore != nullptr) {
        xSemaphoreGive(frameAckSemaphore);
    }
}

bool frameDeadlineExpired(uint32_t startedAt)
{
    return millis() - startedAt >= ESPNOW_FRAME_SEND_DEADLINE_MS;
}

bool sendPacketWithRetry(const uint8_t* peerMac,
                         const uint8_t* packet,
                         size_t packetLength,
                         uint32_t frameStartedAt)
{
    for (uint8_t attempt = 0; attempt <= ESPNOW_SEND_RETRY_COUNT; ++attempt) {
        if (frameDeadlineExpired(frameStartedAt)) {
            incrementStat(&BaseEspnowStats::frameDeadlineDrops);
            return false;
        }

        if (xSemaphoreTake(espnowSendMutex, pdMS_TO_TICKS(ESPNOW_SEND_TIMEOUT_MS)) != pdTRUE) {
            incrementStat(&BaseEspnowStats::sendTimeouts);
            return false;
        }

        drainSendCallbackSemaphore();
        lastSendSucceeded = false;

        const esp_err_t sendResult = esp_now_send(peerMac, packet, packetLength);
        if (sendResult != ESP_OK) {
            xSemaphoreGive(espnowSendMutex);
            incrementStat(&BaseEspnowStats::sendFailures);
            Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_send %s failed: %d\n",
                          macToString(peerMac).c_str(),
                          sendResult);
            delay(5);
            continue;
        }

        const uint32_t elapsedMs = millis() - frameStartedAt;
        const uint32_t deadlineRemaining = elapsedMs < ESPNOW_FRAME_SEND_DEADLINE_MS
                                               ? ESPNOW_FRAME_SEND_DEADLINE_MS - elapsedMs
                                               : 0;
        const uint32_t waitMs = min(ESPNOW_SEND_TIMEOUT_MS, deadlineRemaining);
        const BaseType_t callbackReceived =
            waitMs > 0
                ? xSemaphoreTake(sendCallbackSemaphore, pdMS_TO_TICKS(waitMs))
                : pdFALSE;
        xSemaphoreGive(espnowSendMutex);

        if (callbackReceived == pdTRUE && lastSendSucceeded) {
            incrementStat(&BaseEspnowStats::fragmentsSent);
            return true;
        }

        if (waitMs == 0 || frameDeadlineExpired(frameStartedAt)) {
            incrementStat(&BaseEspnowStats::frameDeadlineDrops);
            return false;
        }

        if (callbackReceived != pdTRUE) {
            incrementStat(&BaseEspnowStats::sendTimeouts);
            Serial.println("[BASE][ESP-NOW][WARN] Send callback timeout");
            return false;
        } else {
            incrementStat(&BaseEspnowStats::sendFailures);
            Serial.println("[BASE][ESP-NOW][WARN] Send callback reported failure");
        }
        delay(5);
    }

    return false;
}

bool sendFrameToPeer(const uint8_t* peerMac,
                     const uint8_t* frame,
                     size_t length,
                     uint16_t currentStreamId,
                     uint32_t currentSequence)
{
    uint8_t packet[RTCM_ESPNOW_MAX_PACKET_SIZE] = {};
    const uint8_t fragmentCount = static_cast<uint8_t>(
        (length + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) / RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD);

    while (xSemaphoreTake(frameAckSemaphore, 0) == pdTRUE) {
    }
    std::memcpy(expectedAckMac, peerMac, 6);
    expectedAckStreamId = currentStreamId;
    expectedAckSequence = currentSequence;
    waitingForFrameAck = true;

    for (uint8_t frameAttempt = 0; frameAttempt <= ESPNOW_FRAME_RETRY_COUNT; ++frameAttempt) {
        const uint32_t frameAttemptStartedAt = millis();
        bool allFragmentsSent = true;

        for (uint8_t fragmentIndex = 0; fragmentIndex < fragmentCount; ++fragmentIndex) {
            const size_t offset =
                static_cast<size_t>(fragmentIndex) * RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;
            const size_t payloadLength = min(RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD, length - offset);

            auto* header = reinterpret_cast<RtcmEspNowHeader*>(packet);
            header->magic = RTCM_ESPNOW_MAGIC;
            header->version = RTCM_ESPNOW_VERSION;
            header->packetType = RTCM_ESPNOW_PACKET_TYPE_RTCM_DATA;
            header->streamId = currentStreamId;
            header->frameSequence = currentSequence;
            header->frameLength = static_cast<uint16_t>(length);
            header->fragmentIndex = fragmentIndex;
            header->fragmentCount = fragmentCount;
            header->payloadLength = static_cast<uint16_t>(payloadLength);

            std::memcpy(packet + sizeof(RtcmEspNowHeader), frame + offset, payloadLength);

            const size_t packetLength = sizeof(RtcmEspNowHeader) + payloadLength;
            if (!sendPacketWithRetry(peerMac, packet, packetLength, frameAttemptStartedAt)) {
                allFragmentsSent = false;
                Serial.printf("[BASE][ESP-NOW][WARN] Peer=%s seq=%lu attempt=%u stopped at fragment %u/%u\n",
                              macToString(peerMac).c_str(),
                              static_cast<unsigned long>(currentSequence),
                              frameAttempt + 1,
                              fragmentIndex + 1,
                              fragmentCount);
                break;
            }
        }

        BaseType_t ackReceived = xSemaphoreTake(frameAckSemaphore, 0);
        if (ackReceived != pdTRUE && allFragmentsSent) {
            ackReceived = xSemaphoreTake(frameAckSemaphore,
                                         pdMS_TO_TICKS(ESPNOW_FRAME_ACK_TIMEOUT_MS));
        }
        if (ackReceived == pdTRUE) {
            waitingForFrameAck = false;
            return true;
        }

        if (allFragmentsSent) {
            incrementStat(&BaseEspnowStats::frameAckTimeouts);
            Serial.printf("[BASE][ESP-NOW][WARN] Frame ACK timeout peer=%s seq=%lu attempt=%u\n",
                          macToString(peerMac).c_str(),
                          static_cast<unsigned long>(currentSequence),
                          frameAttempt + 1);
        }
        if (frameAttempt < ESPNOW_FRAME_RETRY_COUNT) {
            incrementStat(&BaseEspnowStats::frameRetries);
            delay(10);
        }
    }

    waitingForFrameAck = false;
    return false;
}

void setupPairingButton()
{
    if (!ESPNOW_PAIRING_ENABLED || pairingButtonReady) {
        return;
    }
    pinMode(PAIRING_BUTTON_PIN, PAIRING_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    pairingButtonReady = true;
    Serial.printf("[BASE][PAIR] Pairing button GPIO=%d active_%s hold_ms=%lu\n",
                  PAIRING_BUTTON_PIN,
                  PAIRING_BUTTON_ACTIVE_LOW ? "low" : "high",
                  static_cast<unsigned long>(PAIRING_BUTTON_HOLD_MS));
}

void startPairingMode()
{
    const uint32_t now = millis();
    portENTER_CRITICAL(&pairingMux);
    pairingActive = true;
    pendingPairResponse = false;
    pairingEndsAtMs = now + PAIRING_WINDOW_MS;
    pairingBaseNonce = esp_random();
    lastDiscoverySentAtMs = 0;
    portEXIT_CRITICAL(&pairingMux);
    setPairingActiveStat(true);
    Serial.printf("[BASE][PAIR] Pairing mode ON for %lu ms, network_id=0x%08lX\n",
                  static_cast<unsigned long>(PAIRING_WINDOW_MS),
                  static_cast<unsigned long>(ESPNOW_NETWORK_ID));
}

void stopPairingMode(const char* reason)
{
    portENTER_CRITICAL(&pairingMux);
    pairingActive = false;
    pendingPairResponse = false;
    portEXIT_CRITICAL(&pairingMux);
    setPairingActiveStat(false);
    Serial.print("[BASE][PAIR] Pairing mode OFF");
    if (reason != nullptr) {
        Serial.print(": ");
        Serial.print(reason);
    }
    Serial.println();
}

void sendPairDiscoveryIfDue()
{
    bool active = false;
    uint32_t nonce = 0;
    uint32_t lastSent = 0;
    const uint32_t now = millis();

    portENTER_CRITICAL(&pairingMux);
    active = pairingActive;
    nonce = pairingBaseNonce;
    lastSent = lastDiscoverySentAtMs;
    if (active && (lastSent == 0 || now - lastSent >= PAIR_DISCOVERY_INTERVAL_MS)) {
        lastDiscoverySentAtMs = now;
        lastSent = 0;
    }
    portEXIT_CRITICAL(&pairingMux);

    if (!active || lastSent != 0) {
        return;
    }

    RtcmEspNowPairDiscovery discovery{};
    discovery.common.magic = RTCM_ESPNOW_MAGIC;
    discovery.common.version = RTCM_ESPNOW_VERSION;
    discovery.common.packetType = RTCM_ESPNOW_PACKET_TYPE_PAIR_DISCOVERY;
    discovery.role = RTCM_ESPNOW_ROLE_BASE;
    discovery.networkId = ESPNOW_NETWORK_ID;
    discovery.baseDeviceId = localDeviceId();
    discovery.baseNonce = nonce;
    discovery.pairingWindowMs = PAIRING_WINDOW_MS;
    discovery.authTag = rtcmEspNowPairingAuthTag(discovery,
                                                 ESPNOW_PAIRING_KEY,
                                                 sizeof(ESPNOW_PAIRING_KEY));

    sendControlPacket(BROADCAST_MAC,
                      reinterpret_cast<const uint8_t*>(&discovery),
                      sizeof(discovery));
}

void processPendingPairResponse()
{
    RtcmEspNowPairResponse response{};
    uint8_t sourceMac[6] = {};
    uint32_t expectedNonce = 0;
    bool hasPending = false;

    portENTER_CRITICAL(&pairingMux);
    hasPending = pendingPairResponse;
    expectedNonce = pairingBaseNonce;
    if (hasPending) {
        response = pendingPairResponsePacket;
        std::memcpy(sourceMac, pendingPairResponseMac, sizeof(sourceMac));
        pendingPairResponse = false;
    }
    portEXIT_CRITICAL(&pairingMux);

    if (!hasPending) {
        return;
    }

    if (!rtcmEspNowValidatePairResponse(response,
                                        sizeof(response),
                                        ESPNOW_NETWORK_ID,
                                        expectedNonce,
                                        ESPNOW_PAIRING_KEY,
                                        sizeof(ESPNOW_PAIRING_KEY))) {
        incrementStat(&BaseEspnowStats::pairAuthFailures);
        Serial.println("[BASE][PAIR][WARN] Invalid PAIR_RESPONSE from " + macToString(sourceMac));
        return;
    }

    if (!addEspNowPeer(sourceMac, false)) {
        return;
    }

    RtcmEspNowPairConfirm confirm{};
    confirm.common.magic = RTCM_ESPNOW_MAGIC;
    confirm.common.version = RTCM_ESPNOW_VERSION;
    confirm.common.packetType = RTCM_ESPNOW_PACKET_TYPE_PAIR_CONFIRM;
    confirm.role = RTCM_ESPNOW_ROLE_BASE;
    confirm.networkId = ESPNOW_NETWORK_ID;
    confirm.baseNonce = response.baseNonceEcho;
    confirm.roverNonce = response.roverNonce;
    confirm.authTag = rtcmEspNowPairingAuthTag(confirm,
                                               ESPNOW_PAIRING_KEY,
                                               sizeof(ESPNOW_PAIRING_KEY));

    if (!sendControlPacket(sourceMac,
                           reinterpret_cast<const uint8_t*>(&confirm),
                           sizeof(confirm))) {
        Serial.println("[BASE][PAIR][ERROR] Gui PAIR_CONFIRM that bai");
        return;
    }

    incrementStat(&BaseEspnowStats::pairResponsesReceived);
    incrementStat(&BaseEspnowStats::pairConfirmsSent);
    if (!saveRoverToNvs(sourceMac)) {
        Serial.println("[BASE][PAIR][ERROR] Luu Rover MAC vao NVS that bai");
        return;
    }

    Serial.println("[BASE][PAIR] Paired Rover " + macToString(sourceMac));
    stopPairingMode("paired");
}

void queueLocalGnssCommandTimeout(const QueuedGnssCommand& command)
{
    BaseGnssCommandResultEvent event{};
    std::memcpy(event.roverMac, command.roverMac, sizeof(event.roverMac));
    event.transactionId = command.transactionId;
    event.commandId = command.commandId;
    event.ecefXScaled = command.ecefXScaled;
    event.ecefYScaled = command.ecefYScaled;
    event.ecefZScaled = command.ecefZScaled;
    event.receivedAtMs = millis();
    event.responseTimedOut = true;
    if (gnssCommandResultQueue != nullptr &&
        xQueueSend(gnssCommandResultQueue, &event, 0) != pdTRUE) {
        BaseGnssCommandResultEvent discarded{};
        xQueueReceive(gnssCommandResultQueue, &discarded, 0);
        xQueueSend(gnssCommandResultQueue, &event, 0);
    }
}

void processNextGnssCommand()
{
    if (gnssCommandQueue == nullptr || gnssCommandResultSemaphore == nullptr) {
        return;
    }

    QueuedGnssCommand command{};
    if (xQueueReceive(gnssCommandQueue, &command, 0) != pdTRUE) {
        return;
    }

    while (xSemaphoreTake(gnssCommandResultSemaphore, 0) == pdTRUE) {
    }
    portENTER_CRITICAL(&gnssCommandMux);
    std::memcpy(expectedGnssCommandMac, command.roverMac,
                sizeof(expectedGnssCommandMac));
    expectedGnssCommandTransactionId = command.transactionId;
    expectedGnssCommandId = command.commandId;
    expectedGnssCommandEcefXScaled = command.ecefXScaled;
    expectedGnssCommandEcefYScaled = command.ecefYScaled;
    expectedGnssCommandEcefZScaled = command.ecefZScaled;
    waitingForGnssCommandResult = true;
    portEXIT_CRITICAL(&gnssCommandMux);

    GnssCommandRequestPacket packet{};
    packet.common.magic = RTCM_ESPNOW_MAGIC;
    packet.common.version = RTCM_ESPNOW_VERSION;
    packet.common.packetType = RTCM_ESPNOW_PACKET_TYPE_GNSS_COMMAND_REQUEST;
    packet.networkId = ESPNOW_NETWORK_ID;
    packet.transactionId = command.transactionId;
    packet.ecefXScaled = command.ecefXScaled;
    packet.ecefYScaled = command.ecefYScaled;
    packet.ecefZScaled = command.ecefZScaled;
    packet.commandId = command.commandId;
    packet.targetPort = RTCM_ESPNOW_GNSS_PORT_COM2;
    packet.authTag = rtcmEspNowPairingAuthTag(packet,
                                              ESPNOW_PAIRING_KEY,
                                              sizeof(ESPNOW_PAIRING_KEY));

    if (command.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF) {
        beginTempPreparing(command.roverMac);
    }

    bool resultReceived = false;
    for (uint8_t attempt = 0;
         attempt <= ESPNOW_GNSS_COMMAND_SEND_RETRY_COUNT && !resultReceived;
         ++attempt) {
        if (!sendControlPacket(command.roverMac,
                               reinterpret_cast<const uint8_t*>(&packet),
                               sizeof(packet))) {
            Serial.printf("[BASE][GNSS_CMD][WARN] Radio send failed txn=%lu attempt=%u\n",
                          static_cast<unsigned long>(command.transactionId),
                          attempt + 1);
            delay(20);
            continue;
        }
        incrementStat(&BaseEspnowStats::gnssCommandSent);
        Serial.printf("[BASE][GNSS_CMD] Sent action=%s txn=%lu rover=%s "
                      "ecef_m=(%.4f,%.4f,%.4f) attempt=%u\n",
                      gnssCommandName(command.commandId),
                      static_cast<unsigned long>(command.transactionId),
                      macToString(command.roverMac).c_str(),
                      static_cast<double>(command.ecefXScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(command.ecefYScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(command.ecefZScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      attempt + 1);
        resultReceived =
            xSemaphoreTake(gnssCommandResultSemaphore,
                           pdMS_TO_TICKS(ESPNOW_GNSS_COMMAND_RESULT_TIMEOUT_MS)) == pdTRUE;
    }

    portENTER_CRITICAL(&gnssCommandMux);
    waitingForGnssCommandResult = false;
    portEXIT_CRITICAL(&gnssCommandMux);
    if (!resultReceived) {
        incrementStat(&BaseEspnowStats::gnssCommandTimeouts);
        if (command.commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF) {
            switchToLocalFallback("temp_base_command_timeout");
        }
        queueLocalGnssCommandTimeout(command);
        Serial.printf("[BASE][GNSS_CMD][ERROR] Result timeout txn=%lu rover=%s\n",
                      static_cast<unsigned long>(command.transactionId),
                      macToString(command.roverMac).c_str());
    }
}
}

bool setupEspNowBase()
{
    if (ESPNOW_ENCRYPTION_ENABLED && !espnowSecurityKeysAreConfigured()) {
        Serial.println("[BASE][ESP-NOW][ERROR] Encryption enabled but PMK/LMK are not provisioned");
        return false;
    }

    sendCallbackSemaphore = xSemaphoreCreateBinary();
    frameAckSemaphore = xSemaphoreCreateBinary();
    espnowSendMutex = xSemaphoreCreateMutex();
    gnssCommandResultSemaphore = xSemaphoreCreateBinary();
    gnssCommandQueue = xQueueCreate(ESPNOW_GNSS_COMMAND_QUEUE_LENGTH,
                                    sizeof(QueuedGnssCommand));
    gnssCommandResultQueue = xQueueCreate(ESPNOW_GNSS_COMMAND_RESULT_QUEUE_LENGTH,
                                          sizeof(BaseGnssCommandResultEvent));
    tempRtcmRxQueue = xQueueCreate(TEMP_RTCM_RX_QUEUE_LENGTH,
                                   sizeof(TempRtcmRxPacket));
    tempRtcmFrameQueue = xQueueCreate(TEMP_RTCM_FRAME_QUEUE_LENGTH,
                                      sizeof(BaseTempRtcmFrame));
    if (sendCallbackSemaphore == nullptr || frameAckSemaphore == nullptr ||
        espnowSendMutex == nullptr || gnssCommandResultSemaphore == nullptr ||
        gnssCommandQueue == nullptr || gnssCommandResultQueue == nullptr ||
        tempRtcmRxQueue == nullptr || tempRtcmFrameQueue == nullptr) {
        Serial.println("[BASE][ESP-NOW][ERROR] Failed to create semaphores");
        return false;
    }

    setupPairingButton();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    delay(50);
    if (!configureMaxTxPower()) {
        return false;
    }

    esp_wifi_set_ps(WIFI_PS_NONE);

    uint8_t protocol = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N;
#ifdef WIFI_PROTOCOL_LR
    if (ESPNOW_USE_LR_250KBPS) {
        protocol |= WIFI_PROTOCOL_LR;
    }
#endif
    esp_wifi_set_protocol(WIFI_IF_STA, protocol);

    esp_err_t result = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (result != ESP_OK) {
        Serial.printf("[BASE][ESP-NOW][ERROR] esp_wifi_set_channel failed: %d\n", result);
        return false;
    }

    result = esp_now_init();
    if (result != ESP_OK) {
        Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_init failed: %d\n", result);
        return false;
    }

    result = esp_now_register_send_cb(onDataSent);
    if (result != ESP_OK) {
        Serial.printf("[BASE][ESP-NOW][ERROR] register send callback failed: %d\n", result);
        return false;
    }
    result = esp_now_register_recv_cb(onDataReceived);
    if (result != ESP_OK) {
        Serial.printf("[BASE][ESP-NOW][ERROR] register receive callback failed: %d\n", result);
        return false;
    }

    if (ESPNOW_ENCRYPTION_ENABLED) {
        result = esp_now_set_pmk(ESPNOW_PMK);
        if (result != ESP_OK) {
            Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_set_pmk failed: %d\n", result);
            return false;
        }
    }

    if (ESPNOW_PAIRING_ENABLED) {
        addEspNowPeer(BROADCAST_MAC, false);
    }
    loadStoredRovers();
    updatePeerStats();
    const BaseEspnowStats loadedPeerStats = getBaseEspnowStats();
    if (loadedPeerStats.activeRoverCount == 0) {
        if (ESPNOW_PAIRING_ENABLED) {
            Serial.println("[BASE][PAIR][WARN] No Rover MAC in NVS; hold pairing button to pair");
        } else {
            Serial.println("[BASE][ESP-NOW][ERROR] No Rover MAC in NVS and pairing disabled");
            return false;
        }
    }

#if defined(WIFI_PHY_RATE_LORA_250K)
    if (ESPNOW_USE_LR_250KBPS) {
        result = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K);
        if (result != ESP_OK) {
            Serial.printf("[BASE][ESP-NOW][WARN] espnow LR rate config failed: %d\n", result);
        }
    }
#endif

    portENTER_CRITICAL(&sourceMux);
    streamId = newStreamId();
    frameSequence = 0;
    rtcmSource.state = BaseRtcmSourceState::LocalActive;
    rtcmSource.epoch = 1;
    portEXIT_CRITICAL(&sourceMux);

    if (xTaskCreatePinnedToCore(tempRtcmReceiveTask,
                                "Temp RTCM RX",
                                TEMP_RTCM_RX_TASK_STACK_BYTES,
                                nullptr,
                                4,
                                nullptr,
                                1) != pdPASS) {
        Serial.println("[BASE][ESP-NOW][ERROR] Failed to create Temp RTCM RX task");
        return false;
    }

    Serial.println("[WIFI] STA radio ready for ESP-NOW; Internet transport starts separately");
    Serial.print("[WIFI] Local STA MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("[WIFI] ESP-NOW fixed channel: %u\n", ESPNOW_WIFI_CHANNEL);
    updatePeerStats();
    const BaseEspnowStats setupStats = getBaseEspnowStats();
    Serial.printf("[BASE][ESP-NOW] Ready, channel=%u, LR=%s, security=%s, streamId=%u, rover_count=%lu\n",
                  ESPNOW_WIFI_CHANNEL,
                  ESPNOW_USE_LR_250KBPS ? "250 Kbps" : "off",
                  ESPNOW_ENCRYPTION_ENABLED ? "CCMP" : "plaintext",
                  streamId,
                  static_cast<unsigned long>(setupStats.activeRoverCount));

    return true;
}

void baseEspNowLoop()
{
    bool shouldBeginReset = false;
    portENTER_CRITICAL(&sourceMux);
    shouldBeginReset = resetCohortRequested;
    resetCohortRequested = false;
    portEXIT_CRITICAL(&sourceMux);
    if (shouldBeginReset) {
        beginResetCohort();
    }

    const uint32_t sourceNow = millis();
    progressResetCohort(sourceNow);
    BaseRtcmSourceSnapshot sourceSnapshot{};
    uint32_t preparingStartedAt = 0;
    portENTER_CRITICAL(&sourceMux);
    sourceSnapshot = rtcmSource;
    preparingStartedAt = tempPreparingStartedAtMs;
    portEXIT_CRITICAL(&sourceMux);
    if (sourceSnapshot.state == BaseRtcmSourceState::TempActive &&
        sourceSnapshot.lastTempFrameAtMs != 0 &&
        sourceNow - sourceSnapshot.lastTempFrameAtMs > TEMP_RTCM_SOURCE_TIMEOUT_MS) {
        switchToLocalFallback("temp_rtcm_timeout");
    } else if (sourceSnapshot.state == BaseRtcmSourceState::TempPreparing &&
               preparingStartedAt != 0 &&
               sourceNow - preparingStartedAt >
                   TEMP_RTCM_PREPARING_TIMEOUT_MS) {
        switchToLocalFallback("temp_not_ready");
    }

    if constexpr (ESPNOW_PAIRING_ENABLED) {
        static uint32_t pressedSinceMs = 0;
        static bool pairingStartHandled = false;
        const bool rawLevel = digitalRead(PAIRING_BUTTON_PIN) == HIGH;
        const bool pressed = PAIRING_BUTTON_ACTIVE_LOW ? !rawLevel : rawLevel;
        const uint32_t now = millis();

        if (pressed) {
            if (pressedSinceMs == 0) {
                pressedSinceMs = now;
            } else if (!pairingStartHandled &&
                       now - pressedSinceMs >= PAIRING_BUTTON_HOLD_MS) {
                startPairingMode();
                pairingStartHandled = true;
            }
        } else {
            pressedSinceMs = 0;
            pairingStartHandled = false;
        }

        bool active = false;
        uint32_t endsAt = 0;
        portENTER_CRITICAL(&pairingMux);
        active = pairingActive;
        endsAt = pairingEndsAtMs;
        portEXIT_CRITICAL(&pairingMux);
        if (active && static_cast<int32_t>(now - endsAt) >= 0) {
            stopPairingMode("timeout");
        }

        sendPairDiscoveryIfDue();
        processPendingPairResponse();
    }
    processNextGnssCommand();
}

bool baseEspNowSendRtcmFrame(const uint8_t* frame, size_t length)
{
    if (frame == nullptr || length < 6 || length > RTCM_ESPNOW_MAX_FRAME_LENGTH) {
        incrementStat(&BaseEspnowStats::framesDropped);
        return false;
    }

    const uint8_t fragmentCount = static_cast<uint8_t>(
        (length + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) / RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD);
    if (fragmentCount == 0 || fragmentCount > RTCM_ESPNOW_MAX_FRAGMENT_COUNT) {
        incrementStat(&BaseEspnowStats::framesDropped);
        return false;
    }

    RoverPeer peers[ESPNOW_MAX_PAIRED_ROVERS] = {};
    const size_t peerCount = copyRoverPeers(peers, ESPNOW_MAX_PAIRED_ROVERS);
    if (peerCount == 0) {
        incrementStat(&BaseEspnowStats::framesDropped);
        Serial.println("[BASE][ESP-NOW][WARN] Drop RTCM frame: no paired Rover peer");
        return false;
    }

    uint16_t currentStreamId = 0;
    uint32_t currentSequence = 0;
    portENTER_CRITICAL(&sourceMux);
    currentStreamId = streamId;
    currentSequence = frameSequence++;
    portEXIT_CRITICAL(&sourceMux);
    const uint32_t sendStartedAt = millis();
    bool allPeersAcked = true;
    size_t attemptedPeers = 0;
    const size_t startIndex = nextPeerStartIndex % peerCount;
    nextPeerStartIndex = (startIndex + 1) % peerCount;
    const BaseRtcmSourceSnapshot sourceSnapshot =
        getBaseRtcmSourceSnapshot();
    const bool hasSelectedTemp =
        sourceSnapshot.state == BaseRtcmSourceState::TempPreparing ||
        sourceSnapshot.state == BaseRtcmSourceState::TempResetting ||
        sourceSnapshot.state == BaseRtcmSourceState::TempActive;

    for (size_t offset = 0; offset < peerCount; ++offset) {
        const size_t index = (startIndex + offset) % peerCount;
        if (hasSelectedTemp &&
            macEquals(peers[index].mac, sourceSnapshot.tempBaseMac)) {
            continue;
        }
        if (!peers[index].rtcmEnabled) {
            continue;
        }
        if (peerCooldownActive(peers[index], millis())) {
            incrementStat(&BaseEspnowStats::peerCooldownSkips);
            continue;
        }
        ++attemptedPeers;
        const bool delivered = sendFrameToPeer(peers[index].mac,
                                               frame,
                                               length,
                                               currentStreamId,
                                               currentSequence);
        recordPeerDeliveryResult(peers[index].mac,
                                 delivered,
                                 currentStreamId,
                                 currentSequence);
        if (!delivered) {
            allPeersAcked = false;
            Serial.printf("[BASE][ESP-NOW][ERROR] Drop seq=%lu for peer=%s without application ACK\n",
                          static_cast<unsigned long>(currentSequence),
                          macToString(peers[index].mac).c_str());
        }
    }

    waitingForFrameAck = false;
    if (attemptedPeers == 0) {
        incrementStat(&BaseEspnowStats::framesDropped);
        const uint32_t now = millis();
        if (lastNoRtcmPeerWarningAtMs == 0 ||
            now - lastNoRtcmPeerWarningAtMs >= HEALTH_INTERVAL) {
            lastNoRtcmPeerWarningAtMs = now;
            Serial.println("[BASE][ESP-NOW][WARN] Drop RTCM frame: all RTCM peers disabled or in cooldown");
        }
        return false;
    }
    recordFrameSendDuration(millis() - sendStartedAt);

    if (allPeersAcked) {
        incrementStat(&BaseEspnowStats::framesSent);
        return true;
    }

    incrementStat(&BaseEspnowStats::framesDropped);
    return false;
}

BaseEspnowStats getBaseEspnowStats()
{
    updatePeerStats();
    portENTER_CRITICAL(&statsMux);
    const BaseEspnowStats copy = stats;
    portEXIT_CRITICAL(&statsMux);
    return copy;
}

uint16_t getBaseEspNowStreamId()
{
    portENTER_CRITICAL(&sourceMux);
    const uint16_t copy = streamId;
    portEXIT_CRITICAL(&sourceMux);
    return copy;
}

bool baseEspNowShouldForwardLocalRtcm()
{
    portENTER_CRITICAL(&sourceMux);
    const bool forward =
        rtcmSource.state != BaseRtcmSourceState::TempResetting &&
        rtcmSource.state != BaseRtcmSourceState::TempActive;
    portEXIT_CRITICAL(&sourceMux);
    return forward;
}

bool baseEspNowPopTempRtcmFrame(BaseTempRtcmFrame& frame, TickType_t waitTicks)
{
    return tempRtcmFrameQueue != nullptr &&
           xQueueReceive(tempRtcmFrameQueue, &frame, waitTicks) == pdTRUE;
}

void baseEspNowCompleteTempRtcmForward(const BaseTempRtcmFrame& frame,
                                       bool downstreamDelivered)
{
    bool shouldAck = false;
    portENTER_CRITICAL(&tempAckMux);
    if (deferredTempAck.pending &&
        macEquals(deferredTempAck.sourceMac, frame.sourceMac) &&
        deferredTempAck.streamId == frame.upstreamStreamId &&
        deferredTempAck.sequence == frame.upstreamSequence) {
        deferredTempAck.pending = false;
        deferredTempAck.completed = true;
        shouldAck = true;
    }
    portEXIT_CRITICAL(&tempAckMux);
    if (!shouldAck) {
        return;
    }

    incrementStat(&BaseEspnowStats::tempForwardCompleted);
    if (!downstreamDelivered) {
        incrementStat(&BaseEspnowStats::tempForwardFailed);
    }
    sendTempRtcmAck(frame.sourceMac,
                    frame.upstreamStreamId,
                    frame.upstreamSequence);
}

BaseRtcmSourceSnapshot getBaseRtcmSourceSnapshot()
{
    portENTER_CRITICAL(&sourceMux);
    const BaseRtcmSourceSnapshot copy = rtcmSource;
    portEXIT_CRITICAL(&sourceMux);
    return copy;
}

bool baseEspNowGnssCommandPending()
{
    return waitingForGnssCommandResult ||
           (gnssCommandQueue != nullptr &&
            uxQueueMessagesWaiting(gnssCommandQueue) != 0);
}

const char* baseRtcmSourceStateToString(BaseRtcmSourceState state)
{
    switch (state) {
    case BaseRtcmSourceState::LocalActive: return "LOCAL_ACTIVE";
    case BaseRtcmSourceState::TempPreparing: return "TEMP_PREPARING";
    case BaseRtcmSourceState::TempResetting: return "TEMP_RESETTING";
    case BaseRtcmSourceState::TempActive: return "TEMP_ACTIVE";
    case BaseRtcmSourceState::LocalFallback: return "LOCAL_FALLBACK";
    default: return "UNKNOWN";
    }
}

size_t baseEspNowCopyLatestRoverEcef(BaseRoverEcefStatus* destination,
                                     size_t capacity)
{
    if (destination == nullptr || capacity == 0) {
        return 0;
    }
    const size_t count = ESPNOW_MAX_ECEF_SOURCES < capacity
                             ? ESPNOW_MAX_ECEF_SOURCES
                             : capacity;
    portENTER_CRITICAL(&ecefMux);
    for (size_t index = 0; index < count; ++index) {
        destination[index] = latestRoverEcef[index];
    }
    portEXIT_CRITICAL(&ecefMux);
    return count;
}

size_t baseEspNowCopyOnlineRovers(
    BaseOnlineRoverStatus* destination,
    size_t capacity,
    uint32_t now,
    uint32_t onlineWindowMs)
{
    if (destination == nullptr || capacity == 0 || onlineWindowMs == 0) {
        return 0;
    }

    RoverPeer peers[ESPNOW_MAX_PAIRED_ROVERS] = {};
    const size_t peerCount = copyRoverPeers(peers, ESPNOW_MAX_PAIRED_ROVERS);
    BaseRoverEcefStatus ecefSnapshots[ESPNOW_MAX_ECEF_SOURCES] = {};
    const size_t ecefCount = baseEspNowCopyLatestRoverEcef(
        ecefSnapshots, ESPNOW_MAX_ECEF_SOURCES);

    size_t copied = 0;
    for (size_t peerIndex = 0;
         peerIndex < peerCount && copied < capacity;
         ++peerIndex) {
        const RoverPeer& peer = peers[peerIndex];
        if (!peer.stored || !peer.rtcmEnabled || peer.lastRtcmAckAtMs == 0 ||
            now - peer.lastRtcmAckAtMs > onlineWindowMs) {
            continue;
        }

        BaseOnlineRoverStatus& output = destination[copied++];
        output = {};
        std::memcpy(output.mac, peer.mac, sizeof(output.mac));
        output.lastAckStreamId = peer.lastAckStreamId;
        output.lastAckFrameSequence = peer.lastAckFrameSequence;
        output.lastRtcmAckAtMs = peer.lastRtcmAckAtMs;

        // A relayed status belongs to a child Rover and must not be used as
        // the telemetry of the direct ESP-NOW peer queried here.
        for (size_t ecefIndex = 0; ecefIndex < ecefCount; ++ecefIndex) {
            const BaseRoverEcefStatus& ecef = ecefSnapshots[ecefIndex];
            if (!ecef.valid || ecef.viaRelay || !macEquals(ecef.mac, peer.mac)) {
                continue;
            }
            output.correctionStreamId = ecef.correctionStreamId;
            output.fixQuality = ecef.fixQuality;
            output.fixQualityReceivedAtMs = ecef.receivedAtMs;
            output.hasFixQuality = true;
            break;
        }
    }

    RelayedRoverAckState relayedSnapshots[ESPNOW_MAX_ECEF_SOURCES] = {};
    portENTER_CRITICAL(&relayedAckMux);
    for (size_t index = 0; index < ESPNOW_MAX_ECEF_SOURCES; ++index) {
        relayedSnapshots[index] = relayedRoverAcks[index];
    }
    portEXIT_CRITICAL(&relayedAckMux);

    for (size_t ackIndex = 0;
         ackIndex < ESPNOW_MAX_ECEF_SOURCES && copied < capacity;
         ++ackIndex) {
        const RelayedRoverAckState& ack = relayedSnapshots[ackIndex];
        if (!ack.valid || ack.lastRtcmAckAtMs == 0 ||
            now - ack.lastRtcmAckAtMs > onlineWindowMs) {
            continue;
        }
        bool alreadyDirect = false;
        for (size_t outputIndex = 0; outputIndex < copied; ++outputIndex) {
            if (macEquals(destination[outputIndex].mac, ack.roverMac)) {
                alreadyDirect = true;
                break;
            }
        }
        if (alreadyDirect) {
            continue;
        }

        BaseOnlineRoverStatus& output = destination[copied++];
        output = {};
        std::memcpy(output.mac, ack.roverMac, sizeof(output.mac));
        std::memcpy(output.relayMac, ack.relayMac, sizeof(output.relayMac));
        output.viaRelay = true;
        output.lastAckStreamId = ack.streamId;
        output.lastAckFrameSequence = ack.frameSequence;
        output.lastRtcmAckAtMs = ack.lastRtcmAckAtMs;

        for (size_t ecefIndex = 0; ecefIndex < ecefCount; ++ecefIndex) {
            const BaseRoverEcefStatus& ecef = ecefSnapshots[ecefIndex];
            if (!ecef.valid || !ecef.viaRelay ||
                !macEquals(ecef.mac, ack.roverMac) ||
                !macEquals(ecef.relayMac, ack.relayMac)) {
                continue;
            }
            output.correctionStreamId = ecef.correctionStreamId;
            output.fixQuality = ecef.fixQuality;
            output.fixQualityReceivedAtMs = ecef.receivedAtMs;
            output.hasFixQuality = true;
            break;
        }
    }
    return copied;
}

static BaseGnssCommandQueueResult queueRoverGnssCommand(
    uint8_t commandId,
    const uint8_t* requestedTargetMac,
    uint32_t transactionId,
    int64_t ecefXScaled,
    int64_t ecefYScaled,
    int64_t ecefZScaled,
    uint8_t selectedTargetMac[6])
{
    const bool validEcef =
        rtcmEspNowValidEcef(ecefXScaled, ecefYScaled, ecefZScaled);
    const bool validArguments =
        transactionId != 0 &&
        ((commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF &&
         validEcef) ||
         ((commandId == RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER ||
           commandId == RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK ||
           commandId == RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK) &&
          ecefXScaled == 0 &&
          ecefYScaled == 0 &&
          ecefZScaled == 0));
    if (!validArguments) {
        return BaseGnssCommandQueueResult::InvalidArgument;
    }
    if (gnssCommandQueue == nullptr) {
        return BaseGnssCommandQueueResult::NotReady;
    }

    QueuedGnssCommand command{};
    bool targetFound = false;
    portENTER_CRITICAL(&peerMux);
    if (requestedTargetMac == nullptr && roverPeerCount > 0) {
        std::memcpy(command.roverMac, roverPeers[0].mac, sizeof(command.roverMac));
        targetFound = true;
    } else if (requestedTargetMac != nullptr) {
        for (size_t index = 0; index < roverPeerCount; ++index) {
            if (macEquals(roverPeers[index].mac, requestedTargetMac)) {
                std::memcpy(command.roverMac,
                            roverPeers[index].mac,
                            sizeof(command.roverMac));
                targetFound = true;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&peerMux);
    if (!targetFound || !macIsConfigured(command.roverMac)) {
        if (requestedTargetMac != nullptr) {
            return BaseGnssCommandQueueResult::TargetNotPaired;
        }
        return BaseGnssCommandQueueResult::NoPairedRover;
    }
    command.transactionId = transactionId;
    command.ecefXScaled = ecefXScaled;
    command.ecefYScaled = ecefYScaled;
    command.ecefZScaled = ecefZScaled;
    command.commandId = commandId;
    if (xQueueSend(gnssCommandQueue, &command, 0) != pdTRUE) {
        return BaseGnssCommandQueueResult::QueueFull;
    }
    if (selectedTargetMac != nullptr) {
        std::memcpy(selectedTargetMac, command.roverMac, 6);
    }
    incrementStat(&BaseEspnowStats::gnssCommandQueued);
    return BaseGnssCommandQueueResult::Queued;
}

void beginResetCohort()
{
    const uint32_t now = millis();
    BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
    if (source.state != BaseRtcmSourceState::TempPreparing) {
        return;
    }

    const size_t snapshotCount =
        baseEspNowCopyLatestRoverEcef(resetCohortSnapshots,
                                     ESPNOW_MAX_ECEF_SOURCES);
    RoverPeer peers[ESPNOW_MAX_PAIRED_ROVERS] = {};
    const size_t peerCount =
        copyRoverPeers(peers, ESPNOW_MAX_PAIRED_ROVERS);

    portENTER_CRITICAL(&resetMux);
    std::memset(resetCohort, 0, sizeof(resetCohort));
    std::memset(resetBranches, 0, sizeof(resetBranches));
    resetCohortCount = 0;
    resetBranchCount = 0;
    resetGateStartedAtMs = now;
    resetWaitingForFreshTempLogged = false;
    for (size_t index = 0; index < snapshotCount; ++index) {
        const BaseRoverEcefStatus& status = resetCohortSnapshots[index];
        if (!status.valid ||
            macEquals(status.mac, source.tempBaseMac) ||
            resetCohortCount >= ESPNOW_MAX_ECEF_SOURCES) {
            continue;
        }

        const uint8_t* branchMac = status.viaRelay
                                       ? status.relayMac
                                       : status.mac;
        if (!macIsConfigured(branchMac)) {
            continue;
        }
        ResetCohortEntry& entry = resetCohort[resetCohortCount++];
        std::memcpy(entry.roverMac, status.mac, sizeof(entry.roverMac));
        std::memcpy(entry.branchMac, branchMac, sizeof(entry.branchMac));
        entry.baselineReceivedAtMs = status.receivedAtMs;

        bool branchExists = false;
        for (size_t branchIndex = 0;
             branchIndex < resetBranchCount;
             ++branchIndex) {
            branchExists =
                macEquals(resetBranches[branchIndex].mac, branchMac);
            if (branchExists) {
                break;
            }
        }
        if (!branchExists && resetBranchCount < ESPNOW_MAX_PAIRED_ROVERS) {
            ResetCohortBranch& branch = resetBranches[resetBranchCount++];
            std::memcpy(branch.mac, branchMac, sizeof(branch.mac));
            do {
                branch.transactionId = esp_random();
            } while (branch.transactionId == 0);
        }
    }

    // A paired Rover remains part of the handover gate even if its last ECEF
    // status is stale or it rebooted immediately before this transition.
    for (size_t peerIndex = 0; peerIndex < peerCount; ++peerIndex) {
        const RoverPeer& peer = peers[peerIndex];
        if (!macIsConfigured(peer.mac) ||
            macEquals(peer.mac, source.tempBaseMac)) {
            continue;
        }

        bool roverExists = false;
        for (size_t cohortIndex = 0;
             cohortIndex < resetCohortCount;
             ++cohortIndex) {
            if (macEquals(resetCohort[cohortIndex].roverMac, peer.mac)) {
                roverExists = true;
                break;
            }
        }
        if (!roverExists &&
            resetCohortCount < ESPNOW_MAX_ECEF_SOURCES) {
            ResetCohortEntry& entry =
                resetCohort[resetCohortCount++];
            std::memcpy(entry.roverMac, peer.mac,
                        sizeof(entry.roverMac));
            std::memcpy(entry.branchMac, peer.mac,
                        sizeof(entry.branchMac));
        }

        bool branchExists = false;
        for (size_t branchIndex = 0;
             branchIndex < resetBranchCount;
             ++branchIndex) {
            if (macEquals(resetBranches[branchIndex].mac, peer.mac)) {
                branchExists = true;
                break;
            }
        }
        if (!branchExists &&
            resetBranchCount < ESPNOW_MAX_PAIRED_ROVERS) {
            ResetCohortBranch& branch =
                resetBranches[resetBranchCount++];
            std::memcpy(branch.mac, peer.mac, sizeof(branch.mac));
            do {
                branch.transactionId = esp_random();
            } while (branch.transactionId == 0);
        }
    }
    portEXIT_CRITICAL(&resetMux);

    portENTER_CRITICAL(&sourceMux);
    if (rtcmSource.state == BaseRtcmSourceState::TempPreparing &&
        macEquals(rtcmSource.tempBaseMac, source.tempBaseMac)) {
        rtcmSource.state = BaseRtcmSourceState::TempResetting;
    }
    const bool resetting =
        rtcmSource.state == BaseRtcmSourceState::TempResetting;
    portEXIT_CRITICAL(&sourceMux);
    if (!resetting) {
        return;
    }

    ResetCohortBranch branches[ESPNOW_MAX_PAIRED_ROVERS] = {};
    size_t branchCount = 0;
    size_t cohortCount = 0;
    portENTER_CRITICAL(&resetMux);
    branchCount = resetBranchCount;
    cohortCount = resetCohortCount;
    std::memcpy(branches, resetBranches,
                branchCount * sizeof(ResetCohortBranch));
    portEXIT_CRITICAL(&resetMux);

    Serial.printf("[BASE][HANDOVER_RESET] state=START rovers=%u branches=%u "
                  "timeout_ms=%lu\n",
                  static_cast<unsigned>(cohortCount),
                  static_cast<unsigned>(branchCount),
                  static_cast<unsigned long>(TEMP_RESET_GATE_TIMEOUT_MS));
    for (size_t index = 0; index < branchCount; ++index) {
        setRoverRtcmEnabled(branches[index].mac, false);
        uint8_t selectedMac[6] = {};
        const BaseGnssCommandQueueResult result = queueRoverGnssCommand(
            RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK,
            branches[index].mac,
            branches[index].transactionId,
            0,
            0,
            0,
            selectedMac);
        portENTER_CRITICAL(&resetMux);
        for (size_t branchIndex = 0;
             branchIndex < resetBranchCount;
             ++branchIndex) {
            if (macEquals(resetBranches[branchIndex].mac,
                          branches[index].mac)) {
                resetBranches[branchIndex].commandQueued =
                    result == BaseGnssCommandQueueResult::Queued;
                break;
            }
        }
        portEXIT_CRITICAL(&resetMux);
        Serial.printf("[BASE][HANDOVER_RESET] branch=%s txn=%lu queue=%s\n",
                      macToString(branches[index].mac).c_str(),
                      static_cast<unsigned long>(
                          branches[index].transactionId),
                      baseGnssCommandQueueResultToString(result));
    }
}

void observeResetCohortStatus(const uint8_t* roverMac,
                              uint8_t fixQuality,
                              uint32_t receivedAtMs)
{
    if (roverMac == nullptr || fixQuality == 4 || fixQuality == 5) {
        return;
    }
    bool observed = false;
    portENTER_CRITICAL(&resetMux);
    for (size_t index = 0; index < resetCohortCount; ++index) {
        ResetCohortEntry& entry = resetCohort[index];
        if (macEquals(entry.roverMac, roverMac) &&
            receivedAtMs > entry.baselineReceivedAtMs) {
            entry.nonRtkObservedAtMs = receivedAtMs;
            observed = true;
            break;
        }
    }
    portEXIT_CRITICAL(&resetMux);
    if (observed) {
        Serial.printf("[BASE][HANDOVER_RESET] rover=%s fresh_fix=%u\n",
                      macToString(roverMac).c_str(),
                      static_cast<unsigned>(fixQuality));
    }
}

void markResetBranchAcked(const uint8_t* branchMac)
{
    if (branchMac == nullptr) {
        return;
    }
    bool marked = false;
    const uint32_t now = millis();
    portENTER_CRITICAL(&resetMux);
    for (size_t index = 0; index < resetBranchCount; ++index) {
        if (macEquals(resetBranches[index].mac, branchMac)) {
            resetBranches[index].commandAcked = true;
            resetBranches[index].ackedAtMs = now;
            marked = true;
            break;
        }
    }
    portEXIT_CRITICAL(&resetMux);
    if (marked) {
        Serial.printf("[BASE][HANDOVER_RESET] branch=%s command_ack=1\n",
                      macToString(branchMac).c_str());
    }
}

void queueRtkResumeForBranches(const ResetCohortBranch* branches,
                               size_t branchCount,
                               const char* reason)
{
    for (size_t index = 0; index < branchCount; ++index) {
        setRoverRtcmEnabled(branches[index].mac, false);
        uint32_t transactionId = 0;
        do {
            transactionId = esp_random();
        } while (transactionId == 0);
        uint8_t selectedMac[6] = {};
        const BaseGnssCommandQueueResult result = queueRoverGnssCommand(
            RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK,
            branches[index].mac,
            transactionId,
            0,
            0,
            0,
            selectedMac);
        Serial.printf("[BASE][HANDOVER_RESUME] branch=%s txn=%lu queue=%s reason=%s\n",
                      macToString(branches[index].mac).c_str(),
                      static_cast<unsigned long>(transactionId),
                      baseGnssCommandQueueResultToString(result),
                      reason == nullptr ? "unknown" : reason);
    }
}

void progressResetCohort(uint32_t now)
{
    BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
    if (source.state != BaseRtcmSourceState::TempResetting) {
        return;
    }

    bool ready = true;
    bool gateTimedOut = false;
    ResetCohortBranch branches[ESPNOW_MAX_PAIRED_ROVERS] = {};
    size_t branchCount = 0;
    portENTER_CRITICAL(&resetMux);
    branchCount = resetBranchCount;
    std::memcpy(branches, resetBranches,
                branchCount * sizeof(ResetCohortBranch));
    for (size_t index = 0; index < resetBranchCount; ++index) {
        ready = ready && resetBranches[index].commandQueued &&
                resetBranches[index].commandAcked;
    }
    for (size_t index = 0; index < resetCohortCount; ++index) {
        uint32_t branchAckedAtMs = 0;
        for (size_t branchIndex = 0;
             branchIndex < resetBranchCount;
             ++branchIndex) {
            if (macEquals(resetCohort[index].branchMac,
                          resetBranches[branchIndex].mac)) {
                branchAckedAtMs = resetBranches[branchIndex].ackedAtMs;
                break;
            }
        }
        ready = ready &&
                resetCohort[index].nonRtkObservedAtMs >= branchAckedAtMs &&
                branchAckedAtMs != 0;
    }
    gateTimedOut =
        now - resetGateStartedAtMs > TEMP_RESET_GATE_TIMEOUT_MS;
    portEXIT_CRITICAL(&resetMux);

    const bool tempFrameFresh =
        source.lastTempFrameAtMs != 0 &&
        now - source.lastTempFrameAtMs <= TEMP_RTCM_SOURCE_TIMEOUT_MS;

    // The reset ACK/fix gate and temp RTCM freshness can become ready on
    // different loop iterations. Do not activate a source whose last frame
    // is already stale: that would make the next baseEspNowLoop() iteration
    // immediately fall back to the local Base. Keep the cohort reset and wait
    // for a fresh temp frame, bounded by the overall reset-gate timeout.
    if (!gateTimedOut && (!ready || !tempFrameFresh)) {
        if (ready && !tempFrameFresh && !resetWaitingForFreshTempLogged) {
            resetWaitingForFreshTempLogged = true;
            const uint32_t tempAgeMs =
                source.lastTempFrameAtMs == 0
                    ? UINT32_MAX
                    : now - source.lastTempFrameAtMs;
            Serial.printf("[BASE][HANDOVER_RESET] state=WAIT_TEMP_FRESH "
                          "temp_age_ms=%lu timeout_ms=%lu\n",
                          static_cast<unsigned long>(tempAgeMs),
                          static_cast<unsigned long>(
                              TEMP_RESET_GATE_TIMEOUT_MS -
                              (now - resetGateStartedAtMs)));
        }
        return;
    }
    if (!gateTimedOut && ready && tempFrameFresh) {
        if (tempRtcmFrameQueue != nullptr) {
            xQueueReset(tempRtcmFrameQueue);
        }
        Serial.println("[BASE][HANDOVER_RESET] state=COMPLETE");
        queueRtkResumeForBranches(branches, branchCount, "temp_active");
        activateTempSource();
    } else {
        Serial.println("[BASE][HANDOVER_RESET][ERROR] state=TIMEOUT; keep local RTCM");
        queueRtkResumeForBranches(branches, branchCount, "local_fallback");
        switchToLocalFallback("reset_gate_timeout");
        uint8_t ignoredMac[6] = {};
        uint32_t transactionId = 0;
        do {
            transactionId = esp_random();
        } while (transactionId == 0);
        const BaseGnssCommandQueueResult result = queueRoverGnssCommand(
            RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER,
            source.tempBaseMac,
            transactionId,
            0,
            0,
            0,
            ignoredMac);
        Serial.printf("[BASE][HANDOVER_RESET] rollback_temp=%s queue=%s\n",
                      macToString(source.tempBaseMac).c_str(),
                      baseGnssCommandQueueResultToString(result));
    }

    portENTER_CRITICAL(&resetMux);
    std::memset(resetCohort, 0, sizeof(resetCohort));
    std::memset(resetBranches, 0, sizeof(resetBranches));
    resetCohortCount = 0;
    resetBranchCount = 0;
    resetGateStartedAtMs = 0;
    resetWaitingForFreshTempLogged = false;
    portEXIT_CRITICAL(&resetMux);
}

BaseGnssCommandQueueResult baseEspNowQueueFirstRoverMode(
    uint32_t transactionId,
    uint8_t targetMac[6])
{
    return queueRoverGnssCommand(
        RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER,
        nullptr,
        transactionId,
        0,
        0,
        0,
        targetMac);
}

void baseEspNowMarkLocalBaseReady()
{
    portENTER_CRITICAL(&sourceMux);
    if (rtcmSource.state == BaseRtcmSourceState::LocalFallback) {
        rtcmSource.state = BaseRtcmSourceState::LocalActive;
        std::memset(rtcmSource.tempBaseMac, 0,
                    sizeof(rtcmSource.tempBaseMac));
    }
    portEXIT_CRITICAL(&sourceMux);
}

BaseGnssCommandQueueResult baseEspNowQueueRoverMode(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    uint8_t selectedTargetMac[6])
{
    return queueRoverGnssCommand(
        RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER,
        requestedTargetMac,
        transactionId,
        0,
        0,
        0,
        selectedTargetMac);
}

BaseGnssCommandQueueResult baseEspNowQueueRoverBaseFixedEcef(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    int64_t ecefXScaled,
    int64_t ecefYScaled,
    int64_t ecefZScaled,
    uint8_t selectedTargetMac[6])
{
    return queueRoverGnssCommand(
        RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF,
        requestedTargetMac,
        transactionId,
        ecefXScaled,
        ecefYScaled,
        ecefZScaled,
        selectedTargetMac);
}

BaseGnssCommandQueueResult baseEspNowQueueRoverRtkReset(
    const uint8_t requestedTargetMac[6],
    uint32_t transactionId,
    uint8_t selectedTargetMac[6])
{
    return queueRoverGnssCommand(
        RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK,
        requestedTargetMac,
        transactionId,
        0,
        0,
        0,
        selectedTargetMac);
}

bool baseEspNowIsDirectRoverPaired(const uint8_t targetMac[6])
{
    size_t ignoredIndex = 0;
    return targetMac != nullptr && findRoverPeerIndex(targetMac, ignoredIndex);
}

bool baseEspNowPopGnssCommandResult(BaseGnssCommandResultEvent& result)
{
    return gnssCommandResultQueue != nullptr &&
           xQueueReceive(gnssCommandResultQueue, &result, 0) == pdTRUE;
}

const char* baseGnssCommandQueueResultToString(BaseGnssCommandQueueResult result)
{
    switch (result) {
    case BaseGnssCommandQueueResult::Queued: return "queued";
    case BaseGnssCommandQueueResult::InvalidArgument: return "invalid_argument";
    case BaseGnssCommandQueueResult::NotReady: return "not_ready";
    case BaseGnssCommandQueueResult::NoPairedRover: return "no_paired_rover";
    case BaseGnssCommandQueueResult::TargetNotPaired: return "target_not_paired";
    case BaseGnssCommandQueueResult::QueueFull: return "queue_full";
    default: return "unknown";
    }
}
