#include <Arduino.h>
#include <cstring>
#include <freertos/queue.h>

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "functions/NetworkMqttManager.h"
#include "functions/BaseGnssRoleController.h"
#include "functions/Rtcm_Frame_Reader.h"
#include "hardware/BaseEspnow_sender.h"

namespace {
struct RtcmFrameEnvelope {
    uint16_t length;
    uint16_t messageId;
    uint32_t receivedAtMs;
    uint8_t data[RTCM_ESPNOW_MAX_FRAME_LENGTH];
};

struct RtcmMessageStats {
    uint16_t messageId;
    uint32_t count;
    uint32_t lastReceivedAtMs;
};

struct BasePipelineStats {
    uint32_t rtcmValidFrames;
    uint32_t rtcmCrcErrors;
    uint32_t rtcmTooLarge;
    uint32_t rtcmSendOk;
    uint32_t rtcmSendFail;
    uint32_t queueDrops;
    uint32_t staleDrops;
    uint32_t queueHighWater;
    uint32_t lastQueueAgeMs;
    uint32_t maxQueueAgeMs;
    uint32_t localFramesSuppressed;
};

QueueHandle_t rtcmFrameQueue = nullptr;
BasePipelineStats pipelineStats{};
RtcmFrameEnvelope senderEnvelope{};
BaseTempRtcmFrame senderTempFrame{};
RtcmMessageStats messageStats[] = {
    {1005, 0, 0},
    {1006, 0, 0},
    {1074, 0, 0},
    {1084, 0, 0},
    {1094, 0, 0},
    {1124, 0, 0},
    {1230, 0, 0},
    {0, 0, 0}, // Other RTCM message IDs.
};
portMUX_TYPE pipelineStatsMux = portMUX_INITIALIZER_UNLOCKED;

uint16_t getRtcmMessageId(const uint8_t* frame, size_t frameLength)
{
    if (frame == nullptr || frameLength < 5) {
        return 0;
    }
    return static_cast<uint16_t>((static_cast<uint16_t>(frame[3]) << 4) |
                                 (static_cast<uint16_t>(frame[4]) >> 4));
}

void recordValidFrame(uint16_t messageId, uint32_t receivedAtMs)
{
    portENTER_CRITICAL(&pipelineStatsMux);
    ++pipelineStats.rtcmValidFrames;

    RtcmMessageStats* selected = &messageStats[sizeof(messageStats) / sizeof(messageStats[0]) - 1];
    for (RtcmMessageStats& entry : messageStats) {
        if (entry.messageId == messageId) {
            selected = &entry;
            break;
        }
    }
    ++selected->count;
    selected->lastReceivedAtMs = receivedAtMs;
    portEXIT_CRITICAL(&pipelineStatsMux);
}

void recordQueueDepth()
{
    const uint32_t depth = static_cast<uint32_t>(uxQueueMessagesWaiting(rtcmFrameQueue));
    portENTER_CRITICAL(&pipelineStatsMux);
    if (depth > pipelineStats.queueHighWater) {
        pipelineStats.queueHighWater = depth;
    }
    portEXIT_CRITICAL(&pipelineStatsMux);
}

void enqueueRtcmFrame(const uint8_t* frame, size_t frameLength)
{
    RtcmFrameEnvelope envelope{};
    envelope.length = static_cast<uint16_t>(frameLength);
    envelope.messageId = getRtcmMessageId(frame, frameLength);
    envelope.receivedAtMs = millis();
    std::memcpy(envelope.data, frame, frameLength);
    recordValidFrame(envelope.messageId, envelope.receivedAtMs);

    if (!baseEspNowShouldForwardLocalRtcm()) {
        portENTER_CRITICAL(&pipelineStatsMux);
        ++pipelineStats.localFramesSuppressed;
        portEXIT_CRITICAL(&pipelineStatsMux);
        return;
    }

    if (xQueueSend(rtcmFrameQueue, &envelope, 0) != pdTRUE) {
        RtcmFrameEnvelope discarded{};
        if (xQueueReceive(rtcmFrameQueue, &discarded, 0) == pdTRUE) {
            portENTER_CRITICAL(&pipelineStatsMux);
            ++pipelineStats.queueDrops;
            portEXIT_CRITICAL(&pipelineStatsMux);
        }
        if (xQueueSend(rtcmFrameQueue, &envelope, 0) != pdTRUE) {
            portENTER_CRITICAL(&pipelineStatsMux);
            ++pipelineStats.queueDrops;
            portEXIT_CRITICAL(&pipelineStatsMux);
            return;
        }
    }
    recordQueueDepth();
}

BasePipelineStats getPipelineStats()
{
    portENTER_CRITICAL(&pipelineStatsMux);
    const BasePipelineStats copy = pipelineStats;
    portEXIT_CRITICAL(&pipelineStatsMux);
    return copy;
}

void copyMessageStats(RtcmMessageStats* destination, size_t count)
{
    portENTER_CRITICAL(&pipelineStatsMux);
    std::memcpy(destination, messageStats, min(count, sizeof(messageStats)));
    portEXIT_CRITICAL(&pipelineStatsMux);
}

void dumpRtcmFrameHex(const char* label, const uint8_t* frame, size_t frameLength)
{
    if (!DEBUG_RTCM_HEX_DUMP || frame == nullptr || frameLength == 0) {
        return;
    }

    Serial.printf("[BASE][GNSS][RTCM_HEX] %s length=%u\n",
                  label,
                  static_cast<unsigned>(frameLength));

    for (size_t i = 0; i < frameLength; ++i) {
        if ((i % DEBUG_RTCM_HEX_BYTES_PER_LINE) == 0) {
            Serial.printf("[BASE][GNSS][RTCM_HEX] %04u: ", static_cast<unsigned>(i));
        }

        Serial.printf("%02X", frame[i]);
        if ((i % DEBUG_RTCM_HEX_BYTES_PER_LINE) == (DEBUG_RTCM_HEX_BYTES_PER_LINE - 1) ||
            i == (frameLength - 1)) {
            Serial.println();
        } else {
            Serial.print(' ');
        }
    }
}

[[noreturn]] void taskRtcmReader(void*)
{
    static uint8_t parserFrame[RTCM_ESPNOW_MAX_FRAME_LENGTH] = {};

    while (true) {
        if (baseGnssRoleConsumesNmea()) {
            while (Serial1.available()) {
                const int value = Serial1.read();
                if (value >= 0) {
                    baseGnssRoleConsumeByte(static_cast<uint8_t>(value));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        size_t frameLength = 0;
        const RtcmReadResult result =
            readRtcmFrame(Serial1, parserFrame, sizeof(parserFrame), frameLength);

        switch (result) {
        case RtcmReadResult::FrameValid:
            baseGnssRoleRecordLocalRtcm(parserFrame, frameLength);
            flushRtcmDebugLine();
            if (DEBUG_RTCM_FRAME_LOG) {
                Serial.printf("[BASE][GNSS] RTCM id=%u length=%u\n",
                              getRtcmMessageId(parserFrame, frameLength),
                              static_cast<unsigned>(frameLength));
            }
            dumpRtcmFrameHex("valid", parserFrame, frameLength);
            enqueueRtcmFrame(parserFrame, frameLength);
            break;

        case RtcmReadResult::CrcError:
            portENTER_CRITICAL(&pipelineStatsMux);
            ++pipelineStats.rtcmCrcErrors;
            portEXIT_CRITICAL(&pipelineStatsMux);
            flushRtcmDebugLine();
            Serial.printf("[BASE][GNSS][WARN] RTCM CRC error, length=%u\n",
                          static_cast<unsigned>(frameLength));
            dumpRtcmFrameHex("crc_error", parserFrame, frameLength);
            break;

        case RtcmReadResult::FrameTooLarge:
            portENTER_CRITICAL(&pipelineStatsMux);
            ++pipelineStats.rtcmTooLarge;
            portEXIT_CRITICAL(&pipelineStatsMux);
            flushRtcmDebugLine();
            Serial.println("[BASE][GNSS][WARN] RTCM frame too large");
            break;

        case RtcmReadResult::None:
            vTaskDelay(pdMS_TO_TICKS(1));
            break;
        }
    }
}

[[noreturn]] void taskRtcmSender(void*)
{
    uint32_t observedSourceEpoch = getBaseRtcmSourceSnapshot().epoch;
    while (true) {
        RtcmFrameEnvelope& envelope = senderEnvelope;
        BaseTempRtcmFrame& activeTempFrame = senderTempFrame;
        envelope = {};
        activeTempFrame = {};
        BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
        if (source.epoch != observedSourceEpoch) {
            observedSourceEpoch = source.epoch;
            while (xQueueReceive(rtcmFrameQueue, &envelope, 0) == pdTRUE) {
            }
        }
        if (source.state == BaseRtcmSourceState::TempResetting) {
            while (xQueueReceive(rtcmFrameQueue, &envelope, 0) == pdTRUE) {
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        bool haveFrame = false;
        bool hasDeferredTempAck = false;
        const bool fromTemp = source.state == BaseRtcmSourceState::TempActive;
        if (fromTemp) {
            if (baseEspNowPopTempRtcmFrame(activeTempFrame,
                                          pdMS_TO_TICKS(50))) {
                hasDeferredTempAck = true;
                envelope.length = activeTempFrame.length;
                envelope.messageId = activeTempFrame.messageId;
                envelope.receivedAtMs = activeTempFrame.receivedAtMs;
                std::memcpy(envelope.data, activeTempFrame.data,
                            activeTempFrame.length);
                baseGnssRoleInjectTempRtcm(activeTempFrame.data,
                                           activeTempFrame.length);
                haveFrame = true;
            }
        } else {
            haveFrame = xQueueReceive(rtcmFrameQueue,
                                      &envelope,
                                      pdMS_TO_TICKS(50)) == pdTRUE;
        }
        if (!haveFrame) {
            continue;
        }

        const BaseRtcmSourceSnapshot sourceBeforeSend = getBaseRtcmSourceSnapshot();
        if (sourceBeforeSend.epoch != observedSourceEpoch ||
            sourceBeforeSend.state == BaseRtcmSourceState::TempResetting ||
            (fromTemp && sourceBeforeSend.state != BaseRtcmSourceState::TempActive) ||
            (!fromTemp && sourceBeforeSend.state == BaseRtcmSourceState::TempActive)) {
            continue;
        }

        const uint32_t queueAgeMs = millis() - envelope.receivedAtMs;
        portENTER_CRITICAL(&pipelineStatsMux);
        pipelineStats.lastQueueAgeMs = queueAgeMs;
        if (queueAgeMs > pipelineStats.maxQueueAgeMs) {
            pipelineStats.maxQueueAgeMs = queueAgeMs;
        }
        portEXIT_CRITICAL(&pipelineStatsMux);

        if (queueAgeMs > RTCM_MAX_QUEUE_AGE_MS) {
            portENTER_CRITICAL(&pipelineStatsMux);
            ++pipelineStats.staleDrops;
            portEXIT_CRITICAL(&pipelineStatsMux);
            if (hasDeferredTempAck) {
                baseEspNowCompleteTempRtcmForward(activeTempFrame, false);
            }
            continue;
        }

        const bool sent = baseEspNowSendRtcmFrame(envelope.data, envelope.length);
        portENTER_CRITICAL(&pipelineStatsMux);
        if (sent) {
            ++pipelineStats.rtcmSendOk;
        } else {
            ++pipelineStats.rtcmSendFail;
        }
        portEXIT_CRITICAL(&pipelineStatsMux);
        if (hasDeferredTempAck) {
            baseEspNowCompleteTempRtcmForward(activeTempFrame, sent);
        }
    }
}

[[noreturn]] void healthLogTask(void*)
{
    uint32_t previousLogAt = millis();
    uint32_t previousRawBytes = getRtcmRawByteCount();
    uint32_t previousRtcmValid = 0;
    uint32_t previousFramesSent = 0;
    uint32_t previousFramesDropped = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));

        const uint32_t now = millis();
        const uint32_t periodMs = now - previousLogAt;
        const BasePipelineStats pipeline = getPipelineStats();
        const BaseEspnowStats espnow = getBaseEspnowStats();
        const uint32_t rawBytes = getRtcmRawByteCount();

        const uint32_t deltaRawBytes = rawBytes - previousRawBytes;
        const uint32_t deltaRtcmValid = pipeline.rtcmValidFrames - previousRtcmValid;
        const uint32_t deltaFramesSent = espnow.framesSent - previousFramesSent;
        const uint32_t deltaFramesDropped = espnow.framesDropped - previousFramesDropped;
        const uint32_t deltaDeliveryTotal = deltaFramesSent + deltaFramesDropped;
        const float seconds = periodMs > 0 ? static_cast<float>(periodMs) / 1000.0F : 1.0F;
        const float deliveryPercent = deltaDeliveryTotal > 0
                                          ? (100.0F * static_cast<float>(deltaFramesSent) /
                                             static_cast<float>(deltaDeliveryTotal))
                                          : 0.0F;

        flushRtcmDebugLine();
        Serial.printf(
            "[BASE][HEALTH] period_ms=%lu uart_Bps=%.1f rtcm_fps=%.2f acked_fps=%.2f "
            "delivery=%.1f%% uart_available=%d queue=%u queue_hwm=%lu queue_drop=%lu "
            "stale_drop=%lu local_suppressed=%lu queue_age_ms=%lu queue_age_max_ms=%lu uart_raw_bytes=%lu "
            "rtcm_valid=%lu crc_error=%lu too_large=%lu frames_acked=%lu frames_dropped=%lu "
            "fragments_sent=%lu send_fail=%lu send_timeout=%lu frame_retry=%lu ack_timeout=%lu "
            "ack_rx=%lu ack_invalid=%lu frame_deadline=%lu task_send_ok=%lu task_send_fail=%lu "
            "rovers=%lu stored_rovers=%lu rtcm_rovers=%lu pairing=%u pair_resp=%lu pair_confirm=%lu "
            "pair_auth_fail=%lu llh_rx=%lu llh_relayed=%lu llh_invalid=%lu "
            "llh_unknown=%lu llh_capacity_drop=%lu "
            "cmd_queued=%lu cmd_sent=%lu cmd_result=%lu cmd_timeout=%lu cmd_invalid=%lu "
            "temp_frag_rx=%lu temp_frame_ok=%lu temp_invalid=%lu temp_queue_drop=%lu "
            "temp_ack=%lu temp_ack_fail=%lu temp_ack_deferred=%lu temp_ack_wait_dup=%lu "
            "temp_forward_done=%lu temp_forward_fail=%lu "
            "source=%s source_epoch=%lu source_switch=%lu "
            "source_fallback=%lu peer_cooldown=%lu peer_skip=%lu peer_recovery=%lu "
            "send_ms=%lu send_max_ms=%lu free_heap=%u\n",
            static_cast<unsigned long>(periodMs),
            static_cast<double>(deltaRawBytes) / seconds,
            static_cast<double>(deltaRtcmValid) / seconds,
            static_cast<double>(deltaFramesSent) / seconds,
            static_cast<double>(deliveryPercent),
            Serial1.available(),
            static_cast<unsigned>(uxQueueMessagesWaiting(rtcmFrameQueue)),
            static_cast<unsigned long>(pipeline.queueHighWater),
            static_cast<unsigned long>(pipeline.queueDrops),
            static_cast<unsigned long>(pipeline.staleDrops),
            static_cast<unsigned long>(pipeline.localFramesSuppressed),
            static_cast<unsigned long>(pipeline.lastQueueAgeMs),
            static_cast<unsigned long>(pipeline.maxQueueAgeMs),
            static_cast<unsigned long>(rawBytes),
            static_cast<unsigned long>(pipeline.rtcmValidFrames),
            static_cast<unsigned long>(pipeline.rtcmCrcErrors),
            static_cast<unsigned long>(pipeline.rtcmTooLarge),
            static_cast<unsigned long>(espnow.framesSent),
            static_cast<unsigned long>(espnow.framesDropped),
            static_cast<unsigned long>(espnow.fragmentsSent),
            static_cast<unsigned long>(espnow.sendFailures),
            static_cast<unsigned long>(espnow.sendTimeouts),
            static_cast<unsigned long>(espnow.frameRetries),
            static_cast<unsigned long>(espnow.frameAckTimeouts),
            static_cast<unsigned long>(espnow.ackPacketsReceived),
            static_cast<unsigned long>(espnow.ackPacketsInvalid),
            static_cast<unsigned long>(espnow.frameDeadlineDrops),
            static_cast<unsigned long>(pipeline.rtcmSendOk),
            static_cast<unsigned long>(pipeline.rtcmSendFail),
            static_cast<unsigned long>(espnow.activeRoverCount),
            static_cast<unsigned long>(espnow.storedRoverCount),
            static_cast<unsigned long>(espnow.rtcmEnabledRoverCount),
            espnow.pairingActive ? 1U : 0U,
            static_cast<unsigned long>(espnow.pairResponsesReceived),
            static_cast<unsigned long>(espnow.pairConfirmsSent),
            static_cast<unsigned long>(espnow.pairAuthFailures),
            static_cast<unsigned long>(espnow.llhStatusReceived),
            static_cast<unsigned long>(espnow.llhStatusRelayedReceived),
            static_cast<unsigned long>(espnow.llhStatusInvalid),
            static_cast<unsigned long>(espnow.llhStatusUnknownSource),
            static_cast<unsigned long>(espnow.llhStatusCapacityDrops),
            static_cast<unsigned long>(espnow.gnssCommandQueued),
            static_cast<unsigned long>(espnow.gnssCommandSent),
            static_cast<unsigned long>(espnow.gnssCommandResults),
            static_cast<unsigned long>(espnow.gnssCommandTimeouts),
            static_cast<unsigned long>(espnow.gnssCommandInvalidResults),
            static_cast<unsigned long>(espnow.tempFragmentsReceived),
            static_cast<unsigned long>(espnow.tempFramesValid),
            static_cast<unsigned long>(espnow.tempFramesInvalid),
            static_cast<unsigned long>(espnow.tempQueueDrops),
            static_cast<unsigned long>(espnow.tempAcksSent),
            static_cast<unsigned long>(espnow.tempAckFailures),
            static_cast<unsigned long>(espnow.tempAcksDeferred),
            static_cast<unsigned long>(espnow.tempDeferredDuplicates),
            static_cast<unsigned long>(espnow.tempForwardCompleted),
            static_cast<unsigned long>(espnow.tempForwardFailed),
            baseRtcmSourceStateToString(getBaseRtcmSourceSnapshot().state),
            static_cast<unsigned long>(getBaseRtcmSourceSnapshot().epoch),
            static_cast<unsigned long>(espnow.sourceSwitches),
            static_cast<unsigned long>(espnow.sourceFallbacks),
            static_cast<unsigned long>(espnow.peerCooldownEvents),
            static_cast<unsigned long>(espnow.peerCooldownSkips),
            static_cast<unsigned long>(espnow.peerRecoveries),
            static_cast<unsigned long>(espnow.lastFrameSendMs),
            static_cast<unsigned long>(espnow.maxFrameSendMs),
            ESP.getFreeHeap());

        RtcmMessageStats types[sizeof(messageStats) / sizeof(messageStats[0])]{};
        copyMessageStats(types, sizeof(types));
        Serial.print("[BASE][RTCM_TYPES]");
        for (const RtcmMessageStats& entry : types) {
            const uint32_t ageMs = entry.lastReceivedAtMs == 0
                                       ? UINT32_MAX
                                       : now - entry.lastReceivedAtMs;
            if (entry.messageId == 0) {
                Serial.printf(" other=%lu(age=%lu)",
                              static_cast<unsigned long>(entry.count),
                              static_cast<unsigned long>(ageMs));
            } else {
                Serial.printf(" %u=%lu(age=%lu)",
                              entry.messageId,
                              static_cast<unsigned long>(entry.count),
                              static_cast<unsigned long>(ageMs));
            }
        }
        Serial.println();

        const NetworkMqttStats network = getNetworkMqttStats();
        Serial.printf(
            "[BASE][NETWORK_HEALTH] transport=%s configured=%u internet=%u mqtt=%u "
            "signal_dbm=%ld network_attempt=%lu mqtt_attempt=%lu mqtt_connect=%lu "
            "mqtt_disconnect=%lu ecef_published=%lu ecef_publish_fail=%lu "
            "ecef_publish_age_ms=%lu cmd_rx=%lu cmd_reject=%lu "
            "cmd_result_pub=%lu cmd_result_pub_fail=%lu stack_hwm_bytes=%lu\n",
            networkTransportName(),
            network.configured ? 1U : 0U,
            network.internetConnected ? 1U : 0U,
            network.mqttConnected ? 1U : 0U,
            static_cast<long>(network.signalDbm),
            static_cast<unsigned long>(network.networkAttempts),
            static_cast<unsigned long>(network.mqttAttempts),
            static_cast<unsigned long>(network.mqttConnects),
            static_cast<unsigned long>(network.mqttDisconnects),
            static_cast<unsigned long>(network.llhPublished),
            static_cast<unsigned long>(network.llhPublishFailures),
            static_cast<unsigned long>(network.lastLlhPublishedAtMs == 0
                                           ? UINT32_MAX
                                           : now - network.lastLlhPublishedAtMs),
            static_cast<unsigned long>(network.commandsReceived),
            static_cast<unsigned long>(network.commandsRejected),
            static_cast<unsigned long>(network.commandResultsPublished),
            static_cast<unsigned long>(network.commandResultPublishFailures),
            static_cast<unsigned long>(network.stackHighWaterBytes));

        previousLogAt = now;
        previousRawBytes = rawBytes;
        previousRtcmValid = pipeline.rtcmValidFrames;
        previousFramesSent = espnow.framesSent;
        previousFramesDropped = espnow.framesDropped;
    }
}

[[noreturn]] void networkMqttTask(void*)
{
    setupNetworkMqtt();
    while (true) {
        networkMqttLoop();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

[[noreturn]] void roverEcefLogTask(void*)
{
    uint32_t lastSequences[ESPNOW_MAX_ECEF_SOURCES] = {};
    bool haveSequence[ESPNOW_MAX_ECEF_SOURCES] = {};
    static BaseRoverEcefStatus snapshots[ESPNOW_MAX_ECEF_SOURCES] = {};
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(200));
        const size_t count = baseEspNowCopyLatestRoverEcef(
            snapshots, ESPNOW_MAX_ECEF_SOURCES);
        for (size_t index = 0; index < count; ++index) {
            const BaseRoverEcefStatus& status = snapshots[index];
            if (!status.valid ||
                (haveSequence[index] && lastSequences[index] == status.sequence)) {
                continue;
            }
            haveSequence[index] = true;
            lastSequences[index] = status.sequence;
            char relayMacText[18] = {};
            if (status.viaRelay) {
                snprintf(relayMacText, sizeof(relayMacText),
                         "%02X:%02X:%02X:%02X:%02X:%02X",
                         status.relayMac[0], status.relayMac[1], status.relayMac[2],
                         status.relayMac[3], status.relayMac[4], status.relayMac[5]);
            }
            char tempBaseMacText[18] = {};
            if (status.usesTempBase) {
                snprintf(tempBaseMacText, sizeof(tempBaseMacText),
                         "%02X:%02X:%02X:%02X:%02X:%02X",
                         status.tempBaseMac[0], status.tempBaseMac[1],
                         status.tempBaseMac[2], status.tempBaseMac[3],
                         status.tempBaseMac[4], status.tempBaseMac[5]);
            }
            Serial.printf(
                "[BASE][ROVER_ECEF] mac=%02X:%02X:%02X:%02X:%02X:%02X "
                "seq=%lu gnss_ms=%lu stream=%u "
                "ecef_m=(%.4f,%.4f,%.4f) fix_quality=%u "
                "via=%s relay_mac=%s rtcm_source=%s temp_base_mac=%s "
                "source_epoch=%lu age_ms=%lu\n",
                status.mac[0], status.mac[1], status.mac[2],
                status.mac[3], status.mac[4], status.mac[5],
                static_cast<unsigned long>(status.sequence),
                static_cast<unsigned long>(status.gnssTimeMsOfDay),
                static_cast<unsigned>(status.correctionStreamId),
                static_cast<double>(status.ecefXScaled) /
                    RTCM_ESPNOW_ECEF_SCALE,
                static_cast<double>(status.ecefYScaled) /
                    RTCM_ESPNOW_ECEF_SCALE,
                static_cast<double>(status.ecefZScaled) /
                    RTCM_ESPNOW_ECEF_SCALE,
                static_cast<unsigned>(status.fixQuality),
                status.viaRelay ? "relay" : "direct",
                relayMacText,
                status.usesTempBase ? "temp_base" : "local_base",
                tempBaseMacText,
                static_cast<unsigned long>(status.rtcmSourceEpoch),
                static_cast<unsigned long>(millis() - status.receivedAtMs));
        }
    }
}

bool createTask(TaskFunction_t function,
                const char* name,
                uint32_t stackSize,
                UBaseType_t priority,
                BaseType_t core)
{
    if (xTaskCreatePinnedToCore(function, name, stackSize, nullptr, priority, nullptr, core) ==
        pdPASS) {
        return true;
    }
    Serial.printf("[BASE][SETUP][ERROR] Failed to create task %s\n", name);
    return false;
}
}

void setup()
{
    Serial.begin(115200);
    const unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 3000) {
        delay(10);
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println();
    Serial.println("=========================================");
    Serial.println("       ESP32 GNSS BASE ESP-NOW           ");
    Serial.println("=========================================");

    if (Serial1.setRxBufferSize(GNSS_RX_BUFFER_SIZE) != GNSS_RX_BUFFER_SIZE) {
        Serial.println("[BASE][GNSS][ERROR] Failed to set UART RX buffer");
    }
    if (Serial1.setTxBufferSize(GNSS_TX_BUFFER_SIZE) != GNSS_TX_BUFFER_SIZE) {
        Serial.println("[BASE][GNSS][WARN] Failed to set UART TX buffer");
    }
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    Serial.printf("[BASE][GNSS] ESP32 Serial1 reading %s, baud=%lu RX=%d TX=%d rx_buffer=%u\n",
                  GNSS_UART_PORT_NAME,
                  static_cast<unsigned long>(GNSS_BAUD),
                  RX_GNSS,
                  TX_GNSS,
                  static_cast<unsigned>(GNSS_RX_BUFFER_SIZE));
    Serial.printf("[BASE][DEBUG] uart_raw_dump=%s rtcm_hex_dump=%s frame_log=%s\n",
                  DEBUG_GNSS_UART_RAW_DUMP ? "on" : "off",
                  DEBUG_RTCM_HEX_DUMP ? "on" : "off",
                  DEBUG_RTCM_FRAME_LOG ? "on" : "off");

    rtcmFrameQueue = xQueueCreate(RTCM_FRAME_QUEUE_LENGTH, sizeof(RtcmFrameEnvelope));
    if (rtcmFrameQueue == nullptr) {
        Serial.println("[BASE][SETUP][ERROR] Failed to create RTCM frame queue");
        delay(5000);
        ESP.restart();
    }

    if (!setupEspNowBase()) {
        Serial.println("[BASE][SETUP][ERROR] ESP-NOW init failed, restarting in 5s");
        delay(5000);
        ESP.restart();
    }
    if (!baseGnssRoleSetup()) {
        Serial.println("[BASE][SETUP][ERROR] Local GNSS role controller failed");
        delay(5000);
        ESP.restart();
    }

    const bool tasksReady =
        createTask(taskRtcmReader, "RTCM Reader", 6144, 4, 1) &&
        createTask(taskRtcmSender, "RTCM Sender", 6144, 3, 1) &&
        createTask(healthLogTask, "Health Task", 4096, 1, 1) &&
        createTask(roverEcefLogTask, "Rover ECEF", 4096, 1, 1) &&
        createTask(networkMqttTask, "Network MQTT", 10240, 1, 0);
    if (!tasksReady) {
        delay(5000);
        ESP.restart();
    }

    Serial.println("[BASE][SETUP] Khoi dong hoan tat");
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
}

void loop()
{
    baseEspNowLoop();
    baseGnssRoleLoop();
    vTaskDelay(pdMS_TO_TICKS(20));
}
