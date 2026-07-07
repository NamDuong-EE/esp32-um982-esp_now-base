#include "hardware/BaseEspnow_sender.h"

#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/semphr.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"

namespace {
BaseEspnowStats stats;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
uint16_t streamId = 0;
uint32_t frameSequence = 0;
volatile bool lastSendSucceeded = false;
volatile bool waitingForFrameAck = false;
volatile uint16_t expectedAckStreamId = 0;
volatile uint32_t expectedAckSequence = 0;
SemaphoreHandle_t sendCallbackSemaphore = nullptr;
SemaphoreHandle_t frameAckSemaphore = nullptr;

void incrementStat(uint32_t BaseEspnowStats::*member, uint32_t amount = 1)
{
    portENTER_CRITICAL(&statsMux);
    stats.*member += amount;
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

bool roverMacConfigured()
{
    for (uint8_t byte : ESPNOW_ROVER_MAC) {
        if (byte != 0) {
            return true;
        }
    }
    return false;
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

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int length)
{
    const uint8_t* sourceMac = info == nullptr ? nullptr : info->src_addr;
#else
void onDataReceived(const uint8_t* sourceMac, const uint8_t* data, int length)
#endif
{
    if (sourceMac == nullptr || std::memcmp(sourceMac, ESPNOW_ROVER_MAC, 6) != 0 ||
        data == nullptr || length != static_cast<int>(sizeof(RtcmEspNowAck))) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    RtcmEspNowAck ack{};
    std::memcpy(&ack, data, sizeof(ack));
    if (ack.magic != RTCM_ESPNOW_MAGIC ||
        ack.version != RTCM_ESPNOW_VERSION ||
        ack.packetType != RTCM_ESPNOW_PACKET_TYPE_FRAME_ACK ||
        ack.status != RTCM_ESPNOW_ACK_STATUS_WRITTEN) {
        incrementStat(&BaseEspnowStats::ackPacketsInvalid);
        return;
    }

    if (!waitingForFrameAck || ack.streamId != expectedAckStreamId ||
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

bool sendPacketWithRetry(const uint8_t* packet, size_t packetLength, uint32_t frameStartedAt)
{
    for (uint8_t attempt = 0; attempt <= ESPNOW_SEND_RETRY_COUNT; ++attempt) {
        if (frameDeadlineExpired(frameStartedAt)) {
            incrementStat(&BaseEspnowStats::frameDeadlineDrops);
            return false;
        }

        while (xSemaphoreTake(sendCallbackSemaphore, 0) == pdTRUE) {
        }
        lastSendSucceeded = false;

        const esp_err_t sendResult = esp_now_send(ESPNOW_ROVER_MAC, packet, packetLength);
        if (sendResult != ESP_OK) {
            incrementStat(&BaseEspnowStats::sendFailures);
            Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_send failed: %d\n", sendResult);
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
}

bool setupEspNowBase()
{
    if (!roverMacConfigured()) {
        Serial.println("[BASE][ESP-NOW][WARN] ESPNOW_ROVER_MAC is still 00:00:00:00:00:00");
    }

    sendCallbackSemaphore = xSemaphoreCreateBinary();
    frameAckSemaphore = xSemaphoreCreateBinary();
    if (sendCallbackSemaphore == nullptr || frameAckSemaphore == nullptr) {
        Serial.println("[BASE][ESP-NOW][ERROR] Failed to create callback semaphores");
        return false;
    }

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

    esp_now_peer_info_t peerInfo = {};
    std::memcpy(peerInfo.peer_addr, ESPNOW_ROVER_MAC, sizeof(peerInfo.peer_addr));
    peerInfo.channel = ESPNOW_WIFI_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = ESPNOW_ENCRYPTION_ENABLED;
    if (ESPNOW_ENCRYPTION_ENABLED) {
        std::memcpy(peerInfo.lmk, ESPNOW_LMK, sizeof(peerInfo.lmk));
    }

    result = esp_now_add_peer(&peerInfo);
    if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_add_peer failed: %d\n", result);
        return false;
    }

#if defined(WIFI_PHY_RATE_LORA_250K)
    if (ESPNOW_USE_LR_250KBPS) {
        result = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_250K);
        if (result != ESP_OK) {
            Serial.printf("[BASE][ESP-NOW][WARN] espnow LR rate config failed: %d\n", result);
        }
    }
#endif

    streamId = static_cast<uint16_t>(esp_random() & 0xFFFFU);

    Serial.println("[WIFI] Khong ket noi router/AP; chi dung STA radio cho ESP-NOW");
    Serial.print("[WIFI] Local STA MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("[WIFI] ESP-NOW fixed channel: %u\n", ESPNOW_WIFI_CHANNEL);
    Serial.printf("[BASE][ESP-NOW] Ready, channel=%u, LR=%s, streamId=%u\n",
                  ESPNOW_WIFI_CHANNEL,
                  ESPNOW_USE_LR_250KBPS ? "250 Kbps" : "off",
                  streamId);

    return true;
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

    uint8_t packet[RTCM_ESPNOW_MAX_PACKET_SIZE] = {};
    const uint32_t currentSequence = frameSequence;
    const uint32_t sendStartedAt = millis();

    while (xSemaphoreTake(frameAckSemaphore, 0) == pdTRUE) {
    }
    expectedAckStreamId = streamId;
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
            header->streamId = streamId;
            header->frameSequence = currentSequence;
            header->frameLength = static_cast<uint16_t>(length);
            header->fragmentIndex = fragmentIndex;
            header->fragmentCount = fragmentCount;
            header->payloadLength = static_cast<uint16_t>(payloadLength);

            std::memcpy(packet + sizeof(RtcmEspNowHeader), frame + offset, payloadLength);

            const size_t packetLength = sizeof(RtcmEspNowHeader) + payloadLength;
            if (!sendPacketWithRetry(packet, packetLength, frameAttemptStartedAt)) {
                allFragmentsSent = false;
                Serial.printf("[BASE][ESP-NOW][WARN] Frame seq=%lu attempt=%u stopped at fragment %u/%u\n",
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
            incrementStat(&BaseEspnowStats::framesSent);
            ++frameSequence;
            recordFrameSendDuration(millis() - sendStartedAt);
            return true;
        }

        if (allFragmentsSent) {
            incrementStat(&BaseEspnowStats::frameAckTimeouts);
            Serial.printf("[BASE][ESP-NOW][WARN] Frame ACK timeout seq=%lu attempt=%u\n",
                          static_cast<unsigned long>(currentSequence),
                          frameAttempt + 1);
        }
        if (frameAttempt < ESPNOW_FRAME_RETRY_COUNT) {
            incrementStat(&BaseEspnowStats::frameRetries);
            delay(10);
        }
    }

    waitingForFrameAck = false;
    incrementStat(&BaseEspnowStats::framesDropped);
    ++frameSequence;
    recordFrameSendDuration(millis() - sendStartedAt);
    Serial.printf("[BASE][ESP-NOW][ERROR] Drop frame seq=%lu without application ACK\n",
                  static_cast<unsigned long>(currentSequence));
    return false;
}

BaseEspnowStats getBaseEspnowStats()
{
    portENTER_CRITICAL(&statsMux);
    const BaseEspnowStats copy = stats;
    portEXIT_CRITICAL(&statsMux);
    return copy;
}

uint16_t getBaseEspNowStreamId()
{
    return streamId;
}
