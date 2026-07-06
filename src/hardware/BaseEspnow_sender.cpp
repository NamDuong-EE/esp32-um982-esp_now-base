#include "hardware/BaseEspnow_sender.h"

#include <WiFi.h>
#include <cstring>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"

namespace {
BaseEspnowStats stats;
uint16_t streamId = 0;
uint32_t frameSequence = 0;
volatile bool sendCallbackReceived = false;
volatile bool lastSendSucceeded = false;

bool roverMacConfigured()
{
    for (uint8_t byte : ESPNOW_ROVER_MAC) {
        if (byte != 0) {
            return true;
        }
    }
    return false;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t*, esp_now_send_status_t status)
#else
void onDataSent(const uint8_t*, esp_now_send_status_t status)
#endif
{
    lastSendSucceeded = (status == ESP_NOW_SEND_SUCCESS);
    sendCallbackReceived = true;
}

bool waitForSendCallback()
{
    const uint32_t startedAt = millis();
    while (!sendCallbackReceived && (millis() - startedAt) < ESPNOW_SEND_TIMEOUT_MS) {
        delay(1);
    }
    return sendCallbackReceived && lastSendSucceeded;
}

bool sendPacketWithRetry(const uint8_t* packet, size_t packetLength)
{
    for (uint8_t attempt = 0; attempt <= ESPNOW_SEND_RETRY_COUNT; ++attempt) {
        sendCallbackReceived = false;
        lastSendSucceeded = false;

        const esp_err_t sendResult = esp_now_send(ESPNOW_ROVER_MAC, packet, packetLength);
        if (sendResult != ESP_OK) {
            ++stats.sendFailures;
            Serial.printf("[BASE][ESP-NOW][ERROR] esp_now_send failed: %d\n", sendResult);
            delay(5);
            continue;
        }

        if (waitForSendCallback()) {
            ++stats.fragmentsSent;
            return true;
        }

        if (!sendCallbackReceived) {
            ++stats.sendTimeouts;
            Serial.println("[BASE][ESP-NOW][WARN] Send callback timeout");
        } else {
            ++stats.sendFailures;
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

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    delay(50);

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

    esp_now_register_send_cb(onDataSent);

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
        ++stats.framesDropped;
        return false;
    }

    const uint8_t fragmentCount = static_cast<uint8_t>(
        (length + RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD - 1) / RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD);
    if (fragmentCount == 0 || fragmentCount > RTCM_ESPNOW_MAX_FRAGMENT_COUNT) {
        ++stats.framesDropped;
        return false;
    }

    uint8_t packet[RTCM_ESPNOW_MAX_PACKET_SIZE] = {};
    const uint32_t currentSequence = frameSequence;

    for (uint8_t fragmentIndex = 0; fragmentIndex < fragmentCount; ++fragmentIndex) {
        const size_t offset = static_cast<size_t>(fragmentIndex) * RTCM_ESPNOW_MAX_FRAGMENT_PAYLOAD;
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
        if (!sendPacketWithRetry(packet, packetLength)) {
            ++stats.framesDropped;
            ++frameSequence;
            Serial.printf("[BASE][ESP-NOW][ERROR] Drop frame seq=%lu at fragment %u/%u\n",
                          static_cast<unsigned long>(currentSequence),
                          fragmentIndex + 1,
                          fragmentCount);
            return false;
        }
    }

    ++stats.framesSent;
    ++frameSequence;
    return true;
}

const BaseEspnowStats& getBaseEspnowStats()
{
    return stats;
}

uint16_t getBaseEspNowStreamId()
{
    return streamId;
}
