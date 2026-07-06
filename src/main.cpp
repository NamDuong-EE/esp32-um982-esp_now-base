#include <Arduino.h>

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "functions/Rtcm_Frame_Reader.h"
#include "hardware/BaseEspnow_sender.h"

namespace {
uint8_t rtcmFrame[RTCM_ESPNOW_MAX_FRAME_LENGTH] = {};
uint32_t rtcmValidFrames = 0;
uint32_t rtcmCrcErrors = 0;
uint32_t rtcmTooLarge = 0;
uint32_t rtcmSendOk = 0;
uint32_t rtcmSendFail = 0;

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

[[noreturn]] void taskRtcm(void*)
{
    while (true) {
        size_t frameLength = 0;
        const RtcmReadResult result =
            readRtcmFrame(Serial1, rtcmFrame, sizeof(rtcmFrame), frameLength);

        switch (result) {
        case RtcmReadResult::FrameValid:
            ++rtcmValidFrames;
            flushRtcmDebugLine();
            Serial.printf("[BASE][GNSS] RTCM frame valid, length=%u\n",
                          static_cast<unsigned>(frameLength));
            dumpRtcmFrameHex("valid", rtcmFrame, frameLength);
            if (baseEspNowSendRtcmFrame(rtcmFrame, frameLength)) {
                ++rtcmSendOk;
            } else {
                ++rtcmSendFail;
            }
            break;

        case RtcmReadResult::CrcError:
            ++rtcmCrcErrors;
            flushRtcmDebugLine();
            Serial.printf("[BASE][GNSS][WARN] RTCM CRC error, length=%u\n",
                          static_cast<unsigned>(frameLength));
            dumpRtcmFrameHex("crc_error", rtcmFrame, frameLength);
            break;

        case RtcmReadResult::FrameTooLarge:
            ++rtcmTooLarge;
            flushRtcmDebugLine();
            Serial.println("[BASE][GNSS][WARN] RTCM frame too large");
            break;

        case RtcmReadResult::None:
            vTaskDelay(pdMS_TO_TICKS(2));
            break;
        }
    }
}

[[noreturn]] void healthLogTask(void*)
{
    uint32_t previousLogAt = millis();
    uint32_t previousRawBytes = getRtcmRawByteCount();
    uint32_t previousRtcmValid = rtcmValidFrames;
    uint32_t previousFramesSent = getBaseEspnowStats().framesSent;
    uint32_t previousFramesDropped = getBaseEspnowStats().framesDropped;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));

        const uint32_t now = millis();
        const uint32_t periodMs = now - previousLogAt;
        const BaseEspnowStats& espnow = getBaseEspnowStats();
        const uint32_t rawBytes = getRtcmRawByteCount();
        const uint32_t validFrames = rtcmValidFrames;
        const uint32_t framesSent = espnow.framesSent;
        const uint32_t framesDropped = espnow.framesDropped;

        const uint32_t deltaRawBytes = rawBytes - previousRawBytes;
        const uint32_t deltaRtcmValid = validFrames - previousRtcmValid;
        const uint32_t deltaFramesSent = framesSent - previousFramesSent;
        const uint32_t deltaFramesDropped = framesDropped - previousFramesDropped;
        const uint32_t deltaDeliveryTotal = deltaFramesSent + deltaFramesDropped;
        const float seconds = periodMs > 0 ? static_cast<float>(periodMs) / 1000.0F : 1.0F;
        const float deliveryPercent = deltaDeliveryTotal > 0
                                          ? (100.0F * static_cast<float>(deltaFramesSent) /
                                             static_cast<float>(deltaDeliveryTotal))
                                          : 0.0F;

        flushRtcmDebugLine();
        Serial.printf(
            "[BASE][HEALTH] period_ms=%lu uart_Bps=%.1f rtcm_fps=%.2f send_fps=%.2f "
            "delivery=%.1f%% uart_available=%d uart_raw_bytes=%lu rtcm_valid=%lu crc_error=%lu "
            "too_large=%lu frames_sent=%lu frames_dropped=%lu fragments_sent=%lu send_fail=%lu "
            "send_timeout=%lu task_send_ok=%lu task_send_fail=%lu\n",
            static_cast<unsigned long>(periodMs),
            static_cast<double>(deltaRawBytes) / seconds,
            static_cast<double>(deltaRtcmValid) / seconds,
            static_cast<double>(deltaFramesSent) / seconds,
            static_cast<double>(deliveryPercent),
            Serial1.available(),
            static_cast<unsigned long>(rawBytes),
            static_cast<unsigned long>(validFrames),
            static_cast<unsigned long>(rtcmCrcErrors),
            static_cast<unsigned long>(rtcmTooLarge),
            static_cast<unsigned long>(framesSent),
            static_cast<unsigned long>(framesDropped),
            static_cast<unsigned long>(espnow.fragmentsSent),
            static_cast<unsigned long>(espnow.sendFailures),
            static_cast<unsigned long>(espnow.sendTimeouts),
            static_cast<unsigned long>(rtcmSendOk),
            static_cast<unsigned long>(rtcmSendFail));

        previousLogAt = now;
        previousRawBytes = rawBytes;
        previousRtcmValid = validFrames;
        previousFramesSent = framesSent;
        previousFramesDropped = framesDropped;
    }
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

    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    Serial.printf("[BASE][GNSS] ESP32 Serial1 reading %s, baud=%lu RX=%d TX=%d\n",
                  GNSS_UART_PORT_NAME,
                  static_cast<unsigned long>(GNSS_BAUD),
                  RX_GNSS,
                  TX_GNSS);
    Serial.printf("[BASE][DEBUG] uart_raw_dump=%s rtcm_hex_dump=%s\n",
                  DEBUG_GNSS_UART_RAW_DUMP ? "on" : "off",
                  DEBUG_RTCM_HEX_DUMP ? "on" : "off");

    if (!setupEspNowBase()) {
        Serial.println("[BASE][SETUP][ERROR] ESP-NOW init failed, restarting in 5s");
        delay(5000);
        ESP.restart();
    }

    xTaskCreatePinnedToCore(taskRtcm, "RTCM Task", 4096, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(healthLogTask, "Health Task", 4096, nullptr, 1, nullptr, 1);

    Serial.println("[BASE][SETUP] Khoi dong hoan tat");
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
