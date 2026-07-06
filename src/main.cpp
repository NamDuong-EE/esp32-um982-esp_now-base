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

[[noreturn]] void taskRtcm(void*)
{
    while (true) {
        size_t frameLength = 0;
        const RtcmReadResult result =
            readRtcmFrame(Serial1, rtcmFrame, sizeof(rtcmFrame), frameLength);

        switch (result) {
        case RtcmReadResult::FrameValid:
            ++rtcmValidFrames;
            Serial.printf("[BASE][GNSS] RTCM frame valid, length=%u\n",
                          static_cast<unsigned>(frameLength));
            if (baseEspNowSendRtcmFrame(rtcmFrame, frameLength)) {
                ++rtcmSendOk;
            } else {
                ++rtcmSendFail;
            }
            break;

        case RtcmReadResult::CrcError:
            ++rtcmCrcErrors;
            Serial.printf("[BASE][GNSS][WARN] RTCM CRC error, length=%u\n",
                          static_cast<unsigned>(frameLength));
            break;

        case RtcmReadResult::FrameTooLarge:
            ++rtcmTooLarge;
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
    while (true) {
        const BaseEspnowStats& espnow = getBaseEspnowStats();
        Serial.printf(
            "[BASE][HEALTH] rtcm_valid=%lu crc_error=%lu too_large=%lu frames_sent=%lu "
            "frames_dropped=%lu fragments_sent=%lu send_fail=%lu send_timeout=%lu task_send_ok=%lu task_send_fail=%lu\n",
            static_cast<unsigned long>(rtcmValidFrames),
            static_cast<unsigned long>(rtcmCrcErrors),
            static_cast<unsigned long>(rtcmTooLarge),
            static_cast<unsigned long>(espnow.framesSent),
            static_cast<unsigned long>(espnow.framesDropped),
            static_cast<unsigned long>(espnow.fragmentsSent),
            static_cast<unsigned long>(espnow.sendFailures),
            static_cast<unsigned long>(espnow.sendTimeouts),
            static_cast<unsigned long>(rtcmSendOk),
            static_cast<unsigned long>(rtcmSendFail));

        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
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
    Serial.printf("[BASE][GNSS] UART1 baud=%lu RX=%d TX=%d\n",
                  static_cast<unsigned long>(GNSS_BAUD),
                  RX_GNSS,
                  TX_GNSS);

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
