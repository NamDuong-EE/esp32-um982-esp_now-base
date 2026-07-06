#include "helper.h"
#include "functions/RTCM_Receiver.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern PubSubClient mqtt;
#if CONNECT_USING_4G && RTCM_COMMUNICATION_PROTOCOL==TCP_IP
extern TinyGsmClient ntripClient;
extern TinyGsm modem;
#endif

#if CONNECT_USING_WIFI && RTCM_COMMUNICATION_PROTOCOL==TCP_IP
extern WiFiClient ntripClient;
#endif

#if RTCM_COMMUNICATION_PROTOCOL==LORA_SERIAL
String rtcmBuffer = ""; // Bộ đệm đọc RTCM từ UM980 để gửi lên Caster qua NTRIP
#endif
unsigned long lastHealthCheck = 0;
String latestRtcm = "";

// Semaphore
SemaphoreHandle_t rtcmBufferMutex = nullptr;
SemaphoreHandle_t tcpStreamMutex = nullptr;

/* ===================== NGUYÊN MẪU HÀM ======================== */

#if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
__attribute__((noreturn)) void taskLora(void* parameter);
__attribute__((noreturn)) void taskRtcm(void* parameter);
#else
__attribute__((noreturn)) void taskNtrip(void* parameter);
#endif
__attribute__((noreturn)) void healthCheckTask(void* parameter);

/* ==================SETUP VÀ LOOP======================== */

void setup()
{
    Serial.begin(115200);
    #if BOARD_HELTEC
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    #endif
    delay(100);

    // Debug marker: confirm Serial is working immediately after begin()
    Serial.println("[DEBUG] Serial initialized");
    Serial.println("\n=========================================");
    Serial.println("     ESP32 GNSS GATEWAY KHOI DONG        ");
    Serial.println("=========================================");

    // Khởi tạo giao tiếp với UM980
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    bool networkConnected = false;

    #ifndef NATIVE_BUILD
    #if CONNECT_USING_4G
    SerialAT.begin(115200, SERIAL_8N1, RX_TO_MODEM_TX, TX_TO_MODEM_RX);
    delay(500);
    #endif
    #endif

    #if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
    int loraSetupResult = loraSetup();
    if (loraSetupResult != 0) {
        Serial.println("[SETUP][ERROR] Khoi dong LoRa that bai! Vui long kiem tra cau hinh va thu lai.");
    } else {
        Serial.println("[SETUP] Khoi dong LoRa thanh cong!");
    }
    #endif

    while (!networkConnected) {
#if CONNECT_USING_WIFI
        Serial.println("[SETUP] Su dung ket noi WIFI");
        networkConnected = setupWiFi();
#endif
#if CONNECT_USING_4G
        Serial.println("[SETUP] Su dung ket noi SIM/GSM");
        if (startSIM()) {
            if (connectGSM()) {
                networkConnected = true;
            }
        }
#endif
        if (networkConnected) {
            Serial.println("[SETUP] Ket noi mang thanh cong!");
            setupMQTT();
            #if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
            setupNTRIP();
            connectNTRIP();
            #endif
        } else {
            Serial.println("[ERROR] Khong the ket noi mang. Vui long kiem tra cau hinh va thu lai.");
        }
    }

    Serial.println("[SETUP] Khoi dong cac task...");

    Serial.println("[Setup] Tao mutex de dong bo hoa tai nguyen chung");
    
    rtcmBufferMutex = xSemaphoreCreateMutex();
    while (rtcmBufferMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex rtcmDataMutex that bai! Dang thu lai...");
        rtcmBufferMutex = xSemaphoreCreateMutex();
    }
    Serial.println("[SETUP] Tao mutex rtcmDataMutex thanh cong!");

    #if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
    tcpStreamMutex = xSemaphoreCreateMutex();
    while (tcpStreamMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex tcpStreamMutex that bai! Dang thu lai...");
        tcpStreamMutex = xSemaphoreCreateMutex();
    }
    Serial.println("[SETUP] Tao mutex tcpStreamMutex thanh cong!");
    #endif

    #if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
    Serial.println("[SETUP] Task LoRa: Truyen du lieu RTCM qua LoRa.");
    xTaskCreatePinnedToCore(taskLora, "LoRa Task", 4096, nullptr, 1, nullptr, 0);
    Serial.println("[SETUP] Da khoi dong Task LoRa!");

    Serial.println("[SETUP] Task RTCM: Doc du lieu RTCM tu UM980.");
    xTaskCreatePinnedToCore(taskRtcm, "RTCM Task", 4096, nullptr, 2, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task RTCM!");
    #elif RTCM_COMMUNICATION_PROTOCOL == TCP_IP
    Serial.println("[SETUP] Task NTRIP: Gui du lieu RTCM qua NTRIP.");
    xTaskCreatePinnedToCore(taskNtrip, "NTRIP Task", 4096, nullptr, 2, nullptr, 0);
    Serial.println("[SETUP] Da khoi dong Task NTRIP!");
    #endif


    Serial.println("[SETUP] Task Health: Gui thong tin suc khoe thiet bi len MQTT moi 30s");
    xTaskCreatePinnedToCore(healthCheckTask, "Health Task", 4096, nullptr, 1, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task Health!");

    Serial.println("=========================================");
    Serial.println("        KHOI DONG HOAN TAT               ");
    Serial.println("=========================================\n");

    digitalWrite(LED_PIN, HIGH);

    delay(1000);

    digitalWrite(LED_PIN, LOW);
}

#if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
/* ================= TRIỂN KHAI HÀM TASK ====================== */
__attribute__((noreturn)) void taskRtcm(void* parameter) {
    // Sử dụng chung rtcmBuffer với taskLora, cần mutex
    while (true) {
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            if (!rtcmBuffer.isEmpty()) {
                #if PROGRAM_DEBUG
                Serial.println("[RTCM TASK] LoRa chua kip gui xong du lieu RTCM truoc do, dang doi de gui tiep...");
                #endif
                goto giveUpMutexRtcm;
            }

            rtcmBuffer = receiveRtcmFromGnss();
            if (!rtcmBuffer.isEmpty()) {
                Serial.println("[RTCM TASK] Da nhan du lieu RTCM tu mach RTK. So byte: " + String(rtcmBuffer.length()));
            } else {
                Serial.println("[RTCM TASK] Du lieu RTCM rong.");

            }
            Serial.println();

            giveUpMutexRtcm:
            xSemaphoreGive(rtcmBufferMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#else
__attribute__((noreturn)) void taskNtrip(void* parameter) {
    Serial.println("[NTRIP TASK] Bat dau task NTRIP...");
    int loopStatus = 0;
    String rtcmRead = "";
    while (true) {
        
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            latestRtcm = receiveRtcmFromGnss();
            rtcmRead = latestRtcm;
            xSemaphoreGive(rtcmBufferMutex);
        }
        if (xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            if (loopStatus == 504) {
                Serial.println("[NTRIP TASK] Dang thu ket noi lai NTRIP...");
                connectNTRIP();
            }
            loopStatus = loopNTRIP(rtcmRead);
            #if PROGRAM_DEBUG
            Serial.println("[NTRIP TASK] loopNTRIP() tra ve: " + String(loopStatus));
            #endif
            xSemaphoreGive(tcpStreamMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif

#if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
__attribute__((noreturn))void taskLora(void* parameter) {
    // Sử dụng chung rtcmBuffer với taskRtcm, cần mutex
    String tempRtcm = "";
    bool lastStateWasEmpty = true;
    while (true) {
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            #if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
            loopNTRIP(latestGGA);
            #else
            if (rtcmBuffer.isEmpty()) {
                #if PROGRAM_DEBUG
                Serial.println("[LORA TASK] Chua co du lieu RTCM de truyen qua LoRa.");
                #endif
                lastStateWasEmpty = true;
                goto giveUpMutexLora;
            }

            if (lastStateWasEmpty) {
                tempRtcm = rtcmBuffer;
                lastStateWasEmpty = false;
            }

            #if PROGRAM_DEBUG
            Serial.println("[LORA TASK] Chuan bi truyen du lieu RTCM qua LoRA...");

            Serial.println("[LORA TASK] Noi dung con lai trong buffer duoc in ra theo hexa:");

            for (int i = 0; i < rtcmBuffer.length(); i++) {
                Serial.printf("%02X ", static_cast<uint8_t>(rtcmBuffer[i]));

                if ((i + 1) % 16 == 0) {
                    Serial.println();
                }
            }

            Serial.println();
            #endif // PROGRAM_DEBUG

            lora_packet_process(rtcmBuffer);

            if (rtcmBuffer.isEmpty() && !lastStateWasEmpty) {
                latestRtcm = tempRtcm;
                lastStateWasEmpty = true;
            }

            #if PROGRAM_DEBUG
            Serial.println("[LORA TASK] Da xoa du lieu RTCM trong buffer sau khi gui.");
            #endif // PROGRAM_DEBUG
            #endif

            giveUpMutexLora:
            xSemaphoreGive(rtcmBufferMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif

__attribute__((noreturn)) void healthCheckTask(void* parameter) {
    // Có tranh chấp tài nguyên với task RTCM và NTRIP publish
    String healthPayload = "";
    uint32_t loopStartTime = 0;
    uint32_t remainingWait = 0;
    while (true) {
        loopStartTime = millis();
        #if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            healthPayload = formDeviceHealthString();
            xSemaphoreGive(rtcmBufferMutex);
        }
        #else
            healthPayload = formDeviceHealthString();
        #endif
        Serial.print("[HEALTH CHECK] ");
        Serial.println(healthPayload);

        if (healthPayload.isEmpty())
        {
            if (HEALTH_INTERVAL > (millis() - loopStartTime)) {
                remainingWait = HEALTH_INTERVAL - (millis() - loopStartTime);
            } else {
                remainingWait = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(remainingWait));
            continue;
        }

        #if PROGRAM_DEBUG
        Serial.println("[HEALTH CHECK] Kiem tra ket noi MQTT de gui thong tin suc khoe...");
        #endif

        if (xSemaphoreTake(tcpStreamMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            // Perform MQTT operations here
            
            if (!mqtt.connected()) {
                digitalWrite(LED_PIN, HIGH);
                Serial.println("[LOOP] MQTT mat ket noi, dang thu ket noi lai...");
                connectMQTT();
                digitalWrite(LED_PIN, LOW);
            }

            #if PROGRAM_DEBUG
            Serial.println("[HEALTH CHECK] MQTT dang ket noi, dang kich hoat loop...");
            #endif

            mqtt.loop();

            #if PROGRAM_DEBUG
            Serial.println("[HEALTH CHECK] Dang gui thong tin suc khoe len MQTT...");
            #endif

            publishHealth(healthPayload);

            if (!latestRtcm.isEmpty()) {
                #if PROGRAM_DEBUG
                Serial.println("[GNSS PUBLISH] Dang kich hoat loop...");
                #endif
                mqtt.loop();
                #if PROGRAM_DEBUG
                Serial.println("[GNSS PUBLISH] Dang gui du lieu NMEA len MQTT...");
                #endif
                publishRaw(latestRtcm); // publishRaw accepts String&

                /*Xóa tọa độ sau khi đã dùng để đánh giá sức khoẻ, nếu còn giữ, 
                trong trường hợp không có dữ liệu mới, sẽ luôn báo GNSS OK dù 
                thực tế đã mất tín hiệu. Việc này giúp phản ánh tình trạng thực tế hơn.*/ 
                latestRtcm = "";
            }

            xSemaphoreGive(tcpStreamMutex);
        }
        if (HEALTH_INTERVAL > (millis() - loopStartTime)) {
            remainingWait = HEALTH_INTERVAL - (millis() - loopStartTime);
        } else {
            remainingWait = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(remainingWait));
    }
}

void loop() {
    #if CONNECT_USING_4G
    if (!modem.isGprsConnected()) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("[LOOP] GPRS mat ket noi, dang thu ket noi lai...");
        connectGSM();
        digitalWrite(LED_PIN, LOW);
    }
    #endif
    #if CONNECT_USING_WIFI
    if (WiFiClass::status() != WL_CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("[LOOP] WiFi mat ket noi, dang thu ket noi lai...");
        setupWiFi();
        digitalWrite(LED_PIN, LOW);
    }
    #endif
    vTaskDelay(pdMS_TO_TICKS(1000)); // loop trống, tất cả logic đã được xử lý trong các task
}
