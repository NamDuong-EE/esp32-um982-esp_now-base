#include "helper.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern PubSubClient mqtt;

String rtcmBuffer = "";
unsigned long lastHealthCheck = 0;
String latestRtcm = "";

// Semaphore
SemaphoreHandle_t rtcmBufferMutex = nullptr;

/* ===================== NGUYÊN MẪU HÀM ======================== */

__attribute__((noreturn)) void taskLora(void* parameter);
__attribute__((noreturn)) void taskRtcm(void* parameter);
__attribute__((noreturn)) void gnssPublishTask(void* parameter);
__attribute__((noreturn)) void healthCheckTask(void* parameter);

/* ==================SETUP VÀ LOOP======================== */

void setup()
{
    Serial.begin(115200);
    unsigned long serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 5000) {
        delay(10);
    }

    // Debug marker: confirm Serial is working immediately after begin()
    Serial.println("[DEBUG] Serial initialized");
    Serial.println("\n=========================================");
    Serial.println("     ESP32 GNSS GATEWAY KHOI DONG        ");
    Serial.println("=========================================");

    // Khởi tạo giao tiếp với UM980
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    bool networkConnected = false;

    int loraSetupResult = loraSetup();
    if (loraSetupResult != 0) {
        Serial.println("[SETUP][ERROR] Khoi dong LoRa that bai! Vui long kiem tra cau hinh va thu lai.");
    } else {
        Serial.println("[SETUP] Khoi dong LoRa thanh cong!");
    }

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
            #if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
            setupNTRIP();
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
    
    Serial.println("[SETUP] Task LoRa: Truyen du lieu RTCM qua LoRa.");
    xTaskCreatePinnedToCore(taskLora, "LoRa Task", 4096, nullptr, 1, nullptr, 0);
    Serial.println("[SETUP] Da khoi dong Task LoRa!");

    Serial.println("[SETUP] Task RTCM: Doc du lieu RTCM tu UM980.");
    xTaskCreatePinnedToCore(taskRtcm, "RTCM Task", 4096, nullptr, 2, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task RTCM!");


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

/* ================= TRIỂN KHAI HÀM TASK ====================== */
__attribute__((noreturn)) void taskRtcm(void* parameter) {
    // Sử dụng chung rtcmBuffer với taskLora, cần mutex
    while (true) {
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            rtcmBuffer = receiveRtcmFromGnss();
            if (!rtcmBuffer.isEmpty()) {
                Serial.println("[RTCM TASK] Da nhan du lieu RTCM tu mach RTK. So byte: " + String(rtcmBuffer.length()));
            } else {
                Serial.println("[RTCM TASK] Du lieu RTCM rong.");

            }
            Serial.println();
            xSemaphoreGive(rtcmBufferMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

__attribute__((noreturn))void taskLora(void* parameter) {
    // Sử dụng chung rtcmBuffer với taskRtcm, cần mutex
    char* rtcmCharArray = nullptr;
    while (true) {
        if (xSemaphoreTake(rtcmBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            #if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
            loopNTRIP(latestGGA);
            #else
            if (!rtcmBuffer.isEmpty()) {
                Serial.println("[LORA TASK] Chuan bi truyen du lieu RTCM qua LoRA...");
                rtcmCharArray = new char[rtcmBuffer.length() + 1];
                rtcmBuffer.toCharArray(rtcmCharArray, rtcmBuffer.length() + 1);
                loraSend(rtcmCharArray);
                Serial.printf("[LORA TASK] Da truyen du lieu RTCM qua LoRa. So byte: %d\n", strlen(rtcmCharArray));
                delete[] rtcmCharArray;
                rtcmCharArray = nullptr;
                latestRtcm = rtcmBuffer; // Cập nhật chuỗi RTCM mới nhất đã gửi đi
                rtcmBuffer = ""; // Dọn buffer sau khi gửi
                Serial.println("[LORA TASK] Da xoa du lieu RTCM trong buffer sau khi gui.");
            }
            else {
                Serial.println("[LORA TASK] Chua co du lieu RTCM de truyen qua LoRa.");
            }
            #endif
            xSemaphoreGive(rtcmBufferMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

__attribute__((noreturn)) void healthCheckTask(void* parameter) {
    // không sử dụng tài nguyên chung, không cần mutex
    String healthPayload = "";
    while (true) {
        healthPayload = formDeviceHealthString();
        Serial.print("[HEALTH CHECK] ");
        Serial.println(healthPayload);

        if (healthPayload.isEmpty())
        {
            vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
            continue;
        }

        #if PROGRAM_DEBUG
        Serial.println("[HEALTH CHECK] Kiem tra ket noi MQTT de gui thong tin suc khoe...");
        #endif
        if (!mqtt.connected()) {
            vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
            continue;
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

        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
    }
}

void loop() {
    if (!mqtt.connected()) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("[LOOP] MQTT mat ket noi, dang thu ket noi lai...");
        connectMQTT();
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // loop trống, tất cả logic đã được xử lý trong các task
}
