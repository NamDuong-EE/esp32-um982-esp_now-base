#include "helper.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern PubSubClient mqtt;

String rtcmBuffer = "";
unsigned long lastHealthCheck = 0;
String latestGGA = "";

// Toạ độ của mục tiêu, tạm thời để giá trị mẫu
String targetGGA = "$GNGGA,045151.00,2104.44183385,N,10546.62503715,E,1,28,0.7,22.4381,M,-28.2448,M,,*6C";

// Semaphore
SemaphoreHandle_t mqttClientMutex = nullptr;
SemaphoreHandle_t nmeaBufferMutex = nullptr;

/* ===================== NGUYÊN MẪU HÀM ======================== */

__attribute__((noreturn)) void taskRtcm(void* parameter);
__attribute__((noreturn))void gnssParseTask(void* parameter);
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
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    delay(3000);

    // Debug marker: confirm Serial is working immediately after begin()
    Serial.println("[DEBUG] Serial initialized");
    Serial.println("\n=========================================");
    Serial.println("     ESP32 GNSS GATEWAY KHOI DONG        ");
    Serial.println("=========================================");

    // Khởi tạo giao tiếp với UM980
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    bool networkConnected = false;

    digitalWrite(LED_PIN, LOW);

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

    digitalWrite(LED_PIN, HIGH);

    delay(1000);

    digitalWrite(LED_PIN, LOW);

    Serial.println("[SETUP] Khoi dong cac task...");

    Serial.println("[Setup] Tao mutex de dong bo hoa tai nguyen chung");
    mqttClientMutex = xSemaphoreCreateMutex();
    while (mqttClientMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex mqttClientMutex that bai! Dang thu lai...");
        mqttClientMutex = xSemaphoreCreateMutex();
    }
    nmeaBufferMutex = xSemaphoreCreateMutex();

    while (nmeaBufferMutex == nullptr) {
        Serial.println("[ERROR] Tao mutex nmeaBufferMutex that bai! Dang thu lai...");
        nmeaBufferMutex = xSemaphoreCreateMutex();
    }

    digitalWrite(LED_PIN, HIGH);

    delay(1000);

    digitalWrite(LED_PIN, LOW);

    Serial.println("[SETUP] Task RTCM: Doc du lieu RTCM tu UM980");
    xTaskCreatePinnedToCore(taskRtcm, "RTCM Task", 4096, nullptr, 2, nullptr, 1);
    Serial.println("[SETUP] Da khoi dong Task RTCM!");

    // Serial.println("[SETUP] Task GNSS Parse: Phan tich du lieu RTCM va chuan bi payload");
    // xTaskCreatePinnedToCore(gnssParseTask, "GNSS Parse Task", 4096, nullptr, 3, nullptr, 0);
    // Serial.println("[SETUP] Da khoi dong Task GNSS Parse!");

    // Serial.println("[SETUP] Task GNSS Publish: Gui du lieu da duoc phan tich len MQTT");
    // xTaskCreatePinnedToCore(gnssPublishTask, "GNSS Publish Task", 4096, nullptr, 2, nullptr, 0);
    // Serial.println("[SETUP] Da khoi dong Task GNSS Publish!");

    // Serial.println("[SETUP] Task Health: Gui thong tin suc khoe thiet bi len MQTT moi 30s");
    // xTaskCreatePinnedToCore(healthCheckTask, "Health Task", 4096, nullptr, 1, nullptr, 1);
    // Serial.println("[SETUP] Da khoi dong Task Health!");

    Serial.println("=========================================");
    Serial.println("        KHOI DONG HOAN TAT               ");
    Serial.println("=========================================\n");

    digitalWrite(LED_PIN, HIGH);

    delay(1000);

    digitalWrite(LED_PIN, LOW);
}

/* ================= TRIỂN KHAI HÀM TASK ====================== */

__attribute__((noreturn))void taskRtcm(void* parameter) {
    // không sử dụng tài nguyên chung, không cần mutex
    while (true) {
        #if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
        loopNTRIP(latestGGA);
        #else
        rtcmBuffer = receiveRtcmFromGnss();
        if (!rtcmBuffer.isEmpty()) {
            Serial.println("[RTCM TASK] Da nhan du lieu RTCM, dang truyen qua LoRA...");
            loraWanMain();
        }
        #endif
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

__attribute__((noreturn)) void gnssParseTask(void* parameter) {
    // sử dụng nmeaBuffer làm tài nguyên chung với publishTask, cần mutex để tránh xung đột
    while (true) {
        if (xSemaphoreTake(nmeaBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            while (Serial1.available()) {
                auto c = (char)Serial1.read();
                rtcmBuffer += c;
                if (c == '\n' || c == '\0' || c == '$') {
                    break; // đọc đến cuối dòng, sẵn sàng cho việc phân tích
                }
            }
            xSemaphoreGive(nmeaBufferMutex);
            if (!rtcmBuffer.isEmpty()) {
                Serial.print("[GNSS PARSE] Doc duoc du lieu NMEA: ");
                Serial.println(rtcmBuffer);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

__attribute__((noreturn))void gnssPublishTask(void* parameter) {
    String localBuf = "";
    String topic = "";
    // sử dụng nmeaBuffer làm tài nguyên chung với gnssParseTask
    // sử dụng chung đối tượng lớp PubSubClient là mqtt với healthCheckTask
    while (true) {
        if (xSemaphoreTake(nmeaBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            localBuf = rtcmBuffer;    // copy
            rtcmBuffer = "";         // clear shared buffer
            xSemaphoreGive(nmeaBufferMutex);
        }

        if (!localBuf.isEmpty()) {
            #if PROGRAM_DEBUG
            Serial.println("[GNSS PUBLISH] Khong co du lieu NMEA de gui, cho 2000ms...");
            #endif
            vTaskDelay(pdMS_TO_TICKS(500)); // nothing to publish, yield longer
            continue;
        }

        if (xSemaphoreTake(mqttClientMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            #if PROGRAM_DEBUG
            Serial.println("[GNSS PUBLISH] Kiem tra ket noi MQTT de gui du lieu NMEA...");
            #endif
            if (mqtt.connected()) {
                #if PROGRAM_DEBUG
                Serial.println("[GNSS PUBLISH] MQTT dang ket noi, dang kich hoat loop...");
                #endif
                mqtt.loop();
                #if PROGRAM_DEBUG
                Serial.println("[GNSS PUBLISH] Dang gui du lieu NMEA len MQTT...");
                #endif
                publishGGA(localBuf); // publishGGA accepts String&
            }
            xSemaphoreGive(mqttClientMutex);
        }
        localBuf = "";
        vTaskDelay(pdMS_TO_TICKS(100));        
    }
}

__attribute__((noreturn)) void healthCheckTask(void* parameter) {
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

        if (xSemaphoreTake(mqttClientMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)))
        {
            #if PROGRAM_DEBUG
            Serial.println("[HEALTH CHECK] Kiem tra ket noi MQTT de gui thong tin suc khoe...");
            #endif
            if (mqtt.connected()) {
                #if PROGRAM_DEBUG
                Serial.println("[HEALTH CHECK] MQTT dang ket noi, dang kich hoat loop...");
                #endif
                mqtt.loop();
                #if PROGRAM_DEBUG
                Serial.println("[HEALTH CHECK] Dang gui thong tin suc khoe len MQTT...");
                #endif
                publishHealth(healthPayload);
            }
            xSemaphoreGive(mqttClientMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
    }
}

void loop() {
    if (!mqtt.connected()) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("[LOOP] MQTT mat ket noi, dang thu ket noi lai...");
        connectMQTT();
        if (mqtt.connected()) {
            digitalWrite(LED_PIN, LOW);
            Serial.println("[LOOP] Ket noi MQTT thanh cong!");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // loop trống, tất cả logic đã được xử lý trong các task
}
