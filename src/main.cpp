#include "helper.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern PubSubClient mqtt;

String nmeaBuffer = "";
unsigned long lastHealthCheck = 0;
String latestGGA = "";

// Toạ độ của mục tiêu, tạm thời để giá trị mẫu
String targetGGA = "$GNGGA,045151.00,2104.44183385,N,10546.62503715,E,1,28,0.7,22.4381,M,-28.2448,M,,*6C";

// Semaphore
SemaphoreHandle_t mqttClientMutex = NULL;
SemaphoreHandle_t nmeaBufferMutex = NULL;

/* ===================== NGUYÊN MẪU HÀM ======================== */

void taskNmea(void* parameter);
void gnssParseTask(void* parameter);
void gnssPublishTask(void* parameter);
void healthCheckTask(void* parameter);

/* ==================SETUP VÀ LOOP======================== */

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=========================================");
    Serial.println("     ESP32 GNSS GATEWAY KHOI DONG        ");
    Serial.println("=========================================");

    // Khởi tạo giao tiếp với UM980
    Serial1.begin(GNSS_BAUD, SERIAL_8N1, RX_GNSS, TX_GNSS);
    bool networkConnected = false;

    connection_init:
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
        goto connection_init;
    }

    Serial.println("[SETUP] Khoi dong cac task...");

    Serial.println("[Setup] Tao mutex de dong bo hoa tai nguyen chung");
    mqttClientMutex = xSemaphoreCreateMutex();
    while (mqttClientMutex == NULL) {
        Serial.println("[ERROR] Tao mutex mqttClientMutex that bai! Dang thu lai...");
        mqttClientMutex = xSemaphoreCreateMutex();
    }
    nmeaBufferMutex = xSemaphoreCreateMutex();

    while (nmeaBufferMutex == NULL) {
        Serial.println("[ERROR] Tao mutex nmeaBufferMutex that bai! Dang thu lai...");
        nmeaBufferMutex = xSemaphoreCreateMutex();
    }

    Serial.println("[SETUP] Task NMEA: Doc du lieu NMEA tu UM980");
    xTaskCreatePinnedToCore(taskNmea, "NMEA Task", 4096, NULL, 2, NULL, 1);
    Serial.println("[SETUP] Da khoi dong Task NMEA!");

    Serial.println("[SETUP] Task GNSS Parse: Phan tich du lieu NMEA va chuan bi payload");
    xTaskCreatePinnedToCore(gnssParseTask, "GNSS Parse Task", 4096, NULL, 3, NULL, 0);
    Serial.println("[SETUP] Da khoi dong Task GNSS Parse!");

    Serial.println("[SETUP] Task GNSS Publish: Gui du lieu da duoc phan tich len MQTT");
    xTaskCreatePinnedToCore(gnssPublishTask, "GNSS Publish Task", 4096, NULL, 2, NULL, 0);
    Serial.println("[SETUP] Da khoi dong Task GNSS Publish!");

    Serial.println("[SETUP] Task Health: Gui thong tin suc khoe thiet bi len MQTT moi 30s");
    xTaskCreatePinnedToCore(healthCheckTask, "Health Task", 4096, NULL, 1, NULL, 1);
    Serial.println("[SETUP] Da khoi dong Task Health!");

    Serial.println("=========================================");
    Serial.println("        KHOI DONG HOAN TAT               ");
    Serial.println("=========================================\n");
}

/* ================= TRIỂN KHAI HÀM TASK ====================== */

void taskNmea(void* parameter) {
    // không sử dụng tài nguyên chung, không cần mutex
    while (true) {
        #if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
        loopNTRIP(latestGGA);
        #else
        loraWanMain();
        #endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void gnssParseTask(void* parameter) {
    // sử dụng nmeaBuffer làm tài nguyên chung với publishTask, cần mutex để tránh xung đột
    
    while (true) {
        int readError = -2;
        if (xSemaphoreTake(nmeaBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            if (Serial1.available()) {
                readError = roverReadCharFromRtk(nmeaBuffer);
            }
            xSemaphoreGive(nmeaBufferMutex);
        }
        if (readError == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            #if PROGRAM_DEBUG
            Serial.println("[GNSS PARSE] Se doc cac char tiep theo tu Serial1...");
            #endif
            continue;
        }
        #if PROGRAM_DEBUG
        switch (readError) {
            case 1:
                Serial.println("[GNSS PARSE] Da doc duoc mot dong NMEA hoan chinh: " + nmeaBuffer);
                break;
            case 2:
                Serial.print("[GNSS PARSE] Doc phai ky tu NULL.");
                if (nmeaBuffer.length() > 0) {
                    Serial.print(" Dong NMEA hien tai: " + nmeaBuffer);
                }
                Serial.println();
                break;
            case -2:
                Serial.println("[GNSS PARSE] Serial1 khong san sang de doc.");
                break;
            default:
                Serial.println("[GNSS PARSE] Loi khong xac dinh khi doc tu Serial1.");
        }
        #else
        if (readError == 1) {
            Serial.print("[GNSS PARSE] Da doc duoc mot dong NMEA hoan chinh.");
            Serial.println("[GNSS PARSE] Du lieu:" + nmeaBuffer);
        }
        #endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void gnssPublishTask(void* parameter) {
    String localBuf = "";
    String topic = "";
    // sử dụng nmeaBuffer làm tài nguyên chung với gnssParseTask
    // sử dụng chung đối tượng lớp PubSubClient là mqtt với healthCheckTask
    while (true) {
        if (xSemaphoreTake(nmeaBufferMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS))) {
            localBuf = nmeaBuffer;    // copy
            nmeaBuffer = "";         // clear shared buffer
            xSemaphoreGive(nmeaBufferMutex);
        }
        if (localBuf.length() > 0) {
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
            continue;
        }
        #if PROGRAM_DEBUG
        Serial.println("[GNSS PUBLISH] Khong co du lieu NMEA de gui, cho 2000ms...");
        #endif
        noParse:
        vTaskDelay(pdMS_TO_TICKS(2000)); // nothing to publish, yield longer
    }
}

void healthCheckTask(void* parameter) {
    String healthPayload = "";
    while (true) {
        healthPayload = formDeviceHealthString();
        Serial.print("[HEALTH CHECK] ");
        Serial.println(healthPayload);

        if (healthPayload.length() > 0) {
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
        }
        vTaskDelay(pdMS_TO_TICKS(HEALTH_INTERVAL));
    }
}

void loop() {
    if (!mqtt.connected()) {
        Serial.println("[LOOP] MQTT mat ket noi, dang thu ket noi lai...");
        connectMQTT();
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // loop trống, tất cả logic đã được xử lý trong các task
}
