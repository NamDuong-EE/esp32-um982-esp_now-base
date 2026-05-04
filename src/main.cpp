#include "helper.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
String nmeaBuffer = "";
unsigned long lastHealthCheck = 0;
String latestGGA = "";

// Toạ độ của mục tiêu, tạm thời để giá trị mẫu
String targetGGA = "$GNGGA,045151.00,2104.44183385,N,10546.62503715,E,1,28,0.7,22.4381,M,-28.2448,M,,*6C";

/* ===================== NGUYÊN MẪU HÀM ======================== */

void taskNmea(void* parameter);
void gnssParseTask(void* parameter);
void gnssPublishTask(void* parameter);

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
    
    xTaskCreatePinnedToCore(taskNmea, "NMEA Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(gnssParseTask, "GNSS Parse Task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(gnssPublishTask, "GNSS Publish Task", 4096, NULL, 1, NULL, 0);

    Serial.println("=========================================");
    Serial.println("        KHOI DONG HOAN TAT               ");
    Serial.println("=========================================\n");
}

/* ================= TRIỂN KHAI HÀM TASK ====================== */

void taskNmea(void* parameter) {
    while (true) {
        #if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
        loopNTRIP(latestGGA);
        #else
        loraWanMain();
        #endif
    }
}

void gnssParseTask(void* parameter) {
    while (true) {
        if (Serial1.available()) {
            gnssRoverParse(nmeaBuffer);
        }
    }
}

void gnssPublishTask(void* parameter) {
    while (true) {
        loopMQTT();
        publishGGA(nmeaBuffer);
    }
}

void loop()
{
    loopMQTT();
    sendDeviceHealth();
    delay(HEALTH_INTERVAL);
}
