#ifndef PROG_CONFIG_H
#define PROG_CONFIG_H

#include <cstdint>
#include "Top_Lvl_Config.h"

// ================= CẤU HÌNH CHÂN CẮM VÀ TỐC ĐỘ SERIAL =================
inline constexpr int RX_GNSS = 26; // Nối TXD (Hàng dưới) của UM980
inline constexpr int TX_GNSS = 27; // Nối RXD (Hàng dưới) của UM980
inline constexpr int GNSS_BAUD = 115200;

// ================= CẤU HÌNH KẾT NỐI =================
#if CONNECT_USING_WIFI
inline constexpr char WIFI_SSID[] = "AITOGY";
inline constexpr char WIFI_PASSWORD[] = "aitogy@aitogy";
#endif

#if CONNECT_USING_4G
inline constexpr uint8_t TX_TO_MODEM_RX = 16;
inline constexpr uint8_t RX_TO_MODEM_TX = 17;
inline constexpr uint8_t MODEM_DC_PIN = 15;
inline constexpr uint8_t MODEM_DTR_PIN = 4;

inline constexpr char APN[] = "v-internet"; // Thay bằng APN của nhà mạng bạn
inline constexpr char GPRS_USER[] = "";     // Thường để trống
inline constexpr char GPRS_PASS[] = "";
#endif

// ================= CẤU HÌNH NTRIP =================
inline constexpr int NTRIP_MODE = 3; // 1: Chỉ gửi GGA khi có yêu cầu; 2: Gửi GGA mỗi khi có thay đổi; 3: Gửi GGA đều đặn mỗi 10s

#if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
inline constexpr char NTRIP_CASTER_IP[] = "aitogy.com.vn";
inline constexpr uint16_t NTRIP_CASTER_PORT = 2101;
#elif NMEA_COMMUNICATION_PROTOCOL == LORA_SERIAL
#define NTRIP_LORA_SERIAL_CONFIG
#endif

inline constexpr char NTRIP_MOUNTPOINT[] = "/humga";
// Base64 của "trung:12345"
inline constexpr char NTRIP_AUTH[] = "dHJ1bmc6MTIzNDU=";

// ================ CẤU HÌNH MQTT =================

inline constexpr char MQTT_SERVER[] = "aitogy.asia";
inline constexpr uint16_t MQTT_PORT = 1883;
inline constexpr char MQTT_USER[] = "mqttUser";
inline constexpr char MQTT_PASS[] = "MqttPassword123$%^";

inline constexpr char TOPIC_PUB_DATA_GGA[] = "tdm2402/um980/data/gga";
inline constexpr char TOPIC_PUB_DATA_KSXT[] = "tdm2402/um980/data/ksxt";
inline constexpr char TOPIC_SUB_CMD[] = "tdm2402/um980/cmd";
inline constexpr char TOPIC_PUB_RAW_GGA[] = "tdm2402/um980/raw/gga";
inline constexpr char TOPIC_PUB_RAW_KSXT[] = "tdm2402/um980/raw/ksxt";
inline constexpr char TOPIC_PUB_HEALTH[] = "tdm2402/um980/health";

// ================= CẤU HÌNH KIỂM TRA SỨC KHOẺ =================
const unsigned long HEALTH_INTERVAL = 30000; // chu kỳ gửi thông tin sức khoẻ (ms)

#endif // PROG_CONFIG_H