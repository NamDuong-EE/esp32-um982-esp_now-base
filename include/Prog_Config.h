#ifndef PROG_CONFIG_H
#define PROG_CONFIG_H

#include <cstdint>
#include "Top_Lvl_Config.h"

// ================= CẤU HÌNH CHÂN CẮM VÀ TỐC ĐỘ SERIAL =================
#if BOARD_HELTEC
inline constexpr int RX_GNSS = 41; // Nối TXD (Hàng dưới) của UM980
inline constexpr int TX_GNSS = 42; // Nối RXD (Hàng dưới) của UM980
inline constexpr int LED_PIN = 35;
#elif BOARD_2A53N
inline constexpr int RX_GNSS = 22; // Nối TXD (Hàng dưới) của UM980
inline constexpr int TX_GNSS = 23; // Nối RXD (Hàng dưới) của UM980
inline constexpr int LED_PIN = 2;
#elif BOARD_TDM_240X
inline constexpr int RX_GNSS = 26; // Nối TXD (Hàng dưới) của UM980
inline constexpr int TX_GNSS = 27; // Nối RXD (Hàng dưới) của UM980
inline constexpr int LED_PIN = 2;
#endif
inline constexpr int GNSS_BAUD = 115200;

// ================= CẤU HÌNH CÁC TASK =================
inline constexpr int MUTEX_TIMEOUT_MS = 1500; // Thời gian tối đa để chờ mutex (ms)

// ================= CẤU HÌNH KẾT NỐI =================
#if CONNECT_USING_WIFI
inline constexpr char WIFI_SSID[] = "AITOGY";
inline constexpr char WIFI_PASSWORD[] = "aitogy@aitogy";
#endif

#if CONNECT_USING_4G
inline constexpr uint8_t TX_TO_MODEM_RX = 17;
inline constexpr uint8_t RX_TO_MODEM_TX = 16;
inline constexpr uint8_t MODEM_DC_PIN = 15;
inline constexpr uint8_t MODEM_DTR_PIN = 4;

inline constexpr char APN[] = "v-internet"; // Thay bằng APN của nhà mạng bạn
inline constexpr char GPRS_USER[] = "";     // Thường để trống
inline constexpr char GPRS_PASS[] = "";
#endif

// ================ CẤU HÌNH LORA =================
#if RTCM_COMMUNICATION_PROTOCOL == LORA_SERIAL
inline constexpr int RF_FREQUENCY = 433000000; // Hz
inline constexpr int TX_OUTPUT_POWER = 24;        // dBm
inline constexpr int LORA_BANDWIDTH = 0;         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
inline constexpr int LORA_SPREADING_FACTOR = 11;         // [SF7..SF12]
inline constexpr int LORA_CODINGRATE = 1;         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
inline constexpr int LORA_PREAMBLE_LENGTH = 8;         // Same for Tx and Rx
inline constexpr int LORA_SYMBOL_TIMEOUT = 0;         // Symbols
inline constexpr bool LORA_FIX_LENGTH_PAYLOAD_ON = false;
inline constexpr bool LORA_IQ_INVERSION_ON = false;
inline constexpr int LORA_TX_TIMEOUT = 3000;         // ms
#endif

// ================= CẤU HÌNH NTRIP =================
inline constexpr int NTRIP_MODE = 1; // 1: Chỉ gửi GGA khi có yêu cầu; 2: Gửi GGA mỗi khi có thay đổi; 3: Gửi GGA đều đặn mỗi 10s

#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP
inline constexpr char NTRIP_CASTER_IP[] = "aitogy.com.vn";
inline constexpr uint16_t NTRIP_CASTER_PORT = 2101;
#endif

#ifdef PROGRAM_TEST
inline constexpr char NTRIP_MOUNTPOINT[] = "/test";
inline constexpr char NTRIP_AUTH[] = "YWl0b2d5OmFpdG9neQ==";
inline constexpr char NTRIP_AUTH_BASE_STATION[] = "12345";
#else
inline constexpr char NTRIP_MOUNTPOINT[] = "/humga";
// Base64 của "trung:12345"
inline constexpr char NTRIP_AUTH[] = "dHJ1bmc6MTIzNDU=";
inline constexpr char NTRIP_AUTH_BASE_STATION[] = "12345";
#endif

// ================ CẤU HÌNH MQTT =================

inline constexpr char MQTT_SERVER[] = "aitogy.asia";
inline constexpr uint16_t MQTT_PORT = 1883;
inline constexpr char MQTT_USER[] = "mqttUser";
inline constexpr char MQTT_PASS[] = "MqttPassword123$%^";

inline constexpr char TOPIC_PUB_DATA_GGA[] = "tdm2402/um980/data/gga";
inline constexpr char TOPIC_PUB_DATA_KSXT[] = "tdm2402/um980/data/ksxt";
inline constexpr char TOPIC_SUB_CMD[] = "tdm2402/um980/cmd";
inline constexpr char TOPIC_PUB_RAW_RTCM[] = "tdm2402/um980_base/raw/last_rtcm";
inline constexpr char TOPIC_PUB_HEALTH[] = "tdm2402/um980/health";

// ================= CẤU HÌNH KIỂM TRA SỨC KHOẺ =================
const unsigned long HEALTH_INTERVAL = 30000; // chu kỳ gửi thông tin sức khoẻ (ms)

#endif // PROG_CONFIG_H