#ifndef PROG_CONFIG_H
#define PROG_CONFIG_H

#include <cstddef>
#include <cstdint>
#include "Top_Lvl_Config.h"

#if __has_include("Network_Secrets.h")
#include "Network_Secrets.h"
#endif

#ifndef BASE_WIFI_SSID
#define BASE_WIFI_SSID ""
#endif
#ifndef BASE_WIFI_PASSWORD
#define BASE_WIFI_PASSWORD ""
#endif
#ifndef BASE_MQTT_HOST
#define BASE_MQTT_HOST ""
#endif
#ifndef BASE_MQTT_PORT
#define BASE_MQTT_PORT 1883
#endif
#ifndef BASE_MQTT_USER
#define BASE_MQTT_USER ""
#endif
#ifndef BASE_MQTT_PASSWORD
#define BASE_MQTT_PASSWORD ""
#endif
#ifndef BASE_MODEM_APN
#define BASE_MODEM_APN ""
#endif
#ifndef BASE_MODEM_GPRS_USER
#define BASE_MODEM_GPRS_USER ""
#endif
#ifndef BASE_MODEM_GPRS_PASSWORD
#define BASE_MODEM_GPRS_PASSWORD ""
#endif

// UART2 from UM980/UM982 Base to ESP32.
inline constexpr int LED_PIN = 2;
inline constexpr char GNSS_UART_PORT_NAME[] = "UM980 UART2 TX2/RX2";
inline constexpr int RX_GNSS = 26; // UM980/UM982 TX2 -> ESP32 RX GPIO26
inline constexpr int TX_GNSS = 27; // UM980/UM982 RX2 <- ESP32 TX GPIO27
inline constexpr uint32_t GNSS_BAUD = 115200;
inline constexpr size_t GNSS_RX_BUFFER_SIZE = 4096;

// Task and health logging.
inline constexpr int MUTEX_TIMEOUT_MS = 1500;
inline constexpr unsigned long HEALTH_INTERVAL = 30000;
inline constexpr bool DEBUG_GNSS_UART_RAW_DUMP = false;
inline constexpr bool DEBUG_RTCM_HEX_DUMP = false;
inline constexpr bool DEBUG_RTCM_FRAME_LOG = false;
inline constexpr uint8_t DEBUG_RTCM_HEX_BYTES_PER_LINE = 16;
inline constexpr size_t RTCM_FRAME_QUEUE_LENGTH = 8;
inline constexpr uint32_t RTCM_MAX_QUEUE_AGE_MS = 1000;

// Internet and MQTT. Wi-Fi is used during development; 4G is selected by the
// esp32u_base_4g_mqtt PlatformIO environment when the modem board is ready.
inline constexpr char NETWORK_WIFI_SSID[] = BASE_WIFI_SSID;
inline constexpr char NETWORK_WIFI_PASSWORD[] = BASE_WIFI_PASSWORD;
inline constexpr char MQTT_HOST[] = BASE_MQTT_HOST;
inline constexpr uint16_t MQTT_PORT = BASE_MQTT_PORT;
inline constexpr char MQTT_USER[] = BASE_MQTT_USER;
inline constexpr char MQTT_PASSWORD[] = BASE_MQTT_PASSWORD;
inline constexpr char MQTT_TOPIC_STATUS[] = "aitogy/base/test/status";
inline constexpr char MQTT_TOPIC_COMMAND[] = "aitogy/base/test/command";
inline constexpr char MQTT_TOPIC_COMMAND_RESULT[] = "aitogy/base/test/command-result";
inline constexpr char MQTT_TOPIC_ROVER_ECEF_PREFIX[] = "aitogy/base/rovers";
inline constexpr uint32_t NETWORK_RECONNECT_INTERVAL_MS = 10000;
inline constexpr uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000;
inline constexpr uint32_t MQTT_ECEF_RETRY_INTERVAL_MS = 1000;
inline constexpr uint16_t MQTT_KEEPALIVE_SECONDS = 30;
inline constexpr uint16_t MQTT_SOCKET_TIMEOUT_SECONDS = 2;
inline constexpr uint16_t MQTT_BUFFER_SIZE = 1024;

// SIM7600 defaults for the future 4G board. Confirm these pins against the PCB.
inline constexpr int MODEM_RX_PIN = 16; // ESP32 RX <- modem TX.
inline constexpr int MODEM_TX_PIN = 17; // ESP32 TX -> modem RX.
// Mirrors the power-on sequence used by Long's SIM7600 implementation:
// drive the modem control input active for 1 second, release it, then wait
// for the modem to finish booting before issuing AT commands.
inline constexpr bool MODEM_POWER_CONTROL_ENABLED = true;
inline constexpr int MODEM_POWER_CONTROL_PIN = 15;
inline constexpr bool MODEM_POWER_CONTROL_ACTIVE_HIGH = true;
inline constexpr uint32_t MODEM_POWER_PULSE_MS = 1000;
inline constexpr uint32_t MODEM_BOOT_WAIT_MS = 8000;
inline constexpr uint32_t MODEM_BAUD = 115200;
inline constexpr uint32_t MODEM_NETWORK_TIMEOUT_MS = 10000;
inline constexpr char MODEM_APN[] = BASE_MODEM_APN;
inline constexpr char MODEM_GPRS_USER[] = BASE_MODEM_GPRS_USER;
inline constexpr char MODEM_GPRS_PASSWORD[] = BASE_MODEM_GPRS_PASSWORD;

// Wired UART bridge between the Wi-Fi/GNSS Base and the dedicated 4G gateway.
// Wi-Fi/GNSS board: RX16/TX17. 4G board: RX26/TX27.
inline constexpr int UART_GATEWAY_CLIENT_RX_PIN = 16;
inline constexpr int UART_GATEWAY_CLIENT_TX_PIN = 17;
inline constexpr int UART_GATEWAY_MODEM_BOARD_RX_PIN = 26;
inline constexpr int UART_GATEWAY_MODEM_BOARD_TX_PIN = 27;
inline constexpr uint32_t UART_GATEWAY_BAUD = 115200;

// ESP-NOW Base field-mode configuration.
inline constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;
inline constexpr bool ESPNOW_USE_LR_250KBPS = true;

inline constexpr bool ESPNOW_ENCRYPTION_ENABLED = false;
inline constexpr uint8_t ESPNOW_PMK[16] = {0};
inline constexpr uint8_t ESPNOW_LMK[16] = {0};

inline constexpr uint32_t ESPNOW_SEND_TIMEOUT_MS = 250;
inline constexpr uint8_t ESPNOW_SEND_RETRY_COUNT = 2;
inline constexpr uint32_t ESPNOW_FRAME_SEND_DEADLINE_MS = 1000;
inline constexpr uint32_t ESPNOW_FRAME_ACK_TIMEOUT_MS = 300;
inline constexpr uint8_t ESPNOW_FRAME_RETRY_COUNT = 1;
inline constexpr uint8_t ESPNOW_PEER_FAILURES_BEFORE_COOLDOWN = 2;
inline constexpr uint32_t ESPNOW_PEER_FAILURE_COOLDOWN_MS = 3000;
inline constexpr size_t ESPNOW_GNSS_COMMAND_QUEUE_LENGTH = 8;
inline constexpr size_t ESPNOW_GNSS_COMMAND_RESULT_QUEUE_LENGTH = 4;
inline constexpr uint32_t ESPNOW_GNSS_COMMAND_RESULT_TIMEOUT_MS = 10000;
inline constexpr uint8_t ESPNOW_GNSS_COMMAND_SEND_RETRY_COUNT = 1;
inline constexpr uint32_t TEMP_BASE_FIXED_WAIT_DEFAULT_SECONDS = 120;
inline constexpr uint32_t TEMP_BASE_FIXED_WAIT_MAX_SECONDS = 3600;
inline constexpr uint32_t TEMP_BASE_FIXED_ECEF_MAX_AGE_MS = 3000;
inline constexpr size_t TEMP_RTCM_RX_QUEUE_LENGTH = 16;
inline constexpr size_t TEMP_RTCM_FRAME_QUEUE_LENGTH = 3;
inline constexpr uint32_t TEMP_RTCM_RX_TASK_STACK_BYTES = 8192;
inline constexpr uint32_t TEMP_RTCM_REASSEMBLY_TIMEOUT_MS = 1500;
inline constexpr uint32_t TEMP_RTCM_SOURCE_TIMEOUT_MS = 3500;
inline constexpr uint32_t TEMP_RTCM_PREPARING_TIMEOUT_MS = 15000;
inline constexpr uint32_t TEMP_RTCM_FIXED_GUARD_MS = 3000;
inline constexpr uint8_t TEMP_RTCM_READY_CYCLES = 2;
inline constexpr uint32_t TEMP_RESET_GATE_TIMEOUT_MS = 45000;
inline constexpr uint8_t BASE_ECEF_CORRECTION_WARMUP_SAMPLES = 3;
inline constexpr int64_t BASE_ECEF_CORRECTION_MAX_ABS_SCALED = 50000;
inline constexpr int64_t BASE_ECEF_CORRECTION_MAX_STEP_SCALED = 5000;
inline constexpr uint32_t BASE_ECEF_CORRECTION_MAX_AGE_MS = 3000;
inline constexpr uint32_t BASE_ECEF_CORRECTION_MAX_TIME_DELTA_MS = 1500;
inline constexpr uint32_t BASE_GNSS_ROLE_COMMAND_DELAY_MS = 1000;
inline constexpr uint32_t BASE_GNSS_OUTPUT_COMMAND_DELAY_MS = 100;
inline constexpr size_t GNSS_TX_BUFFER_SIZE = 2048;

// ESP-NOW pairing. Hold the physical pairing button on Base and one Rover to
// discover MAC addresses by broadcast, then switch back to unicast runtime.
inline constexpr bool ESPNOW_PAIRING_ENABLED = true;
inline constexpr int PAIRING_BUTTON_PIN = 0; // BOOT on many ESP32 boards; change for PCB.
inline constexpr bool PAIRING_BUTTON_ACTIVE_LOW = true;
inline constexpr uint32_t PAIRING_BUTTON_HOLD_MS = 1500;
inline constexpr uint32_t PAIRING_WINDOW_MS = 60000;
inline constexpr uint32_t PAIR_DISCOVERY_INTERVAL_MS = 500;
inline constexpr uint8_t ESPNOW_MAX_PAIRED_ROVERS = 5;
inline constexpr uint8_t ESPNOW_MAX_CHILDREN_PER_RELAY = 5;
inline constexpr uint8_t ESPNOW_MAX_ECEF_SOURCES =
    ESPNOW_MAX_PAIRED_ROVERS * (1 + ESPNOW_MAX_CHILDREN_PER_RELAY);
inline constexpr uint32_t ESPNOW_NETWORK_ID = 0xA1700001UL;
inline constexpr uint8_t ESPNOW_PAIRING_KEY[16] = {
    0x41, 0x49, 0x54, 0x4F, 0x47, 0x59, 0x5F, 0x50,
    0x41, 0x49, 0x52, 0x5F, 0x56, 0x30, 0x30, 0x31,
};
inline constexpr char ESPNOW_NVS_NAMESPACE[] = "espnow";
inline constexpr char ESPNOW_NVS_ROVER_COUNT_KEY[] = "rover_count";
inline constexpr char ESPNOW_NVS_ROVER_MAC_PREFIX[] = "rover";

#endif // PROG_CONFIG_H
