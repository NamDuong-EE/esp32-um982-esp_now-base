#ifndef PROG_CONFIG_H
#define PROG_CONFIG_H

#include <cstddef>
#include <cstdint>
#include "Top_Lvl_Config.h"

// UART2 from UM980/UM982 Base to ESP32.
inline constexpr int LED_PIN = 2;
inline constexpr char GNSS_UART_PORT_NAME[] = "UM980 UART2 TX2/RX2";
inline constexpr int RX_GNSS = 16; // UM980/UM982 TX2 -> ESP32 RX GPIO16
inline constexpr int TX_GNSS = 17; // UM980/UM982 RX2 <- ESP32 TX GPIO17
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

// ESP-NOW Base field-mode configuration.
inline constexpr uint8_t ESPNOW_WIFI_CHANNEL = 6;
inline constexpr bool ESPNOW_USE_LR_250KBPS = true;

// Rover STA MAC: 58:2A:BD:71:E4:F0.
inline constexpr uint8_t ESPNOW_ROVER_MAC[6] = {
    0x58, 0x2A, 0xBD, 0x71, 0xE4, 0xF0
};

inline constexpr bool ESPNOW_ENCRYPTION_ENABLED = false;
inline constexpr uint8_t ESPNOW_PMK[16] = {0};
inline constexpr uint8_t ESPNOW_LMK[16] = {0};

inline constexpr uint32_t ESPNOW_SEND_TIMEOUT_MS = 250;
inline constexpr uint8_t ESPNOW_SEND_RETRY_COUNT = 2;
inline constexpr uint32_t ESPNOW_FRAME_SEND_DEADLINE_MS = 1000;
inline constexpr uint32_t ESPNOW_FRAME_ACK_TIMEOUT_MS = 300;
inline constexpr uint8_t ESPNOW_FRAME_RETRY_COUNT = 1;

#endif // PROG_CONFIG_H
