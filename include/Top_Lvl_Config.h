/*
    Cấu hình cho thiết bị ĐỂ SỬ DỤNG TẠI MAIN.CPP
*/
#ifndef TOP_LVL_CONFIG_H
#define TOP_LVL_CONFIG_H

// ================= CẤU HÌNH KHỞI TẠO =================
#ifndef WIFI_LORA_32_V4
#define WIFI_LORA_32_V4
#endif

#ifndef LORAWAN_DEBUG_LEVEL
#define LORAWAN_DEBUG_LEVEL 0
#endif

#ifndef CONNECT_USING_WIFI
#define CONNECT_USING_WIFI 0
#endif

#ifndef CONNECT_USING_4G
#define CONNECT_USING_4G 1
#endif

#define TCP_IP 0
#define LORA_SERIAL 1

#ifndef NMEA_COMMUNICATION_PROTOCOL
#define NMEA_COMMUNICATION_PROTOCOL TCP_IP // Chọn giữa TCP_IP hoặc LORA_SERIAL
#endif

// ==== CHỌN 1 TRONG 2 PHƯƠNG THỨC KẾT NỐI (KHÔNG ĐƯỢC CHỌN CẢ HAI) ====
#if (!CONNECT_USING_WIFI && !CONNECT_USING_4G)
    #warning "Không chọn phương thức kết nối nào! Sẽ sử dụng WiFi."
    #undef CONNECT_USING_WIFI
    #define CONNECT_USING_WIFI 1
#endif

#if ((CONNECT_USING_WIFI + CONNECT_USING_4G) > 1)
    #error "Chỉ được chọn một phương thức kết nối! Vui lòng chỉnh sửa DEVICE_CONFIG.h"
#endif

#endif