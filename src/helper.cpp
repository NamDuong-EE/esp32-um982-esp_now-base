#include "helper.h"

extern String latestGGA;

gga_data_struct ggaData;
gga_data_struct targetGgaData;
ksxt_data_struct ksxtData;

int roverReadCharFromRtk(String &nmeaBuffer)
{

parse_start:
    char c = Serial1.read();
    nmeaBuffer += c;
    if (c == '\n')
    {
        return 1;
    }
    if (c == NULL)
    {
        return 2;
    }
    return 0;
}

int publishGGA(String &nmeaBuffer)
{
    nmeaBuffer.trim();

    // Bắt dòng tọa độ
    if (nmeaBuffer.startsWith("$GNGGA") || nmeaBuffer.startsWith("$GPGGA") || nmeaBuffer.startsWith("$KSXT"))
    {
        // Cập nhật tọa độ mới nhất để NTRIP dùng xác thực (Mode 3)
        latestGGA = nmeaBuffer;

        // Đẩy lên MQTT
        String jsonPayload = "";
        if (nmeaBuffer.startsWith("$KSXT"))
        {
            bool parseOk = parseKSXT_toStruct(nmeaBuffer, ksxtData);
            if (parseOk)
            {
                jsonPayload = parseKSXT_toJSON(ksxtData);
            }
            publishData(jsonPayload, false);
        }
        else if (nmeaBuffer.startsWith("$GNGGA"))
        {
            publishRaw(nmeaBuffer, true);
            bool parseOk = parseGGA_toStruct(nmeaBuffer, ggaData);
            if (parseOk)
            {
                jsonPayload = parseGGA_toJSON(ggaData);
            }
            publishData(jsonPayload, true);
        }
        nmeaBuffer = "";
        return 0;
    }
    // Bắt dòng phản hồi lệnh
    else if (nmeaBuffer.startsWith("#"))
    {
        Serial.print("[UM980 RESPONSE] ");
        Serial.println(nmeaBuffer);
        nmeaBuffer = "";
        return -1;
    }
    nmeaBuffer = "";
    return -1;
}

String formDeviceHealthString()
{
    // 1. Lấy các thông số hệ thống
    unsigned long uptime_s = millis() / 1000;
    uint32_t freeHeap = ESP.getFreeHeap();

#if CONNECT_USING_WIFI
    int32_t rssi = WiFi.RSSI();
    String connected_via = "WiFi";
#endif
#if CONNECT_USING_4G
    int32_t rssi = modem.getSignalQuality();
    String connected_via = "GSM";
#endif

    bool mqttOk = isMqttConnected();
#if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
    bool ntripOk = isNtripConnected();
#else
    // Nếu dùng LoRa thì không có NTRIP qua TCP/IP, sẽ có cách khác để kiểm tra. Hiện chưa có mã nguồn cho LoRa nên tạm thời để false.
    bool ntripOk = false;
#endif
    bool gnssOk = (latestGGA.length() > 10); // Nếu có chuỗi NMEA hợp lệ

    // 2. Đóng gói thành JSON
    char healthPayload[256];
    snprintf(healthPayload, sizeof(healthPayload),
             "{\"uptime_s\":%lu,\"free_heap_bytes\":%u,\"connected_via\":\"%s\",\"rssi_dbm\":%d,\"mqtt_ok\":%s,\"ntrip_ok\":%s,\"gnss_data_ok\":%s}",
             uptime_s, freeHeap, connected_via, rssi,
             mqttOk ? "true" : "false",
             ntripOk ? "true" : "false",
             gnssOk ? "true" : "false");
    /*Xóa tọa độ sau khi đã dùng để đánh giá sức khoẻ, nếu còn giữ, 
    trong trường hợp không có dữ liệu mới, sẽ luôn báo GNSS OK dù 
    thực tế đã mất tín hiệu. Việc này giúp phản ánh tình trạng thực tế hơn.*/ 
    latestGGA = "";
    // 3. Trả về payload để có thể log hoặc dùng cho mục đích khác nếu cần
    return String(healthPayload);
}