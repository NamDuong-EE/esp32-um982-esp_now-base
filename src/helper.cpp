#include "helper.h"

extern String latestGGA;

gga_data_t ggaData;
gga_data_t targetGgaData;
ksxt_data_t ksxtData;


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
            if (bool parseOk = parseKSXT_toStruct(nmeaBuffer, ksxtData))
            {
                jsonPayload = parseKSXT_toJSON(ksxtData);
            }
            publishData(jsonPayload, false);
        }
        else if (nmeaBuffer.startsWith("$GNGGA"))
        {
            publishRaw(nmeaBuffer, true);
            if (bool parseOk = parseGGA_toStruct(nmeaBuffer, ggaData))
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
    std::string healthPayload = "{";
    healthPayload += "\"uptime_s\":" + std::to_string(uptime_s);
    healthPayload += ",\"free_heap_bytes\":" + std::to_string(freeHeap);
    healthPayload += R"(,"connected_via":")" + std::string(connected_via.c_str()) + "\"";
    healthPayload += ",\"rssi_dbm\":" + std::to_string(rssi);
    healthPayload += ",\"mqtt_ok\":" + std::string(mqttOk ? "true" : "false");
    healthPayload += ",\"ntrip_ok\":" + std::string(ntripOk ? "true" : "false");
    healthPayload += ",\"gnss_data_ok\":" + std::string(gnssOk ? "true" : "false");
    healthPayload += "}";
    /*Xóa tọa độ sau khi đã dùng để đánh giá sức khoẻ, nếu còn giữ, 
    trong trường hợp không có dữ liệu mới, sẽ luôn báo GNSS OK dù 
    thực tế đã mất tín hiệu. Việc này giúp phản ánh tình trạng thực tế hơn.*/ 
    latestGGA = "";
    // 3. Trả về payload để có thể log hoặc dùng cho mục đích khác nếu cần
    return String(healthPayload.c_str());
}