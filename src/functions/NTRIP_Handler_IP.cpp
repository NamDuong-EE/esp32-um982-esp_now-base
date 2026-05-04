#if NMEA_COMMUNICATION_PROTOCOL == TCP_IP || defined(UNIT_TEST)
#define NTRIP_HANDLER_IP_CODE

#include "functions/NTRIP_Handler_IP.h"
#include "Prog_Config.h"

// ================= BIẾN TOÀN CỤC =================
static bool isIcyOk = false;
static unsigned long lastReconnect = 0;
static bool isNmeaSent = false; // Cờ kiểm tra xem đã gửi NMEA xác thực chưa

// ================= CÁC ĐỐI TƯỢNG KẾT NỐI =================
#if CONNECT_USING_WIFI
#include "hardware/Wifi_handler.h"
WiFiClient ntripClient;
#endif
#if CONNECT_USING_4G
#include "hardware/Sim_handler.h"
extern TinyGsm modem;
TinyGsmClient ntripClient(modem);
#endif

// ================= ĐỊNH NGHĨA HÀM =================

int setupNTRIP() {
  isIcyOk = false;
  isNmeaSent = false;
  return 0;
}

bool isNtripConnected() {
  return isIcyOk; // Trả về true nếu đã xác thực thành công với Caster
}

int connectNTRIP() {
  Serial.print("\n[NTRIP] Dang mo TCP den: ");
  Serial.println(NTRIP_CASTER_IP);

  if (ntripClient.connect(NTRIP_CASTER_IP, NTRIP_CASTER_PORT)) {
    Serial.println("[NTRIP] Da ket noi TCP! Dang gui Header...");
    
    String request = "";
    if (NTRIP_MODE == 1) {
      request = "SOURCE " + String(NTRIP_AUTH) + " " + String(NTRIP_MOUNTPOINT) + "\r\n";
      request += "Source-Agent: NTRIP NtripServerCMD/1.0\r\n\r\n";
    } else {
      request = "GET " + String(NTRIP_MOUNTPOINT) + " HTTP/1.0\r\n";
      request += "User-Agent: NTRIP AitogyNTRIPClient/20131124\r\n";
      request += "Authorization: Basic " + String(NTRIP_AUTH) + "\r\n";
      request += "Accept: */*\r\n";
      request += "Connection: close\r\n\r\n";
    }
    ntripClient.print(request);
    
    // Đợi server trả lời ICY OK
    unsigned long timeout = millis();
    while (ntripClient.connected() && millis() - timeout < 5000) {
      if (ntripClient.available()) {
        String response = ntripClient.readStringUntil('\n');
        response.trim();
        Serial.print("[CASTER RESP]: ");
        Serial.println(response);
        
        if (response.indexOf("ICY 200 OK") != -1 || response.indexOf("ICY OK") != -1) {
          isIcyOk = true;
          isNmeaSent = false;
          Serial.println("[NTRIP] Xac thuc THANH CONG (ICY OK)!");
          break;
        }
      }
    }
  } else {
    Serial.println("[NTRIP] Loi ket noi TCP socket!");
    return -1;
  }
  return 0;
}

int loopNTRIP(String currentGGA) {
  int returnCode = NTRIP_MODE; // returnCode = NTRIP_MODE + ntripClient.available() * 4
#ifdef UNIT_TEST
  auto& ntripOutput = Serial;
#else
  auto& ntripOutput = Serial1;
#endif
  // 1. Quản lý mất kết nối
  if (!ntripClient.connected()) {
    isIcyOk = false;
    if (millis() - lastReconnect > 7000) { // Thử lại sau 7 giây
      connectNTRIP();
      lastReconnect = millis();
    }
    return 500; // Chưa kết nối, sẽ quay lại ở vòng tiếp theo của loop()
  }

  // 2. Xử lý sau khi kết nối thành công
  if (isIcyOk) {
    
    // Nếu Mode 3 yêu cầu NMEA mà chưa gửi, tiến hành gửi ngay
    // Xóa cái điều kiện !isNmeaSent đi, dùng timer để gửi định kỳ
    static unsigned long lastGgaSentTime = 0;
    
    // Nếu Mode 3 yêu cầu NMEA: Gửi đều đặn mỗi 10 giây
    if (NTRIP_MODE == 3) {
      if (currentGGA.length() > 10 && (millis() - lastGgaSentTime > 10000)) {
        Serial.print("[NTRIP UPLINK] Cap nhat vi tri cho Server: ");
        Serial.println(currentGGA);
        
        ntripClient.print(currentGGA + "\r\n");
        lastGgaSentTime = millis();
      }
    }
    
    // 3. Nhận dữ liệu RTCM từ Caster -> Đẩy thẳng xuống UM980
    if (ntripClient.available()) {
      uint8_t buffer[128];
      int bytesRead = ntripClient.read(buffer, sizeof(buffer));
      
      // Bơm trực tiếp byte thô vào cổng Serial1 cho module định vị
      ntripOutput.write(buffer, bytesRead); 
      
      // In dấu chấm nhỏ để biết đang nhận RTCM (bỏ comment nếu cần debug)
      // Serial.print("*"); 

      returnCode += 4;
    }
  }
  return returnCode;
}
#endif // NTRIP_HANDLER_IP_CODE
