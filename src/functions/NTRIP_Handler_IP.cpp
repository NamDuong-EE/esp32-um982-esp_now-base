#if RTCM_COMMUNICATION_PROTOCOL == TCP_IP || defined(UNIT_TEST)
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

extern String latestRtcm;
extern SemaphoreHandle_t rtcmBufferMutex;
extern SemaphoreHandle_t tcpStreamMutex;

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
    delay(1000); // Đợi một chút để đảm bảo kết nối ổn định
    Serial.println("[NTRIP] Da ket noi TCP! Dang gui Header...");
    
    sendRequest:
    String request = "SOURCE " + String(NTRIP_AUTH_BASE_STATION) + " " + String(NTRIP_MOUNTPOINT) + " \r\n"
          + "Source-Agent: NTRIP NtripServerCMD/1.0\r\n\r\n";
    ntripClient.print(request);

    #if PROGRAM_DEBUG
    Serial.println("[NTRIP] Da gui request header len Caster:\n" + request);
    #endif
    
    // Đợi server trả lời ICY OK
    unsigned long timeout = millis();
    while (ntripClient.connected() && millis() - timeout < 10000) {
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
    if (!isIcyOk) {
      Serial.println("[NTRIP] Khong nhan duoc ICY OK tu Caster!");
    }
    // goto sendRequest; // Thử gửi lại request nếu không nhận được phản hồi
  } else {
    Serial.println("[NTRIP] Loi ket noi TCP socket!");
    return -1;
  }
  return 0;
}

int loopNTRIP(String& rtcmData) {
  // không sử dụng tài nguyên chung, không cần mutex
  int returnCode = NTRIP_MODE; // returnCode = NTRIP_MODE + ntripClient.available() * 4
  // 1. Quản lý mất kết nối
  if (!ntripClient.connected()) {
    isIcyOk = false;
    if (millis() - lastReconnect > 7000) { // Thử lại sau 7 giây
      lastReconnect = millis();
      return 504; // chuẩn bị kết nối lại
    }
    return 500; // Chưa kết nối, sẽ quay lại ở vòng tiếp theo của loop()
  }

  // 2. Xử lý sau khi kết nối thành công
  if (isIcyOk) {
    // 3. Đẩy RTCM lên Caster nếu có dữ liệu
    if (!rtcmData.isEmpty()) {
      ntripClient.print(rtcmData); // Gửi dữ liệu RTCM lên Caster
      #if PROGRAM_DEBUG
      Serial.println("[NTRIP TASK] Da gui du lieu RTCM len Caster!");
      #endif
      returnCode += 4;
    }
    #if PROGRAM_DEBUG
    else {
      Serial.println("[NTRIP TASK] Khong co du lieu RTCM de gui len Caster.");
    }
    #endif
  }
  return returnCode;
}
#endif // NTRIP_HANDLER_IP_CODE
