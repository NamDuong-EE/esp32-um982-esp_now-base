#include "Top_Lvl_Config.h"

#if CONNECT_USING_WIFI
#define WIFI_CODE

#include "Prog_Config.h"
#include "hardware/Wifi_handler.h"

bool setupWiFi() {
  Serial.print("\n[WIFI] Dang ket noi mang: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempt > 10) {
      Serial.println("\n[ERROR] Ket noi WiFi that bai!");
      return false;
    }
  }
  #ifdef NATIVE_BUILD
  Serial.println("\n[WIFI] Ket noi THANH CONG! IP: " + WiFi.ipToString(WiFi.localIP()));
  #else
  Serial.println("\n[WIFI] Ket noi THANH CONG! IP: " + WiFi.localIP().toString());
  #endif
  return true;
}

#endif // WIFI_CODE