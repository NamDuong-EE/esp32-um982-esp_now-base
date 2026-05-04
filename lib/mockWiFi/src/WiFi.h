#ifndef MOCK_WIFI_H
#define MOCK_WIFI_H

#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include <arduino/IPAddress.h>

#define WL_CONNECTED 3
#define WL_IDLE_STATUS 0

typedef int wl_status_t;

class WiFiClass {
 public:
  void begin(const char* ssid, const char* password);
  wl_status_t status() const;
  IPAddress localIP() const;
  String ipToString(const IPAddress& ip) const;

  void mockSetStatus(wl_status_t status);
  void mockSetLocalIP(const IPAddress& ip);
  void mockReset();

  String lastSsid_;
  String lastPassword_;
  int beginCallCount_ = 0;

 private:
  wl_status_t status_ = WL_IDLE_STATUS;
  IPAddress ip_;
};

class WiFiClient {
 public:
  bool connect(const char* host, uint16_t port);
  size_t print(const String& data);
  size_t print(const char* data);
  bool connected() const;
  int available() const;
  String readStringUntil(char terminator);
  int read(uint8_t* buffer, size_t size);

  void mockSetConnectResult(bool result);
  void mockSetConnected(bool connected);
  void mockQueueResponseLine(const String& line);
  void mockQueueReadBytes(const uint8_t* data, size_t length);
  void mockQueueReadString(const String& data);
  void mockReset();

  String lastHost_;
  uint16_t lastPort_ = 0;
  String lastPrinted_;
  int connectCallCount_ = 0;
  int printCallCount_ = 0;
  int readCallCount_ = 0;

 private:
  bool connectResult_ = true;
  bool connected_ = false;
  std::vector<String> responseLines_;
  std::vector<uint8_t> rxBytes_;
};

extern WiFiClass WiFi;

#endif
