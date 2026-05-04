#include "WiFi.h"

WiFiClass WiFi;

void WiFiClass::begin(const char* ssid, const char* password) {
  lastSsid_ = ssid ? String(ssid) : String();
  lastPassword_ = password ? String(password) : String();
  beginCallCount_++;
}

wl_status_t WiFiClass::status() const { return status_; }

IPAddress WiFiClass::localIP() const { return ip_; }

String WiFiClass::ipToString(const IPAddress& ip) const {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

void WiFiClass::mockSetStatus(wl_status_t status) { status_ = status; }

void WiFiClass::mockSetLocalIP(const IPAddress& ip) { ip_ = ip; }

void WiFiClass::mockReset() {
  lastSsid_ = "";
  lastPassword_ = "";
  beginCallCount_ = 0;
  status_ = WL_IDLE_STATUS;
  ip_ = IPAddress();
}

bool WiFiClient::connect(const char* host, uint16_t port) {
  lastHost_ = host ? String(host) : String();
  lastPort_ = port;
  connectCallCount_++;
  connected_ = connectResult_;
  return connectResult_;
}

size_t WiFiClient::print(const String& data) {
  lastPrinted_ = data;
  printCallCount_++;
  return data.length();
}

size_t WiFiClient::print(const char* data) {
  lastPrinted_ = data ? String(data) : String();
  printCallCount_++;
  return lastPrinted_.length();
}

bool WiFiClient::connected() const { return connected_; }

int WiFiClient::available() const {
  if (!responseLines_.empty()) {
    return 1;
  }
  return static_cast<int>(rxBytes_.size());
}

String WiFiClient::readStringUntil(char terminator) {
  (void)terminator;
  if (responseLines_.empty()) {
    return "";
  }

  String line = responseLines_.front();
  responseLines_.erase(responseLines_.begin());
  return line;
}

int WiFiClient::read(uint8_t* buffer, size_t size) {
  readCallCount_++;
  if (buffer == nullptr || size == 0 || rxBytes_.empty()) {
    return 0;
  }

  size_t copyLen = size;
  if (copyLen > rxBytes_.size()) {
    copyLen = rxBytes_.size();
  }

  for (size_t i = 0; i < copyLen; ++i) {
    buffer[i] = rxBytes_[i];
  }

  rxBytes_.erase(rxBytes_.begin(), rxBytes_.begin() + static_cast<long long>(copyLen));
  return static_cast<int>(copyLen);
}

void WiFiClient::mockSetConnectResult(bool result) { connectResult_ = result; }

void WiFiClient::mockSetConnected(bool connected) { connected_ = connected; }

void WiFiClient::mockQueueResponseLine(const String& line) { responseLines_.push_back(line); }

void WiFiClient::mockQueueReadBytes(const uint8_t* data, size_t length) {
  if (data == nullptr || length == 0) {
    return;
  }
  rxBytes_.insert(rxBytes_.end(), data, data + length);
}

void WiFiClient::mockQueueReadString(const String& data) {
  for (size_t i = 0; i < data.length(); ++i) {
    rxBytes_.push_back(static_cast<uint8_t>(data[i]));
  }
}

void WiFiClient::mockReset() {
  lastHost_ = "";
  lastPort_ = 0;
  lastPrinted_ = "";
  connectCallCount_ = 0;
  printCallCount_ = 0;
  readCallCount_ = 0;
  connectResult_ = true;
  connected_ = false;
  responseLines_.clear();
  rxBytes_.clear();
}
