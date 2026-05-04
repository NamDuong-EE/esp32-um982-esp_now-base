#ifndef MOCK_PUBSUBCLIENT_H
#define MOCK_PUBSUBCLIENT_H

#include <Arduino.h>
#include <cstring>
#include <stdint.h>

class PubSubClient {
 public:
  using MQTT_CALLBACK_SIGNATURE = void (*)(char*, uint8_t*, unsigned int);

  PubSubClient();

  void setServer(const char* domain, uint16_t port);

  void setCallback(MQTT_CALLBACK_SIGNATURE callback);

  bool connected() const;

  bool connect(const char* clientId, const char* user, const char* pass);

  bool publish(const char* topic, const char* payload);

  bool subscribe(const char* topic);

  bool loop();

  int state() const;

  void mockSetConnectResult(bool result, int stateCode = 0);

  void mockSetConnected(bool connected);

  void mockSetPublishResult(bool result);

  void mockSetSubscribeResult(bool result);

  void mockReset();

  void mockTriggerMessage(const char* topic, const char* payload);

  String server_;
  uint16_t port_ = 0;

  String lastClientId_;
  String lastUser_;
  String lastPass_;

  String lastPublishedTopic_;
  String lastPublishedPayload_;
  String lastSubscribedTopic_;

  int connectCallCount_ = 0;
  int publishCallCount_ = 0;
  int subscribeCallCount_ = 0;
  int loopCallCount_ = 0;

 private:
  MQTT_CALLBACK_SIGNATURE callback_ = nullptr;
  bool connected_ = false;
  bool connectResult_ = true;
  bool publishResult_ = true;
  bool subscribeResult_ = true;
  int state_ = 0;
};

#endif
