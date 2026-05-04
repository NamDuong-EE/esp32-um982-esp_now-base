#include "mockPubSubClient.h"

PubSubClient::PubSubClient() = default;

void PubSubClient::setServer(const char* domain, uint16_t port) {
  server_ = domain ? String(domain) : String();
  port_ = port;
}

void PubSubClient::setCallback(MQTT_CALLBACK_SIGNATURE callback) { callback_ = callback; }

bool PubSubClient::connected() const { return connected_; }

bool PubSubClient::connect(const char* clientId, const char* user, const char* pass) {
  lastClientId_ = clientId ? String(clientId) : String();
  lastUser_ = user ? String(user) : String();
  lastPass_ = pass ? String(pass) : String();
  ++connectCallCount_;

  connected_ = connectResult_;
  return connectResult_;
}

bool PubSubClient::publish(const char* topic, const char* payload) {
  lastPublishedTopic_ = topic ? String(topic) : String();
  lastPublishedPayload_ = payload ? String(payload) : String();
  ++publishCallCount_;
  return publishResult_;
}

bool PubSubClient::subscribe(const char* topic) {
  lastSubscribedTopic_ = topic ? String(topic) : String();
  ++subscribeCallCount_;
  return subscribeResult_;
}

bool PubSubClient::loop() {
  ++loopCallCount_;
  return true;
}

int PubSubClient::state() const { return state_; }

void PubSubClient::mockSetConnectResult(bool result, int stateCode) {
  connectResult_ = result;
  state_ = stateCode;
  if (!result) {
    connected_ = false;
  }
}

void PubSubClient::mockSetConnected(bool connected) { connected_ = connected; }

void PubSubClient::mockSetPublishResult(bool result) { publishResult_ = result; }

void PubSubClient::mockSetSubscribeResult(bool result) { subscribeResult_ = result; }

void PubSubClient::mockReset() {
  server_ = "";
  port_ = 0;
  callback_ = nullptr;
  connected_ = false;
  connectResult_ = true;
  publishResult_ = true;
  subscribeResult_ = true;
  state_ = 0;

  lastClientId_ = "";
  lastUser_ = "";
  lastPass_ = "";
  lastPublishedTopic_ = "";
  lastPublishedPayload_ = "";
  lastSubscribedTopic_ = "";

  connectCallCount_ = 0;
  publishCallCount_ = 0;
  subscribeCallCount_ = 0;
  loopCallCount_ = 0;
}

void PubSubClient::mockTriggerMessage(const char* topic, const char* payload) {
  if (callback_ == nullptr || topic == nullptr || payload == nullptr) {
    return;
  }

  const size_t payloadLen = strlen(payload);
  uint8_t* buffer = new uint8_t[payloadLen];
  for (size_t i = 0; i < payloadLen; ++i) {
    buffer[i] = static_cast<uint8_t>(payload[i]);
  }

  callback_(const_cast<char*>(topic), buffer, static_cast<unsigned int>(payloadLen));
  delete[] buffer;
}
