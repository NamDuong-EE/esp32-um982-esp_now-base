#ifndef UART_MQTT_CLIENT_H
#define UART_MQTT_CLIENT_H

#include <Arduino.h>

#include "UartMqttBridgeProtocol.h"

using UartMqttCallback = void (*)(char*, uint8_t*, unsigned int);

class UartMqttClient {
public:
    void begin();
    void setServer(const char*, uint16_t) {}
    void setCallback(UartMqttCallback callback) { callback_ = callback; }
    void setKeepAlive(uint16_t) {}
    void setSocketTimeout(uint16_t) {}
    bool setBufferSize(uint16_t) { return true; }

    bool connect(const char*,
                 const char*,
                 const char*,
                 const char*,
                 uint8_t,
                 bool,
                 const char*);
    bool publish(const char* topic, const char* payload, bool retained = false);
    bool subscribe(const char*) { return true; }
    bool connected() const { return started_; }
    void disconnect() {}
    bool loop();
    int state() const { return started_ ? 0 : -1; }

private:
    void processIncoming();
    void processCompleteFrame();
    void sendAck(uint32_t sequence, uint8_t status);
    bool deliverySeen(uint32_t sequence) const;
    void rememberDelivery(uint32_t sequence);

    bool started_ = false;
    uint32_t nextSequence_ = 1;
    UartMqttCallback callback_ = nullptr;
    uint8_t receiveBuffer_[sizeof(UartMqttFrameHeader) +
                           UART_MQTT_MAX_TOPIC_LENGTH +
                           UART_MQTT_MAX_PAYLOAD_LENGTH] = {};
    size_t receiveLength_ = 0;
    size_t expectedLength_ = 0;
    uint32_t recentDeliveries_[8] = {};
    size_t recentDeliveryIndex_ = 0;
    char incomingTopic_[UART_MQTT_MAX_TOPIC_LENGTH + 1] = {};
    uint8_t incomingPayload_[UART_MQTT_MAX_PAYLOAD_LENGTH + 1] = {};
};

#endif // UART_MQTT_CLIENT_H
