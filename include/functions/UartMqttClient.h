#ifndef UART_MQTT_CLIENT_H
#define UART_MQTT_CLIENT_H

#include <Arduino.h>

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

    bool started_ = false;
    uint32_t nextSequence_ = 1;
    UartMqttCallback callback_ = nullptr;
    uint8_t receiveBuffer_[64] = {};
    size_t receiveLength_ = 0;
    size_t expectedLength_ = 0;
};

#endif // UART_MQTT_CLIENT_H
