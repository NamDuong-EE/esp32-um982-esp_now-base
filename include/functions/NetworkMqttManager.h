#ifndef NETWORK_MQTT_MANAGER_H
#define NETWORK_MQTT_MANAGER_H

#include <cstdint>

struct NetworkMqttStats {
    bool configured = false;
    bool internetConnected = false;
    bool mqttConnected = false;
    int32_t signalDbm = 0;
    uint32_t networkAttempts = 0;
    uint32_t mqttAttempts = 0;
    uint32_t mqttConnects = 0;
    uint32_t mqttDisconnects = 0;
    uint32_t llhPublished = 0;
    uint32_t llhPublishFailures = 0;
    uint32_t lastLlhPublishedAtMs = 0;
    uint32_t commandsReceived = 0;
    uint32_t commandsRejected = 0;
    uint32_t commandResultsPublished = 0;
    uint32_t commandResultPublishFailures = 0;
    uint32_t stackHighWaterBytes = 0;
};

void setupNetworkMqtt();
void networkMqttLoop();
NetworkMqttStats getNetworkMqttStats();
const char* networkTransportName();

#endif // NETWORK_MQTT_MANAGER_H
