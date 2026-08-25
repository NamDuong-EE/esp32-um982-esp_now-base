#include "functions/NetworkMqttManager.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <cmath>
#include <cstring>
#include <esp_wifi.h>

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "functions/BaseGnssRoleController.h"
#include "hardware/BaseEspnow_sender.h"

#if CONNECT_USING_WIFI
#include <PubSubClient.h>
#include <WiFi.h>
static WiFiClient transportClient;
#elif CONNECT_USING_4G
#include <PubSubClient.h>
#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif
#include <TinyGsmClient.h>
static HardwareSerial modemSerial(2);
static TinyGsm modem(modemSerial);
static TinyGsmClient transportClient(modem);
#elif CONNECT_USING_UART_GATEWAY
#include "functions/UartMqttClient.h"
#endif

namespace {
#if CONNECT_USING_UART_GATEWAY
UartMqttClient mqtt;
#else
PubSubClient mqtt(transportClient);
#endif
NetworkMqttStats stats{};
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastNetworkAttemptAtMs = 0;
uint32_t lastMqttAttemptAtMs = 0;
bool previousInternetConnected = false;
bool previousMqttConnected = false;
char baseMacTopicId[13] = {};
char mqttTopicStatus[48] = {};
char mqttTopicCommand[48] = {};
char mqttTopicCommandResult[56] = {};
char mqttTopicRoverEcefPrefix[56] = {};
#if CONNECT_USING_UART_GATEWAY
uint32_t lastUartIdentityPublishAtMs = 0;
#endif
bool hasPendingGnssCommandResult = false;
BaseGnssCommandResultEvent pendingGnssCommandResult{};
bool hasPendingLocalCoordinateResult = false;
BaseLocalCoordinateResult pendingLocalCoordinateResult{};

struct PendingFixedBaseCommand {
    uint8_t targetMac[6] = {};
    uint32_t transactionId = 0;
    uint32_t requestedAtMs = 0;
    uint32_t timeoutMs = 0;
    bool active = false;
};

PendingFixedBaseCommand pendingFixedBaseCommand{};

struct PublishedEcefState {
    uint8_t mac[6] = {};
    uint32_t lastPublishedSequence = 0;
    uint32_t lastPublishedReceivedAtMs = 0;
    uint32_t lastAttemptedReceivedAtMs = 0;
    uint32_t lastAttemptAtMs = 0;
    bool assigned = false;
    bool hasPublished = false;
};

PublishedEcefState publishedEcef[ESPNOW_MAX_ECEF_SOURCES] = {};

struct NetworkMqttWorkBuffers {
    BaseRoverEcefStatus snapshots[ESPNOW_MAX_ECEF_SOURCES] = {};
    char topic[96] = {};
    char payload[768] = {};
    char commandPayload[448] = {};
    char ecefJson[128] = {};
    char correctionJson[128] = {};
    char correctedJson[128] = {};
    char rawX[32] = {};
    char rawY[32] = {};
    char rawZ[32] = {};
    char deltaX[32] = {};
    char deltaY[32] = {};
    char deltaZ[32] = {};
    char correctedX[32] = {};
    char correctedY[32] = {};
    char correctedZ[32] = {};
};

NetworkMqttWorkBuffers workBuffers{};
#if CONNECT_USING_4G
bool modemInitialized = false;
bool modemUartStarted = false;
bool modemPowerSequenceCompleted = false;
#endif

template <typename Member>
void incrementStat(Member member)
{
    portENTER_CRITICAL(&statsMux);
    ++(stats.*member);
    portEXIT_CRITICAL(&statsMux);
}

bool initializeMqttTopics()
{
    uint8_t mac[6] = {};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) != ESP_OK) {
        Serial.println("[BASE][MQTT][ERROR] Cannot read Base STA MAC for topics");
        return false;
    }
    snprintf(baseMacTopicId,
             sizeof(baseMacTopicId),
             "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(mqttTopicStatus, sizeof(mqttTopicStatus), "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE, baseMacTopicId, MQTT_TOPIC_STATUS_SUFFIX);
    snprintf(mqttTopicCommand, sizeof(mqttTopicCommand), "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE, baseMacTopicId, MQTT_TOPIC_COMMAND_SUFFIX);
    snprintf(mqttTopicCommandResult,
             sizeof(mqttTopicCommandResult),
             "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE,
             baseMacTopicId,
             MQTT_TOPIC_COMMAND_RESULT_SUFFIX);
    snprintf(mqttTopicRoverEcefPrefix,
             sizeof(mqttTopicRoverEcefPrefix),
             "%s/%s/base/%s",
             MQTT_TOPIC_NAMESPACE,
             baseMacTopicId,
             MQTT_TOPIC_ROVERS_SUFFIX);
    Serial.printf("[BASE][MQTT] topic_root=%s/%s/base\n",
                  MQTT_TOPIC_NAMESPACE,
                  baseMacTopicId);
    return true;
}

void setConnectionStats(bool internetConnected, bool mqttConnected, int32_t signalDbm)
{
    portENTER_CRITICAL(&statsMux);
    stats.internetConnected = internetConnected;
    stats.mqttConnected = mqttConnected;
    stats.signalDbm = signalDbm;
    portEXIT_CRITICAL(&statsMux);
}

bool credentialsConfigured()
{
#if CONNECT_USING_UART_GATEWAY
    return true;
#else
    if (MQTT_HOST[0] == '\0') {
        return false;
    }
#if CONNECT_USING_WIFI
    return NETWORK_WIFI_SSID[0] != '\0';
#else
    return MODEM_APN[0] != '\0';
#endif
#endif
}

const char* gnssCommandAction(uint8_t commandId)
{
    switch (commandId) {
    case RTCM_ESPNOW_GNSS_COMMAND_RESET_RTK:
        return "rtk_reset";
    case RTCM_ESPNOW_GNSS_COMMAND_RESUME_RTK:
        return "rtk_resume";
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER:
        return "switch_to_rover";
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF:
        return "switch_to_base_fixed_ecef";
    default:
        return "unknown";
    }
}

int8_t hexNibble(char value)
{
    if (value >= '0' && value <= '9') {
        return static_cast<int8_t>(value - '0');
    }
    if (value >= 'A' && value <= 'F') {
        return static_cast<int8_t>(value - 'A' + 10);
    }
    if (value >= 'a' && value <= 'f') {
        return static_cast<int8_t>(value - 'a' + 10);
    }
    return -1;
}

bool parseUnicastMac(const char* text, uint8_t mac[6])
{
    if (text == nullptr || std::strlen(text) != 17) {
        return false;
    }
    for (size_t index = 0; index < 6; ++index) {
        const size_t offset = index * 3;
        const int8_t high = hexNibble(text[offset]);
        const int8_t low = hexNibble(text[offset + 1]);
        if (high < 0 || low < 0 || (index < 5 && text[offset + 2] != ':')) {
            return false;
        }
        mac[index] = static_cast<uint8_t>((high << 4) | low);
    }
    const bool allZero = mac[0] == 0 && mac[1] == 0 && mac[2] == 0 &&
                         mac[3] == 0 && mac[4] == 0 && mac[5] == 0;
    return !allZero && (mac[0] & 0x01U) == 0;
}

bool macEquals(const uint8_t left[6], const uint8_t right[6])
{
    return left != nullptr && right != nullptr && std::memcmp(left, right, 6) == 0;
}

void formatEcefScaled(int64_t value, char* destination, size_t capacity)
{
    if (destination == nullptr || capacity == 0) {
        return;
    }
    const bool negative = value < 0;
    const uint64_t magnitude = negative
                                   ? static_cast<uint64_t>(-(value + 1)) + 1U
                                   : static_cast<uint64_t>(value);
    snprintf(destination,
             capacity,
             "%s%llu.%04llu",
             negative ? "-" : "",
             static_cast<unsigned long long>(magnitude / 10000ULL),
             static_cast<unsigned long long>(magnitude % 10000ULL));
}

bool jsonFiniteNumber(JsonVariantConst value, double& destination)
{
    if (value.isNull() || !value.is<double>()) {
        return false;
    }
    destination = value.as<double>();
    return std::isfinite(destination);
}

bool parseLocalFixedCoordinates(JsonDocument& document,
                                int64_t& xScaled,
                                int64_t& yScaled,
                                int64_t& zScaled)
{
    const char* coordinateSystem = document["coordinate_system"] | "";
    if (std::strcmp(coordinateSystem, "ecef") == 0) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        const JsonObjectConst ecef = document["ecef_m"].as<JsonObjectConst>();
        if (ecef.isNull() || !jsonFiniteNumber(ecef["x"], x) ||
            !jsonFiniteNumber(ecef["y"], y) ||
            !jsonFiniteNumber(ecef["z"], z)) {
            return false;
        }
        constexpr double ecefLimitM =
            static_cast<double>(RTCM_ESPNOW_ECEF_SCALED_LIMIT) /
            RTCM_ESPNOW_ECEF_SCALE;
        if (std::abs(x) > ecefLimitM || std::abs(y) > ecefLimitM ||
            std::abs(z) > ecefLimitM) {
            return false;
        }
        xScaled = static_cast<int64_t>(std::llround(
            x * RTCM_ESPNOW_ECEF_SCALE));
        yScaled = static_cast<int64_t>(std::llround(
            y * RTCM_ESPNOW_ECEF_SCALE));
        zScaled = static_cast<int64_t>(std::llround(
            z * RTCM_ESPNOW_ECEF_SCALE));
        return rtcmEspNowValidEcef(xScaled, yScaled, zScaled);
    }
    if (std::strcmp(coordinateSystem, "llh") == 0) {
        double latitude = 0.0;
        double longitude = 0.0;
        double height = 0.0;
        const JsonObjectConst llh = document["llh"].as<JsonObjectConst>();
        return !llh.isNull() &&
               jsonFiniteNumber(llh["latitude_deg"], latitude) &&
               jsonFiniteNumber(llh["longitude_deg"], longitude) &&
               jsonFiniteNumber(llh["ellipsoid_height_m"], height) &&
               height >= -30000.0 && height <= 30000.0 &&
               baseGnssGeodeticToEcef(latitude,
                                      longitude,
                                      height,
                                      xScaled,
                                      yScaled,
                                      zScaled);
    }
    return false;
}

void processPendingFixedBaseCommand(uint32_t now)
{
    if (!pendingFixedBaseCommand.active) {
        return;
    }
    if (now - pendingFixedBaseCommand.requestedAtMs >
        pendingFixedBaseCommand.timeoutMs) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu target="
                      "%02X:%02X:%02X:%02X:%02X:%02X result=rtk_fixed_timeout\n",
                      static_cast<unsigned long>(
                          pendingFixedBaseCommand.transactionId),
                      pendingFixedBaseCommand.targetMac[0],
                      pendingFixedBaseCommand.targetMac[1],
                      pendingFixedBaseCommand.targetMac[2],
                      pendingFixedBaseCommand.targetMac[3],
                      pendingFixedBaseCommand.targetMac[4],
                      pendingFixedBaseCommand.targetMac[5]);
        pendingFixedBaseCommand = {};
        return;
    }

    const size_t count =
        baseEspNowCopyLatestRoverEcef(workBuffers.snapshots,
                                     ESPNOW_MAX_ECEF_SOURCES);
    const BaseRoverEcefStatus* selected = nullptr;
    for (size_t index = 0; index < count; ++index) {
        if (workBuffers.snapshots[index].valid &&
            !workBuffers.snapshots[index].viaRelay &&
            macEquals(workBuffers.snapshots[index].mac,
                      pendingFixedBaseCommand.targetMac)) {
            selected = &workBuffers.snapshots[index];
            break;
        }
    }
    if (selected == nullptr || selected->fixQuality != 4 ||
        now - selected->receivedAtMs > TEMP_BASE_FIXED_ECEF_MAX_AGE_MS) {
        return;
    }

    uint8_t selectedMac[6] = {};
    const BaseGnssCommandQueueResult queueResult =
        baseEspNowQueueRoverBaseFixedEcef(
            pendingFixedBaseCommand.targetMac,
            pendingFixedBaseCommand.transactionId,
            selected->ecefXScaled,
            selected->ecefYScaled,
            selected->ecefZScaled,
            selectedMac);
    if (queueResult == BaseGnssCommandQueueResult::QueueFull ||
        queueResult == BaseGnssCommandQueueResult::NotReady) {
        return;
    }
    if (queueResult != BaseGnssCommandQueueResult::Queued) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu result=%s\n",
                      static_cast<unsigned long>(
                          pendingFixedBaseCommand.transactionId),
                      baseGnssCommandQueueResultToString(queueResult));
        pendingFixedBaseCommand = {};
        return;
    }

    formatEcefScaled(selected->ecefXScaled, workBuffers.rawX,
                     sizeof(workBuffers.rawX));
    formatEcefScaled(selected->ecefYScaled, workBuffers.rawY,
                     sizeof(workBuffers.rawY));
    formatEcefScaled(selected->ecefZScaled, workBuffers.rawZ,
                     sizeof(workBuffers.rawZ));
    Serial.printf("[BASE][ECEF] txn=%lu target=%02X:%02X:%02X:%02X:%02X:%02X "
                  "gnss_ms=%lu ecef_m=(%s,%s,%s)\n",
                  static_cast<unsigned long>(
                      pendingFixedBaseCommand.transactionId),
                  selectedMac[0], selectedMac[1], selectedMac[2],
                  selectedMac[3], selectedMac[4], selectedMac[5],
                  static_cast<unsigned long>(selected->gnssTimeMsOfDay),
                  workBuffers.rawX,
                  workBuffers.rawY,
                  workBuffers.rawZ);
    pendingFixedBaseCommand = {};
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length)
{
    constexpr unsigned int maxLogLength = 160;
    const unsigned int logLength = min(length, maxLogLength);
    Serial.printf("[BASE][MQTT] RX topic=%s bytes=%u payload=", topic, length);
    Serial.write(payload, logLength);
    if (length > logLength) {
        Serial.print("...");
    }
    Serial.println();

    if (std::strcmp(topic, mqttTopicCommand) != 0) {
        return;
    }
    incrementStat(&NetworkMqttStats::commandsReceived);

    JsonDocument document;
    const DeserializationError error = deserializeJson(document, payload, length);
    if (error) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] Invalid JSON: %s\n",
                      error.c_str());
        return;
    }

    const char* action = document["action"] | "";
    const bool switchToBaseFixed =
        std::strcmp(action, "switch_to_base_fixed_ecef") == 0;
    const bool switchToRover = std::strcmp(action, "switch_to_rover") == 0;
    const bool setLocalCoordinates =
        std::strcmp(action, "set_local_base_coordinates") == 0;
    const bool clearLocalCoordinates =
        std::strcmp(action, "clear_local_base_coordinates") == 0;
    if (!switchToBaseFixed && !switchToRover && !setLocalCoordinates &&
        !clearLocalCoordinates) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] Unsupported action=%s\n", action);
        return;
    }

    uint32_t transactionId = document["transaction_id"] | 0U;
    if ((setLocalCoordinates || clearLocalCoordinates) && transactionId == 0) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][LOCAL_COORD][REJECT] action=%s "
                      "transaction_id required\n",
                      action);
        return;
    }
    if (transactionId == 0) {
        do {
            transactionId = esp_random();
        } while (transactionId == 0);
    }

    if (setLocalCoordinates || clearLocalCoordinates) {
        if (pendingFixedBaseCommand.active ||
            baseEspNowGnssCommandPending()) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][LOCAL_COORD][REJECT] txn=%lu "
                          "result=temp_base_command_pending\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        int64_t xScaled = 0;
        int64_t yScaled = 0;
        int64_t zScaled = 0;
        if (setLocalCoordinates &&
            !parseLocalFixedCoordinates(document,
                                        xScaled,
                                        yScaled,
                                        zScaled)) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][LOCAL_COORD][REJECT] txn=%lu "
                          "result=invalid_coordinates\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        const BaseLocalCoordinateQueueResult queueResult =
            setLocalCoordinates
                ? baseGnssQueueSetFixedCoordinates(transactionId,
                                                   xScaled,
                                                   yScaled,
                                                   zScaled)
                : baseGnssQueueClearCoordinates(transactionId);
        if (queueResult != BaseLocalCoordinateQueueResult::Queued) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][LOCAL_COORD][REJECT] txn=%lu result=%s\n",
                          static_cast<unsigned long>(transactionId),
                          baseLocalCoordinateQueueResultToString(queueResult));
            return;
        }
        Serial.printf("[BASE][MQTT][LOCAL_COORD] Queued txn=%lu action=%s\n",
                      static_cast<unsigned long>(transactionId),
                      action);
        return;
    }

    const JsonVariantConst targetMacValue = document["target_mac"];
    const bool hasRequestedTarget = !targetMacValue.isNull();
    uint8_t requestedTargetMac[6] = {};
    if (hasRequestedTarget &&
        (!targetMacValue.is<const char*>() ||
         !parseUnicastMac(targetMacValue.as<const char*>(), requestedTargetMac))) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu invalid target_mac\n",
                      static_cast<unsigned long>(transactionId));
        return;
    }
    if (switchToBaseFixed) {
        if (baseGnssLocalCoordinateCommandBusy()) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "result=local_coordinate_command_busy\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        if (!hasRequestedTarget) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "target_mac required for fixed ECEF\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        if (!baseGnssRoleHasReference()) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "result=local_rtcm1006_reference_unavailable\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        if (!baseEspNowIsDirectRoverPaired(requestedTargetMac)) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "result=target_not_paired\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        if (pendingFixedBaseCommand.active) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "result=fixed_command_pending\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        const uint32_t timeoutSeconds =
            document["fix_timeout_s"] |
            TEMP_BASE_FIXED_WAIT_DEFAULT_SECONDS;
        if (timeoutSeconds == 0 ||
            timeoutSeconds > TEMP_BASE_FIXED_WAIT_MAX_SECONDS) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "invalid fix_timeout_s\n",
                          static_cast<unsigned long>(transactionId));
            return;
        }
        std::memcpy(pendingFixedBaseCommand.targetMac,
                    requestedTargetMac,
                    sizeof(pendingFixedBaseCommand.targetMac));
        pendingFixedBaseCommand.transactionId = transactionId;
        pendingFixedBaseCommand.requestedAtMs = millis();
        pendingFixedBaseCommand.timeoutMs = timeoutSeconds * 1000UL;
        pendingFixedBaseCommand.active = true;
        Serial.printf("[BASE][MQTT][GNSS_CMD] Waiting RTK Fixed txn=%lu "
                      "target=%02X:%02X:%02X:%02X:%02X:%02X timeout_s=%lu\n",
                      static_cast<unsigned long>(transactionId),
                      requestedTargetMac[0], requestedTargetMac[1],
                      requestedTargetMac[2], requestedTargetMac[3],
                      requestedTargetMac[4], requestedTargetMac[5],
                      static_cast<unsigned long>(timeoutSeconds));
        return;
    }

    uint8_t targetMac[6] = {};
    const BaseGnssCommandQueueResult queueResult =
        baseEspNowQueueRoverMode(hasRequestedTarget ? requestedTargetMac : nullptr,
                                 transactionId,
                                 targetMac);
    if (queueResult != BaseGnssCommandQueueResult::Queued) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu result=%s\n",
                      static_cast<unsigned long>(transactionId),
                      baseGnssCommandQueueResultToString(queueResult));
        return;
    }
    if (pendingFixedBaseCommand.active &&
        macEquals(targetMac, pendingFixedBaseCommand.targetMac)) {
        Serial.printf("[BASE][MQTT][GNSS_CMD] Cancel pending fixed ECEF txn=%lu "
                      "target=%02X:%02X:%02X:%02X:%02X:%02X replacement=%s\n",
                      static_cast<unsigned long>(
                          pendingFixedBaseCommand.transactionId),
                      targetMac[0], targetMac[1], targetMac[2],
                      targetMac[3], targetMac[4], targetMac[5],
                      action);
        pendingFixedBaseCommand = {};
    }

    Serial.printf("[BASE][MQTT][GNSS_CMD] Queued txn=%lu action=%s "
                  "target=%02X:%02X:%02X:%02X:%02X:%02X selection=%s\n",
                  static_cast<unsigned long>(transactionId),
                  action,
                  targetMac[0], targetMac[1], targetMac[2],
                  targetMac[3], targetMac[4], targetMac[5],
                  hasRequestedTarget ? "mqtt" : "first_paired");
}

bool internetConnected()
{
#if CONNECT_USING_WIFI
    return WiFi.status() == WL_CONNECTED && WiFi.channel() == ESPNOW_WIFI_CHANNEL;
#elif CONNECT_USING_4G
    return modemInitialized && modem.isNetworkConnected() && modem.isGprsConnected();
#else
    return mqtt.connected();
#endif
}

int32_t signalDbm()
{
#if CONNECT_USING_WIFI
    return WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
#elif CONNECT_USING_4G
    if (!modemInitialized) {
        return 0;
    }
    const int16_t csq = modem.getSignalQuality();
    return (csq >= 0 && csq <= 31) ? (-113 + (2 * csq)) : 0;
#else
    return 0;
#endif
}

#if CONNECT_USING_4G
void startModemUart()
{
    if (modemUartStarted) {
        return;
    }
    Serial.printf("[BASE][4G] Starting SIM7600 UART2 baud=%lu RX=%d TX=%d\n",
                  static_cast<unsigned long>(MODEM_BAUD),
                  MODEM_RX_PIN,
                  MODEM_TX_PIN);
    modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    modemUartStarted = true;
}

void runModemPowerOnSequence()
{
    if (modemPowerSequenceCompleted) {
        return;
    }
    modemPowerSequenceCompleted = true;
    if (!MODEM_POWER_CONTROL_ENABLED) {
        Serial.println("[BASE][4G] Modem power control disabled");
        return;
    }

    const uint8_t activeLevel =
        MODEM_POWER_CONTROL_ACTIVE_HIGH ? HIGH : LOW;
    const uint8_t inactiveLevel =
        MODEM_POWER_CONTROL_ACTIVE_HIGH ? LOW : HIGH;
    pinMode(MODEM_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(MODEM_POWER_CONTROL_PIN, inactiveLevel);
    delay(50);
    Serial.printf("[BASE][4G] Power-on pulse pin=%d active=%s pulse_ms=%lu\n",
                  MODEM_POWER_CONTROL_PIN,
                  MODEM_POWER_CONTROL_ACTIVE_HIGH ? "HIGH" : "LOW",
                  static_cast<unsigned long>(MODEM_POWER_PULSE_MS));
    digitalWrite(MODEM_POWER_CONTROL_PIN, activeLevel);
    delay(MODEM_POWER_PULSE_MS);
    digitalWrite(MODEM_POWER_CONTROL_PIN, inactiveLevel);
    Serial.printf("[BASE][4G] Waiting %lu ms for modem boot\n",
                  static_cast<unsigned long>(MODEM_BOOT_WAIT_MS));
    delay(MODEM_BOOT_WAIT_MS);
}
#endif

void startNetworkConnection()
{
    incrementStat(&NetworkMqttStats::networkAttempts);
#if CONNECT_USING_WIFI
    Serial.printf("[BASE][WIFI] Connecting SSID=%s fixed_channel=%u\n",
                  NETWORK_WIFI_SSID,
                  ESPNOW_WIFI_CHANNEL);
    WiFi.setAutoReconnect(false);
    WiFi.begin(NETWORK_WIFI_SSID,
               NETWORK_WIFI_PASSWORD,
               ESPNOW_WIFI_CHANNEL,
               nullptr,
               true);
#elif CONNECT_USING_4G
    if (!modemInitialized) {
        startModemUart();
        runModemPowerOnSequence();
        // The modem was just powered on and allowed to boot. TinyGSM's
        // SIM7600 restart() sends AT+CRESET, waits for a response and then
        // blocks for another 24 seconds before init(). That reset is
        // redundant here and can starve the ESP32 idle task long enough to
        // trigger the task watchdog. Probe and configure the running modem
        // directly instead.
        modemInitialized = modem.init();
        if (!modemInitialized) {
            Serial.println("[BASE][4G][WARN] Modem init failed; check power, UART pins and baud");
            return;
        }
        Serial.println("[BASE][4G] Modem=" + modem.getModemInfo());
    }

    if (!modem.isNetworkConnected() &&
        !modem.waitForNetwork(MODEM_NETWORK_TIMEOUT_MS, true)) {
        Serial.println("[BASE][4G][WARN] Mobile network registration failed");
        return;
    }
    if (!modem.isGprsConnected() &&
        !modem.gprsConnect(MODEM_APN, MODEM_GPRS_USER, MODEM_GPRS_PASSWORD)) {
        Serial.println("[BASE][4G][WARN] APN data connection failed");
        return;
    }
    Serial.println("[BASE][4G] Data connection ready");
#else
    mqtt.begin();
    Serial.println("[BASE][UART-GW] Wired gateway transport ready");
#endif
}

String mqttClientId()
{
    return "base-" + String(baseMacTopicId);
}

void connectMqtt()
{
    incrementStat(&NetworkMqttStats::mqttAttempts);
    const String clientId = mqttClientId();
    const char* user = MQTT_USER[0] == '\0' ? nullptr : MQTT_USER;
    const char* password = MQTT_PASSWORD[0] == '\0' ? nullptr : MQTT_PASSWORD;
    Serial.printf("[BASE][MQTT] Connecting broker=%s:%u client_id=%s\n",
                  MQTT_HOST,
                  MQTT_PORT,
                  clientId.c_str());

    if (!mqtt.connect(clientId.c_str(),
                      user,
                      password,
                       mqttTopicStatus,
                      0,
                      true,
                      "offline")) {
        Serial.printf("[BASE][MQTT][WARN] Connect failed state=%d\n", mqtt.state());
        return;
    }

    incrementStat(&NetworkMqttStats::mqttConnects);
    mqtt.publish(mqttTopicStatus, "online", true);
    mqtt.subscribe(mqttTopicCommand);
    Serial.printf("[BASE][MQTT] Connected, rover_ecef_filter=%s/+/ecef\n",
                  mqttTopicRoverEcefPrefix);
}

PublishedEcefState* publishedStateFor(const uint8_t* mac)
{
    PublishedEcefState* freeSlot = nullptr;
    for (PublishedEcefState& state : publishedEcef) {
        if (state.assigned && std::memcmp(state.mac, mac, sizeof(state.mac)) == 0) {
            return &state;
        }
        if (!state.assigned && freeSlot == nullptr) {
            freeSlot = &state;
        }
    }
    if (freeSlot != nullptr) {
        std::memcpy(freeSlot->mac, mac, sizeof(freeSlot->mac));
        freeSlot->assigned = true;
    }
    return freeSlot;
}

uint32_t gnssTimeDeltaMs(uint32_t left, uint32_t right)
{
    const uint32_t direct = left > right ? left - right : right - left;
    const uint32_t wrapped =
        RTCM_ESPNOW_GNSS_MILLISECONDS_PER_DAY - direct;
    return direct < wrapped ? direct : wrapped;
}

void publishLatestRoverEcef(uint32_t now)
{
    const size_t count = baseEspNowCopyLatestRoverEcef(
        workBuffers.snapshots, ESPNOW_MAX_ECEF_SOURCES);
    const BaseEcefCorrectionSnapshot correction =
        getBaseEcefCorrectionSnapshot();
    const uint16_t activeStreamId = getBaseEspNowStreamId();

    for (size_t index = 0; index < count; ++index) {
        const BaseRoverEcefStatus& status = workBuffers.snapshots[index];
        if (!status.valid) {
            continue;
        }

        PublishedEcefState* state = publishedStateFor(status.mac);
        if (state == nullptr ||
            (state->hasPublished &&
             state->lastPublishedSequence == status.sequence &&
             state->lastPublishedReceivedAtMs == status.receivedAtMs)) {
            continue;
        }
        if (state->lastAttemptedReceivedAtMs == status.receivedAtMs &&
            now - state->lastAttemptAtMs < MQTT_ECEF_RETRY_INTERVAL_MS) {
            continue;
        }
        state->lastAttemptedReceivedAtMs = status.receivedAtMs;
        state->lastAttemptAtMs = now;

        char macText[18] = {};
        snprintf(macText,
                 sizeof(macText),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 status.mac[0], status.mac[1], status.mac[2],
                 status.mac[3], status.mac[4], status.mac[5]);
        char relayMacText[18] = {};
        if (status.viaRelay) {
            snprintf(relayMacText,
                     sizeof(relayMacText),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     status.relayMac[0], status.relayMac[1], status.relayMac[2],
                     status.relayMac[3], status.relayMac[4], status.relayMac[5]);
        }
        char tempBaseMacText[18] = {};
        char tempBaseJson[24] = "null";
        if (status.usesTempBase) {
            snprintf(tempBaseMacText,
                     sizeof(tempBaseMacText),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     status.tempBaseMac[0], status.tempBaseMac[1],
                     status.tempBaseMac[2], status.tempBaseMac[3],
                     status.tempBaseMac[4], status.tempBaseMac[5]);
            snprintf(tempBaseJson, sizeof(tempBaseJson), "\"%s\"", tempBaseMacText);
        }
        snprintf(workBuffers.topic,
                 sizeof(workBuffers.topic),
                 "%s/%02X%02X%02X%02X%02X%02X/ecef",
                 mqttTopicRoverEcefPrefix,
                 status.mac[0], status.mac[1], status.mac[2],
                 status.mac[3], status.mac[4], status.mac[5]);

        const bool correctionValid =
            correction.correctionValid &&
            status.fixQuality == 4 &&
            status.usesTempBase &&
            !macEquals(status.mac, status.tempBaseMac) &&
            status.correctionStreamId == activeStreamId &&
            gnssTimeDeltaMs(status.gnssTimeMsOfDay,
                            correction.gnssTimeMsOfDay) <=
                BASE_ECEF_CORRECTION_MAX_TIME_DELTA_MS;
        snprintf(workBuffers.correctionJson,
                 sizeof(workBuffers.correctionJson),
                 "null");
        snprintf(workBuffers.correctedJson,
                 sizeof(workBuffers.correctedJson),
                 "null");
        formatEcefScaled(status.ecefXScaled,
                         workBuffers.rawX,
                         sizeof(workBuffers.rawX));
        formatEcefScaled(status.ecefYScaled,
                         workBuffers.rawY,
                         sizeof(workBuffers.rawY));
        formatEcefScaled(status.ecefZScaled,
                         workBuffers.rawZ,
                         sizeof(workBuffers.rawZ));
        if (correctionValid) {
            formatEcefScaled(correction.deltaXScaled,
                             workBuffers.deltaX,
                             sizeof(workBuffers.deltaX));
            formatEcefScaled(correction.deltaYScaled,
                             workBuffers.deltaY,
                             sizeof(workBuffers.deltaY));
            formatEcefScaled(correction.deltaZScaled,
                             workBuffers.deltaZ,
                             sizeof(workBuffers.deltaZ));
            formatEcefScaled(status.ecefXScaled + correction.deltaXScaled,
                             workBuffers.correctedX,
                             sizeof(workBuffers.correctedX));
            formatEcefScaled(status.ecefYScaled + correction.deltaYScaled,
                             workBuffers.correctedY,
                             sizeof(workBuffers.correctedY));
            formatEcefScaled(status.ecefZScaled + correction.deltaZScaled,
                             workBuffers.correctedZ,
                             sizeof(workBuffers.correctedZ));
            snprintf(workBuffers.correctionJson,
                     sizeof(workBuffers.correctionJson),
                     "{\"dx\":%s,\"dy\":%s,\"dz\":%s}",
                     workBuffers.deltaX,
                     workBuffers.deltaY,
                     workBuffers.deltaZ);
            snprintf(workBuffers.correctedJson,
                     sizeof(workBuffers.correctedJson),
                     "{\"x\":%s,\"y\":%s,\"z\":%s}",
                     workBuffers.correctedX,
                     workBuffers.correctedY,
                     workBuffers.correctedZ);
        }
        snprintf(workBuffers.payload,
                 sizeof(workBuffers.payload),
                 "{\"rover_mac\":\"%s\",\"sequence\":%lu,"
                 "\"gnss_time_ms\":%lu,\"fix_quality\":%u,"
                 "\"ecef_raw_m\":{\"x\":%s,\"y\":%s,\"z\":%s},"
                 "\"correction_valid\":%s,\"correction_ecef_m\":%s,"
                 "\"ecef_corrected_m\":%s,"
                 "\"via_relay\":%s,"
                 "\"relay_mac\":\"%s\",\"rtcm_source\":\"%s\","
                 "\"temp_base_mac\":%s,\"rtcm_source_epoch\":%lu,"
                 "\"correction_stream_id\":%u,\"source_age_ms\":%lu,"
                 "\"correction_age_ms\":%lu}",
                 macText,
                 static_cast<unsigned long>(status.sequence),
                 static_cast<unsigned long>(status.gnssTimeMsOfDay),
                 static_cast<unsigned>(status.fixQuality),
                 workBuffers.rawX,
                 workBuffers.rawY,
                 workBuffers.rawZ,
                 correctionValid ? "true" : "false",
                 workBuffers.correctionJson,
                 workBuffers.correctedJson,
                 status.viaRelay ? "true" : "false",
                 relayMacText,
                 status.usesTempBase ? "temp_base" : "local_base",
                 tempBaseJson,
                 static_cast<unsigned long>(status.rtcmSourceEpoch),
                 static_cast<unsigned>(status.correctionStreamId),
                 static_cast<unsigned long>(now - status.receivedAtMs),
                 correction.observationValid
                     ? static_cast<unsigned long>(
                           now - correction.observedAtMs)
                     : 0UL);

        if (mqtt.publish(workBuffers.topic, workBuffers.payload, false)) {
            state->lastPublishedSequence = status.sequence;
            state->lastPublishedReceivedAtMs = status.receivedAtMs;
            state->hasPublished = true;
            portENTER_CRITICAL(&statsMux);
            ++stats.llhPublished;
            stats.lastLlhPublishedAtMs = now;
            portEXIT_CRITICAL(&statsMux);
            Serial.printf("[BASE][MQTT][ECEF] Published topic=%s seq=%lu "
                          "corrected=%u bytes=%u\n",
                          workBuffers.topic,
                          static_cast<unsigned long>(status.sequence),
                          correctionValid ? 1U : 0U,
                          static_cast<unsigned>(strlen(workBuffers.payload)));
        } else {
            incrementStat(&NetworkMqttStats::llhPublishFailures);
            Serial.printf("[BASE][MQTT][ECEF][WARN] Publish failed mac=%s "
                          "seq=%lu\n",
                          macText,
                          static_cast<unsigned long>(status.sequence));
        }
    }
}

const char* gnssCommandStatusText(const BaseGnssCommandResultEvent& result)
{
    if (result.responseTimedOut) {
        return "response_timeout";
    }
    switch (result.status) {
    case RTCM_ESPNOW_GNSS_COMMAND_STATUS_VERIFIED:
        return "verified";
    case RTCM_ESPNOW_GNSS_COMMAND_STATUS_REJECTED:
        return "rejected";
    case RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_ERROR:
        return "uart_error";
    case RTCM_ESPNOW_GNSS_COMMAND_STATUS_BUSY:
        return "busy";
    default:
        return "unknown";
    }
}

void publishLocalCoordinateResult(uint32_t now)
{
    if (!hasPendingLocalCoordinateResult) {
        hasPendingLocalCoordinateResult =
            baseGnssPopLocalCoordinateResult(pendingLocalCoordinateResult);
    }
    if (!hasPendingLocalCoordinateResult) {
        return;
    }
    snprintf(workBuffers.ecefJson, sizeof(workBuffers.ecefJson), "null");
    if (pendingLocalCoordinateResult.command ==
        BaseLocalCoordinateCommand::SetFixed) {
        formatEcefScaled(pendingLocalCoordinateResult.ecefXScaled,
                         workBuffers.rawX,
                         sizeof(workBuffers.rawX));
        formatEcefScaled(pendingLocalCoordinateResult.ecefYScaled,
                         workBuffers.rawY,
                         sizeof(workBuffers.rawY));
        formatEcefScaled(pendingLocalCoordinateResult.ecefZScaled,
                         workBuffers.rawZ,
                         sizeof(workBuffers.rawZ));
        snprintf(workBuffers.ecefJson,
                 sizeof(workBuffers.ecefJson),
                 "{\"x\":%s,\"y\":%s,\"z\":%s}",
                 workBuffers.rawX,
                 workBuffers.rawY,
                 workBuffers.rawZ);
    }
    const char* action =
        pendingLocalCoordinateResult.command ==
                BaseLocalCoordinateCommand::SetFixed
            ? "set_local_base_coordinates"
            : "clear_local_base_coordinates";
    snprintf(workBuffers.commandPayload,
             sizeof(workBuffers.commandPayload),
             "{\"transaction_id\":%lu,\"action\":\"%s\","
             "\"status\":\"%s\",\"ecef_m\":%s,\"result_age_ms\":%lu}",
             static_cast<unsigned long>(
                 pendingLocalCoordinateResult.transactionId),
             action,
             baseLocalCoordinateStatusToString(
                 pendingLocalCoordinateResult.status),
             workBuffers.ecefJson,
             static_cast<unsigned long>(
                 now - pendingLocalCoordinateResult.completedAtMs));
    if (!mqtt.publish(mqttTopicCommandResult,
                      workBuffers.commandPayload,
                      false)) {
        incrementStat(&NetworkMqttStats::commandResultPublishFailures);
        return;
    }
    incrementStat(&NetworkMqttStats::commandResultsPublished);
    Serial.printf("[BASE][MQTT][LOCAL_COORD] Result published txn=%lu status=%s\n",
                  static_cast<unsigned long>(
                      pendingLocalCoordinateResult.transactionId),
                  baseLocalCoordinateStatusToString(
                      pendingLocalCoordinateResult.status));
    hasPendingLocalCoordinateResult = false;
}

void publishGnssCommandResult(uint32_t now)
{
    if (!hasPendingGnssCommandResult) {
        hasPendingGnssCommandResult =
            baseEspNowPopGnssCommandResult(pendingGnssCommandResult);
    }
    if (!hasPendingGnssCommandResult) {
        return;
    }

    char macText[18] = {};
    snprintf(macText,
             sizeof(macText),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             pendingGnssCommandResult.roverMac[0],
             pendingGnssCommandResult.roverMac[1],
             pendingGnssCommandResult.roverMac[2],
             pendingGnssCommandResult.roverMac[3],
             pendingGnssCommandResult.roverMac[4],
             pendingGnssCommandResult.roverMac[5]);
    snprintf(workBuffers.ecefJson, sizeof(workBuffers.ecefJson), "null");
    if (pendingGnssCommandResult.commandId ==
        RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF) {
        formatEcefScaled(pendingGnssCommandResult.ecefXScaled,
                         workBuffers.rawX,
                         sizeof(workBuffers.rawX));
        formatEcefScaled(pendingGnssCommandResult.ecefYScaled,
                         workBuffers.rawY,
                         sizeof(workBuffers.rawY));
        formatEcefScaled(pendingGnssCommandResult.ecefZScaled,
                         workBuffers.rawZ,
                         sizeof(workBuffers.rawZ));
        snprintf(workBuffers.ecefJson,
                 sizeof(workBuffers.ecefJson),
                 "{\"x\":%s,\"y\":%s,\"z\":%s}",
                 workBuffers.rawX,
                 workBuffers.rawY,
                 workBuffers.rawZ);
    }
    snprintf(workBuffers.commandPayload,
             sizeof(workBuffers.commandPayload),
             "{\"transaction_id\":%lu,\"target_mac\":\"%s\","
             "\"action\":\"%s\",\"status\":\"%s\","
             "\"completed_step\":%u,\"total_steps\":%u,"
             "\"detail_code\":%u,\"ecef_m\":%s,\"result_age_ms\":%lu}",
             static_cast<unsigned long>(pendingGnssCommandResult.transactionId),
             macText,
             gnssCommandAction(pendingGnssCommandResult.commandId),
             gnssCommandStatusText(pendingGnssCommandResult),
             pendingGnssCommandResult.completedStep,
             pendingGnssCommandResult.totalSteps,
             pendingGnssCommandResult.detailCode,
             workBuffers.ecefJson,
             static_cast<unsigned long>(now - pendingGnssCommandResult.receivedAtMs));

    if (!mqtt.publish(mqttTopicCommandResult,
                      workBuffers.commandPayload,
                      false)) {
        incrementStat(&NetworkMqttStats::commandResultPublishFailures);
        return;
    }
    incrementStat(&NetworkMqttStats::commandResultsPublished);
    Serial.printf("[BASE][MQTT][GNSS_CMD] Result published txn=%lu status=%s\n",
                  static_cast<unsigned long>(pendingGnssCommandResult.transactionId),
                  gnssCommandStatusText(pendingGnssCommandResult));
    hasPendingGnssCommandResult = false;
}
}

void setupNetworkMqtt()
{
    const bool topicsReady = initializeMqttTopics();
    const bool configured = topicsReady && credentialsConfigured();
    portENTER_CRITICAL(&statsMux);
    stats.configured = configured;
    portEXIT_CRITICAL(&statsMux);

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_SECONDS);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SECONDS);
    mqtt.setBufferSize(MQTT_BUFFER_SIZE);
#if CONNECT_USING_UART_GATEWAY
    mqtt.begin();
    if (configured) {
        mqtt.publish(mqttTopicStatus, "online", true);
        lastUartIdentityPublishAtMs = millis();
    }
#endif

    Serial.printf("[BASE][NETWORK] transport=%s configured=%s\n",
                  networkTransportName(),
                  configured ? "yes" : "no");
    if (!configured) {
        Serial.println("[BASE][NETWORK][WARN] Fill include/Network_Secrets.h to enable Internet/MQTT");
    }
}

void networkMqttLoop()
{
    const uint32_t stackHighWaterBytes =
        static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
    portENTER_CRITICAL(&statsMux);
    stats.stackHighWaterBytes = stackHighWaterBytes;
    portEXIT_CRITICAL(&statsMux);

    processPendingFixedBaseCommand(millis());
    const NetworkMqttStats snapshot = getNetworkMqttStats();
    if (!snapshot.configured) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    const uint32_t now = millis();
#if CONNECT_USING_UART_GATEWAY
    if (now - lastUartIdentityPublishAtMs >= 30000) {
        mqtt.publish(mqttTopicStatus, "online", true);
        lastUartIdentityPublishAtMs = now;
    }
#endif
    bool online = internetConnected();
    if (!online &&
        (lastNetworkAttemptAtMs == 0 ||
         now - lastNetworkAttemptAtMs >= NETWORK_RECONNECT_INTERVAL_MS)) {
        lastNetworkAttemptAtMs = now;
        startNetworkConnection();
        online = internetConnected();
    }

    if (online != previousInternetConnected) {
#if CONNECT_USING_WIFI
        if (online) {
            Serial.printf("[BASE][WIFI] Connected IP=%s channel=%u RSSI=%ld dBm\n",
                          WiFi.localIP().toString().c_str(),
                          WiFi.channel(),
                          static_cast<long>(WiFi.RSSI()));
        } else {
            Serial.println("[BASE][WIFI][WARN] Connection lost; ESP-NOW remains active");
        }
#elif CONNECT_USING_4G
        Serial.printf("[BASE][4G] Internet %s\n", online ? "connected" : "disconnected");
#else
        Serial.printf("[BASE][UART-GW] Link %s\n",
                      online ? "ready" : "disconnected");
#endif
        previousInternetConnected = online;
    }

    if (!online) {
        if (mqtt.connected()) {
            mqtt.disconnect();
        }
        if (previousMqttConnected) {
            incrementStat(&NetworkMqttStats::mqttDisconnects);
            Serial.println("[BASE][MQTT][WARN] Connection lost");
            previousMqttConnected = false;
        }
        setConnectionStats(false, false, signalDbm());
        return;
    }

    if (!mqtt.connected() &&
        (lastMqttAttemptAtMs == 0 ||
         now - lastMqttAttemptAtMs >= MQTT_RECONNECT_INTERVAL_MS)) {
        lastMqttAttemptAtMs = now;
        connectMqtt();
    }

    const bool mqttOnline = mqtt.connected();
    if (previousMqttConnected && !mqttOnline) {
        incrementStat(&NetworkMqttStats::mqttDisconnects);
        Serial.println("[BASE][MQTT][WARN] Connection lost");
    }
    previousMqttConnected = mqttOnline;
    setConnectionStats(true, mqttOnline, signalDbm());

    if (!mqttOnline) {
        return;
    }

    mqtt.loop();
    if (mqtt.connected()) {
        publishLocalCoordinateResult(now);
        publishGnssCommandResult(now);
        publishLatestRoverEcef(now);
    }
}

NetworkMqttStats getNetworkMqttStats()
{
    portENTER_CRITICAL(&statsMux);
    const NetworkMqttStats copy = stats;
    portEXIT_CRITICAL(&statsMux);
    return copy;
}

const char* networkTransportName()
{
#if CONNECT_USING_WIFI
    return "wifi";
#elif CONNECT_USING_4G
    return "4g";
#else
    return "uart-4g-gateway";
#endif
}
