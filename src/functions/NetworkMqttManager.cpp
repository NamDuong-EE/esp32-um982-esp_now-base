#include "functions/NetworkMqttManager.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <cmath>
#include <cstring>

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "hardware/BaseEspnow_sender.h"

#if CONNECT_USING_WIFI
#include <WiFi.h>
static WiFiClient transportClient;
#elif CONNECT_USING_4G
#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif
#include <TinyGsmClient.h>
static HardwareSerial modemSerial(2);
static TinyGsm modem(modemSerial);
static TinyGsmClient transportClient(modem);
#endif

namespace {
PubSubClient mqtt(transportClient);
NetworkMqttStats stats{};
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastNetworkAttemptAtMs = 0;
uint32_t lastMqttAttemptAtMs = 0;
bool previousInternetConnected = false;
bool previousMqttConnected = false;
bool hasPendingGnssCommandResult = false;
BaseGnssCommandResultEvent pendingGnssCommandResult{};

struct PendingFixedBaseCommand {
    uint8_t targetMac[6] = {};
    uint32_t transactionId = 0;
    uint32_t requestedAtMs = 0;
    uint32_t timeoutMs = 0;
    bool active = false;
};

PendingFixedBaseCommand pendingFixedBaseCommand{};

struct PublishedLlhState {
    uint8_t mac[6] = {};
    uint32_t lastPublishedSequence = 0;
    uint32_t lastPublishedReceivedAtMs = 0;
    uint32_t lastAttemptedReceivedAtMs = 0;
    uint32_t lastAttemptAtMs = 0;
    bool assigned = false;
    bool hasPublished = false;
};

PublishedLlhState publishedLlh[ESPNOW_MAX_LLH_SOURCES] = {};
#if CONNECT_USING_4G
bool modemInitialized = false;
#endif

template <typename Member>
void incrementStat(Member member)
{
    portENTER_CRITICAL(&statsMux);
    ++(stats.*member);
    portEXIT_CRITICAL(&statsMux);
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
    if (MQTT_HOST[0] == '\0') {
        return false;
    }
#if CONNECT_USING_WIFI
    return NETWORK_WIFI_SSID[0] != '\0';
#else
    return MODEM_APN[0] != '\0';
#endif
}

const char* gnssCommandAction(uint8_t commandId)
{
    switch (commandId) {
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF:
        return "switch_to_base_fixed_ecef";
    case RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_ROVER:
        return "switch_to_rover";
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

void geodeticToEcef(double latitudeDegrees,
                    double longitudeDegrees,
                    double ellipsoidHeightM,
                    double ecefM[3])
{
    constexpr double reWgs84 = 6378137.0;
    constexpr double feWgs84 = 1.0 / 298.257223563;
    constexpr double degreesToRadians = 0.017453292519943295769;
    const double latitude = latitudeDegrees * degreesToRadians;
    const double longitude = longitudeDegrees * degreesToRadians;
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double sinLongitude = std::sin(longitude);
    const double cosLongitude = std::cos(longitude);
    const double eccentricitySquared = feWgs84 * (2.0 - feWgs84);
    const double primeVerticalRadius =
        reWgs84 / std::sqrt(1.0 - eccentricitySquared *
                                     sinLatitude * sinLatitude);

    ecefM[0] = (primeVerticalRadius + ellipsoidHeightM) *
               cosLatitude * cosLongitude;
    ecefM[1] = (primeVerticalRadius + ellipsoidHeightM) *
               cosLatitude * sinLongitude;
    ecefM[2] = (primeVerticalRadius * (1.0 - eccentricitySquared) +
                ellipsoidHeightM) * sinLatitude;
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

    BaseRoverLlhStatus snapshots[ESPNOW_MAX_LLH_SOURCES] = {};
    const size_t count =
        baseEspNowCopyLatestRoverLlh(snapshots, ESPNOW_MAX_LLH_SOURCES);
    const BaseRoverLlhStatus* selected = nullptr;
    for (size_t index = 0; index < count; ++index) {
        if (snapshots[index].valid && !snapshots[index].viaRelay &&
            macEquals(snapshots[index].mac,
                      pendingFixedBaseCommand.targetMac)) {
            selected = &snapshots[index];
            break;
        }
    }
    if (selected == nullptr || selected->fixQuality != 4 ||
        now - selected->receivedAtMs > TEMP_BASE_FIXED_LLH_MAX_AGE_MS) {
        return;
    }

    const double latitude =
        static_cast<double>(selected->latitudeE7) /
        RTCM_ESPNOW_LLH_COORDINATE_SCALE;
    const double longitude =
        static_cast<double>(selected->longitudeE7) /
        RTCM_ESPNOW_LLH_COORDINATE_SCALE;
    const double ellipsoidHeightM =
        static_cast<double>(selected->ellipsoidHeightMm) /
        RTCM_ESPNOW_LLH_HEIGHT_SCALE;
    double ecefM[3] = {};
    geodeticToEcef(latitude, longitude, ellipsoidHeightM, ecefM);
    const int64_t ecefXmm = static_cast<int64_t>(std::llround(ecefM[0] * 1000.0));
    const int64_t ecefYmm = static_cast<int64_t>(std::llround(ecefM[1] * 1000.0));
    const int64_t ecefZmm = static_cast<int64_t>(std::llround(ecefM[2] * 1000.0));

    uint8_t selectedMac[6] = {};
    const BaseGnssCommandQueueResult queueResult =
        baseEspNowQueueRoverBaseFixedEcef(
            pendingFixedBaseCommand.targetMac,
            pendingFixedBaseCommand.transactionId,
            ecefXmm,
            ecefYmm,
            ecefZmm,
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

    Serial.printf("[BASE][ECEF] txn=%lu target=%02X:%02X:%02X:%02X:%02X:%02X "
                  "llh=(%.7f,%.7f,%.3f) ecef_m=(%.4f,%.4f,%.4f)\n",
                  static_cast<unsigned long>(
                      pendingFixedBaseCommand.transactionId),
                  selectedMac[0], selectedMac[1], selectedMac[2],
                  selectedMac[3], selectedMac[4], selectedMac[5],
                  latitude, longitude, ellipsoidHeightM,
                  ecefM[0], ecefM[1], ecefM[2]);
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

    if (std::strcmp(topic, MQTT_TOPIC_COMMAND) != 0) {
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
    if (!switchToBaseFixed && !switchToRover) {
        incrementStat(&NetworkMqttStats::commandsRejected);
        Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] Unsupported action=%s\n", action);
        return;
    }

    uint32_t transactionId = document["transaction_id"] | 0U;
    if (transactionId == 0) {
        do {
            transactionId = esp_random();
        } while (transactionId == 0);
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
        if (!hasRequestedTarget) {
            incrementStat(&NetworkMqttStats::commandsRejected);
            Serial.printf("[BASE][MQTT][GNSS_CMD][REJECT] txn=%lu "
                          "target_mac required for fixed ECEF\n",
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
#else
    return modemInitialized && modem.isNetworkConnected() && modem.isGprsConnected();
#endif
}

int32_t signalDbm()
{
#if CONNECT_USING_WIFI
    return WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
#else
    if (!modemInitialized) {
        return 0;
    }
    const int16_t csq = modem.getSignalQuality();
    return (csq >= 0 && csq <= 31) ? (-113 + (2 * csq)) : 0;
#endif
}

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
#else
    if (!modemInitialized) {
        Serial.printf("[BASE][4G] Starting SIM7600 UART2 baud=%lu RX=%d TX=%d\n",
                      static_cast<unsigned long>(MODEM_BAUD),
                      MODEM_RX_PIN,
                      MODEM_TX_PIN);
        modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
        modemInitialized = modem.restart();
        if (!modemInitialized) {
            Serial.println("[BASE][4G][WARN] Modem restart failed");
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
#endif
}

String mqttClientId()
{
#if CONNECT_USING_WIFI
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    return "base-" + mac;
#else
    String imei = modem.getIMEI();
    if (imei.length() == 0) {
        imei = String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
    }
    return "base-" + imei;
#endif
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
                      MQTT_TOPIC_STATUS,
                      0,
                      true,
                      "offline")) {
        Serial.printf("[BASE][MQTT][WARN] Connect failed state=%d\n", mqtt.state());
        return;
    }

    incrementStat(&NetworkMqttStats::mqttConnects);
    mqtt.publish(MQTT_TOPIC_STATUS, "online", true);
    mqtt.subscribe(MQTT_TOPIC_COMMAND);
    Serial.printf("[BASE][MQTT] Connected, rover_llh_filter=%s/+/llh\n",
                  MQTT_TOPIC_ROVER_LLH_PREFIX);
}

PublishedLlhState* publishedStateFor(const uint8_t* mac)
{
    PublishedLlhState* freeSlot = nullptr;
    for (PublishedLlhState& state : publishedLlh) {
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

void publishLatestRoverLlh(uint32_t now)
{
    BaseRoverLlhStatus snapshots[ESPNOW_MAX_LLH_SOURCES] = {};
    const size_t count = baseEspNowCopyLatestRoverLlh(
        snapshots, ESPNOW_MAX_LLH_SOURCES);

    for (size_t index = 0; index < count; ++index) {
        const BaseRoverLlhStatus& status = snapshots[index];
        if (!status.valid) {
            continue;
        }

        PublishedLlhState* state = publishedStateFor(status.mac);
        if (state == nullptr ||
            (state->hasPublished &&
             state->lastPublishedSequence == status.sequence &&
             state->lastPublishedReceivedAtMs == status.receivedAtMs)) {
            continue;
        }
        if (state->lastAttemptedReceivedAtMs == status.receivedAtMs &&
            now - state->lastAttemptAtMs < MQTT_LLH_RETRY_INTERVAL_MS) {
            continue;
        }
        state->lastAttemptedReceivedAtMs = status.receivedAtMs;
        state->lastAttemptAtMs = now;

        char topic[96] = {};
        char payload[448] = {};
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
        snprintf(topic,
                 sizeof(topic),
                 "%s/%02X%02X%02X%02X%02X%02X/llh",
                 MQTT_TOPIC_ROVER_LLH_PREFIX,
                 status.mac[0], status.mac[1], status.mac[2],
                 status.mac[3], status.mac[4], status.mac[5]);

        const double latitude =
            static_cast<double>(status.latitudeE7) / RTCM_ESPNOW_LLH_COORDINATE_SCALE;
        const double longitude =
            static_cast<double>(status.longitudeE7) / RTCM_ESPNOW_LLH_COORDINATE_SCALE;
        const double heightM =
            static_cast<double>(status.heightMm) / RTCM_ESPNOW_LLH_HEIGHT_SCALE;
        const double ellipsoidHeightM =
            static_cast<double>(status.ellipsoidHeightMm) /
            RTCM_ESPNOW_LLH_HEIGHT_SCALE;
        snprintf(payload,
                 sizeof(payload),
                 "{\"rover_mac\":\"%s\",\"sequence\":%lu,"
                 "\"latitude\":%.7f,\"longitude\":%.7f,"
                 "\"height_m\":%.3f,\"ellipsoid_height_m\":%.3f,"
                 "\"fix_quality\":%u,"
                 "\"via_relay\":%s,"
                 "\"relay_mac\":\"%s\",\"rtcm_source\":\"%s\","
                 "\"temp_base_mac\":%s,\"rtcm_source_epoch\":%lu,"
                 "\"source_age_ms\":%lu}",
                 macText,
                 static_cast<unsigned long>(status.sequence),
                 latitude,
                 longitude,
                 heightM,
                 ellipsoidHeightM,
                 static_cast<unsigned>(status.fixQuality),
                 status.viaRelay ? "true" : "false",
                 relayMacText,
                 status.usesTempBase ? "temp_base" : "local_base",
                 tempBaseJson,
                 static_cast<unsigned long>(status.rtcmSourceEpoch),
                 static_cast<unsigned long>(now - status.receivedAtMs));

        if (mqtt.publish(topic, payload, false)) {
            state->lastPublishedSequence = status.sequence;
            state->lastPublishedReceivedAtMs = status.receivedAtMs;
            state->hasPublished = true;
            portENTER_CRITICAL(&statsMux);
            ++stats.llhPublished;
            stats.lastLlhPublishedAtMs = now;
            portEXIT_CRITICAL(&statsMux);
            Serial.printf("[BASE][MQTT][LLH] Published topic=%s seq=%lu bytes=%u\n",
                          topic,
                          static_cast<unsigned long>(status.sequence),
                          static_cast<unsigned>(strlen(payload)));
        } else {
            incrementStat(&NetworkMqttStats::llhPublishFailures);
            Serial.printf("[BASE][MQTT][LLH][WARN] Publish failed mac=%s seq=%lu\n",
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
    case RTCM_ESPNOW_GNSS_COMMAND_STATUS_UART_SEQUENCE_WRITTEN:
        return "uart_sequence_written";
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
    char ecefJson[128] = "null";
    if (pendingGnssCommandResult.commandId ==
        RTCM_ESPNOW_GNSS_COMMAND_SWITCH_TO_BASE_FIXED_ECEF) {
        snprintf(ecefJson,
                 sizeof(ecefJson),
                 "{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}",
                 static_cast<double>(pendingGnssCommandResult.ecefXmm) / 1000.0,
                 static_cast<double>(pendingGnssCommandResult.ecefYmm) / 1000.0,
                 static_cast<double>(pendingGnssCommandResult.ecefZmm) / 1000.0);
    }
    char resultPayload[448] = {};
    snprintf(resultPayload,
             sizeof(resultPayload),
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
             ecefJson,
             static_cast<unsigned long>(now - pendingGnssCommandResult.receivedAtMs));

    if (!mqtt.publish(MQTT_TOPIC_COMMAND_RESULT, resultPayload, false)) {
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
    const bool configured = credentialsConfigured();
    portENTER_CRITICAL(&statsMux);
    stats.configured = configured;
    portEXIT_CRITICAL(&statsMux);

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(MQTT_KEEPALIVE_SECONDS);
    mqtt.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SECONDS);
    mqtt.setBufferSize(MQTT_BUFFER_SIZE);

    Serial.printf("[BASE][NETWORK] transport=%s configured=%s\n",
                  networkTransportName(),
                  configured ? "yes" : "no");
    if (!configured) {
        Serial.println("[BASE][NETWORK][WARN] Fill include/Network_Secrets.h to enable Internet/MQTT");
    }
}

void networkMqttLoop()
{
    processPendingFixedBaseCommand(millis());
    const NetworkMqttStats snapshot = getNetworkMqttStats();
    if (!snapshot.configured) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    const uint32_t now = millis();
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
#else
        Serial.printf("[BASE][4G] Internet %s\n", online ? "connected" : "disconnected");
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
        publishGnssCommandResult(now);
        publishLatestRoverLlh(now);
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
#else
    return "4g";
#endif
}
