#include "functions/BaseGnssRoleController.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <Preferences.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "Prog_Config.h"
#include "RtcmEspNowProtocol.h"
#include "hardware/BaseEspnow_sender.h"

namespace {

enum class LocalGnssRole : uint8_t {
    LocalBase,
    SwitchingToRover,
    Rover,
    SwitchingToBase,
};

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
SemaphoreHandle_t uartMutex = nullptr;
LocalGnssRole role = LocalGnssRole::LocalBase;
BaseEcefCorrectionSnapshot correction{};
String nmeaLine;
QueueHandle_t localCommandQueue = nullptr;
QueueHandle_t localResultQueue = nullptr;
volatile bool localCommandBusy = false;
uint32_t referenceSequence = 0;

struct SavedBaseCoordinate {
    uint32_t magic = BASE_GNSS_SAVED_COORDINATE_MAGIC;
    int64_t xScaled = 0;
    int64_t yScaled = 0;
    int64_t zScaled = 0;
    uint32_t checksum = 0;
};

struct LocalCoordinateRequest {
    uint32_t transactionId = 0;
    BaseLocalCoordinateCommand command = BaseLocalCoordinateCommand::SetFixed;
    int64_t xScaled = 0;
    int64_t yScaled = 0;
    int64_t zScaled = 0;
};

uint32_t coordinateChecksum(const SavedBaseCoordinate& coordinate)
{
    uint32_t hash = 2166136261UL;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&coordinate);
    const size_t length = offsetof(SavedBaseCoordinate, checksum);
    for (size_t index = 0; index < length; ++index) {
        hash ^= bytes[index];
        hash *= 16777619UL;
    }
    return hash;
}

bool loadSavedCoordinate(SavedBaseCoordinate& coordinate)
{
    Preferences preferences;
    if (!preferences.begin(BASE_GNSS_NVS_NAMESPACE, true)) {
        return false;
    }
    const bool present =
        preferences.getBytesLength(BASE_GNSS_NVS_COORDINATE_KEY) ==
            sizeof(coordinate) &&
        preferences.getBytes(BASE_GNSS_NVS_COORDINATE_KEY,
                             &coordinate,
                             sizeof(coordinate)) == sizeof(coordinate);
    preferences.end();
    return present && coordinate.magic == BASE_GNSS_SAVED_COORDINATE_MAGIC &&
           coordinate.checksum == coordinateChecksum(coordinate) &&
           rtcmEspNowValidEcef(coordinate.xScaled,
                               coordinate.yScaled,
                               coordinate.zScaled);
}

bool saveCoordinate(int64_t xScaled, int64_t yScaled, int64_t zScaled)
{
    SavedBaseCoordinate coordinate{};
    coordinate.xScaled = xScaled;
    coordinate.yScaled = yScaled;
    coordinate.zScaled = zScaled;
    coordinate.checksum = coordinateChecksum(coordinate);
    Preferences preferences;
    if (!preferences.begin(BASE_GNSS_NVS_NAMESPACE, false)) {
        return false;
    }
    const bool ok = preferences.putBytes(BASE_GNSS_NVS_COORDINATE_KEY,
                                         &coordinate,
                                         sizeof(coordinate)) ==
                    sizeof(coordinate);
    preferences.end();
    return ok;
}

bool clearSavedCoordinate()
{
    Preferences preferences;
    if (!preferences.begin(BASE_GNSS_NVS_NAMESPACE, false)) {
        return false;
    }
    const bool ok = !preferences.isKey(BASE_GNSS_NVS_COORDINATE_KEY) ||
                    preferences.remove(BASE_GNSS_NVS_COORDINATE_KEY);
    preferences.end();
    return ok;
}

uint64_t getUnsignedBits(const uint8_t* data, size_t bitOffset, size_t bitLength)
{
    uint64_t value = 0;
    for (size_t bit = 0; bit < bitLength; ++bit) {
        const size_t absoluteBit = bitOffset + bit;
        value = (value << 1U) |
                ((data[absoluteBit / 8U] >>
                  (7U - (absoluteBit % 8U))) & 0x01U);
    }
    return value;
}

int64_t getSignedBits(const uint8_t* data, size_t bitOffset, size_t bitLength)
{
    const uint64_t raw = getUnsignedBits(data, bitOffset, bitLength);
    const uint64_t signBit = 1ULL << (bitLength - 1U);
    if ((raw & signBit) == 0) {
        return static_cast<int64_t>(raw);
    }
    return static_cast<int64_t>(raw | (~0ULL << bitLength));
}

bool parseUtcMs(const String& field, uint32_t& millisecondsOfDay)
{
    if (field.length() < 6) {
        return false;
    }
    const int hours = field.substring(0, 2).toInt();
    const int minutes = field.substring(2, 4).toInt();
    const double seconds = field.substring(4).toDouble();
    if (hours < 0 || hours > 23 ||
        minutes < 0 || minutes > 59 ||
        !std::isfinite(seconds) || seconds < 0.0 || seconds >= 60.0) {
        return false;
    }
    millisecondsOfDay =
        static_cast<uint32_t>(hours) * 3600000UL +
        static_cast<uint32_t>(minutes) * 60000UL +
        static_cast<uint32_t>(std::llround(seconds * 1000.0));
    return millisecondsOfDay < RTCM_ESPNOW_GNSS_MILLISECONDS_PER_DAY;
}

double nmeaCoordinateToDegrees(const String& value, const String& direction)
{
    const int dot = value.indexOf('.');
    if (dot < 2) {
        return NAN;
    }
    const int degrees = value.substring(0, dot - 2).toInt();
    const double minutes = value.substring(dot - 2).toDouble();
    double result = static_cast<double>(degrees) + minutes / 60.0;
    if (direction == "S" || direction == "W") {
        result = -result;
    }
    return result;
}

bool geodeticToEcef(double latitudeDegrees,
                    double longitudeDegrees,
                    double ellipsoidHeightM,
                    int64_t& xScaled,
                    int64_t& yScaled,
                    int64_t& zScaled)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double toRadians = 0.017453292519943295769;
    if (!std::isfinite(latitudeDegrees) ||
        !std::isfinite(longitudeDegrees) ||
        !std::isfinite(ellipsoidHeightM)) {
        return false;
    }
    const double latitude = latitudeDegrees * toRadians;
    const double longitude = longitudeDegrees * toRadians;
    const double sinLatitude = std::sin(latitude);
    const double cosLatitude = std::cos(latitude);
    const double e2 = f * (2.0 - f);
    const double v = a / std::sqrt(1.0 - e2 * sinLatitude * sinLatitude);
    xScaled = static_cast<int64_t>(std::llround(
        (v + ellipsoidHeightM) * cosLatitude * std::cos(longitude) *
        RTCM_ESPNOW_ECEF_SCALE));
    yScaled = static_cast<int64_t>(std::llround(
        (v + ellipsoidHeightM) * cosLatitude * std::sin(longitude) *
        RTCM_ESPNOW_ECEF_SCALE));
    zScaled = static_cast<int64_t>(std::llround(
        (v * (1.0 - e2) + ellipsoidHeightM) * sinLatitude *
        RTCM_ESPNOW_ECEF_SCALE));
    return rtcmEspNowValidEcef(xScaled, yScaled, zScaled);
}

bool parseGga(const String& line,
              uint32_t& gnssTimeMs,
              uint8_t& fixQuality,
              int64_t& xScaled,
              int64_t& yScaled,
              int64_t& zScaled)
{
    if (!line.startsWith("$GNGGA") && !line.startsWith("$GPGGA")) {
        return false;
    }
    int comma[14] = {};
    size_t count = 0;
    for (int index = 0; index < line.length() && count < 14; ++index) {
        if (line[index] == ',') {
            comma[count++] = index;
        }
    }
    if (count < 12) {
        return false;
    }
    const String utc = line.substring(comma[0] + 1, comma[1]);
    const String latitudeField = line.substring(comma[1] + 1, comma[2]);
    const String latitudeDirection = line.substring(comma[2] + 1, comma[3]);
    const String longitudeField = line.substring(comma[3] + 1, comma[4]);
    const String longitudeDirection = line.substring(comma[4] + 1, comma[5]);
    const String fixField = line.substring(comma[5] + 1, comma[6]);
    const String altitudeField = line.substring(comma[8] + 1, comma[9]);
    const String geoidField = line.substring(comma[10] + 1, comma[11]);
    if (latitudeField.isEmpty() || longitudeField.isEmpty() ||
        altitudeField.isEmpty() || geoidField.isEmpty() ||
        !parseUtcMs(utc, gnssTimeMs)) {
        return false;
    }
    fixQuality = static_cast<uint8_t>(fixField.toInt());
    const double latitude =
        nmeaCoordinateToDegrees(latitudeField, latitudeDirection);
    const double longitude =
        nmeaCoordinateToDegrees(longitudeField, longitudeDirection);
    const double ellipsoidHeight =
        altitudeField.toDouble() + geoidField.toDouble();
    return geodeticToEcef(latitude,
                          longitude,
                          ellipsoidHeight,
                          xScaled,
                          yScaled,
                          zScaled);
}

bool writeCommand(const char* command)
{
    if (uartMutex == nullptr ||
        xSemaphoreTake(uartMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }
    const size_t length = std::strlen(command);
    const size_t written = Serial1.write(
        reinterpret_cast<const uint8_t*>(command), length);
    Serial1.flush();
    xSemaphoreGive(uartMutex);
    Serial.printf("[BASE][LOCAL_GNSS][CMD] %s", command);
    return written == length;
}

bool writeBaseConfiguration(bool fixed,
                            int64_t xScaled,
                            int64_t yScaled,
                            int64_t zScaled,
                            bool persist)
{
    char modeCommand[128] = {};
    if (fixed) {
        snprintf(modeCommand,
                 sizeof(modeCommand),
                 "mode base %.4f %.4f %.4f\r\n",
                 static_cast<double>(xScaled) / RTCM_ESPNOW_ECEF_SCALE,
                 static_cast<double>(yScaled) / RTCM_ESPNOW_ECEF_SCALE,
                 static_cast<double>(zScaled) / RTCM_ESPNOW_ECEF_SCALE);
    } else {
        std::strcpy(modeCommand, "mode base\r\n");
    }
    const char* commands[] = {
        "unlogall\r\n", modeCommand, "gpgga com2 1\r\n",
        "rtcm1006 com2 1\r\n", "rtcm1033 com2 1\r\n",
        "rtcm1074 com2 1\r\n", "rtcm1124 com2 1\r\n",
        "rtcm1084 com2 1\r\n", "rtcm1094 com2 1\r\n",
        "rtcm1042 com2 1\r\n", "rtcm1019 com2 1\r\n",
        "rtcm1020 com2 1\r\n", "rtcm1045 com2 1\r\n",
    };
    bool ok = true;
    for (const char* command : commands) {
        ok = writeCommand(command) && ok;
        vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_OUTPUT_COMMAND_DELAY_MS));
    }
    if (persist) {
        ok = writeCommand("saveconfig\r\n") && ok;
        vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_OUTPUT_COMMAND_DELAY_MS));
    }
    return ok;
}

bool waitForFixedReference(int64_t xScaled,
                           int64_t yScaled,
                           int64_t zScaled)
{
    uint32_t lastSequence = 0;
    portENTER_CRITICAL(&stateMux);
    lastSequence = referenceSequence;
    portEXIT_CRITICAL(&stateMux);
    uint8_t matchingSamples = 0;
    const uint32_t startedAtMs = millis();
    while (millis() - startedAtMs < BASE_GNSS_FIXED_VERIFY_TIMEOUT_MS) {
        uint32_t sequence = 0;
        int64_t referenceX = 0;
        int64_t referenceY = 0;
        int64_t referenceZ = 0;
        portENTER_CRITICAL(&stateMux);
        sequence = referenceSequence;
        referenceX = correction.referenceXScaled;
        referenceY = correction.referenceYScaled;
        referenceZ = correction.referenceZScaled;
        portEXIT_CRITICAL(&stateMux);
        if (sequence != lastSequence) {
            lastSequence = sequence;
            const bool matches =
                std::llabs(referenceX - xScaled) <=
                    BASE_GNSS_FIXED_VERIFY_TOLERANCE_SCALED &&
                std::llabs(referenceY - yScaled) <=
                    BASE_GNSS_FIXED_VERIFY_TOLERANCE_SCALED &&
                std::llabs(referenceZ - zScaled) <=
                    BASE_GNSS_FIXED_VERIFY_TOLERANCE_SCALED;
            matchingSamples = matches ? matchingSamples + 1U : 0U;
            if (matchingSamples >= BASE_GNSS_FIXED_VERIFY_SAMPLES) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

void pushLocalResult(const LocalCoordinateRequest& request,
                     BaseLocalCoordinateStatus status)
{
    if (localResultQueue == nullptr) {
        return;
    }
    BaseLocalCoordinateResult result{};
    result.transactionId = request.transactionId;
    result.command = request.command;
    result.status = status;
    result.ecefXScaled = request.xScaled;
    result.ecefYScaled = request.yScaled;
    result.ecefZScaled = request.zScaled;
    result.completedAtMs = millis();
    if (xQueueSend(localResultQueue, &result, 0) != pdTRUE) {
        BaseLocalCoordinateResult discarded{};
        xQueueReceive(localResultQueue, &discarded, 0);
        xQueueSend(localResultQueue, &result, 0);
    }
}

[[noreturn]] void localCoordinateCommandTask(void*)
{
    LocalCoordinateRequest request{};
    while (true) {
        if (xQueueReceive(localCommandQueue, &request, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        localCommandBusy = true;
        const BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
        if (source.state != BaseRtcmSourceState::LocalActive) {
            pushLocalResult(request, BaseLocalCoordinateStatus::UartError);
            localCommandBusy = false;
            continue;
        }

        SavedBaseCoordinate previous{};
        const bool previousWasFixed = loadSavedCoordinate(previous);
        if (request.command == BaseLocalCoordinateCommand::ClearToSurvey) {
            const bool uartOk = writeBaseConfiguration(false, 0, 0, 0, true);
            const bool storageOk = uartOk && clearSavedCoordinate();
            if (storageOk) {
                portENTER_CRITICAL(&stateMux);
                correction.referenceValid = false;
                correction.correctionValid = false;
                correction.stableSamples = 0;
                portEXIT_CRITICAL(&stateMux);
                baseEspNowMarkLocalBaseReady();
                pushLocalResult(request,
                                BaseLocalCoordinateStatus::SurveyStarted);
            } else {
                pushLocalResult(request,
                                uartOk ? BaseLocalCoordinateStatus::StorageError
                                       : BaseLocalCoordinateStatus::UartError);
            }
            localCommandBusy = false;
            continue;
        }

        if (!writeBaseConfiguration(true,
                                    request.xScaled,
                                    request.yScaled,
                                    request.zScaled,
                                    true)) {
            pushLocalResult(request, BaseLocalCoordinateStatus::UartError);
            localCommandBusy = false;
            continue;
        }
        if (waitForFixedReference(request.xScaled,
                                  request.yScaled,
                                  request.zScaled)) {
            if (saveCoordinate(request.xScaled,
                               request.yScaled,
                               request.zScaled)) {
                baseEspNowMarkLocalBaseReady();
                pushLocalResult(request,
                                BaseLocalCoordinateStatus::AppliedVerified);
            } else {
                pushLocalResult(request,
                                BaseLocalCoordinateStatus::StorageError);
            }
            localCommandBusy = false;
            continue;
        }

        bool rollbackOk = false;
        if (previousWasFixed) {
            rollbackOk = writeBaseConfiguration(true,
                                                previous.xScaled,
                                                previous.yScaled,
                                                previous.zScaled,
                                                true) &&
                         waitForFixedReference(previous.xScaled,
                                               previous.yScaled,
                                               previous.zScaled);
        } else {
            rollbackOk = writeBaseConfiguration(false, 0, 0, 0, true);
            if (rollbackOk) {
                portENTER_CRITICAL(&stateMux);
                correction.referenceValid = false;
                correction.correctionValid = false;
                correction.stableSamples = 0;
                portEXIT_CRITICAL(&stateMux);
            }
        }
        if (rollbackOk) {
            baseEspNowMarkLocalBaseReady();
        }
        pushLocalResult(request,
                        rollbackOk
                            ? BaseLocalCoordinateStatus::VerifyTimeoutRolledBack
                            : BaseLocalCoordinateStatus::RollbackFailed);
        localCommandBusy = false;
    }
}

bool switchLocalGnssToRover()
{
    portENTER_CRITICAL(&stateMux);
    role = LocalGnssRole::SwitchingToRover;
    correction.observationValid = false;
    correction.correctionValid = false;
    correction.stableSamples = 0;
    portEXIT_CRITICAL(&stateMux);
    bool ok = writeCommand("unlogall\r\n");
    vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_ROLE_COMMAND_DELAY_MS));
    ok = writeCommand("mode rover survey\r\n") && ok;
    vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_ROLE_COMMAND_DELAY_MS));
    ok = writeCommand("config rtk reset\r\n") && ok;
    vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_OUTPUT_COMMAND_DELAY_MS));
    ok = writeCommand("gpgga com2 1\r\n") && ok;
    portENTER_CRITICAL(&stateMux);
    role = ok ? LocalGnssRole::Rover : LocalGnssRole::LocalBase;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("[BASE][LOCAL_GNSS] role=%s\n", ok ? "ROVER" : "BASE_ERROR");
    return ok;
}

bool switchLocalGnssToBase()
{
    BaseEcefCorrectionSnapshot snapshot = getBaseEcefCorrectionSnapshot();
    if (!snapshot.referenceValid) {
        Serial.println("[BASE][LOCAL_GNSS][ERROR] Missing RTCM1006 reference");
        return false;
    }
    portENTER_CRITICAL(&stateMux);
    role = LocalGnssRole::SwitchingToBase;
    correction.observationValid = false;
    correction.correctionValid = false;
    correction.stableSamples = 0;
    portEXIT_CRITICAL(&stateMux);

    char modeCommand[128] = {};
    snprintf(modeCommand,
             sizeof(modeCommand),
             "mode base %.4f %.4f %.4f\r\n",
             static_cast<double>(snapshot.referenceXScaled) /
                 RTCM_ESPNOW_ECEF_SCALE,
             static_cast<double>(snapshot.referenceYScaled) /
                 RTCM_ESPNOW_ECEF_SCALE,
             static_cast<double>(snapshot.referenceZScaled) /
                 RTCM_ESPNOW_ECEF_SCALE);
    const char* commands[] = {
        "unlogall\r\n", modeCommand, "gpgga com2 1\r\n",
        "rtcm1006 com2 1\r\n", "rtcm1033 com2 1\r\n",
        "rtcm1074 com2 1\r\n", "rtcm1124 com2 1\r\n",
        "rtcm1084 com2 1\r\n", "rtcm1094 com2 1\r\n",
        "rtcm1042 com2 1\r\n", "rtcm1019 com2 1\r\n",
        "rtcm1020 com2 1\r\n", "rtcm1045 com2 1\r\n",
    };
    bool ok = true;
    for (const char* command : commands) {
        ok = writeCommand(command) && ok;
        vTaskDelay(pdMS_TO_TICKS(BASE_GNSS_OUTPUT_COMMAND_DELAY_MS));
    }
    portENTER_CRITICAL(&stateMux);
    role = ok ? LocalGnssRole::LocalBase : LocalGnssRole::Rover;
    portEXIT_CRITICAL(&stateMux);
    if (ok) {
        baseEspNowMarkLocalBaseReady();
    }
    Serial.printf("[BASE][LOCAL_GNSS] role=%s\n", ok ? "BASE" : "BASE_ERROR");
    return ok;
}

void updateCorrection(uint32_t gnssTimeMs,
                      uint8_t fixQuality,
                      int64_t xScaled,
                      int64_t yScaled,
                      int64_t zScaled)
{
    portENTER_CRITICAL(&stateMux);
    correction.observationValid = true;
    correction.observedXScaled = xScaled;
    correction.observedYScaled = yScaled;
    correction.observedZScaled = zScaled;
    correction.gnssTimeMsOfDay = gnssTimeMs;
    correction.observedAtMs = millis();
    correction.fixQuality = fixQuality;
    const int64_t dx = correction.referenceXScaled - xScaled;
    const int64_t dy = correction.referenceYScaled - yScaled;
    const int64_t dz = correction.referenceZScaled - zScaled;
    const bool absoluteValid =
        std::llabs(dx) <= BASE_ECEF_CORRECTION_MAX_ABS_SCALED &&
        std::llabs(dy) <= BASE_ECEF_CORRECTION_MAX_ABS_SCALED &&
        std::llabs(dz) <= BASE_ECEF_CORRECTION_MAX_ABS_SCALED;
    const bool stepValid =
        correction.stableSamples == 0 ||
        (std::llabs(dx - correction.deltaXScaled) <=
             BASE_ECEF_CORRECTION_MAX_STEP_SCALED &&
         std::llabs(dy - correction.deltaYScaled) <=
             BASE_ECEF_CORRECTION_MAX_STEP_SCALED &&
         std::llabs(dz - correction.deltaZScaled) <=
             BASE_ECEF_CORRECTION_MAX_STEP_SCALED);
    if (fixQuality == 4 && correction.referenceValid &&
        absoluteValid && stepValid) {
        correction.deltaXScaled = dx;
        correction.deltaYScaled = dy;
        correction.deltaZScaled = dz;
        if (correction.stableSamples < UINT8_MAX) {
            ++correction.stableSamples;
        }
        correction.correctionValid =
            correction.stableSamples >=
            BASE_ECEF_CORRECTION_WARMUP_SAMPLES;
    } else {
        correction.stableSamples = 0;
        correction.correctionValid = false;
    }
    const BaseEcefCorrectionSnapshot logSnapshot = correction;
    portEXIT_CRITICAL(&stateMux);

    Serial.printf("[BASE][CORRECTION] gnss_ms=%lu fix=%u valid=%u samples=%u "
                  "observed=(%.4f,%.4f,%.4f) delta=(%.4f,%.4f,%.4f)\n",
                  static_cast<unsigned long>(gnssTimeMs),
                  static_cast<unsigned>(fixQuality),
                  logSnapshot.correctionValid ? 1U : 0U,
                  static_cast<unsigned>(logSnapshot.stableSamples),
                  static_cast<double>(xScaled) / RTCM_ESPNOW_ECEF_SCALE,
                  static_cast<double>(yScaled) / RTCM_ESPNOW_ECEF_SCALE,
                  static_cast<double>(zScaled) / RTCM_ESPNOW_ECEF_SCALE,
                  static_cast<double>(dx) / RTCM_ESPNOW_ECEF_SCALE,
                  static_cast<double>(dy) / RTCM_ESPNOW_ECEF_SCALE,
                  static_cast<double>(dz) / RTCM_ESPNOW_ECEF_SCALE);
}

} // namespace

bool baseGnssRoleSetup()
{
    uartMutex = xSemaphoreCreateMutex();
    localCommandQueue = xQueueCreate(BASE_GNSS_LOCAL_COMMAND_QUEUE_LENGTH,
                                     sizeof(LocalCoordinateRequest));
    localResultQueue = xQueueCreate(BASE_GNSS_LOCAL_RESULT_QUEUE_LENGTH,
                                    sizeof(BaseLocalCoordinateResult));
    nmeaLine.reserve(256);
    if (uartMutex == nullptr || localCommandQueue == nullptr ||
        localResultQueue == nullptr) {
        return false;
    }

    SavedBaseCoordinate saved{};
    const bool hasSavedCoordinate = loadSavedCoordinate(saved);
    const bool configured = hasSavedCoordinate
                                ? writeBaseConfiguration(true,
                                                         saved.xScaled,
                                                         saved.yScaled,
                                                         saved.zScaled,
                                                         false)
                                : writeBaseConfiguration(false, 0, 0, 0, false);
    if (hasSavedCoordinate) {
        portENTER_CRITICAL(&stateMux);
        correction.referenceValid = true;
        correction.referenceXScaled = saved.xScaled;
        correction.referenceYScaled = saved.yScaled;
        correction.referenceZScaled = saved.zScaled;
        portEXIT_CRITICAL(&stateMux);
        Serial.printf("[BASE][LOCAL_GNSS] Boot fixed ecef_m=(%.4f,%.4f,%.4f)\n",
                      static_cast<double>(saved.xScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(saved.yScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(saved.zScaled) /
                          RTCM_ESPNOW_ECEF_SCALE);
    } else {
        Serial.println("[BASE][LOCAL_GNSS] Boot survey-in (no saved coordinate)");
    }
    if (!configured) {
        return false;
    }
    return xTaskCreate(localCoordinateCommandTask,
                       "Local GNSS Cmd",
                       BASE_GNSS_LOCAL_COMMAND_TASK_STACK_BYTES,
                       nullptr,
                       1,
                       nullptr) == pdPASS;
}

void baseGnssRoleLoop()
{
    if (localCommandBusy ||
        (localCommandQueue != nullptr &&
         uxQueueMessagesWaiting(localCommandQueue) != 0)) {
        return;
    }
    const BaseRtcmSourceSnapshot source = getBaseRtcmSourceSnapshot();
    LocalGnssRole currentRole{};
    portENTER_CRITICAL(&stateMux);
    currentRole = role;
    portEXIT_CRITICAL(&stateMux);
    if (source.state == BaseRtcmSourceState::TempActive &&
        currentRole == LocalGnssRole::LocalBase) {
        switchLocalGnssToRover();
    } else if (source.state != BaseRtcmSourceState::TempActive &&
               currentRole == LocalGnssRole::Rover) {
        switchLocalGnssToBase();
    }
}

bool baseGnssRoleConsumesNmea()
{
    portENTER_CRITICAL(&stateMux);
    const bool consumes = role != LocalGnssRole::LocalBase;
    portEXIT_CRITICAL(&stateMux);
    return consumes;
}

void baseGnssRoleConsumeByte(uint8_t value)
{
    if (value == '\n') {
        nmeaLine.trim();
        uint32_t gnssTimeMs = 0;
        uint8_t fixQuality = 0;
        int64_t xScaled = 0;
        int64_t yScaled = 0;
        int64_t zScaled = 0;
        if (parseGga(nmeaLine,
                     gnssTimeMs,
                     fixQuality,
                     xScaled,
                     yScaled,
                     zScaled)) {
            updateCorrection(gnssTimeMs,
                             fixQuality,
                             xScaled,
                             yScaled,
                             zScaled);
        }
        nmeaLine = "";
    } else if (value != '\r' && value != '\0') {
        nmeaLine += static_cast<char>(value);
        if (nmeaLine.length() > 512) {
            nmeaLine = "";
        }
    }
}

void baseGnssRoleRecordLocalRtcm(const uint8_t* frame, size_t length)
{
    if (frame == nullptr || length < 27 ||
        ((static_cast<uint16_t>(frame[3]) << 4U) |
         (frame[4] >> 4U)) != 1006) {
        return;
    }
    const uint8_t* payload = frame + 3;
    const int64_t xScaled = getSignedBits(payload, 34, 38);
    const int64_t yScaled = getSignedBits(payload, 74, 38);
    const int64_t zScaled = getSignedBits(payload, 114, 38);
    if (!rtcmEspNowValidEcef(xScaled, yScaled, zScaled)) {
        return;
    }
    bool changed = false;
    portENTER_CRITICAL(&stateMux);
    changed = !correction.referenceValid ||
              correction.referenceXScaled != xScaled ||
              correction.referenceYScaled != yScaled ||
              correction.referenceZScaled != zScaled;
    correction.referenceValid = true;
    correction.referenceXScaled = xScaled;
    correction.referenceYScaled = yScaled;
    correction.referenceZScaled = zScaled;
    ++referenceSequence;
    portEXIT_CRITICAL(&stateMux);
    if (changed) {
        Serial.printf("[BASE][REFERENCE] RTCM1006 ecef_m=(%.4f,%.4f,%.4f)\n",
                      static_cast<double>(xScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(yScaled) /
                          RTCM_ESPNOW_ECEF_SCALE,
                      static_cast<double>(zScaled) /
                          RTCM_ESPNOW_ECEF_SCALE);
    }
}

bool baseGnssRoleInjectTempRtcm(const uint8_t* frame, size_t length)
{
    if (frame == nullptr || length == 0 || uartMutex == nullptr ||
        !baseGnssRoleConsumesNmea() ||
        xSemaphoreTake(uartMutex, 0) != pdTRUE) {
        return false;
    }
    const size_t written = Serial1.write(frame, length);
    xSemaphoreGive(uartMutex);
    return written == length;
}

bool baseGnssRoleHasReference()
{
    portENTER_CRITICAL(&stateMux);
    const bool valid = correction.referenceValid;
    portEXIT_CRITICAL(&stateMux);
    return valid;
}

BaseEcefCorrectionSnapshot getBaseEcefCorrectionSnapshot()
{
    portENTER_CRITICAL(&stateMux);
    BaseEcefCorrectionSnapshot snapshot = correction;
    portEXIT_CRITICAL(&stateMux);
    if (snapshot.correctionValid &&
        millis() - snapshot.observedAtMs >
            BASE_ECEF_CORRECTION_MAX_AGE_MS) {
        snapshot.correctionValid = false;
    }
    return snapshot;
}

bool baseGnssGeodeticToEcef(double latitudeDegrees,
                            double longitudeDegrees,
                            double ellipsoidHeightM,
                            int64_t& xScaled,
                            int64_t& yScaled,
                            int64_t& zScaled)
{
    if (latitudeDegrees < -90.0 || latitudeDegrees > 90.0 ||
        longitudeDegrees < -180.0 || longitudeDegrees > 180.0) {
        return false;
    }
    return geodeticToEcef(latitudeDegrees,
                          longitudeDegrees,
                          ellipsoidHeightM,
                          xScaled,
                          yScaled,
                          zScaled);
}

BaseLocalCoordinateQueueResult baseGnssQueueSetFixedCoordinates(
    uint32_t transactionId,
    int64_t xScaled,
    int64_t yScaled,
    int64_t zScaled)
{
    if (transactionId == 0 ||
        !rtcmEspNowValidEcef(xScaled, yScaled, zScaled)) {
        return BaseLocalCoordinateQueueResult::InvalidCoordinates;
    }
    if (localCommandQueue == nullptr) {
        return BaseLocalCoordinateQueueResult::NotReady;
    }
    if (getBaseRtcmSourceSnapshot().state !=
        BaseRtcmSourceState::LocalActive) {
        return BaseLocalCoordinateQueueResult::TempSourceActive;
    }
    LocalCoordinateRequest request{};
    request.transactionId = transactionId;
    request.command = BaseLocalCoordinateCommand::SetFixed;
    request.xScaled = xScaled;
    request.yScaled = yScaled;
    request.zScaled = zScaled;
    return xQueueSend(localCommandQueue, &request, 0) == pdTRUE
               ? BaseLocalCoordinateQueueResult::Queued
               : BaseLocalCoordinateQueueResult::Busy;
}

BaseLocalCoordinateQueueResult baseGnssQueueClearCoordinates(
    uint32_t transactionId)
{
    if (transactionId == 0) {
        return BaseLocalCoordinateQueueResult::InvalidCoordinates;
    }
    if (localCommandQueue == nullptr) {
        return BaseLocalCoordinateQueueResult::NotReady;
    }
    if (getBaseRtcmSourceSnapshot().state !=
        BaseRtcmSourceState::LocalActive) {
        return BaseLocalCoordinateQueueResult::TempSourceActive;
    }
    LocalCoordinateRequest request{};
    request.transactionId = transactionId;
    request.command = BaseLocalCoordinateCommand::ClearToSurvey;
    return xQueueSend(localCommandQueue, &request, 0) == pdTRUE
               ? BaseLocalCoordinateQueueResult::Queued
               : BaseLocalCoordinateQueueResult::Busy;
}

bool baseGnssPopLocalCoordinateResult(BaseLocalCoordinateResult& result)
{
    return localResultQueue != nullptr &&
           xQueueReceive(localResultQueue, &result, 0) == pdTRUE;
}

bool baseGnssLocalCoordinateCommandBusy()
{
    return localCommandBusy ||
           (localCommandQueue != nullptr &&
            uxQueueMessagesWaiting(localCommandQueue) != 0);
}

const char* baseLocalCoordinateQueueResultToString(
    BaseLocalCoordinateQueueResult result)
{
    switch (result) {
    case BaseLocalCoordinateQueueResult::Queued:
        return "queued";
    case BaseLocalCoordinateQueueResult::Busy:
        return "local_coordinate_command_busy";
    case BaseLocalCoordinateQueueResult::TempSourceActive:
        return "temporary_base_active";
    case BaseLocalCoordinateQueueResult::InvalidCoordinates:
        return "invalid_coordinates";
    case BaseLocalCoordinateQueueResult::NotReady:
        return "local_gnss_not_ready";
    }
    return "unknown";
}

const char* baseLocalCoordinateStatusToString(BaseLocalCoordinateStatus status)
{
    switch (status) {
    case BaseLocalCoordinateStatus::AppliedVerified:
        return "applied_verified";
    case BaseLocalCoordinateStatus::SurveyStarted:
        return "survey_started";
    case BaseLocalCoordinateStatus::VerifyTimeoutRolledBack:
        return "verify_timeout_rolled_back";
    case BaseLocalCoordinateStatus::RollbackFailed:
        return "rollback_failed";
    case BaseLocalCoordinateStatus::UartError:
        return "uart_error";
    case BaseLocalCoordinateStatus::StorageError:
        return "storage_error";
    }
    return "unknown";
}
