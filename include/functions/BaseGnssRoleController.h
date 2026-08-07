#ifndef BASE_GNSS_ROLE_CONTROLLER_H
#define BASE_GNSS_ROLE_CONTROLLER_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

struct BaseEcefCorrectionSnapshot {
    bool referenceValid = false;
    bool observationValid = false;
    bool correctionValid = false;
    int64_t referenceXScaled = 0;
    int64_t referenceYScaled = 0;
    int64_t referenceZScaled = 0;
    int64_t observedXScaled = 0;
    int64_t observedYScaled = 0;
    int64_t observedZScaled = 0;
    int64_t deltaXScaled = 0;
    int64_t deltaYScaled = 0;
    int64_t deltaZScaled = 0;
    uint32_t gnssTimeMsOfDay = 0;
    uint32_t observedAtMs = 0;
    uint8_t fixQuality = 0;
    uint8_t stableSamples = 0;
};

enum class BaseLocalCoordinateCommand : uint8_t {
    SetFixed = 1,
    ClearToSurvey = 2,
};

enum class BaseLocalCoordinateQueueResult : uint8_t {
    Queued = 0,
    Busy,
    TempSourceActive,
    InvalidCoordinates,
    NotReady,
};

enum class BaseLocalCoordinateStatus : uint8_t {
    AppliedVerified = 1,
    SurveyStarted,
    VerifyTimeoutRolledBack,
    RollbackFailed,
    UartError,
    StorageError,
};

struct BaseLocalCoordinateResult {
    uint32_t transactionId = 0;
    BaseLocalCoordinateCommand command = BaseLocalCoordinateCommand::SetFixed;
    BaseLocalCoordinateStatus status = BaseLocalCoordinateStatus::UartError;
    int64_t ecefXScaled = 0;
    int64_t ecefYScaled = 0;
    int64_t ecefZScaled = 0;
    uint32_t completedAtMs = 0;
};

bool baseGnssRoleSetup();
void baseGnssRoleLoop();
bool baseGnssRoleConsumesNmea();
void baseGnssRoleConsumeByte(uint8_t value);
void baseGnssRoleRecordLocalRtcm(const uint8_t* frame, size_t length);
bool baseGnssRoleInjectTempRtcm(const uint8_t* frame, size_t length);
bool baseGnssRoleHasReference();
BaseEcefCorrectionSnapshot getBaseEcefCorrectionSnapshot();
bool baseGnssGeodeticToEcef(double latitudeDegrees,
                            double longitudeDegrees,
                            double ellipsoidHeightM,
                            int64_t& xScaled,
                            int64_t& yScaled,
                            int64_t& zScaled);
BaseLocalCoordinateQueueResult baseGnssQueueSetFixedCoordinates(
    uint32_t transactionId,
    int64_t xScaled,
    int64_t yScaled,
    int64_t zScaled);
BaseLocalCoordinateQueueResult baseGnssQueueClearCoordinates(
    uint32_t transactionId);
bool baseGnssPopLocalCoordinateResult(BaseLocalCoordinateResult& result);
bool baseGnssLocalCoordinateCommandBusy();
const char* baseLocalCoordinateQueueResultToString(
    BaseLocalCoordinateQueueResult result);
const char* baseLocalCoordinateStatusToString(BaseLocalCoordinateStatus status);

#endif
