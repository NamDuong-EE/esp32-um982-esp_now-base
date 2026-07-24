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

bool baseGnssRoleSetup();
void baseGnssRoleLoop();
bool baseGnssRoleConsumesNmea();
void baseGnssRoleConsumeByte(uint8_t value);
void baseGnssRoleRecordLocalRtcm(const uint8_t* frame, size_t length);
bool baseGnssRoleInjectTempRtcm(const uint8_t* frame, size_t length);
bool baseGnssRoleHasReference();
BaseEcefCorrectionSnapshot getBaseEcefCorrectionSnapshot();

#endif
