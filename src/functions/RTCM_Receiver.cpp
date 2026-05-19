#include "functions/RTCM_Receiver.h"
#include "HardwareSerial.h"

String receiveRtcmFromGnss() {
    String rtcmData = Serial1.readString();
    if (!rtcmData.isEmpty()) {
        Serial1.println("[UM980] Da nhan du lieu RTCM tu mach RTK:");
        Serial1.println("[UM980] " + rtcmData);
    } else {
        Serial1.println("[UM980] Khong co du lieu RTCM hop le.");
    }
    Serial1.println();
    return rtcmData;
}
