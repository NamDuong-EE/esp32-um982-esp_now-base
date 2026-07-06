#include "functions/RTCM_Receiver.h"
#include "HardwareSerial.h"

String receiveRtcmFromGnss() {
    #ifdef PROGRAM_TEST
    String rtcmData = "THIS IS NOT A REAL RTCM DATA. THIS IS A TEST STRING FOR UNIT TESTING PURPOSES.";
    #else
    String rtcmData = Serial1.readString();
    #endif
    if (!rtcmData.isEmpty()) {
        Serial1.println("[UM980] Da nhan du lieu RTCM tu mach RTK:");
        Serial1.println("[UM980] " + rtcmData);
    } else {
        Serial1.println("[UM980] Khong co du lieu RTCM hop le.");
    }
    Serial1.println();
    return rtcmData;
}
