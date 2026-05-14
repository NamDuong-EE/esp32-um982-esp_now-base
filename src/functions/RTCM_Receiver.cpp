#include "functions/RTCM_Receiver.h"
#include "HardwareSerial.h"

#ifdef NATIVE_BUILD
#include <ArduinoFake.h>
#endif

String receiveRtcmFromGnss() {
    #ifdef NATIVE_BUILD
    auto& rtcmIn = Serial;
    #else
    auto& rtcmIn = Serial1;
    #endif
    String rtcmData = rtcmIn.readString();
    if (!rtcmData.isEmpty()) {
        rtcmIn.println("[UM980] Da nhan du lieu RTCM tu mach RTK:");
        rtcmIn.println("[UM980] " + rtcmData);
    } else {
        rtcmIn.println("[UM980] Khong co du lieu RTCM hop le.");
    }
    rtcmIn.println();
    return rtcmData;
}
