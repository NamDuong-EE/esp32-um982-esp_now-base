#include "functions/Nmea_Receiver.h"
#include "HardwareSerial.h"

#ifdef NATIVE_BUILD
#include <ArduinoFake.h>
#endif

String receiveNmeaFromGnss() {
    #ifdef NATIVE_BUILD
    auto& nmeaIn = Serial;
    #else
    auto& nmeaIn = Serial1;
    #endif
    String nmeaData = nmeaIn.readString();
    nmeaIn.println("[NMEA over LoRA] Da nhan du lieu NMEA tu mach RTK:");
    nmeaIn.println(nmeaData);
    nmeaIn.println();
    return nmeaData;
}
