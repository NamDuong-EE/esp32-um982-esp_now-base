#if (NMEA_COMMUNICATION_PROTOCOL == LORA_SERIAL || NMEA_COMMUNICATION_PROTOCOL == 1)
#define NTRIP_HANDLER_LORA_CODE

#include "functions/Nmea_Handler_LoRa.h"
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

#endif