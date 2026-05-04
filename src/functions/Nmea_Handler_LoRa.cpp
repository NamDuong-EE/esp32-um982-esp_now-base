#if (NMEA_COMMUNICATION_PROTOCOL == LORA_SERIAL || NMEA_COMMUNICATION_PROTOCOL == 1)
#define NTRIP_HANDLER_LORA_CODE

#include "functions/Nmea_Handler_LoRa.h"
#include <LoRaWan_APP.h>

#ifdef NATIVE_BUILD
#include <ArduinoFake.h>
#endif

int pushNmeaLoRaToGnss(McpsIndication_t *mcpsIndication) {
    #ifdef NATIVE_BUILD
    auto& nmeaOut = Serial;
    #else
    auto& nmeaOut = Serial1;
    #endif
    if (mcpsIndication->BufferSize > 0)
    {
        if (mcpsIndication->Buffer == nullptr) {
            nmeaOut.println("[NMEA over LoRA] Loi: Buffer rong!");
            return -2;
        }

        nmeaOut.write(mcpsIndication->Buffer, mcpsIndication->BufferSize);
        nmeaOut.println("[NMEA over LoRA] Da nhan duoc NMEA qua LoRA, da gui den mach RTK!");
        nmeaOut.println("[NMEA over LoRA] Kich thuoc du lieu: " + String(mcpsIndication->BufferSize) + " bytes");
        nmeaOut.println("[NMEA over LoRA] Noi dung du lieu: " + String((char*)mcpsIndication->Buffer));
        return 0;
    }
    nmeaOut.println("[NMEA over LoRA] Loi: Buffer co do dai bang 0!");
    return -1;
}

#endif