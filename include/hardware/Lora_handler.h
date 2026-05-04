#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#ifndef LORAWAN_CLASS
#define LORAWAN_CLASS CLASS_A
#endif

#ifndef ACTIVE_REGION
#define ACTIVE_REGION LORAMAC_REGION_AS923
#endif

#ifndef LORAWAN_OTA
#define LORAWAN_OTA true
#endif

#ifndef LORAWAN_ADR
#define LORAWAN_ADR true
#endif

#ifndef LORAWAN_NET_RESERVE
#define LORAWAN_NET_RESERVE false
#endif

#ifndef TX_CONFIRMED
#define TX_CONFIRMED false
#endif

#include <LoRaWan_APP.h>

// ================= KHAI BÁO CÁC BIẾN TOÀN CỤC =================

extern DeviceClass_t loraWanClass;
extern LoRaMacRegion_t loraWanRegion;

extern uint8_t devEui[];
extern uint8_t appEui[];
extern uint8_t appKey[];
extern uint8_t nwkSKey[];
extern uint8_t appSKey[];
extern uint32_t devAddr;
extern uint16_t userChannelsMask[6];
extern uint8_t appPort;
extern uint8_t confirmedNbTrials;

extern bool overTheAirActivation;
extern bool loraWanAdr;
extern bool keepNet;
extern bool isTxConfirmed;

// ================= KHAI BÁO HÀM =================
extern void downLinkDataHandle(McpsIndication_t *mcpsIndication);
int loraWanMain();
#endif