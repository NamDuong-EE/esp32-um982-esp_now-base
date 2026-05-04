#ifndef NTRIP_HANDLER_LORA_H
#define NTRIP_HANDLER_LORA_H

#include "Top_Lvl_Config.h"
#include "Prog_Config.h"
#include <LoRaWan_APP.h>

int pushNmeaLoRaToGnss(McpsIndication_t *mcpsIndication);

#endif