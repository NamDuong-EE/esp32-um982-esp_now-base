#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Prog_Config.h"

inline constexpr uint8_t BUFFER_SIZE = 255; // Define the payload size here

void OnTxDone( void );
void OnTxTimeout( void );
void loraSend(char* txData, int length);
int loraSetup();
#endif