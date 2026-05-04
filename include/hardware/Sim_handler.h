#ifndef SIM_HANDLER_H
#define SIM_HANDLER_H

// xác định loại modem
#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console
#define SerialMon Serial

// Set serial for AT commands
#define SerialAT Serial2

// See all AT commands
// #define DUMP_AT_COMMANDS
#define TINY_GSM_DEBUG SerialMon

// #define MODEM_RESET_PIN 4

#ifdef NATIVE_BUILD
#include <Mocks/TinyGsmClient.h>
#else
#include <TinyGsmClient.h>
#endif
#include "Top_Lvl_Config.h"

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
extern StreamDebugger debugger;
extern TinyGsm        modem;
#else
extern TinyGsm        modem;
#endif

bool startSIM();
bool connectGSM();

#endif