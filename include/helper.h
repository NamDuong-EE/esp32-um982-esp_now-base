#ifndef HELPER_H
#define HELPER_H

#include "Top_Lvl_Config.h"
#include "Prog_Config.h"
#include <Arduino.h>

#if CONNECT_USING_WIFI
#include "hardware/Wifi_handler.h"
#endif
#if CONNECT_USING_4G
#include "hardware/Sim_handler.h"
extern TinyGsm modem;
#endif

#include "functions/MQTT_Manager.h"

#if NMEA_COMMUNICATION_PROTOCOL == TCP_IP
#include "functions/NTRIP_Handler_IP.h"
#else
#include "hardware/Lora_handler.h"
#include "functions/RTCM_Receiver.h"
#endif

#include "functions/NMEA_Parser.h"
#include "DataStructs.h"

// ================= ĐỊNH NGHĨA CÁC BIẾN TOÀN CỤC =================
extern String latestGGA;
extern String targetGGA;
extern bool mqttHealthMode;

// ================= ĐỊNH NGHĨA CÁC HÀM =================
String formDeviceHealthString();
int publishGGA(String& rtcmBuffer);

#endif