#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

/* Heltec Automation send communication test example
 *
 * Function:
 * 1. Send data from a esp32 device over hardware 
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Prog_Config.h"

inline constexpr char RX_TIMEOUT_VALUE = 1000;
inline constexpr char BUFFER_SIZE = 256; // Define the payload size here

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void loraSend(char* txData);
int loraSetup();
#endif