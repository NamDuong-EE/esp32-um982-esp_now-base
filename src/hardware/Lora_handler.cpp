#if NMEA_COMMUNICATION_PROTOCOL == 1
#include "hardware/Lora_handler.h"
#include "functions/RTCM_Receiver.h"

static bool lora_idle;

static double txNumber;

int loraSetup( void ) {
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
	
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, false, 0, LORA_IQ_INVERSION_ON, LORA_TX_TIMEOUT );
    lora_idle = true;
    return 0;
}

void loraSend(char* txData)
{
	if(lora_idle == true)
	{
		txNumber += 0.01;

        Serial.printf("[LoRa] Noi dung \"%s\" , do dai: %d\r\n", txData, strlen(txData));

		Radio.Send( (uint8_t *)txData, 
            strlen(txData) > BUFFER_SIZE ? BUFFER_SIZE : (uint8_t)strlen(txData) );
        
        lora_idle = false;
	}
    Radio.IrqProcess( );
}

void OnTxDone( void )
{
	Serial.println("[LoRa] Hoan thanh Tx......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("[LoRa] Het thoi gian cho TX......");
    lora_idle = true;
}
#endif