#if NMEA_COMMUNICATION_PROTOCOL == 1
#include "hardware/Lora_handler.h"

static bool lora_idle;

static double txNumber;

static RadioEvents_t RadioEvents;

int loraSetup( void ) {
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

void loraSend(char* txData, int length)
{
	if(lora_idle == true)
	{
		txNumber += 0.01;

        Serial.printf("[LoRa] Chuan bi gui du lieu co do dai: %d byte.\r\n", length);

        Serial.println("[LoRa] Noi dung duoc in ra theo hexa:");

        for (int i = 0; i < length; i++) {
            Serial.printf("%02X ", static_cast<uint8_t>(txData[i]));

            if ((i + 1) % 16 == 0) {
                Serial.println();
            }
        }

        Serial.println();

        if (length > BUFFER_SIZE - 12) {
            Serial.println("[LoRa] Do dai du lieu vuot qua BUFFER_SIZE, se phan doan thanh nhieu goi!");
        }

        while (length > BUFFER_SIZE - 12) {
            Radio.Send( (uint8_t *)txData, BUFFER_SIZE - 12 );
            txData += BUFFER_SIZE - 12;
            length -= BUFFER_SIZE - 12;
            Serial.printf("[LoRa] Da gui %d byte, con lai: %d\r\n", BUFFER_SIZE - 12, length);

            Serial.println("[LoRa] Noi dung doan duoc in ra theo hexa: ");

            for (int i = 0; i < BUFFER_SIZE - 12; i++) {
                Serial.printf("%02X ", static_cast<uint8_t>(txData[i]));

                if ((i + 1) % 16 == 0) {
                    Serial.println();
                }
            }

            Serial.println();
        }

		Radio.Send( (uint8_t *)txData, 
            length > BUFFER_SIZE - 12 ? BUFFER_SIZE - 12 : (uint8_t)length );
        
        Serial.printf("[LoRa] Da gui %d byte cuoi cung.\r\n", length);

        Serial.println("[LoRa] Noi dung duoc in ra theo hexa: ");

        for (int i = 0; i < length; i++) {
            Serial.printf("%02X ", static_cast<uint8_t>(txData[i]));

            if ((i + 1) % 16 == 0) {
                Serial.println();
            }
        }

        Serial.println();
        
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