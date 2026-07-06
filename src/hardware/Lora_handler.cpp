#if RTCM_COMMUNICATION_PROTOCOL == 1
#include "hardware/Lora_handler.h"

bool lora_idle;
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

int loraSend(char* txData, int length)
{
	if(!lora_idle)
	{
        Radio.IrqProcess();
        return 1;
    }

    if (length > BUFFER_SIZE - 12) {
        Serial.println("[LoRa Send] Do dai du lieu vuot qua BUFFER_SIZE, khong the gui!");
        Radio.IrqProcess();
        return 2;
    }

    if (length <= 0) {
        Serial.println("[LoRa Send] Do dai du lieu khong hop le, khong the gui!");
        Radio.IrqProcess();
        return 3;
    }

    // else
    txNumber += 0.01;

    #if PROGRAM_DEBUG

    Serial.printf("[LoRa Send] Chuan bi gui du lieu co do dai: %d byte.\r\n", length);

    Serial.println("[LoRa Send] Noi dung duoc in ra theo hexa:");

    for (int i = 0; i < length; i++) {
        Serial.printf("%02X ", static_cast<uint8_t>(txData[i]));

        if ((i + 1) % 16 == 0) {
            Serial.println();
        }
    }

    #endif // PROGRAM_DEBUG

    Serial.println();

    Radio.Send( (uint8_t *)txData, (uint8_t)length );
    
    Serial.printf("[LoRa Send] Da gui %d byte.\r\n", length);

    Serial.printf("[LoRa Send] Noi dung da gui duoc in ra (dang text): %.*s\r\n", length, txData);

    Serial.println("[LoRa Send] Noi dung duoc in ra theo hexa: ");

    for (int i = 0; i < length; i++) {
        Serial.printf("%02X ", static_cast<uint8_t>(txData[i]));

        if ((i + 1) % 16 == 0) {
            Serial.println();
        }
    }

    Serial.println();

    lora_idle = false;
    Radio.IrqProcess( );
    return 0;
}

void OnTxDone( void )
{
	Serial.println("[LoRa Handler] Hoan thanh Tx......");
	lora_idle = true;
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("[LoRa Handler] Het thoi gian cho TX......");
    lora_idle = true;
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
}
#endif