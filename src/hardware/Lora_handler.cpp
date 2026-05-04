#include "hardware/Lora_handler.h"
#include "functions/Nmea_Handler_LoRa.h"

DeviceClass_t loraWanClass = LORAWAN_CLASS;
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

uint8_t devEui[] = {0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02};
uint8_t appEui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t appKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
                    0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01};
uint8_t nwkSKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
                     0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01};
uint8_t appSKey[] = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
                     0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01};
uint32_t devAddr = (uint32_t)0x26011BDA;
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

bool overTheAirActivation = LORAWAN_OTA;
bool loraWanAdr = LORAWAN_ADR;
bool keepNet = LORAWAN_NET_RESERVE;
bool isTxConfirmed = TX_CONFIRMED;

uint32_t appTxDutyCycle = 1000; // 1000ms = 1s

void prepareTxFrame(uint8_t appPort)
{
    String nmeaData = receiveNmeaFromGnss();
    if (nmeaData.length() > 0)
    {
        nmeaData.getBytes(appData, LORAWAN_APP_DATA_MAX_SIZE);
        appDataSize = nmeaData.length() > LORAWAN_APP_DATA_MAX_SIZE ? LORAWAN_APP_DATA_MAX_SIZE : nmeaData.length();
    }
}

int loraWanMain()
{
    switch (deviceState)
    {
        case DEVICE_STATE_INIT:
        {
    #if (LORAWAN_DEVEUI_AUTO)
            LoRaWAN.generateDeveuiByChipID();
    #endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            LoRaWAN.setDefaultDR(3);
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
        case DEVICE_STATE_CYCLE:
        {
            // Schedule next packet transmission
            txDutyCycleTime = appTxDutyCycle;
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            LoRaWAN.sleep(loraWanClass);
            break;
        }
        default:
        {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
    return 0;
}