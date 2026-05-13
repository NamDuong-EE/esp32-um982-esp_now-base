#include "hardware/Lora_handler.h"
#include "functions/RTCM_Receiver.h"

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


extern String rtcmBuffer;

void prepareTxFrame(uint8_t appPort)
{
    rtcmBuffer.getBytes(appData, LORAWAN_APP_DATA_MAX_SIZE);
    appDataSize = rtcmBuffer.length() > LORAWAN_APP_DATA_MAX_SIZE ? LORAWAN_APP_DATA_MAX_SIZE : (uint8_t)rtcmBuffer.length();
    printf("[LORAWAN] Du lieu RTCM duoc gui: %s\n", rtcmBuffer.c_str());
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
    #ifdef PROGRAM_DEBUG
            Serial.println("[LORAWAN] Khoi tao LoRaWAN...");
    #endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            LoRaWAN.setDefaultDR(3);
            break;
        }
        case DEVICE_STATE_JOIN:
        {
    #ifdef PROGRAM_DEBUG
            Serial.println("[LORAWAN] Tham gia mang LoRaWAN...");
    #endif
            LoRaWAN.join();
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_SEND:
		{
    #ifdef PROGRAM_DEBUG
            Serial.println("[LORAWAN] Gui du lieu...");
    #endif
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
        case DEVICE_STATE_CYCLE:
        {
    #ifdef PROGRAM_DEBUG
            Serial.println("[LORAWAN] Vao chu ky...");
    #endif
            // Schedule next packet transmission
            txDutyCycleTime = appTxDutyCycle;
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
    #ifdef PROGRAM_DEBUG
            Serial.println("[LORAWAN] Vao che do ngu...");
    #endif
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