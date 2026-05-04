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

void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
    String slot;
    switch (mcpsIndication->RxSlot)
    {
        case RX_SLOT_WIN_1:
            slot = "RXWIN1";
            break;
        case RX_SLOT_WIN_2:
            slot = "RXWIN2";
            break;
        case RX_SLOT_WIN_CLASS_C:
            slot = "CLASS_C";
            break;
        case RX_SLOT_WIN_PING_SLOT:
            slot = "PING_SLOT";
            break;
        case RX_SLOT_WIN_MULTICAST_SLOT:
            slot = "MULTICAST_SLOT";
            break;
        default:
            slot = "UNKNOWN";
    }
    Serial.println("+REV DATA:" + slot + 
        ",RXSIZE " + String(mcpsIndication->BufferSize) + ",PORT " + 
        String(mcpsIndication->Port)
    );
    String bufferHex = "";
    if (mcpsIndication->Buffer != nullptr && mcpsIndication->BufferSize > 0)
    {
        for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
        {
            if (mcpsIndication->Buffer[i] < 0x10)
            {
                bufferHex += "0";
            }
            bufferHex += String(mcpsIndication->Buffer[i], HEX);
        }
        bufferHex.toUpperCase();
    }
    Serial.println("Buffer: " + bufferHex);
    Serial.println();
    #if (LoraWan_RGB == 1)
        uint32_t color = mcpsIndication->Buffer[0] << 16 | mcpsIndication->Buffer[1] << 8 | mcpsIndication->Buffer[2];
        turnOnRGB(color, 5000);
        turnOffRGB();
    #endif
    #ifndef UNIT_TEST
    pushNmeaLoRaToGnss(mcpsIndication);
    #endif
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