#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "Prog_Config.h"

inline constexpr uint8_t BUFFER_SIZE = 255; // Define the payload size here

void OnTxDone( void );
void OnTxTimeout( void );
int loraSend(char* txData, int length);
int loraSetup();

inline void lora_packet_process(String& packet) {
    int txLength = 0;
    txLength = packet.length() > BUFFER_SIZE - 12 ? BUFFER_SIZE - 12 : packet.length() + 1;
    auto* rtcmCharArray = new char[txLength];

    for (int i = 0; i < txLength; i++) {
        rtcmCharArray[i] = packet[i];
    }

    int result = loraSend(rtcmCharArray, txLength);
    Serial.printf("[LORA HANDLER] Da truyen du lieu RTCM qua LoRa. Do dai: %d byte\n", txLength);
    delete[] rtcmCharArray;

    if (result == 0)
        packet = packet.substring(txLength); // Cắt bỏ phần đã gửi, giữ lại phần chưa gửi (nếu có)
}
#endif