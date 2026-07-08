#ifndef BASE_ESPNOW_SENDER_H
#define BASE_ESPNOW_SENDER_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

struct BaseEspnowStats {
    uint32_t framesSent = 0;
    uint32_t framesDropped = 0;
    uint32_t fragmentsSent = 0;
    uint32_t sendFailures = 0;
    uint32_t sendTimeouts = 0;
    uint32_t frameRetries = 0;
    uint32_t frameAckTimeouts = 0;
    uint32_t ackPacketsReceived = 0;
    uint32_t ackPacketsInvalid = 0;
    uint32_t frameDeadlineDrops = 0;
    uint32_t lastFrameSendMs = 0;
    uint32_t maxFrameSendMs = 0;
    uint32_t activeRoverCount = 0;
    uint32_t storedRoverCount = 0;
    uint32_t pairResponsesReceived = 0;
    uint32_t pairConfirmsSent = 0;
    uint32_t pairAuthFailures = 0;
    bool pairingActive = false;
};

bool setupEspNowBase();
void baseEspNowLoop();
bool baseEspNowSendRtcmFrame(const uint8_t* frame, size_t length);
BaseEspnowStats getBaseEspnowStats();
uint16_t getBaseEspNowStreamId();

#endif // BASE_ESPNOW_SENDER_H
