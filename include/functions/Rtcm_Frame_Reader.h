#ifndef RTCM_FRAME_READER_H
#define RTCM_FRAME_READER_H

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "RtcmEspNowProtocol.h"

enum class RtcmReadResult {
    None,
    FrameValid,
    CrcError,
    FrameTooLarge
};

void resetRtcmFrameReader();
uint32_t rtcmCrc24q(const uint8_t* data, size_t length);
RtcmReadResult readRtcmFrame(Stream& input, uint8_t* frame, size_t capacity, size_t& frameLength);

#endif // RTCM_FRAME_READER_H
