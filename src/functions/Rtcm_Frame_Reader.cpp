#include "functions/Rtcm_Frame_Reader.h"

#include "Prog_Config.h"

namespace {
enum class ReaderState {
    WaitPreamble,
    ReadLengthHigh,
    ReadLengthLow,
    ReadBody
};

ReaderState state = ReaderState::WaitPreamble;
size_t writeIndex = 0;
size_t expectedLength = 0;
uint32_t rawByteOffset = 0;
uint8_t rawLineByteCount = 0;

void resetState()
{
    state = ReaderState::WaitPreamble;
    writeIndex = 0;
    expectedLength = 0;
}

void dumpRawUartByte(uint8_t byte)
{
    if (!DEBUG_GNSS_UART_RAW_DUMP) {
        return;
    }

    if (rawLineByteCount == 0) {
        Serial.printf("[BASE][GNSS][UART_RAW] %08lu: ", static_cast<unsigned long>(rawByteOffset));
    }

    Serial.printf("%02X", byte);
    ++rawByteOffset;
    ++rawLineByteCount;

    if (rawLineByteCount >= DEBUG_RTCM_HEX_BYTES_PER_LINE) {
        Serial.println();
        rawLineByteCount = 0;
    } else {
        Serial.print(' ');
    }
}
}

void resetRtcmFrameReader()
{
    resetState();
}

void flushRtcmDebugLine()
{
    if (DEBUG_GNSS_UART_RAW_DUMP && rawLineByteCount != 0) {
        Serial.println();
        rawLineByteCount = 0;
    }
}

uint32_t getRtcmRawByteCount()
{
    return rawByteOffset;
}

uint32_t rtcmCrc24q(const uint8_t* data, size_t length)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if ((crc & 0x1000000U) != 0) {
                crc ^= 0x1864CFBU;
            }
        }
        crc &= 0xFFFFFFU;
    }
    return crc & 0xFFFFFFU;
}

RtcmReadResult readRtcmFrame(Stream& input, uint8_t* frame, size_t capacity, size_t& frameLength)
{
    frameLength = 0;

    while (input.available() > 0) {
        const int value = input.read();
        if (value < 0) {
            break;
        }

        const uint8_t byte = static_cast<uint8_t>(value);
        dumpRawUartByte(byte);

        switch (state) {
        case ReaderState::WaitPreamble:
            if (byte == 0xD3) {
                if (capacity < 3) {
                    resetState();
                    return RtcmReadResult::FrameTooLarge;
                }
                frame[0] = byte;
                writeIndex = 1;
                state = ReaderState::ReadLengthHigh;
            }
            break;

        case ReaderState::ReadLengthHigh:
            frame[writeIndex++] = byte;
            if ((byte & 0xFC) != 0) {
                resetState();
                break;
            }
            state = ReaderState::ReadLengthLow;
            break;

        case ReaderState::ReadLengthLow: {
            frame[writeIndex++] = byte;
            const size_t payloadLength =
                (static_cast<size_t>(frame[1] & 0x03) << 8) | static_cast<size_t>(frame[2]);
            expectedLength = 3 + payloadLength + 3;

            if (expectedLength > RTCM_ESPNOW_MAX_FRAME_LENGTH || expectedLength > capacity) {
                resetState();
                return RtcmReadResult::FrameTooLarge;
            }

            state = ReaderState::ReadBody;
            break;
        }

        case ReaderState::ReadBody:
            frame[writeIndex++] = byte;
            if (writeIndex >= expectedLength) {
                const uint32_t calculated = rtcmCrc24q(frame, expectedLength - 3);
                const uint32_t received =
                    (static_cast<uint32_t>(frame[expectedLength - 3]) << 16) |
                    (static_cast<uint32_t>(frame[expectedLength - 2]) << 8) |
                    static_cast<uint32_t>(frame[expectedLength - 1]);

                frameLength = expectedLength;
                resetState();
                return calculated == received ? RtcmReadResult::FrameValid : RtcmReadResult::CrcError;
            }
            break;
        }
    }

    return RtcmReadResult::None;
}
