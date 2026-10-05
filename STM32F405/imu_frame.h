#pragma once

#include <cstddef>
#include <cstdint>

inline bool ImuFrameValid(const uint8_t* frame, size_t size)
{
    if (frame == nullptr || size < 82 || frame[0] != 0x5A || frame[1] != 0xA5 || frame[6] != 0x91)
    {
        return false;
    }
    const uint16_t length = uint16_t(frame[2]) | (uint16_t(frame[3]) << 8);
    if (length != 76 || size < size_t(length) + 6)
    {
        return false;
    }
    uint16_t crc = 0;
    for (uint16_t i = 0; i < length + 4; ++i)
    {
        crc ^= uint16_t(frame[i < 4 ? i : i + 2]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
        {
            crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
        }
    }
    return crc == (uint16_t(frame[4]) | (uint16_t(frame[5]) << 8));
}
