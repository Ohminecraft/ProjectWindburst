/*
 * This file is part of Windburst Project
 */

#pragma once
#include <stdint.h>

// ============================================================================
// CAN bit timing from a controller's clock (hal/can/stm32_can.h): the prescaler and the time quanta of one bit - sync (1) +
// seg1 + seg2 - with 8 .. 25 quanta a bit (the most that fit first: the finest resolution), the sample point at 87.5 %
// (CiA 301's recommendation for DroneCAN's 1 Mbit/s and below) and the bit rate within 0.5 % (CAN tolerates about 1 %).
// Portable (test_can).
// ============================================================================
namespace cantiming {

struct Timing { uint32_t presc = 0, seg1 = 0, seg2 = 0; };

inline bool compute(uint32_t clk, uint32_t bitrate, uint32_t max_presc, uint32_t max_seg1, uint32_t max_seg2, Timing& t) {
    if (!clk || !bitrate) return false;
    for (uint32_t tq = 25; tq >= 8; tq--) {
        const uint64_t per = (uint64_t)bitrate * tq;
        const uint32_t presc = (uint32_t)(((uint64_t)clk + per / 2) / per);
        if (presc < 1 || presc > max_presc) continue;
        const uint32_t real = (uint32_t)((uint64_t)clk / ((uint64_t)presc * tq)), err = real > bitrate ? real - bitrate : bitrate - real;
        if ((uint64_t)err * 200u > bitrate) continue;
        const uint32_t seg1 = (tq * 7 + 4) / 8 - 1, seg2 = tq - 1 - seg1;
        if (seg1 < 1 || seg1 > max_seg1 || seg2 < 1 || seg2 > max_seg2) continue;
        t.presc = presc; t.seg1 = seg1; t.seg2 = seg2;
        return true;
    }
    return false;
}

} // namespace cantiming
