/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Stick commands while DISARMED (Betaflight's, for the field: no app needed). Held for 1 s; fires once, and again only after
// the sticks left the pattern.
//   GYRO_CAL   throttle low,  yaw left,   pitch low (stick back), roll centre
//   MAG_CAL    throttle high, yaw centre, pitch low (stick back), roll centre      -> the automatic compass calibration
// low < 1100 us, high > 1900 us, centre 1300 .. 1700 us. Never with the link lost, never armed.
// Portable (test_app).
// ============================================================================
namespace app {

class StickCommands {
public:
    enum Cmd : uint8_t { NONE = 0, GYRO_CAL = 1, MAG_CAL = 2 };
    static constexpr uint32_t kHoldMs = 1000;
    Cmd update(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t throttle, bool armed, bool link_ok, uint32_t now_ms) {
        const Cmd now = (armed || !link_ok) ? NONE : pattern(roll, pitch, yaw, throttle);
        if (now != cur_) { cur_ = now; since_ = now_ms; fired_ = false; return NONE; }
        if (now == NONE || fired_ || (uint32_t)(now_ms - since_) < kHoldMs) return NONE;
        fired_ = true; return now;
    }
    static Cmd pattern(uint16_t roll, uint16_t pitch, uint16_t yaw, uint16_t thr) {
        auto lo = [](uint16_t v) { return v < 1100; }; auto hi = [](uint16_t v) { return v > 1900; }; auto mid = [](uint16_t v) { return v >= 1300 && v <= 1700; };
        if (!mid(roll) || !lo(pitch)) return NONE;
        if (lo(thr) && lo(yaw)) return GYRO_CAL;
        if (hi(thr) && mid(yaw)) return MAG_CAL;
        return NONE;
    }
private:
    Cmd cur_ = NONE; uint32_t since_ = 0; bool fired_ = false;
};

} // namespace app
