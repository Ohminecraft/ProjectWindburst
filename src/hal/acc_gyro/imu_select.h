/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"
#include <stdint.h>
#include "acc_gyro_regbus.h"

// ============================================================================
// Two IMUs (Betaflight gyro_to_use): which one is read, which one the control loop gets, and what happens when one fails.
//
//   FIRST  : the gyro and the accelerometer of IMU 1. IMU 2 is a stand-by: read every 8th cycle (its health is known, its
//            gyro bias gets calibrated), and taken in the cycle IMU 1 does not answer.
//   SECOND : the gyro of IMU 2, the accelerometer of IMU 1 (the one the 6-face calibration is made on). Both read every cycle.
//   BOTH   : the mean of both gyros (uncorrelated noise: about 1/sqrt(2) of one), the accelerometer of IMU 1. Both every cycle.
//
// A sample that one chip does not deliver is taken from the other one in that cycle; a chip that misses kLostAfter samples in a
// row is LOST: it is not read again while the other one works (no back and forth between two chips that disagree); only when
// nothing else is left is it tried again. Only when no chip delivers is the cycle
// lost (FlightApp: 20 in a row = emergency disarm, as with one IMU).
// The accelerometer of IMU 2 is used only when IMU 1 is lost: it has no 6-face calibration of its own (offset / scale 1).
// Pure logic: the reads are handed in (test_accgyro).
// ============================================================================
namespace imusel {

enum Use : uint8_t { FIRST = 0, SECOND = 1, BOTH = 2 };
enum Imu2State : uint8_t { NONE = 0, OK = 1, FAILED = 2, LOST = 3 };      // the schema's enum imu2_state: not configured / working / not found at boot / stopped answering
constexpr uint8_t kImu1 = 1, kImu2 = 2;                                   // gyro_src() bits

class Selector {
public:
    static constexpr uint16_t kLostAfter = 20;          // 5 ms at 4 kHz: the same rule as the single IMU
    static constexpr uint8_t kStandbyEvery = 8;

    // have1 / have2: the chip was found at boot
    void configure(Use use, bool have1, bool have2) {
        use_ = use > BOTH ? FIRST : use; have_[0] = have1; have_[1] = have2;
        lost_[0] = lost_[1] = false; streak_[0] = streak_[1] = 0; n_ = 0; src_ = 0; acc_src_ = 0; fallbacks_ = 0;
    }

    // One cycle. read1 / read2: bool(AgSample&) - read that chip (false = no sample). out: the sample for the control loop.
    template <class R1, class R2> WB_FAST bool read(R1&& read1, R2&& read2, AgSample& out) {
        // a lost chip is not read while the other one works; when nothing else is left it is tried again (and comes back when it answers)
        const bool ok_1 = have_[0] && !lost_[0], ok_2 = have_[1] && !lost_[1];
        const bool can1 = have_[0] && (ok_1 || !ok_2), can2 = have_[1] && (ok_2 || !ok_1);
        // which chips are read in this cycle
        bool want1 = can1, want2 = can2;
        if (use_ == FIRST && can1 && can2) want2 = (++n_ % kStandbyEvery) == 0;
        AgSample a, b; bool ok1 = false, ok2 = false;
        if (want1) ok1 = note(0, read1(a));
        if (want2) ok2 = note(1, read2(b));
        if (want1 && !ok1 && can2 && !want2) { want2 = true; ok2 = note(1, read2(b)); }     // the stand-by in the cycle IMU 1 fails
        if (ok2) last2_ = b;
        // the gyro
        src_ = 0;
        if (use_ == BOTH && ok1 && ok2) { for (int k = 0; k < 3; k++) out.gyro_dps[k] = 0.5f * (a.gyro_dps[k] + b.gyro_dps[k]); src_ = kImu1 | kImu2; }
        else if (use_ != FIRST && ok2) { for (int k = 0; k < 3; k++) out.gyro_dps[k] = b.gyro_dps[k]; src_ = kImu2; }
        else if (ok1) { for (int k = 0; k < 3; k++) out.gyro_dps[k] = a.gyro_dps[k]; src_ = kImu1; }
        else if (ok2) { for (int k = 0; k < 3; k++) out.gyro_dps[k] = b.gyro_dps[k]; src_ = kImu2; }
        else return false;
        if (!(src_ & preferred())) fallbacks_++;
        // the accelerometer
        if (ok1) { for (int k = 0; k < 3; k++) out.acc_g[k] = a.acc_g[k]; acc_src_ = kImu1; }
        else { for (int k = 0; k < 3; k++) out.acc_g[k] = b.acc_g[k]; acc_src_ = kImu2; }
        return true;
    }

    Use use() const { return use_; }
    uint8_t gyro_src() const { return src_; }           // where the last sample's gyro came from (kImu1 / kImu2 bits)
    uint8_t acc_src() const { return acc_src_; }
    bool lost(int i) const { return lost_[i & 1]; }
    bool have(int i) const { return have_[i & 1]; }
    // the chip(s) whose gyro calibration counts for arming: the ones the mode flies on, as far as they work
    WB_FAST uint8_t in_use() const {
        const bool c1 = have_[0] && !lost_[0], c2 = have_[1] && !lost_[1];
        uint8_t u = preferred() & ((c1 ? kImu1 : 0) | (c2 ? kImu2 : 0));
        if (!u) u = c1 ? kImu1 : (c2 ? kImu2 : 0);
        return u;
    }
    uint32_t fallbacks() const { return fallbacks_; }   // samples whose gyro did not come from the chip(s) the mode asks for
    const AgSample& last2() const { return last2_; }    // IMU 2's last sample (board frame), for the app

private:
    uint8_t preferred() const { return use_ == FIRST ? kImu1 : use_ == SECOND ? kImu2 : (kImu1 | kImu2); }
    WB_FAST bool note(int i, bool ok) {
        if (ok) { streak_[i] = 0; lost_[i] = false; } else if (++streak_[i] >= kLostAfter) lost_[i] = true;
        return ok;
    }
    Use use_ = FIRST; bool have_[2] = {true, false}, lost_[2] = {false, false};
    uint16_t streak_[2] = {0, 0}; uint8_t n_ = 0, src_ = 0, acc_src_ = 0; uint32_t fallbacks_ = 0;
    AgSample last2_ = {{0, 0, 0}, {0, 0, 0}};
};

} // namespace imusel
