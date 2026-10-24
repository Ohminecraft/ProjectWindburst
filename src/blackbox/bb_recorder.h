/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"
#include "../config/ram_profile.h"

#include <stdint.h>
#include <atomic>
#include "bb_format.h"
#include "bb_ring.h"
#include "../app/telemetry.h"

// ============================================================================
// Recorder: the flight-loop end of the blackbox. It turns what the loop already has into records and puts them in the queue.
//
//   flight loop, every control cycle:   if (rec.fast_due()) rec.log_fast(sample);     // every rate_div-th cycle while a session runs
//                                       rec.note_state(armed, failsafe, modes, ...);  // edge events (arm, disarm, failsafe, modes)
//   flight loop, ~10 Hz:                rec.log_slow(t_us, telemetry);
//   link task (bb::Service):            rec.start(plan, div) / rec.stop(), rec.ring().pop(...)
//
// Costs in the flight loop when a session runs: one record every rate_div cycles (a few dozen float multiplications and a memcpy);
// when no session runs: one atomic load per cycle.
// start() / stop() are called by the link task: the plan is written BEFORE `active` is set and never changed while it is set.
// ============================================================================
namespace bb {

inline double slow_value(const app::Telemetry& t, uint8_t id, int i) {
    switch (id) {
    case S_ARMED: return t.armed ? 1 : 0;
    case S_FAILSAFE: return t.failsafe;
    case S_AH_STATE: return t.ah_state;
    case S_BATT_STATE: return t.batt_state;
    case S_MODES: return (double)t.modes;
    case S_BLOCKERS: return (double)t.blockers;
    case S_HEALTH: return t.health;
    case S_LOOP_MAX_US: return t.loop_max_us;
    case S_GAP_MAX_US: return t.gap_max_us;
    case S_LOOP_HZ: return t.loop_hz;
    case S_RSSI: return t.rssi;
    case S_RC_RATE: return t.rc_rate;
    case S_BARO_ALT: return (double)t.baro_alt;
    case S_EST_Z: return (double)t.z;
    case S_EST_VZ: return (double)t.vz;
    case S_RANGE: return (double)t.range_m;
    case S_VBAT: return (double)t.batt_v;
    case S_CELL: return (double)t.cell_v;
    case S_CURRENT: return (double)t.batt_amps;
    case S_MAH: return (double)t.batt_mah;
    case S_GPS_FIX: return t.gps_fix;
    case S_GPS_SATS: return t.gps_sats;
    case S_LAT: return t.lat * 1e-7;
    case S_LON: return t.lon * 1e-7;
    case S_GPS_ALT: return t.alt_cm * 0.01;
    case S_GPS_SPEED: return t.gs_cms * 0.01;
    case S_GPS_COURSE: return t.course_ddeg * 0.1;
    case S_HDOP: return t.hdop * 0.01;
    case S_POS_N: return (double)t.pos_n;
    case S_POS_E: return (double)t.pos_e;
    case S_VEL_N: return (double)t.vel_n;
    case S_VEL_E: return (double)t.vel_e;
    case S_HOME_DIST: return (double)t.home_dist_m;
    case S_MAG: return (double)t.mag[i];
    case S_EST_VALID: return app::est_valid_bits(t);
    case S_NAV_FLAGS: return t.nav_flags;
    case S_FLOW_QUALITY: return t.flow_quality;
    case S_FLOW_REJECT: return t.flow_reject;
    case S_FLOW_FWD: return (double)t.flow_fwd;
    case S_FLOW_RIGHT: return (double)t.flow_right;
    case S_NAV_ROLL: return (double)t.nav_roll;
    case S_NAV_PITCH: return (double)t.nav_pitch;
    case S_NAV_TARGET_N: return (double)t.nav_target_n;
    case S_NAV_TARGET_E: return (double)t.nav_target_e;
    case S_RESCUE_PHASE: return t.rescue_phase;
    case S_EKF_ROLL: return t.ekf_roll;
    case S_EKF_PITCH: return t.ekf_pitch;
    case S_EKF_YAW: return app::heading360(t.ekf_yaw);
    case S_EKF_PN: return t.ekf_pn;
    case S_EKF_PE: return t.ekf_pe;
    case S_EKF_VN: return t.ekf_vn;
    case S_EKF_VE: return t.ekf_ve;
    case S_EKF_Z: return t.ekf_z;
    case S_EKF_YAW_SIGMA: return t.ekf_yaw_sigma;
    case S_EKF_TEST_VEL: return t.ekf_test_vel;
    case S_EKF_TEST_MAG: return t.ekf_test_mag;
    case S_EKF_FLAGS: return t.ekf_flags;
    default: return 0.0;
    }
}
inline uint16_t pack_slow(const Plan& plan, uint32_t t_us, const app::Telemetry& t, uint8_t* out) {
    uint8_t* p = out; put_u32(p, t_us);
    for (const FieldDef& f : kSlow) {
        if (!plan.has(f)) continue;
        const int n = plan.count(f);
        for (int i = 0; i < n; i++) put_int(p, f.type, slow_value(t, f.id, i) / f.scale);
    }
    return (uint16_t)(p - out);
}

constexpr uint32_t kRingBytes = WB_LOW_RAM ? 4096 : 16384;      // (config/ram_profile.h)

// A flag the link task sets and the control cycle reads - several times per cycle. The ESP32 build (-Os) does not inline
// std::atomic<bool>::load: it would be a call into flash from code that otherwise sits in RAM. The compiler builtin is
// the same atomic access and always inline.
struct FastFlag {
    bool v;
    constexpr FastFlag(bool b = false) : v(b) {}
    __attribute__((always_inline)) inline bool load(std::memory_order = std::memory_order_acquire) const { return __atomic_load_n(&v, __ATOMIC_ACQUIRE); }
    __attribute__((always_inline)) inline void store(bool b, std::memory_order = std::memory_order_release) { __atomic_store_n(&v, b, __ATOMIC_RELEASE); }
};

class Recorder {
public:
    // ---------------- link task
    // Begin producing records. div: every div-th control cycle gets a fast record.
    void start(const Plan& plan, uint8_t div) {
        active_.store(false, std::memory_order_release);
        plan_ = plan; div_ = div ? div : 1; n_ = 0; have_state_ = false; gap_max_ = 0;
        ring_.clear();
        active_.store(true, std::memory_order_release);
    }
    void stop() { active_.store(false, std::memory_order_release); }
    bool active() const { return active_.load(std::memory_order_acquire); }
    Ring<kRingBytes>& ring() { return ring_; }
    const Plan& plan() const { return plan_; }
    uint8_t div() const { return div_; }
    // something that must keep the craft on the ground is going on in the link task (the log memory is being erased)
    void set_busy(bool b) { busy_.store(b, std::memory_order_release); }
    bool busy() const { return busy_.load(std::memory_order_acquire); }
    // the erase has finished: the flight loop beeps (BLACKBOX_ERASE)
    void post_erase_done() { erase_done_.store(true, std::memory_order_release); }
    bool take_erase_done() { return erase_done_.exchange(false, std::memory_order_acq_rel); }      // flight loop

    // ---------------- flight loop
    // Call once per control cycle with the time since the previous cycle started. True when this cycle is to be logged.
    bool fast_due(uint32_t gap_us) {
        if (!active_.load(std::memory_order_acquire)) return false;
        if (gap_us > gap_max_) gap_max_ = gap_us;
        if (++n_ < div_) return false;
        n_ = 0; return true;
    }
    WB_FAST void log_fast(FastSample& s) {
        s.gap_us = gap_max_; gap_max_ = 0;
        uint8_t rec[192];                                   // largest fast record: 1 + 4 + 143 bytes with every group and 8 motors (test_blackbox)
        rec[0] = 'F';
        const uint16_t n = pack_fast(plan_, s, rec + 1);
        ring_.push(rec, (uint8_t)(n + 1));
    }
    void log_slow(uint32_t t_us, const app::Telemetry& t) {
        if (!active_.load(std::memory_order_acquire)) return;
        uint8_t rec[160]; rec[0] = 'S';
        const uint16_t n = pack_slow(plan_, t_us, t, rec + 1);
        ring_.push(rec, (uint8_t)(n + 1));
    }
    WB_FAST void event(uint32_t t_us, uint8_t code, const uint8_t* data = nullptr, uint8_t n = 0) {
        if (!active_.load(std::memory_order_acquire)) return;
        uint8_t rec[24]; if (n > 16) n = 16;
        uint8_t* p = rec; *p++ = 'E'; put_u32(p, t_us); *p++ = code; *p++ = n; if (n) memcpy(p, data, n);
        ring_.push(rec, (uint8_t)(7 + n));
    }
    // Edge events. The first call of a session records the state it starts in.
    WB_FAST void note_state(uint32_t t_us, bool armed, uint8_t failsafe, uint32_t modes, uint8_t batt_state, uint8_t ah_state) {
        if (!active_.load(std::memory_order_acquire)) return;
        const bool first = !have_state_;
        if (first || armed != armed_) event(t_us, armed ? EV_ARM : EV_DISARM);
        if (first || failsafe != failsafe_) event(t_us, EV_FAILSAFE, &failsafe, 1);
        if (first || modes != modes_) event(t_us, EV_MODES, (const uint8_t*)&modes, 4);
        if (first || batt_state != batt_state_) event(t_us, EV_BATTERY, &batt_state, 1);
        if (first || ah_state != ah_state_) event(t_us, EV_AH_STATE, &ah_state, 1);
        armed_ = armed; failsafe_ = failsafe; modes_ = modes; batt_state_ = batt_state; ah_state_ = ah_state; have_state_ = true;
    }

private:
    Ring<kRingBytes> ring_;
    FastFlag active_{false};
    std::atomic<bool> busy_{false}, erase_done_{false};
    Plan plan_; uint8_t div_ = 8, n_ = 0;
    uint32_t gap_max_ = 0, modes_ = 0; bool have_state_ = false, armed_ = false; uint8_t failsafe_ = 0, batt_state_ = 0, ah_state_ = 0;
};

} // namespace bb
