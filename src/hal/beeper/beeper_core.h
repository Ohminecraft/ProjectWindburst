/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Beeper (portable, no Arduino includes): which sound for which situation, and who wins when two want to beep.
// Sound patterns, priorities and the rules below follow Betaflight io/beeper.c, so a pilot hears what he knows:
//
//   * a pattern is a list of on/off durations in units of 10 ms: 1st, 3rd, 5th ... = buzzer on, 2nd, 4th ... = off; a leading 0
//     means "start with the pause"
//   * request(mode) starts a sound unless one of the same or a higher priority is already playing (so asking again every loop
//     does not restart it); when it has finished, a condition that is still true starts it again - that is how RX_LOST repeats
//   * a condition switched off in the settings does not exist for the beeper: it makes no sound AND does not hold back a less
//     important one. (Betaflight runs the pattern silently, so a muted RX_LOST also mutes the battery warning; not copied.)
//
// Mode order = bit order of the settings field beeper.conditions = priority (first = most important).
//
// TWO SOUNDERS can play: a buzzer on a pin, and the MOTORS (the ESCs beep them on a DShot beacon command) - for a craft without a
// buzzer. Each is its own Beeper with its own set of enabled conditions. The motors differ in three ways:
//   * every "on" part of a pattern becomes exactly one tone, and how long a tone sounds is up to the ESC (DShot only says "beacon
//     1 .. 5", never for how long; Betaflight: "the DShot Beacon tone duration is determined by the ESC, and should not exceed
//     250ms"). The motors play the NUMBER of beeps of a pattern (3 = gyro calibrated, 2 = disarmed ...) and its rhythm as far as
//     a tone every 450 ms allows - not the length of each note;
//   * a spinning motor cannot beep: nothing while armed, and nothing in the first 1.2 s after a disarm (Betaflight's guard time);
//   * only with a DShot protocol (the firmware checks that; here it is `motors_ok`).
// ============================================================================
namespace beep {

enum Mode : uint8_t {
    GYRO_CALIBRATED, RX_LOST, RX_LOST_LANDING, DISARMING, ARMING, ARMING_GPS_FIX, ARMING_GPS_NO_FIX, BAT_CRIT_LOW, BAT_LOW, RX_SET,
    ACC_CALIBRATION, ACC_CALIBRATION_FAIL, READY_BEEP, ARMED, SYSTEM_INIT, ARM_REFUSED, BLACKBOX_ERASE,
    ON_USB,                 // not a sound: "beep on the bench too" (app connected and no battery)
    MISSION_WARN,           // the countdown before a mission arms the motors again by itself (always on: FlightApp forces it)
    MODE_COUNT, NO_MODE = 255
};

constexpr uint8_t kStop = 0xFF;
// Betaflight's sequences (io/beeper.c), unit 10 ms
constexpr uint8_t seq_gyro_calibrated[] = {20, 10, 20, 10, 20, 10, kStop};                                  // 3 beeps
constexpr uint8_t seq_rx_lost[] = {50, 50, kStop};                                                          // long beep, long pause
constexpr uint8_t seq_sos[] = {10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, kStop};
constexpr uint8_t seq_disarm[] = {15, 5, 15, 5, kStop};
constexpr uint8_t seq_arming[] = {30, 5, 5, 5, kStop};
constexpr uint8_t seq_arming_gps_fix[] = {5, 5, 15, 5, 5, 5, 15, 30, kStop};
constexpr uint8_t seq_arming_gps_no_fix[] = {30, 5, 30, 5, 30, 5, kStop};
constexpr uint8_t seq_bat_crit[] = {50, 2, kStop};
constexpr uint8_t seq_bat_low[] = {25, 50, kStop};
constexpr uint8_t seq_short[] = {10, 10, kStop};
constexpr uint8_t seq_2short[] = {5, 5, 5, 5, kStop};
constexpr uint8_t seq_2longer[] = {20, 15, 35, 5, kStop};
constexpr uint8_t seq_ready[] = {4, 5, 4, 5, 8, 5, 15, 5, 8, 5, 4, 5, 4, 5, kStop};
constexpr uint8_t seq_armed[] = {0, 245, 10, 5, kStop};                                                     // pause first, then a short beep
// Windburst's own
constexpr uint8_t seq_system_init[] = {3, 5, 3, 5, kStop};                                                  // power-on chirp
constexpr uint8_t seq_arm_refused[] = {40, 10, 10, 10, 10, 10, kStop};                                      // one long, two short
constexpr uint8_t seq_mission_warn[] = {8, 8, kStop};                                                       // fast, for as long as the countdown runs

struct Entry { const uint8_t* sequence; const char* name; };
constexpr Entry kTable[MODE_COUNT] = {
    {seq_gyro_calibrated, "GYRO_CALIBRATED"}, {seq_rx_lost, "RX_LOST"}, {seq_sos, "RX_LOST_LANDING"}, {seq_disarm, "DISARMING"}, {seq_arming, "ARMING"},
    {seq_arming_gps_fix, "ARMING_GPS_FIX"}, {seq_arming_gps_no_fix, "ARMING_GPS_NO_FIX"}, {seq_bat_crit, "BAT_CRIT_LOW"}, {seq_bat_low, "BAT_LOW"},
    {seq_short, "RX_SET"}, {seq_2short, "ACC_CALIBRATION"}, {seq_2longer, "ACC_CALIBRATION_FAIL"}, {seq_ready, "READY_BEEP"}, {seq_armed, "ARMED"},
    {seq_system_init, "SYSTEM_INIT"}, {seq_arm_refused, "ARM_REFUSED"}, {seq_2short, "BLACKBOX_ERASE"}, {nullptr, "USB"}, {seq_mission_warn, "MISSION_WARN"},
};
inline bool has_sound(uint8_t mode) { return mode < MODE_COUNT && kTable[mode].sequence != nullptr; }

class Beeper {
public:
    uint32_t enabled = ((1u << MODE_COUNT) - 1u) & ~(1u << ON_USB);     // bit per Mode: the settings field beeper.conditions (default: every sound, quiet on the bench)
    uint16_t slot_ms = 0;               // 0: patterns as written (a buzzer). > 0: every "on" part lasts at least this long = one ESC tone (the motors)

    // Ask for a sound. Ignored when that condition is switched off, and while a sound of the same or a higher priority plays.
    void request(uint8_t mode) { if (mode < MODE_COUNT && ((enabled >> mode) & 1u)) start(mode, false); }
    // The app's "try it" button: plays even if the condition is switched off, replaces whatever is playing.
    void force(uint8_t mode) { if (has_sound(mode)) { current_ = NO_MODE; start(mode, true); } }
    void silence() { current_ = NO_MODE; pos_ = 0; next_ms_ = 0; waiting_ = false; on_ = false; forced_ = false; started_ = false; }

    // Call every millisecond or so. Returns true while the buzzer has to sound.
    bool update(uint32_t now_ms) {
        if (current_ == NO_MODE) return false;
        if (waiting_ && (int32_t)(next_ms_ - now_ms) > 0) return on_;
        const uint8_t* s = kTable[current_].sequence;
        for (;;) {
            const uint8_t d = s[pos_];
            if (d == kStop) { silence(); return false; }
            const bool on_phase = (pos_ % 2) == 0;       // on/off is strictly index based
            pos_++;
            if (d == 0) continue;                        // a zero duration is skipped (a pattern that starts with its pause)
            // (motors: an "on" part shorter than one ESC tone is stretched to it - the next tone would be dropped otherwise; a longer
            // one keeps its length, so the rhythm of the pattern stays. How long the ESC sounds is the ESC's own: DShot has no length.)
            next_ms_ = now_ms + ((on_phase && slot_ms && 10u * d < slot_ms) ? (uint32_t)slot_ms : 10u * d); waiting_ = true;
            on_ = on_phase; if (on_phase) started_ = true;
            return on_;
        }
    }

    uint8_t current() const { return current_; }                          // the pattern running now, NO_MODE = idle
    bool sounding() const { return on_; }
    // True once for every "on" part that has begun since the last call: the moment to send one beacon tone to the ESCs.
    bool take_start() { const bool st = started_; started_ = false; return st; }
    bool forced() const { return current_ != NO_MODE && forced_; }

private:
    void start(uint8_t mode, bool forced) {
        if (!has_sound(mode)) return;
        if (current_ != NO_MODE && mode >= current_) return;                // lower number = higher priority; equal = already playing
        current_ = mode; pos_ = 0; next_ms_ = 0; waiting_ = false; on_ = false; forced_ = forced;
    }
    uint8_t current_ = NO_MODE; uint16_t pos_ = 0; uint32_t next_ms_ = 0; bool waiting_ = false, on_ = false, forced_ = false, started_ = false;
};

// Conditions the motors can never play: they belong to a craft that is armed (or to no sound at all).
constexpr uint32_t kMotorNever = (1u << RX_LOST_LANDING) | (1u << ARMING) | (1u << ARMING_GPS_FIX) | (1u << ARMING_GPS_NO_FIX) | (1u << ARMED) | (1u << ON_USB);
// Default for the motors: everything they can play except the power-on chirp (the ESCs play their own start-up tones then).
constexpr uint32_t kMotorDefault = (((1u << MODE_COUNT) - 1u) & ~kMotorNever) & ~(1u << SYSTEM_INIT);

// ---------------------------------------------------------------------------------------------------------------------
// Conditions: the state of the craft -> beeper requests, once per call (every ~1 ms). Edge detection lives here.
// One-shot events that have no state to watch (accelerometer calibration finished / failed, log erased) are sent by the
// firmware directly with Beeper::request().
// ---------------------------------------------------------------------------------------------------------------------
struct Input {
    uint32_t now_ms = 0;
    bool armed = false;
    bool rx_ok = false;                 // receiver link up
    bool failsafe_landing = false;      // the failsafe is bringing the craft down
    bool gyro_cal_running = false, gyro_calibrated = false;
    bool gps_enabled = false, gps_ready = false;         // ready = fix with enough satellites for a home point
    uint8_t batt_state = 0;             // batt::State: 0 no battery, 1 ok, 2 warning, 3 critical
    bool beeper_switch = false;         // mode BEEPER on an aux switch
    bool throttle_low = true;
    uint32_t arm_refused = 0;           // Arming::refused(): counts arm attempts that a blocker rejected
    bool app_active = false;            // the configurator is talking to the firmware
    bool mission_warn = false;          // the mission arms the motors by itself soon (the last seconds of a wait on the ground)
};

constexpr uint32_t kBeaconIntervalMs = 450;             // DSHOT_BEACON_MODE_INTERVAL_US: the ESC needs this long for one tone
constexpr uint32_t kBeaconGuardMs = 1200;               // DSHOT_BEACON_GUARD_DELAY_US: no beacon right after a disarm

class Conditions {
public:
    // Feeds the buzzer `b` and, when given, the motor sounder `m` (a Beeper with slot_ms = kBeaconIntervalMs).
    // motors_ok: the ESCs can beep right now (DShot protocol, outputs running, no motor test).
    // The caller then runs both: buzzer pin = b.update(now); if (m.update(now), m.take_start()) send one beacon tone to all ESCs.
    void update(const Input& in, Beeper& b, Beeper* m = nullptr, bool motors_ok = false) {
        const uint32_t now = in.now_ms;
        if (!started_) { started_ = true; prev_armed_ = in.armed; prev_refused_ = in.arm_refused; prev_cal_running_ = in.gyro_cal_running; disarm_ms_ = now - kBeaconGuardMs; }
        if (in.rx_ok) monitoring_ = true;                // like Betaflight's failsafe: nothing is "lost" before it was there once
        const bool rx_lost = monitoring_ && !in.rx_ok;

        // on the bench (app connected, running from USB only) the buzzer stays quiet unless the USB condition is switched on.
        // The motors are never muted by this rule: an ESC can only make a sound when it has power, i.e. a battery is connected -
        // and "no battery" may only mean that nothing measures it (no ADC pin, ESCs without voltage telemetry).
        const bool muted = !((b.enabled >> ON_USB) & 1u) && in.batt_state == 0 && in.app_active;
        if (prev_armed_ && !in.armed) { disarm_ms_ = now; disarm_beep_due_ = true; }
        muted_ = muted;
        // the motors: only while disarmed, with ESCs that can beep, and not right after a disarm (they are still spinning down)
        const bool m_can = m && motors_ok && !in.armed && (uint32_t)(now - disarm_ms_) >= kBeaconGuardMs;
        if (m && !m_can && m->current() != NO_MODE) m->silence();
        m_ = m_can ? m : nullptr;
        if (muted && b.current() != NO_MODE && !b.forced()) b.silence();      // the app's own test sound is not cut by the bench mute
        Beeper& bz = muted ? quiet_ : b;                   // on the bench the buzzer's requests go nowhere

        if (prev_cal_running_ && !in.gyro_cal_running && in.gyro_calibrated) both(bz, GYRO_CALIBRATED);
        if (!prev_armed_ && in.armed) bz.request(!in.gps_enabled ? ARMING : (in.gps_ready ? ARMING_GPS_FIX : ARMING_GPS_NO_FIX));
        if (prev_armed_ && !in.armed) bz.request(DISARMING);
        if (disarm_beep_due_ && m_) { m_->request(DISARMING); disarm_beep_due_ = false; }      // the motors say it once they have stopped
        if (in.arm_refused != prev_refused_) both(bz, ARM_REFUSED);
        if (in.failsafe_landing) bz.request(RX_LOST_LANDING);
        else if (rx_lost && !in.app_active) both(bz, RX_LOST);        // also the lost-model finder: never on the desk
        if (in.batt_state == 3) both(bz, BAT_CRIT_LOW); else if (in.batt_state == 2) both(bz, BAT_LOW);
        if (in.beeper_switch) { bz.request(RX_SET); if (m_ && in.rx_ok) m_->request(RX_SET); }   // a dead link reports stale switches: the motors ignore them
        if (in.mission_warn) both(bz, MISSION_WARN);
        if (!in.armed && in.gps_enabled && in.gps_ready && !ready_beeped_) { both(bz, READY_BEEP); ready_beeped_ = true; }
        // armed with the throttle down: a reminder every 2.5 s that the propellers are live
        if (in.armed && in.throttle_low) { bz.request(ARMED); armed_beep_ = true; }
        else if (armed_beep_) { if (b.current() == ARMED) b.silence(); armed_beep_ = false; }
        quiet_.silence();
        prev_armed_ = in.armed; prev_refused_ = in.arm_refused; prev_cal_running_ = in.gyro_cal_running;
        // keep the stamp close to `now` so that the unsigned difference never wraps after weeks of idling
        if ((uint32_t)(now - disarm_ms_) > 60000u) disarm_ms_ = now - 60000u;
    }
    // A one-shot event with no state to watch (calibration finished, log erased): to both sounders, under the same rules.
    void event(uint8_t mode, Beeper& b) { both(muted_ ? quiet_ : b, mode); }      // on the bench only the motors play it
    bool muted() const { return muted_; }      // result of the last update(): the bench rule keeps the buzzer quiet (never the motors)
    bool motors_ready() const { return m_ != nullptr; }   // result of the last update(): the motors could beep now (ESCs able, disarmed, spun down)

private:
    void both(Beeper& b, uint8_t mode) { b.request(mode); if (m_) m_->request(mode); }
    Beeper quiet_;                                         // where the buzzer's requests go on the bench
    Beeper* m_ = nullptr;
    bool muted_ = false, disarm_beep_due_ = false;
    bool started_ = false, prev_armed_ = false, prev_cal_running_ = false, monitoring_ = false, ready_beeped_ = false, armed_beep_ = false;
    uint32_t prev_refused_ = 0, disarm_ms_ = 0;
};

} // namespace beep
