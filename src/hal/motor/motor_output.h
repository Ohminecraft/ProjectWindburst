/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <string.h>
#include "motor_protocol.h"

// ============================================================================
// MotorOutputManager - everything between "mixer says 0..1" and "raw values for the pins", with no hardware in it.
//
//   mixer (absolute 0..1 of full range, idle already inside)  ->  clamp to [idle, limit]  ->  DShot value / pulse scale value
//   disarmed -> DShot 0 / pulse "disarmed" (app: 0)          armed -> never below idle (a stray 0 must not stop a motor in flight)
//   logical motor -> hardware slot (motor reorder tab), reversed -> DShot spin-direction commands (not a mixer change, as in Betaflight)
//   DShot commands (direction, save, beacon, extended-telemetry enable) are paced like drivers/dshot_command.c and only while DISARMED
//   test sliders: only while disarmed, auto-stop when the app stops refreshing
//   bidirectional telemetry: rpm / temperature / voltage / current per motor, invalid-packet % ("Motor Error"), dynamic idle
//
// Idle & limit live in ONE place per direction: this manager owns motor idle (static + dynamic). Set the FC mixer's idle to
// idle_fraction() every cycle and keep mixer.output_limit equal to cfg.output_limit.
// ============================================================================
namespace motor {

struct DynIdleConfig {            // Betaflight units (pg/pid: dyn_idle_*)
    uint16_t min_rpm_100 = 0;     // in 100 rpm; 0 = off
    uint8_t p_gain = 50, i_gain = 50, d_gain = 50;
    uint16_t max_increase = 150;  // in 0.1 %
};

struct MotorConfig {
    Protocol protocol = PROTO_DSHOT300;
    uint8_t count = 4;
    bool reversed[kMaxMotors] = {};                 // logical motor spins the wrong way -> DShot direction command
    uint8_t order[kMaxMotors] = {0, 1, 2, 3, 4, 5, 6, 7};   // logical motor -> hardware slot (pin index)
    PulseEndpoints pulse;                           // PWM/Oneshot/Multishot: not armed / armed / max throttle
    uint16_t pulse_hz = 0;                          // 0 = protocol default
    bool bidirectional = false;                     // bidirectional DShot (eRPM telemetry)
    bool ext_pullup = false;                        // bidirectional DShot: pull-up resistors on the board, the pins' own pull-up stays off
    uint8_t pole_count = 14;                        // motor poles (magnets), for eRPM -> rpm
    bool edt = false;                               // extended telemetry (voltage/current/temperature) decoding
    bool dir_cmd_extended = false;                  // (not used since 0.18.1: request_direction() always sends 7 / 8 + save)
    float idle = 0.055f;                            // motor idle, fraction of range (BF motor_idle 550 = 5.5 %)
    float output_limit = 1.0f;
    DynIdleConfig dyn;
    uint16_t telem_timeout_ms = 100;
};

struct Frame {
    uint16_t value[kMaxMotors] = {};                // per hardware slot: DShot value 0..2047, or pulse scale value (0 / 1000..2000)
    bool telem_request[kMaxMotors] = {};            // DShot telemetry-request bit (commands; ESC telemetry on its own wire)
    bool send = true;                               // false: hold the line quiet this cycle (paced command gaps)
};

struct MotorTelemetry {
    float rpm = 0; uint8_t temp_c = 0; float volt = 0; float amps = 0;
    uint8_t error_pct = 0;                          // invalid / total packets  (app: "Motor Error")
    bool rpm_valid = false;
};

// Port of Betaflight's dynamic idle controller (flight/mixer.c + mixer_init.c).
class DynamicIdle {
public:
    void configure(const DynIdleConfig& c, float idle_frac, float dt_s, float pid_hz) {
        min_rps_ = c.min_rpm_100 * 100.0f / 60.0f;
        p_ = c.p_gain * 0.00015f; i_ = c.i_gain * 0.01f * dt_s; d_ = c.d_gain * 0.0000003f * pid_hz;
        max_inc_ = c.max_increase * 0.001f; start_inc_ = idle_frac;
        delay_k_ = 800.0f * dt_s / 20.0f;
        reset();
    }
    void reset() { integ_ = 0; prev_ = 0; }
    bool enabled() const { return min_rps_ > 0.0f; }
    // min_hz: slowest motor (rev/s). Returns the increase (fraction of the motor range) to add to the idle floor.
    float update(float min_hz, bool telemetry_ok, bool throttle_raised) {
        if (!enabled()) return 0.0f;
        if (!telemetry_ok) { reset(); return 0.0f; }       // without rpm we can't regulate: fall back to static idle
        const float max_inc = throttle_raised ? max_inc_ : start_inc_;
        float err = min_rps_ - min_hz;
        const float filtered = prev_ + delay_k_ * (min_hz - prev_);
        const float d = (prev_ - filtered) * d_;
        prev_ = filtered;
        const float pterm = err * p_;
        if (err < -0.1f) err = -0.1f;                       // I rises fast, falls slowly
        integ_ += err * i_;
        if (integ_ < 0) integ_ = 0;
        if (integ_ > max_inc) integ_ = max_inc;
        float inc = pterm + integ_ + d;
        if (inc < 0) inc = 0;
        if (inc > max_inc) inc = max_inc;
        return inc;
    }
private:
    float min_rps_ = 0, p_ = 0, i_ = 0, d_ = 0, max_inc_ = 0, start_inc_ = 0, delay_k_ = 0, integ_ = 0, prev_ = 0;
};

class MotorOutputManager {
public:
    MotorConfig cfg;

    // dt_s = flight loop period, used by dynamic idle.
    void begin(float dt_s = 0.00025f) {
        for (int i = 0; i < kMaxMotors; i++) tel_[i] = Slot();
        dyn_.configure(cfg.dyn, cfg.idle, dt_s, 1.0f / dt_s);
        dyn_inc_ = 0; cmd_active_ = false; q_head_ = q_tail_ = 0; test_until_us_ = 0;
        fix_order();
    }
    bool order_valid() const {                          // BF validateAndfixMotorOutputReordering: must be a permutation of 0..count-1
        bool seen[kMaxMotors] = {};
        for (int i = 0; i < cfg.count; i++) { if (cfg.order[i] >= cfg.count || seen[cfg.order[i]]) return false; seen[cfg.order[i]] = true; }
        return true;
    }
    void fix_order() { if (!order_valid()) for (int i = 0; i < kMaxMotors; i++) cfg.order[i] = (uint8_t)i; }
    // A new motor order while running (disarmed; the motor wizard): taken only when it is a permutation of the motors. When the order really
    // changes, a test or a command in progress is dropped: it was aimed at the old order.
    bool set_order(const uint8_t order[kMaxMotors]) {
        const MotorConfig old = cfg;
        for (int i = 0; i < kMaxMotors; i++) cfg.order[i] = order[i];
        if (!order_valid()) { cfg = old; return false; }
        if (memcmp(old.order, cfg.order, sizeof(cfg.order)) != 0) { test_stop(); cmd_active_ = false; q_head_ = q_tail_ = 0; }
        return true;
    }

    // ---------------------------------------------------------------- DShot commands (disarmed only)
    // The spin direction an ESC keeps: DShot 7 / 8, then 12 (save) - BLHeli_S, Bluejay, BLHeli_32 and AM32 all store it. Never 20 / 21
    // (dir_cmd_extended, no longer used): those turn the motor relative to the STORED direction and are not saved (AM32: `forward`
    // only) - the craft forgot it at the next ESC restart, which a flight controller reboot causes (0.18 on the board, AM32).
    bool request_direction(int logical, bool reversed) {
        if (!is_dshot(cfg.protocol) || logical < 0 || logical >= cfg.count) return false;
        const uint8_t cmd = reversed ? CMD_SPIN_DIRECTION_2 : CMD_SPIN_DIRECTION_1;
        return push(cfg.order[logical], cmd) && push(cfg.order[logical], CMD_SAVE_SETTINGS);
    }
    bool apply_directions() { bool ok = is_dshot(cfg.protocol); for (int i = 0; i < cfg.count && ok; i++) ok = request_direction(i, cfg.reversed[i]); return ok; }
    bool beacon(int logical, int n = 2) {              // identify a motor (motor-reorder tab)
        if (!is_dshot(cfg.protocol) || logical < 0 || logical >= cfg.count || n < 1 || n > 5) return false;
        return push(cfg.order[logical], (uint8_t)n);
    }
    // The beeper's sound: ALL motors play one beacon tone at the same moment (Betaflight: dshotCommandWrite(ALL_MOTORS, ...)).
    // Nothing is queued while another command, a motor test or an earlier tone is still on the line: a beep that comes late is noise.
    bool beacon_all(int n, uint32_t now_us) {
        if (!is_dshot(cfg.protocol) || n < 1 || n > 5 || commands_pending() || testing(now_us)) return false;
        return push(kAllSlots, (uint8_t)n);
    }
    bool enable_extended_telemetry(bool on) {
        if (!is_dshot(cfg.protocol)) return false;
        for (int i = 0; i < cfg.count; i++) if (!push(cfg.order[i], on ? CMD_EXTENDED_TELEMETRY_ENABLE : CMD_EXTENDED_TELEMETRY_DISABLE)) return false;
        cfg.edt = on; return true;
    }
    bool commands_pending() const { return cmd_active_ || q_head_ != q_tail_; }

    // ---------------------------------------------------------------- motor test sliders
    void test_motor(int logical, float frac, uint32_t now_us) {
        if (logical < 0 || logical >= cfg.count) return;
        test_frac_[logical] = frac < 0 ? 0 : (frac > 1 ? 1 : frac);
        test_until_us_ = now_us + kTestTimeoutUs;
    }
    WB_FAST void test_stop() { for (auto& f : test_frac_) f = 0; test_until_us_ = 0; }
    // a motor is being driven by the app's test slider right now (the deadline alone is not enough: micros() wraps, a slider at 0 is no test)
    bool testing(uint32_t now_us) const {
        if ((int32_t)(test_until_us_ - now_us) <= 0) return false;
        for (float f : test_frac_) if (f > 0.0f) return true;
        return false;
    }
    // What logical motor i is being driven with right now, 0..1 (flight command when armed, motor-test value when disarmed): for the app's motor bars.
    float output(int logical) const { return logical >= 0 && logical < kMaxMotors ? out_frac_[logical] : 0.0f; }

    // ---------------------------------------------------------------- the per-cycle call
    // cmd[i] = mixer output for LOGICAL motor i, absolute 0..1.  Fills `out` per HARDWARE slot.
    WB_FAST void update(uint32_t now_us, const float cmd[], bool armed, Frame& out) {
        const Endpoints e = endpoints(cfg.protocol, cfg.pulse, cfg.idle, cfg.output_limit);
        out = Frame();
        if (armed) {
            test_stop(); cmd_active_ = false; q_head_ = q_tail_ = 0;
            for (int i = 0; i < cfg.count; i++) { out.value[cfg.order[i]] = to_value(cmd[i], e); out_frac_[i] = cmd[i] < 0 ? 0 : (cmd[i] > 1 ? 1 : cmd[i]); }
            ask_telemetry(out);
            return;
        }
        for (int i = 0; i < kMaxMotors; i++) out_frac_[i] = 0;
        if (is_dshot(cfg.protocol) && run_commands(now_us, out)) return;           // command sequencer owns the line
        const bool testing = (int32_t)(test_until_us_ - now_us) > 0;
        for (int i = 0; i < cfg.count; i++) {
            float v = e.disarm;
            if (testing && test_frac_[i] > 0.0f) { v = to_value(test_frac_[i], e); out_frac_[i] = test_frac_[i]; }
            out.value[cfg.order[i]] = (uint16_t)round_pos(v);
        }
        ask_telemetry(out);
        if (!testing) test_stop();
    }

    // ---------------------------------------------------------------- ESC telemetry on its own wire (esc_sensor.h)
    // The next DShot frame (not a command) carries the telemetry-request bit to this logical motor. (Both calls under the app's lock.)
    void request_telemetry(int logical) { if (logical >= 0 && logical < cfg.count && is_dshot(cfg.protocol)) tlm_ask_ = (int8_t)logical; }
    // An answer of that ESC. Its eRPM counts only without bidirectional DShot: the DShot reply is newer and comes every frame.
    void feed_serial(int logical, uint8_t temp_c, float volt, float amps, uint32_t erpm, uint32_t now_ms) {
        if (logical < 0 || logical >= cfg.count) return;
        Slot& s = tel_[logical];
        s.temp = temp_c; s.volt = volt; s.volt_ms = now_ms; s.amps = amps; s.amps_ms = now_ms; s.amps_seen = true;
        if (!cfg.bidirectional) { s.rpm = erpm * 2.0f / (cfg.pole_count ? cfg.pole_count : 14); s.rpm_ms = now_ms; s.rpm_seen = true; }
    }

    // ---------------------------------------------------------------- ESCs on the CAN bus (esc.Status, index = hardware slot)
    void feed_can(int slot, float temp_c, float volt, float amps, int32_t rpm, uint32_t now_ms) {
        const int m = logical_of(slot);
        if (m < 0) return;
        Slot& s = tel_[m];
        s.temp = (uint8_t)(temp_c < 0 ? 0 : temp_c > 255 ? 255 : temp_c); s.volt = volt; s.volt_ms = now_ms; s.amps = amps; s.amps_ms = now_ms; s.amps_seen = true;
        s.rpm = (float)(rpm < 0 ? -rpm : rpm); s.rpm_ms = now_ms; s.rpm_seen = true;
    }

    // ---------------------------------------------------------------- telemetry in
    // gcr20: the 20 GCR bits captured from the ESC reply on hardware slot `slot` (kGcrInvalid = nothing/garbled).
    WB_FAST void feed_reply(int slot, uint32_t gcr20, uint32_t now_ms) {
        const int m = logical_of(slot);
        if (m < 0) return;
        Slot& s = tel_[m];
        s.total++;
        uint32_t v12 = gcr20 == kGcrInvalid ? kGcrInvalid : gcr_decode(gcr20);
        if (gcr20 != kGcrInvalid && v12 == kGcrInvalid) crc_bad_++;         // clean bits, wrong checksum
        Telem t;
        if (v12 != kGcrInvalid) t = telem_decode((uint16_t)v12, cfg.edt);
        if (v12 == kGcrInvalid || t.value == kTelemInvalid) s.invalid++;
        else {
            switch (t.type) {
            case TT_ERPM: s.rpm = erpm100_to_rpm(t.value, cfg.pole_count); s.rpm_ms = now_ms; s.rpm_seen = true; break;
            case TT_TEMP: s.temp = (uint8_t)t.value; break;
            case TT_VOLT: s.volt = t.value * 0.25f; s.volt_ms = now_ms; break;
            case TT_CURRENT: s.amps = (float)t.value; s.amps_ms = now_ms; s.amps_seen = true; break;
            default: break;
            }
        }
        if (s.total >= 200) { s.total /= 2; s.invalid /= 2; }
    }

    // ---------------------------------------------------------------- telemetry out
    WB_FAST MotorTelemetry telemetry(int logical, uint32_t now_ms) const {
        MotorTelemetry o;
        if (logical < 0 || logical >= cfg.count) return o;
        const Slot& s = tel_[logical];
        o.rpm_valid = s.rpm_seen && (now_ms - s.rpm_ms) <= cfg.telem_timeout_ms;
        o.rpm = o.rpm_valid ? s.rpm : 0.0f; o.temp_c = s.temp; o.amps = s.amps;
        o.volt = (now_ms - s.volt_ms) <= 1000 ? s.volt : 0.0f;
        o.error_pct = s.total ? (uint8_t)(100u * s.invalid / s.total) : 0;
        return o;
    }
    WB_FAST bool rpm_all_valid(uint32_t now_ms) const {
        if (!cfg.bidirectional) return false;
        for (int i = 0; i < cfg.count; i++) if (!telemetry(i, now_ms).rpm_valid) return false;
        return true;
    }
    WB_FAST bool rpm_any_valid(uint32_t now_ms) const {
        if (!cfg.bidirectional) return false;
        for (int i = 0; i < cfg.count; i++) if (telemetry(i, now_ms).rpm_valid) return true;
        return false;
    }
    // for the gyro RPM filter (logical order, every motor of the layout); zeros when not valid -> filter bypasses those notches
    // every motor's speed (0 without telemetry); returns a bit per motor whose speed is known (bidirectional DShot or the wire)
    WB_FAST uint8_t rpm_array(float out[kMaxMotors], uint32_t now_ms) const {
        uint8_t mask = 0;
        for (int i = 0; i < kMaxMotors; i++) {
            if (i >= cfg.count) { out[i] = 0.0f; continue; }
            const MotorTelemetry t = telemetry(i, now_ms); out[i] = t.rpm; if (t.rpm_valid) mask |= (uint8_t)(1u << i);
        }
        return mask;
    }
    // ESC-reported total current (A): the sum over the motors whose current came in within the last second. false: none did.
    bool esc_current(uint32_t now_ms, float& amps) const {
        amps = 0; bool any = false;
        for (int i = 0; i < cfg.count; i++) { const Slot& s = tel_[i]; if (s.amps_seen && (uint32_t)(now_ms - s.amps_ms) <= 1000u) { amps += s.amps; any = true; } }
        return any;
    }
    // ESC-reported pack voltage (highest reading), 0 if the ESCs don't report it
    float esc_voltage(uint32_t now_ms) const {
        float v = 0; for (int i = 0; i < cfg.count; i++) { const float x = telemetry(i, now_ms).volt; if (x > v) v = x; } return v;
    }

    // ---------------------------------------------------------------- idle for the mixer
    // Call once per flight cycle. Returns absolute idle fraction to put into mixer.idle.
    WB_FAST float idle_fraction(uint32_t now_ms, bool throttle_raised) {
        if (cfg.dyn.min_rpm_100 == 0 || !cfg.bidirectional || !is_dshot_wire(cfg.protocol)) return cfg.idle;
        const bool ok = rpm_all_valid(now_ms);
        float min_hz = 1e9f;
        for (int i = 0; i < cfg.count; i++) { const float hz = telemetry(i, now_ms).rpm / 60.0f; if (hz < min_hz) min_hz = hz; }
        dyn_inc_ = dyn_.update(min_hz, ok, throttle_raised);
        return cfg.idle + dyn_inc_ * (cfg.output_limit - cfg.idle);
    }
    float dyn_idle_increase() const { return dyn_inc_; }
    uint32_t reply_crc_errors() const { return crc_bad_; }                  // bidirectional DShot: replies with a bad checksum since start
    // bit i = logical motor i has a fresh rpm (what the rpm filter and dynamic idle use)
    uint8_t rpm_valid_mask(uint32_t now_ms) const { uint8_t m = 0; for (int i = 0; i < cfg.count && i < 8; i++) if (telemetry(i, now_ms).rpm_valid) m |= (uint8_t)(1u << i); return m; }
    uint16_t pulse_hz() const { return cfg.pulse_hz ? cfg.pulse_hz : default_pulse_hz(cfg.protocol); }

private:
    static constexpr uint32_t kTestTimeoutUs = 500000;
    struct Slot { float rpm = 0, volt = 0, amps = 0; uint8_t temp = 0; uint32_t rpm_ms = 0, volt_ms = 0, amps_ms = 0; bool rpm_seen = false, amps_seen = false; uint16_t total = 0, invalid = 0; };
    struct QCmd { uint8_t slot, cmd; };

    WB_FAST float to_value(float f, const Endpoints& e) const {
        f = f < 0 ? 0 : (f > 1 ? 1 : f);
        float v;
        if (is_dshot(cfg.protocol)) v = kDshotMinThrottle + f * kDshotRange;
        else if (is_can(cfg.protocol)) v = 1.0f + f * (kCanMax - 1);
        else v = cfg.pulse.armed + f * (cfg.pulse.max - cfg.pulse.armed);
        v = v < e.low ? e.low : (v > e.high ? e.high : v);        // armed motors never fall below idle, never exceed the limit
        return round_pos(v);
    }
    WB_FAST void ask_telemetry(Frame& out) {
        const int8_t m = tlm_ask_;
        if (m < 0 || !is_dshot(cfg.protocol)) return;
        out.telem_request[cfg.order[m]] = true; tlm_ask_ = -1;
    }
    int logical_of(int slot) const { for (int i = 0; i < cfg.count; i++) if (cfg.order[i] == slot) return i; return -1; }
    bool push(uint8_t slot, uint8_t cmd) {
        const uint8_t n = (uint8_t)((q_tail_ + 1) % kQ);
        if (n == q_head_) return false;
        q_[q_tail_] = {slot, cmd}; q_tail_ = n; return true;
    }
    // returns true when it produced this cycle's frame (or a deliberate gap)
    bool run_commands(uint32_t now_us, Frame& out) {
        if (!cmd_active_) {
            if (q_head_ == q_tail_) return false;
            cur_ = q_[q_head_]; q_head_ = (uint8_t)((q_head_ + 1) % kQ);
            cmd_active_ = true; phase_ = 0; t_phase_ = now_us; sent_ = 0; repeats_ = cmd_repeats(cur_.cmd);
        }
        out.send = false;
        if (phase_ == 0) { if (now_us - t_phase_ < kCmdInitialDelayUs) return true; phase_ = 1; next_due_ = now_us; }
        if (phase_ == 1) {
            if ((int32_t)(now_us - next_due_) < 0) return true;
            out.send = true;
            for (int i = 0; i < cfg.count; i++) out.value[i] = 0;
            if (cur_.slot == kAllSlots) for (int i = 0; i < cfg.count; i++) { out.value[cfg.order[i]] = cur_.cmd; out.telem_request[cfg.order[i]] = true; }
            else { out.value[cur_.slot] = cur_.cmd; out.telem_request[cur_.slot] = true; }
            next_due_ = now_us + kCmdRepeatDelayUs;
            if (++sent_ >= repeats_) { phase_ = 2; t_phase_ = now_us; post_ = (cur_.cmd >= CMD_BEACON1 && cur_.cmd <= CMD_BEACON5) ? kCmdBeepDelayUs : kCmdRepeatDelayUs; }
            return true;
        }
        if (now_us - t_phase_ >= post_) cmd_active_ = false;
        return true;
    }

    static constexpr uint8_t kQ = 24, kAllSlots = 0xFF;
    Slot tel_[kMaxMotors];
    DynamicIdle dyn_; float dyn_inc_ = 0;
    QCmd q_[kQ]; uint8_t q_head_ = 0, q_tail_ = 0;
    int8_t tlm_ask_ = -1;
    uint32_t crc_bad_ = 0;
    bool cmd_active_ = false; QCmd cur_{0, 0}; uint8_t phase_ = 0, sent_ = 0, repeats_ = 0; uint32_t t_phase_ = 0, next_due_ = 0, post_ = 0;
    float test_frac_[kMaxMotors] = {}; uint32_t test_until_us_ = 0; float out_frac_[kMaxMotors] = {};
};

} // namespace motor
