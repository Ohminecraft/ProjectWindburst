#pragma once
#include "../config/fast_code.h"
#include "types.h"

// ============================================================================
// RC layer. Everything internal is standard 1000..2000 us (spec: "fw dung chuan RC 1000-2000").
// Receiver drivers (CRSF/SBUS/IBUS/nRF24/PPM/PWM) convert to us; nothing downstream sees raw protocol values.
// ============================================================================
namespace fc {
constexpr int kAuxCount = 8;
constexpr int kRcMin = 1000, kRcMid = 1500, kRcMax = 2000;
// Betaflight's min_check: a throttle stick below this is "low" (arming, the ground / airmode logic), and the throttle COMMAND is
// zero there - a transmitter whose stick does not come down to exactly 1000 us still gives zero throttle.
constexpr int kRcMinCheck = 1050;


// stick(-1..1) from us, with centre deadband (fraction of half-range, e.g. 0.02).
inline float rc_stick(uint16_t us, float deadband = 0.0f) {
    float v = clampf((float)((int)us - kRcMid) / 500.0f, -1.0f, 1.0f);
    if (fabsf(v) <= deadband) return 0.0f;
    return (v - signf(v) * deadband) / (1.0f - deadband);
}
inline float rc_throttle01(uint16_t us) { return clampf((float)((int)us - kRcMin) / 1000.0f, 0.0f, 1.0f); }        // where the stick IS (0.5 = middle)
// what the stick ASKS FOR as direct throttle (Betaflight fc/rc.c: min_check .. 2000 -> 0 .. 1)
inline float rc_throttle_cmd(uint16_t us) { return clampf((float)((int)us - kRcMinCheck) / (float)(kRcMax - kRcMinCheck), 0.0f, 1.0f); }

// Flight-mode boxes. Altitude Hold / Position Hold are SEPARATE switches and both REQUIRE Angle (they force it on),
// like Betaflight BOXALTHOLD/BOXPOSHOLD. Acro = neither Angle nor Stabilize.
// M_CALIB = in-flight level trim: while it is on in a steady hover the attitude is averaged, and becomes the new "level" after landing.
// M_RESCUE = GPS Rescue on a switch (rescue.h): the craft flies home and lands by itself for as long as it is on.
// M_ACRO_TRAINER = in Acro, roll / pitch are held below an angle (acro_trainer.h, Betaflight ACRO TRAINER).
// M_AIRMODE, M_ANTI_GRAVITY = once a switch is assigned to one of them, the switch turns it on and off (the settings
// general.airmode / pid.anti_gravity then only say how it works / are ignored); without a switch the setting alone decides.
// M_BLACKBOX_ERASE = flipping it on while disarmed erases the log memory (bb_service.h; Betaflight BLACKBOX ERASE).
enum Mode : uint8_t { M_ARM, M_ANGLE, M_STABILIZE, M_ALTHOLD, M_POSHOLD, M_FAILSAFE_SW, M_BEEPER, M_BLACKBOX, M_CALIB, M_RESCUE,
                      M_ACRO_TRAINER, M_AIRMODE, M_ANTI_GRAVITY, M_BLACKBOX_ERASE, M_COUNT };

struct ModeRange { uint8_t mode; uint8_t aux; uint16_t lo_us, hi_us; }; // active when lo <= aux < hi (like BF, 900..2100 clamp)

class ModeMap {
public:
    static constexpr int kMax = 16;
    ModeRange r[kMax]; int n = 0;
    bool add(uint8_t mode, uint8_t aux, uint16_t lo, uint16_t hi) { if (n >= kMax || aux >= kAuxCount) return false; r[n++] = {mode, aux, lo, hi}; return true; }
    bool has(uint8_t mode) const { for (int i = 0; i < n; i++) if (r[i].mode == mode) return true; return false; }      // a switch is assigned to it
    // bit i set => Mode i active. A mode with several ranges is active if ANY matches (BF: OR within the same mode).
    WB_FAST uint32_t resolve(const uint16_t aux_us[kAuxCount]) const {
        uint32_t m = 0;
        for (int i = 0; i < n; i++) if (aux_us[r[i].aux] >= r[i].lo_us && aux_us[r[i].aux] < r[i].hi_us) m |= 1u << r[i].mode;
        if (m & ((1u << M_ALTHOLD) | (1u << M_POSHOLD) | (1u << M_RESCUE))) m |= 1u << M_ANGLE; // hold modes and the rescue force Angle on
        return m;
    }
};

// Betaflight RATES_TYPE_ACTUAL, verified against flight/../rc.c applyActualRates():
//   centerSens = rc_rate*10, maxRate = rate*10, expo in 0..1.  deg/s = s*center + max(0,max-center) * |s|*(s^5*expo + s*(1-expo))
struct RateProfile {
    float center_sens_dps[3] = {70, 70, 70};   // UI "center sensitivity"
    float max_rate_dps[3] = {670, 670, 400};    // UI "max rate"
    float expo[3] = {0.0f, 0.0f, 0.0f};
};
inline float apply_actual_rate(const RateProfile& p, int axis, float s) {
    float a = fabsf(s), s5 = s * s * s * s * s;
    float e = a * (s5 * p.expo[axis] + s * (1.0f - p.expo[axis]));
    float extra = maxf(0.0f, p.max_rate_dps[axis] - p.center_sens_dps[axis]);
    return s * p.center_sens_dps[axis] + extra * e;
}

// ---------------------------------------------------------------------------------------------------------------------
// RcInterpreter: 16 channels in microseconds (receiver driver output) -> pilot command + active flight modes.
//   channel map default AETR (A=ch0, E=ch1, T=ch2, R=ch3), aux1..aux8 = ch4..ch11. Throttle: Betaflight-style mid / expo / limit.
// ---------------------------------------------------------------------------------------------------------------------
struct RcConfig {
    uint8_t ch_roll = 0, ch_pitch = 1, ch_throttle = 2, ch_yaw = 3, ch_aux0 = 4;
    float deadband = 0.02f;
    float throttle_mid = 0.5f, throttle_expo = 0.0f, throttle_limit = 1.0f;     // direct-throttle curve (AltHold uses the raw stick)
};
struct RcCommand {
    float roll = 0, pitch = 0, yaw = 0;           // -1..1 (pitch forward +, roll right +, yaw right +)
    float throttle_raw = 0;                       // 0..1 stick position (Altitude Hold reads it: the middle holds the height)
    float throttle_cmd = 0;                       // 0..1 direct throttle asked for (0 below min_check), before the curve
    float throttle = 0;                           // 0..1 after the curve / limit (direct throttle)
    bool throttle_low = true;                     // the stick is below min_check
    uint16_t aux[kAuxCount] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bool link_ok = false;
    uint32_t modes = 0;                           // bit i set => Mode i active
};
class RcInterpreter {
public:
    RcConfig cfg;
    ModeMap modes;
    WB_FAST RcCommand process(const uint16_t ch[16], bool link_ok) const {
        RcCommand c; c.link_ok = link_ok;
        auto stick = [&](uint16_t us) WB_FAST { const float v = rc_stick(us); if (fabsf(v) <= cfg.deadband) return 0.0f; return (v - signf(v) * cfg.deadband) / (1.0f - cfg.deadband); };
        c.roll = stick(ch[cfg.ch_roll]); c.pitch = stick(ch[cfg.ch_pitch]); c.yaw = stick(ch[cfg.ch_yaw]);
        c.throttle_raw = rc_throttle01(ch[cfg.ch_throttle]);
        c.throttle_cmd = rc_throttle_cmd(ch[cfg.ch_throttle]); c.throttle_low = ch[cfg.ch_throttle] < kRcMinCheck;
        c.throttle = curve(c.throttle_cmd);
        for (int i = 0; i < kAuxCount; i++) c.aux[i] = ch[cfg.ch_aux0 + i];
        c.modes = modes.resolve(c.aux);
        return c;
    }
    float curve(float t) const {
        const float e = t - 0.5f, x = e * (1.0f - cfg.throttle_expo) + cfg.throttle_expo * e * e * e * 4.0f, base = 0.5f + x, m = cfg.throttle_mid;
        const float out = base <= 0.5f ? base * (m / 0.5f) : m + (base - 0.5f) * ((1.0f - m) / 0.5f);
        return clampf(out, 0.0f, 1.0f) * cfg.throttle_limit;
    }
};
} // namespace fc
