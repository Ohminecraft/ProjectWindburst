/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "msp_bridge.h"
#include "telemetry.h"
#include "../proto/wbp_schema.h"
#include "../core/board_align.h"
#include "../core/simplified_tuning.h"

// ============================================================================
// MSP for the configurators (Betaflight Configurator, the SpeedyBee app): the live values they show and the settings they edit,
// in Betaflight 4.5's layouts (msp.c, API 1.46), turned into this firmware's settings. msp::Bridge asks this first
// (Bridge::set_handler); what is not here it answers itself, or with '!' (Betaflight: not supported - the configurators go on).
//
//   live      MSP_RC 105, MSP_RAW_IMU 102, MSP_RAW_GPS 106, MSP_COMP_GPS 107, MSP_ATTITUDE 108, MSP_ALTITUDE 109, MSP_ANALOG 110
//   settings  MSP_NAME 10 / SET 11, MSP_MODE_RANGES 34 / SET 35, MSP_MODE_RANGES_EXTRA 238, MSP_RX_MAP 64 / SET 65,
//             MSP_RC_TUNING 111 / SET 204, MSP_PID 112 / SET 202, MSP_BOXNAMES 116, MSP_PIDNAMES 117, MSP_ACC_TRIM 240
//   actions   MSP_EEPROM_WRITE 250 (save), MSP_REBOOT 68 (after the answer is out: reboot_pending())
//
// Units: the settings already are Betaflight's (PID gains, "Actual" rates, mode ranges in microseconds); only the packing differs.
// Rates go out as rates_type ACTUAL (3); a SET_RC_TUNING with another rates type is refused (no conversion is guessed).
// PIDs written while the tuning sliders own an axis turn the sliders off (msp_config.h follows Betaflight Configurator: typing a
// gain leaves the sliders) - only when a gain really changes, so a tool that writes back what it read changes nothing.
// Writes: refused while armed, and when msp.writes is off. Every write goes through Port::commit(section) - the same steps as
// a SECTION_SET from the app (clamped, rules applied, applied live or "reboot required"); EEPROM_WRITE saves like the app's Save.
// Portable: unit-tested in test/test_msp_config.cpp.
// ============================================================================
namespace msp {

enum ConfigCmd : uint16_t {
    NAME = 10, SET_NAME = 11, MODE_RANGES = 34, SET_MODE_RANGE = 35, RX_MAP = 64, SET_RX_MAP = 65, REBOOT = 68,
    RAW_IMU = 102, RC = 105, RAW_GPS = 106, COMP_GPS = 107, ATTITUDE = 108, ALTITUDE = 109, ANALOG_DATA = 110,   // (MSP_ANALOG: "ANALOG" is an Arduino-ESP32 macro)
    RC_TUNING = 111, PID = 112,
    BATTERY_CONFIG = 32, DATAFLASH_SUMMARY = 70, SDCARD_SUMMARY = 79, BLACKBOX_CONFIG = 80, SET_RTC = 246,
    SET_BATTERY_CONFIG = 33, VOLTAGE_METERS = 128, CURRENT_METERS = 129, VOLTAGE_METER_CONFIG = 56, SET_VOLTAGE_METER_CONFIG = 57,
    CURRENT_METER_CONFIG = 40, SET_CURRENT_METER_CONFIG = 41,
    FAILSAFE_CONFIG = 75, SET_FAILSAFE_CONFIG = 76, RXFAIL_CONFIG = 77, RSSI_CONFIG = 50, GPS_RESCUE = 135, SET_GPS_RESCUE = 225,
    SIMPLIFIED_TUNING = 140, SET_SIMPLIFIED_TUNING = 141, CALCULATE_SIMPLIFIED_PID = 142, CALCULATE_SIMPLIFIED_GYRO = 143, CALCULATE_SIMPLIFIED_DTERM = 144,
    VALIDATE_SIMPLIFIED_TUNING = 145,
    SET_BLACKBOX_CONFIG = 81,
    OSD_CONFIG = 84, SET_OSD_CONFIG = 85, OSD_CANVAS = 189,
    CF_SERIAL_CONFIG = 54, COMMON_SERIAL_CONFIG = 0x1009, COMMON_SET_SERIAL_CONFIG = 0x100A,
    PID_ADVANCED = 94, SET_PID_ADVANCED = 95, FILTER_CONFIG = 92, SET_FILTER_CONFIG = 93, GPS_CONFIG = 132, SET_GPS_CONFIG = 223, GPSSVINFO = 164,
    SET_SENSOR_CONFIG = 97, SENSOR_CONFIG_ACTIVE = 0x300A, GYRO_SENSOR_ACTIVE = 0x300D,                   // (the last two: MSP2_)
    TEXT_GET = 0x3006, TEXT_SET = 0x3007,                                                              // MSP2_GET_TEXT / MSP2_SET_TEXT
    SET_FEATURE_CONFIG = 37, BOARD_ALIGNMENT_CONFIG = 38, SET_BOARD_ALIGNMENT_CONFIG = 39, RX_CONFIG = 44, SET_RX_CONFIG = 45,
    ARMING_CONFIG = 61, SET_ARMING_CONFIG = 62, SET_ADVANCED_CONFIG = 91, SENSOR_CONFIG = 96, SENSOR_ALIGNMENT = 126, COMPASS_CONFIG = 133,
    BEEPER_CONFIG = 184, SET_BEEPER_CONFIG = 185, SET_SENSOR_ALIGNMENT = 220, SET_COMPASS_CONFIG = 224,
    MIXER_CONFIG = 42, SET_MIXER_CONFIG = 43, RC_DEADBAND = 125, SET_MOTOR_3D_CONFIG = 217, SET_RC_DEADBAND = 218,
    SET_ARMING_DISABLED = 99, BOXNAMES = 116, PIDNAMES = 117, SET_PID = 202, SET_RC_TUNING = 204, MODE_RANGES_EXTRA = 238, ACC_TRIM = 240, EEPROM_WRITE = 250,
};
constexpr int kBfModeRanges = 20;                     // MAX_MODE_ACTIVATION_CONDITION_COUNT
constexpr uint16_t kPidAdvancedLen = 61;              // MSP_PID_ADVANCED, master (API 1.49)
constexpr uint16_t kFilterConfigLen = 56;             // MSP_FILTER_CONFIG, master (API 1.49; three RPM harmonic weights)
constexpr uint8_t kBfMeterBattery = 10;              // Betaflight's VOLTAGE / CURRENT_METER_ID_BATTERY_1
constexpr uint16_t kOsdConfigLen = 261;              // MSP_OSD_CONFIG, master (API 1.49)
constexpr uint8_t kBfOsdItems = 99, kBfOsdStats = 31;  // Betaflight's OSD_ITEM_COUNT, OSD_STAT_COUNT
// osd::Element (osd/osd.h order) -> Betaflight's item: RSSI, BATT, CELL, CURRENT, MAH, MODE, NAME, FLY_TIME (timer 2), ON_TIME (timer 1),
// SATS, SPEED, HOME_DIR, HOME_DIST, ALT, VARIO, THROTTLE, CROSSHAIRS, HORIZON, WARNINGS, HEADING, PITCH, ROLL, ESC_TEMP, DISARMED,
// POWER, FLIGHT_DIST, GPS_LAT, GPS_LON, COMPASS_BAR, SIDEBARS, G_FORCE, BATT_USAGE
constexpr uint8_t kBfOsdItem[32] = {0, 1, 22, 11, 12, 7, 8, 6, 5, 14, 13, 30, 31, 15, 33, 9, 2, 3, 21, 32, 26, 27, 35, 29, 19, 47, 24, 23, 34, 4, 42, 28};
constexpr uint8_t kBfOther = 255;                     // a rangefinder / flow sensor this firmware has that is not in Betaflight's list (the App: "on")
constexpr uint8_t kLedStripDefault = 8;             // FEATURE LED_STRIP turned on with no strip set: this many LEDs (the real length: the Windburst app)
constexpr uint8_t kRatesActual = 3;                   // Betaflight RATES_TYPE_ACTUAL (BETAFLIGHT 0, RACEFLIGHT 1, KISS 2, ACTUAL 3, QUICK 4)

// What the handler needs from the firmware for writes (LinkHost + wbp::Server on the board, a fake in the tests)
class ConfigPort {
public:
    virtual ~ConfigPort() = default;
    virtual bool armed() = 0;
    virtual void commit(uint8_t section) = 0;         // a section changed in RAM: clamp, rules, apply live or mark "reboot required"
    virtual bool save() = 0;                          // write the settings to flash
    virtual void arming_disabled(bool on) { (void)on; }   // MSP_SET_ARMING_DISABLED: hold the arming off while the configurator talks
    virtual bool blackbox(wbp::T_blackbox& o) { (void)o; return false; }   // the log memory's state (bb::Service on the link task)
    // the Motors tab's test: n motors, us (1000 = stop); the firmware's own motor test (CMD_MOTOR_TEST: disarmed only, stops by itself)
    virtual bool motor_test(const uint16_t* us, int n) { (void)us; (void)n; return false; }
};

// Betaflight's box names for the permanent ids this firmware has (msp_box.c)
inline const char* bf_box_name(uint8_t id) {
    switch (id) {
    case 0: return "ARM"; case 1: return "ANGLE"; case 3: return "ALTHOLD"; case 4: return "ANTI GRAVITY"; case 11: return "POS HOLD";
    case 13: return "BEEPER"; case 17: return "CALIB"; case 26: return "BLACKBOX"; case 27: return "FAILSAFE"; case 28: return "AIR MODE"; case 29: return "3D DISABLE";
    case 31: return "BLACKBOX ERASE"; case 35: return "FLIP OVER AFTER CRASH"; case 46: return "GPS RESCUE"; case 47: return "ACRO TRAINER"; case 56: return "AUTOPILOT";
    default: return "?";
    }
}

class Config {
public:
    Config(wbp::Settings& s, ConfigPort& port) : s_(s), port_(port) {}
    // The telemetry the live answers come from (the link task's snapshot); set before each request.
    void set_telemetry(const app::Telemetry* t) { t_ = t; }
    // Bridge::Handler
    static bool handler(void* ctx, const Frame& f, uint8_t* p, uint16_t& n, bool& error) { return ((Config*)ctx)->handle(f, p, n, error); }

    bool handle(const Frame& f, uint8_t* p, uint16_t& n, bool& err) {
        n = 0; err = false;
        switch (f.cmd) {
        // ---- live values
        case RC: {                                                         // Betaflight's rcData: roll, pitch, yaw, throttle (the channel map applied), then AUX1 ..
            const wbp::S_receiver& rx = s_.receiver; const uint8_t fn[4] = {rx.ch_roll, rx.ch_pitch, rx.ch_yaw, rx.ch_throttle};
            for (int i = 0; i < 16; i++) { const int ch = i < 4 ? fn[i] : rx.ch_aux0 + (i - 4); u16(p, n, t_ && ch < 16 ? t_->rc[ch] : 1500); }
            return true; }
        case SET_MOTOR: {                                                  // the Motors tab's motor test: disarmed, writes allowed; refused when the firmware refuses
            if (port_.armed() || !s_.msp.writes || f.len < 16) { err = true; return true; }
            uint16_t us[8]; for (int i = 0; i < 8; i++) us[i] = rd16(f.payload, 2 * i);
            err = !port_.motor_test(us, 8); return true; }
        case ATTITUDE: {
            if (!t_) { err = true; return true; }
            int yaw = (int)lroundf(t_->yaw); yaw = ((yaw % 360) + 360) % 360;
            u16(p, n, (uint16_t)(int16_t)lroundf(t_->roll * 10.0f)); u16(p, n, (uint16_t)(int16_t)lroundf(t_->pitch * 10.0f)); u16(p, n, (uint16_t)(int16_t)yaw);
            return true; }
        case ALTITUDE: { const float z = t_ ? t_->z : 0.0f, vz = t_ ? t_->vz : 0.0f; u32(p, n, (uint32_t)(int32_t)lroundf(z * 100.0f)); u16(p, n, (uint16_t)(int16_t)clampf(vz * 100.0f, -32767, 32767)); return true; }
        case ANALOG_DATA: {
            const float v = t_ ? t_->batt_v : 0.0f, a = t_ ? t_->batt_amps : 0.0f, mah = t_ ? t_->batt_mah : 0.0f;
            p[n++] = (uint8_t)clampf(v * 10.0f + 0.5f, 0, 255); u16(p, n, (uint16_t)clampf(mah, 0, 65535));
            u16(p, n, (uint16_t)(t_ ? (uint32_t)t_->rssi * 1023u / 100u : 0u));                                   // (rssi 0..100 % -> 0..1023)
            u16(p, n, (uint16_t)(int16_t)clampf(a * 100.0f, -32767, 32767)); u16(p, n, (uint16_t)clampf(v * 100.0f + 0.5f, 0, 65535));
            return true; }
        case RAW_IMU: {                                                    // the App divides: acc / 2048 (g), gyro x 4 / 16.4 (deg/s); mag in 1/1090 G
            // Betaflight's frame: x forward, y LEFT, z UP (+1 g at rest; the App's sensor graphs and its board alignment wizard read it so)
            static const float f[3] = {1, -1, -1};
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->acc[k] : 0.0f) * f[k] * 2048.0f, -32767, 32767));
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->gyro_f[k] : 0.0f) * f[k] * 4.1f, -32767, 32767));
            for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)clampf((t_ ? t_->mag[k] : 0.0f) * f[k] * 1090.0f, -32767, 32767));
            return true; }
        case RAW_GPS: {
            p[n++] = t_ && t_->gps_fix ? 2 : 0; p[n++] = t_ ? t_->gps_sats : 0;
            u32(p, n, (uint32_t)(t_ ? t_->lat : 0)); u32(p, n, (uint32_t)(t_ ? t_->lon : 0));
            u16(p, n, (uint16_t)(int16_t)clampf(t_ ? t_->alt_cm / 100.0f : 0.0f, -32767, 32767));                     // metres
            u16(p, n, t_ ? t_->gs_cms : 0); u16(p, n, t_ ? t_->course_ddeg : 0);
            u16(p, n, t_ ? t_->hdop : 0); u16(p, n, t_ ? t_->hdop : 0); u16(p, n, 0);        // master: pdop (the hdop here), hdop, vdop (not known)
            p[n++] = t_ ? t_->gps_sats : 0;                                                  // satellites in view (the ones used)
            return true; }
        case COMP_GPS: {
            const uint32_t d = t_ ? t_->home_dist_m : 0; u16(p, n, (uint16_t)(d > 65535u ? 65535u : d));
            int dir = t_ ? (t_->home_dir_ddeg + 5) / 10 : 0; if (dir > 180) dir -= 360;
            u16(p, n, (uint16_t)(int16_t)dir); p[n++] = t_ && t_->gps_fix ? 1 : 0;
            return true; }
        // ---- settings, read
        case NAME: { const size_t l = strnlen((const char*)s_.osd.name, sizeof(s_.osd.name)); memcpy(p, s_.osd.name, l); n = (uint16_t)l; return true; }
        case MODE_RANGES:
            for (int i = 0; i < kBfModeRanges; i++) {
                uint8_t id = 0, aux = 0, lo = 0, hi = 0;
                if (i < 16 && used(i)) { id = kBfBox[s_.modes.mode[i]]; aux = s_.modes.aux[i]; lo = step(s_.modes.lo[i]); hi = step(s_.modes.hi[i]); }
                p[n++] = id; p[n++] = aux; p[n++] = lo; p[n++] = hi;
            }
            return true;
        case MODE_RANGES_EXTRA:                                            // per range: box id, logic (0 OR), linked to (0 none)
            p[n++] = kBfModeRanges;
            for (int i = 0; i < kBfModeRanges; i++) { p[n++] = i < 16 && used(i) ? kBfBox[s_.modes.mode[i]] : 0; p[n++] = 0; p[n++] = 0; }
            return true;
        case RX_MAP: {                                                     // the input channel of roll, pitch, yaw, throttle, AUX1 .. AUX4
            p[n++] = s_.receiver.ch_roll; p[n++] = s_.receiver.ch_pitch; p[n++] = s_.receiver.ch_yaw; p[n++] = s_.receiver.ch_throttle;
            for (int k = 0; k < 4; k++) p[n++] = (uint8_t)(s_.receiver.ch_aux0 + k);
            return true; }
        case RC_TUNING: {                                                  // Betaflight 4.5's layout, rates type ACTUAL
            const wbp::S_rates& r = s_.rates;
            p[n++] = c10(r.center_sens[0]); p[n++] = r.expo[0]; p[n++] = c10(r.max_rate[0]); p[n++] = c10(r.max_rate[1]); p[n++] = c10(r.max_rate[2]);
            p[n++] = 0;                                                    // (the old TPA rate: in MSP_PID_ADVANCED since 4.3)
            p[n++] = r.throttle_mid; p[n++] = r.throttle_expo;
            u16(p, n, 0);                                                  // (the old TPA breakpoint: in MSP_PID_ADVANCED - master sends 0)
            p[n++] = r.expo[2]; p[n++] = c10(r.center_sens[2]); p[n++] = c10(r.center_sens[1]); p[n++] = r.expo[1];
            p[n++] = r.throttle_limit < 100 ? 1 : 0; p[n++] = r.throttle_limit;                                     // throttle limit: SCALE / OFF, percent
            for (int k = 0; k < 3; k++) u16(p, n, 1998);                                                            // rate limits
            p[n++] = kRatesActual;
            p[n++] = 50;                                                   // 1.47: throttle hover (the throttle curve's: Betaflight's default, not used here)
            return true; }
        case PID: {                                                        // ROLL, PITCH, YAW, LEVEL, MAG: P I D each
            for (int a = 0; a < 3; a++) { p[n++] = s_.pid.p[a]; p[n++] = s_.pid.i[a]; p[n++] = s_.pid.d[a]; }
            p[n++] = s_.pid.angle_strength; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0;
            return true; }
        case PID_ADVANCED: {                                               // master's layout (msp.c); 0 where this firmware has nothing
            const wbp::S_pid& pd = s_.pid; const uint16_t n0 = n;
            memset(p + n, 0, kPidAdvancedLen); n += kPidAdvancedLen;
            uint8_t* o = p + n0;
            o[8] = pd.ff_transition; o[17] = s_.rates.angle_limit;
            const uint16_t ag = pd.anti_gravity ? pd.anti_gravity_gain : 0; o[21] = (uint8_t)ag; o[22] = (uint8_t)(ag >> 8);
            o[27] = pd.iterm_relax; o[28] = pd.iterm_relax_type; o[30] = pd.throttle_boost; o[31] = pd.acro_trainer_angle_limit;
            for (int a = 0; a < 3; a++) { o[32 + 2 * a] = (uint8_t)pd.f[a]; o[33 + 2 * a] = (uint8_t)(pd.f[a] >> 8); o[39 + a] = pd.d_max[a]; }
            o[42] = pd.d_max_gain; o[43] = pd.d_max_advance; o[46] = pd.iterm_relax_cutoff;
            o[47] = s_.motors.output_limit; o[49] = (uint8_t)(s_.motors.dyn_idle_min_rpm > 255 ? 255 : s_.motors.dyn_idle_min_rpm);
            o[50] = pd.ff_averaging; o[51] = pd.ff_smooth_factor; o[52] = pd.ff_boost; o[53] = pd.ff_max_rate_limit; o[54] = pd.ff_jitter_factor;
            o[57] = pd.tpa_mode; o[58] = pd.tpa_rate;
            const uint16_t bp = (uint16_t)(1000 + pd.tpa_breakpoint * 10); o[59] = (uint8_t)bp; o[60] = (uint8_t)(bp >> 8);   // (% of the throttle -> us)
            return true; }
        case GPS_CONFIG: {                                                 // provider, SBAS (Betaflight's tables are this firmware's), auto config, auto baud, home once, Galileo
            const wbp::S_gps& g = s_.gps;
            p[n++] = g.provider; p[n++] = g.sbas; p[n++] = g.auto_config; p[n++] = g.auto_baud; p[n++] = g.set_home_once; p[n++] = g.use_galileo; return true; }
        case GPSSVINFO: {                                                  // the module's satellites: as Betaflight's UBX NAV-SAT (chn = gnss id, quality = flags: quality, used)
            const uint8_t k = t_ ? (t_->sat_n > app::Telemetry::kSats ? (uint8_t)app::Telemetry::kSats : t_->sat_n) : 0;
            p[n++] = k;
            for (int i = 0; i < k; i++) { const app::Telemetry::Sat& v = t_->sat[i]; p[n++] = v.gnss; p[n++] = v.sv; p[n++] = v.flags; p[n++] = v.cno; }
            return true; }
        case FILTER_CONFIG: {                                              // master's layout (msp.c); the gyro's hardware LPF and the deprecated bytes 0
            const wbp::S_filters& fl = s_.filters; uint8_t* o = p + n; memset(o, 0, kFilterConfigLen); n += kFilterConfigLen;
            const auto w16 = [o](int at, uint16_t v) { o[at] = (uint8_t)v; o[at + 1] = (uint8_t)(v >> 8); };
            o[0] = (uint8_t)fl.gyro_lpf1_static_hz; w16(1, fl.dterm_lpf1_static_hz); w16(3, fl.yaw_lowpass_hz);
            w16(5, fl.gyro_notch1_hz); w16(7, fl.gyro_notch1_cutoff); w16(9, fl.dterm_notch_hz); w16(11, fl.dterm_notch_cutoff);
            w16(13, fl.gyro_notch2_hz); w16(15, fl.gyro_notch2_cutoff); o[17] = fl.dterm_lpf1_type;
            w16(20, fl.gyro_lpf1_static_hz); w16(22, fl.gyro_lpf2_hz); o[24] = fl.gyro_lpf1_type; o[25] = fl.gyro_lpf2_type; w16(26, fl.dterm_lpf2_hz); o[28] = fl.dterm_lpf2_type;
            w16(29, fl.gyro_lpf1_dyn_min_hz); w16(31, fl.gyro_lpf1_dyn_max_hz); w16(33, fl.dterm_lpf1_dyn_min_hz); w16(35, fl.dterm_lpf1_dyn_max_hz);
            w16(39, fl.dyn_notch_q); w16(41, fl.dyn_notch_min_hz); o[43] = fl.rpm_harmonics; o[44] = fl.rpm_min_hz; w16(45, fl.dyn_notch_max_hz);
            o[47] = fl.dterm_lpf1_dyn_expo; o[48] = fl.dyn_notch_count; w16(49, fl.rpm_fade_range_hz); w16(51, fl.rpm_q);
            for (int k = 0; k < 3; k++) o[53 + k] = fl.rpm_weights[k];
            return true; }
        case PIDNAMES: { const char* t = "ROLL;PITCH;YAW;LEVEL;MAG;"; n = (uint16_t)strlen(t); memcpy(p, t, n); return true; }
        case BOXNAMES: {                                                   // the names, in MSP_BOXIDS' order
            Info in; in.modes_set = modes_set(); uint8_t ids[kModeBoxes]; const int k = Bridge::boxes(in, ids);
            for (int i = 0; i < k; i++) { const char* nm = bf_box_name(ids[i]); const uint16_t l = (uint16_t)strlen(nm); if (n + l + 1 > kMaxReply) break; memcpy(p + n, nm, l); n += l; p[n++] = ';'; }
            return true; }
        case ACC_TRIM: u16(p, n, (uint16_t)s_.calibration.acc_trim_pitch); u16(p, n, (uint16_t)s_.calibration.acc_trim_roll); return true;
        // ---- 0.44, the Betaflight App's connect sequence (master layouts)
        case TEXT_GET: {                                                   // type, then a length-prefixed text; unknown type: refused (master)
            if (f.len < 1) { err = true; return true; }
            const uint8_t ty = f.payload[0];
            if (ty != 1 && ty != 2 && ty != 3 && ty != 4 && ty != 5 && ty != 6 && ty != 11) { err = true; return true; }
            p[n++] = ty;
            if (ty == 2) { const size_t l = strnlen((const char*)s_.osd.name, sizeof(s_.osd.name)); p[n++] = (uint8_t)l; memcpy(p + n, s_.osd.name, l); n = (uint16_t)(n + l); }
            else p[n++] = 0;                                               // pilot / profile names, build key, release name: none here
            return true; }
        case SET_RTC: return true;                                         // (no clock to set: the GPS gives the time)
        case BATTERY_CONFIG: {
            const wbp::S_battery& b = s_.battery;
            p[n++] = (uint8_t)((b.cell_min_mv + 50u) / 100u); p[n++] = (uint8_t)((b.cell_max_mv + 50u) / 100u); p[n++] = (uint8_t)((b.cell_warning_mv + 50u) / 100u);   // (0.1 V)
            u16(p, n, b.capacity_mah);
            // voltage meter: NONE 0, ADC 1, ESC 2; current meter: NONE 0, ADC 1, VIRTUAL 2, ESC 3, MSP 4 (Betaflight)
            p[n++] = bf_vmeter(b.source); p[n++] = bf_cmeter(b.current_source);
            u16(p, n, (uint16_t)((b.cell_min_mv + 5u) / 10u)); u16(p, n, (uint16_t)((b.cell_max_mv + 5u) / 10u)); u16(p, n, (uint16_t)((b.cell_warning_mv + 5u) / 10u));   // (0.01 V)
            return true; }
        case CALCULATE_SIMPLIFIED_PID: {                                   // the PID sliders sent -> P, I, D, D max, F (16 bit) of each axis
            if (f.len < 10) { err = true; return true; }
            const wbp::S_pid& pd = s_.pid; const uint8_t* q = f.payload;
            fc::PidSliders sl; sl.mode = q[0]; sl.master = q[1]; sl.pitch_d = q[2]; sl.i = q[3]; sl.d = q[4]; sl.pi = q[5]; sl.d_max = q[6]; sl.ff = q[8]; sl.pitch_pi = q[9];
            uint8_t pp[3], ii[3], dd[3], dm[3]; uint16_t ff[3];
            for (int a = 0; a < 3; a++) { pp[a] = pd.p[a]; ii[a] = pd.i[a]; dd[a] = pd.d[a]; dm[a] = pd.d_max[a]; ff[a] = pd.f[a]; }
            fc::sliders_to_pid(sl, pp, ii, dd, dm, ff);
            for (int a = 0; a < 3; a++) { p[n++] = pp[a]; p[n++] = ii[a]; p[n++] = dd[a]; p[n++] = dm[a]; u16(p, n, ff[a]); }
            return true; }
        case CALCULATE_SIMPLIFIED_GYRO: case CALCULATE_SIMPLIFIED_DTERM: { // a filter slider sent -> its cutoffs (as sent when the slider is off)
            if (f.len < 10) { err = true; return true; }
            const uint8_t* q = f.payload;
            uint16_t a1 = rd16(q, 2), a4 = rd16(q, 4), a2 = rd16(q, 6), a3 = rd16(q, 8);                 // LPF1 static, LPF2, dynamic min, max
            if (q[0]) { if (f.cmd == CALCULATE_SIMPLIFIED_GYRO) fc::gyro_filter_slider(q[1], a1, a2, a3, a4); else fc::dterm_filter_slider(q[1], a1, a2, a3, a4); }
            memset(p + n, 0, 22); filt_out(p + n, q[0], q[1], a1, a4, a2, a3); n += 22; return true; }
        case SIMPLIFIED_TUNING: {                                          // the PID sliders, then the D-term and the gyro filter sliders (master's 62 bytes)
            const wbp::S_pid& pd = s_.pid; const wbp::S_filters& fl = s_.filters; uint8_t* o = p + n; memset(o, 0, 62); n += 62;
            o[0] = pd.slider_mode; o[1] = pd.slider_master; o[2] = pd.slider_pitch_d; o[3] = pd.slider_i; o[4] = pd.slider_d; o[5] = pd.slider_pi; o[6] = pd.slider_d_max; o[8] = pd.slider_ff; o[9] = pd.slider_pitch_pi;
            filt_out(o + 18, fl.slider_dterm, fl.slider_dterm_mult, fl.dterm_lpf1_static_hz, fl.dterm_lpf2_hz, fl.dterm_lpf1_dyn_min_hz, fl.dterm_lpf1_dyn_max_hz);
            filt_out(o + 40, fl.slider_gyro, fl.slider_gyro_mult, fl.gyro_lpf1_static_hz, fl.gyro_lpf2_hz, fl.gyro_lpf1_dyn_min_hz, fl.gyro_lpf1_dyn_max_hz);
            return true; }
        case VALIDATE_SIMPLIFIED_TUNING: {                                 // are the stored gains / cutoffs what the sliders give? (PIDs, gyro, D-term)
            const wbp::S_pid& pd = s_.pid; const wbp::S_filters& fl = s_.filters;
            uint8_t pp[3], ii[3], dd[3], dm[3]; uint16_t ff[3];
            for (int a = 0; a < 3; a++) { pp[a] = pd.p[a]; ii[a] = pd.i[a]; dd[a] = pd.d[a]; dm[a] = pd.d_max[a]; ff[a] = pd.f[a]; }
            fc::sliders_to_pid(sliders_of(pd), pp, ii, dd, dm, ff);
            bool ok = true; for (int a = 0; a < 3; a++) ok = ok && pp[a] == pd.p[a] && ii[a] == pd.i[a] && dd[a] == pd.d[a] && dm[a] == pd.d_max[a] && ff[a] == pd.f[a];
            p[n++] = ok;
            uint16_t a1 = fl.gyro_lpf1_static_hz, a2 = fl.gyro_lpf1_dyn_min_hz, a3 = fl.gyro_lpf1_dyn_max_hz, a4 = fl.gyro_lpf2_hz;
            if (fl.slider_gyro) fc::gyro_filter_slider(fl.slider_gyro_mult, a1, a2, a3, a4);
            p[n++] = a1 == fl.gyro_lpf1_static_hz && a2 == fl.gyro_lpf1_dyn_min_hz && a3 == fl.gyro_lpf1_dyn_max_hz && a4 == fl.gyro_lpf2_hz;
            uint16_t b1 = fl.dterm_lpf1_static_hz, b2 = fl.dterm_lpf1_dyn_min_hz, b3 = fl.dterm_lpf1_dyn_max_hz, b4 = fl.dterm_lpf2_hz;
            if (fl.slider_dterm) fc::dterm_filter_slider(fl.slider_dterm_mult, b1, b2, b3, b4);
            p[n++] = b1 == fl.dterm_lpf1_static_hz && b2 == fl.dterm_lpf1_dyn_min_hz && b3 == fl.dterm_lpf1_dyn_max_hz && b4 == fl.dterm_lpf2_hz;
            return true; }
        case OSD_CONFIG: {                                                 // master's layout (261 bytes: a v1 jumbo frame); alarms / timers / warnings: Betaflight's defaults
            const wbp::S_osd& o = s_.osd; uint8_t* d = p + n; memset(d, 0, kOsdConfigLen); n += kOsdConfigLen;
            const auto w16 = [d](int at, uint16_t v) { d[at] = (uint8_t)v; d[at + 1] = (uint8_t)(v >> 8); };
            d[0] = o.type != wbp::OSD_TYPE_NONE ? (uint8_t)(1u | (1u << 6)) : 0;  // OSD, MSP DisplayPort device
            d[1] = 3; d[2] = 1; d[3] = 20; w16(4, 2200); d[7] = kBfOsdItems; w16(8, 100);   // video HD, metric, RSSI 20 %, 2200 mAh, (timer alarm byte), items, 100 m
            for (int e = 0; e < 32; e++) w16(10 + 2 * kBfOsdItem[e], bf_osd_pos(o.pos[e]));
            int at = 10 + 2 * kBfOsdItems; d[at++] = kBfOsdStats;
            for (int i = 0; i < kBfOsdStats; i++) d[at++] = o.stats && osd_stat_shown(i) ? 1 : 0;
            d[at++] = 2; w16(at, 0x0A00); w16(at + 2, 0x0A01); at += 4;        // timer 1: power on, timer 2: armed (seconds, alarm 10 min)
            w16(at, 0xFFFF); at += 2; d[at++] = 20; w16(at, 0xFFFF); w16(at + 2, 0x000F); at += 4;   // the 20 warnings: on
            d[at++] = 1; d[at++] = 1; d[at++] = 0; d[at++] = 0; d[at++] = 0; w16(at, 80); w16(at + 2, (uint16_t)(int16_t)-60);   // one profile, no stick overlay / camera frame; LQ 80, -60 dBm
            return true; }
        case COMMON_SERIAL_CONFIG: case CF_SERIAL_CONFIG:                  // UART0..2 (Betaflight's 50..52), no functions (the CLI assigns them, as master), 115200
            if (f.cmd == COMMON_SERIAL_CONFIG) p[n++] = 3;
            for (int u = 0; u < 3; u++) { p[n++] = (uint8_t)(50 + u); if (f.cmd == COMMON_SERIAL_CONFIG) u32(p, n, 0); else u16(p, n, 0); for (int k = 0; k < 4; k++) p[n++] = 5; }
            return true;
        case OSD_CANVAS: {                                                 // the character grid (osd.canvas; AUTO: the HD default)
            const uint8_t c = s_.osd.canvas;
            p[n++] = c == wbp::OSD_CANVAS_HDZERO_50X18 ? 50 : c == wbp::OSD_CANVAS_SD_30X16 ? 30 : 53; p[n++] = c == wbp::OSD_CANVAS_HDZERO_50X18 ? 18 : c == wbp::OSD_CANVAS_SD_30X16 ? 16 : 20;
            return true; }
        case FAILSAFE_CONFIG: {                                            // guard (0.1 s), landing time (s), throttle (us), switch mode, throttle-low delay, procedure
            const wbp::S_general& g = s_.general;
            p[n++] = (uint8_t)((g.fs_guard_ms + 50u) / 100u > 255u ? 255u : (g.fs_guard_ms + 50u) / 100u); p[n++] = (uint8_t)(g.fs_max_descent_s > 255 ? 255 : g.fs_max_descent_s);
            u16(p, n, g.fs_throttle); p[n++] = 0; u16(p, n, 100); p[n++] = bf_procedure(s_.nav.fs_procedure); return true; }   // (switch STAGE1, 10 s: Betaflight's defaults, none here)
        case RXFAIL_CONFIG:                                                // per channel: sticks AUTO, the AUX channels HOLD (the last value is kept)
            for (int i = 0; i < 16; i++) { p[n++] = i < 4 ? 0 : 1; u16(p, n, 1500); }
            return true;
        case RSSI_CONFIG: p[n++] = 0; return true;                         // no RSSI channel (the link quality comes from the receiver itself)
        case GPS_RESCUE: {                                                 // master's 26 bytes; the autopilot throttles: Betaflight's defaults (the hover is the measured one)
            const wbp::S_nav& v = s_.nav; const float hv = t_ ? t_->hover_throttle : 0.0f;
            u16(p, n, v.ph_angle_max); u16(p, n, v.rescue_return_alt); u16(p, n, v.rescue_descent_dist); u16(p, n, (uint16_t)(v.rescue_ground_speed * 10u));
            u16(p, n, 1100); u16(p, n, 1700); u16(p, n, (uint16_t)(hv > 0.0f ? 1000.0f + hv * 1000.0f + 0.5f : 1275.0f));
            p[n++] = 1; p[n++] = v.rescue_min_sats;                            // (sanity checks ON)
            u16(p, n, (uint16_t)(v.rescue_ascend_rate * 10u)); u16(p, n, (uint16_t)(v.rescue_descend_rate * 10u));
            p[n++] = v.rescue_allow_no_fix; p[n++] = bf_rescue_alt(v.rescue_alt_mode); u16(p, n, v.rescue_min_start_dist); u16(p, n, v.rescue_initial_climb);
            return true; }
        case VOLTAGE_METERS:                                               // one meter, battery 1 (id 10): 0.1 V
            p[n++] = kBfMeterBattery; p[n++] = (uint8_t)(t_ && t_->batt_v > 0 ? (t_->batt_v * 10.0f + 0.5f > 255.0f ? 255 : (int)(t_->batt_v * 10.0f + 0.5f)) : 0); return true;
        case CURRENT_METERS: {                                             // battery 1: mAh used, mA (a negative current: 0)
            const float mah = t_ ? t_->batt_mah : 0.0f, ma = t_ ? t_->batt_amps * 1000.0f : 0.0f;
            p[n++] = kBfMeterBattery; u16(p, n, (uint16_t)(mah <= 0 ? 0 : mah > 65535.0f ? 65535 : (int)mah)); u16(p, n, (uint16_t)(ma <= 0 ? 0 : ma > 65535.0f ? 65535 : (int)(ma + 0.5f))); return true; }
        case VOLTAGE_METER_CONFIG: {                                       // one ADC resistor divider: Betaflight's ratio = scale / (divider value x multiplier)
            const uint16_t d = s_.battery.divider;                         // (ratio x 100)
            p[n++] = 1; p[n++] = 5; p[n++] = kBfMeterBattery; p[n++] = 0;
            if (d <= 2550) { p[n++] = (uint8_t)((d + 5u) / 10u); p[n++] = 10; } else { p[n++] = (uint8_t)((d + 50u) / 100u > 255u ? 255u : (d + 50u) / 100u); p[n++] = 1; }
            p[n++] = 1; return true; }
        case CURRENT_METER_CONFIG:                                         // one ADC current sensor: scale (0.1 mV / A), offset (mA) - Betaflight's own units
            p[n++] = 1; p[n++] = 6; p[n++] = kBfMeterBattery; p[n++] = 1; u16(p, n, (uint16_t)s_.battery.current_scale); u16(p, n, (uint16_t)s_.battery.current_offset); return true;
        case BLACKBOX_CONFIG: {                                            // supported, device, rate num, rate denom, P ratio, sample rate, fields masked
            const wbp::S_blackbox& b = s_.blackbox;
            const uint8_t sr = sample_rate(); const uint16_t denom = (uint16_t)(1u << sr);
            p[n++] = 1; p[n++] = b.device; p[n++] = 1; p[n++] = (uint8_t)denom;
            u16(p, n, (uint16_t)((32u * pid_hz() / 1000u) / denom)); p[n++] = sr; u32(p, n, 0);
            return true; }
        case DATAFLASH_SUMMARY: {                                          // flags (READY 1, SUPPORTED 2), sectors, size, used
            wbp::T_blackbox o; const bool flash = port_.blackbox(o) && o.device == wbp::BB_DEVICE_FLASH && o.state != wbp::BB_STATE_NO_STORAGE && o.state != wbp::BB_STATE_OFF;
            p[n++] = flash ? (uint8_t)(2u | (o.state != wbp::BB_STATE_ERASING ? 1u : 0u)) : 0;
            u32(p, n, flash ? o.capacity / 4096u : 0); u32(p, n, flash ? o.capacity : 0); u32(p, n, flash ? o.used : 0);
            return true; }
        case SDCARD_SUMMARY: {                                             // flags (SUPPORTED 1), state (NOT_PRESENT 0, FATAL 1, READY 4), last error, free / total kB
            wbp::T_blackbox o; const bool sd = port_.blackbox(o) && o.device == wbp::BB_DEVICE_SDCARD;
            const bool ready = sd && o.state != wbp::BB_STATE_NO_STORAGE && o.state != wbp::BB_STATE_OFF;
            p[n++] = sd ? 1 : 0; p[n++] = ready ? 4 : 0; p[n++] = 0;
            u32(p, n, ready && o.capacity_kb > o.used_kb ? o.capacity_kb - o.used_kb : 0); u32(p, n, ready ? o.capacity_kb : 0);
            return true; }
        case TEXT_SET:
            if (!s_.msp.writes || port_.armed()) { err = true; refused_++; return true; }
            err = !write(f);
            if (err) refused_++;
            return true;
        // ---- 0.44, the Betaflight App's Configuration / Setup tabs (master layouts)
        case FEATURE_CONFIG: u32(p, n, features()); return true;
        case RX_CONFIG: {                                                  // 39 bytes
            const wbp::S_receiver& rx = s_.receiver;
            p[n++] = bf_provider(rx.protocol);
            u16(p, n, rx.max_check); u16(p, n, rx.mid_rc); u16(p, n, rx.min_check); p[n++] = 0;    // maxcheck, midrc, mincheck (the Receiver tab's stick thresholds), Spektrum bind
            u16(p, n, 885); u16(p, n, 2115); p[n++] = 0; p[n++] = 0;          // rx_min / max_usec (Betaflight's defaults), (old interpolation)
            u16(p, n, (uint16_t)(1000u + s_.general.airmode_start_throttle * 10u));   // airmode starts at this throttle (us)
            p[n++] = 0; u32(p, n, 0); p[n++] = 0;                              // RX SPI protocol, id, channels
            p[n++] = 0; p[n++] = 0; p[n++] = 0;                                // FPV camera angle, (smoothing channels, type)
            p[n++] = 0; p[n++] = 0; p[n++] = rx.rc_smoothing_throttle; p[n++] = 0;   // setpoint / throttle cutoff (0 = auto), throttle smoothness, (derivative type)
            p[n++] = 0; p[n++] = rx.rc_smoothing_factor; p[n++] = rx.rc_smoothing ? 1 : 0;   // USB type, roll / pitch / yaw smoothness, smoothing on
            for (int k = 0; k < 7; k++) p[n++] = 0;                            // ExpressLRS UID, model id
            return true; }
        case ARMING_CONFIG: p[n++] = 5; p[n++] = 0; p[n++] = s_.general.max_arm_angle; p[n++] = 0; return true;   // auto disarm (none here: Betaflight's 5 s), -, small angle, gyro cal on first arm
        case BEEPER_CONFIG: u32(p, n, bf_beeper_off(s_.beeper.conditions)); p[n++] = s_.beeper.dshot_beacon_tone; u32(p, n, bf_beeper_off(s_.beeper.motor_conditions)); return true;
        case SENSOR_CONFIG:                                                // what is set up: acc, baro, mag (DEFAULT 0 / NONE 1), rangefinder, flow (NONE 0), pitot (NONE 1)
            p[n++] = 0; p[n++] = s_.sensors.baro_enabled ? 0 : 1; p[n++] = s_.sensors.mag_enabled ? bf_mag(s_.sensors.mag_model) : 1;
            p[n++] = s_.sensors.range_enabled ? kBfOther : 0; p[n++] = s_.flow.type != wbp::FLOW_TYPE_NONE ? kBfOther : 0; p[n++] = 1; return true;
        case SENSOR_CONFIG_ACTIVE: {                                       // what runs: gyro (NONE 0, DEFAULT 1), acc, baro, mag, rangefinder, flow; pitot not built (0xFF)
            const uint16_t h = health();
            p[n++] = (h & wbp::HEALTH_IMU) ? 1 : 0; p[n++] = (h & wbp::HEALTH_IMU) ? 0 : 1; p[n++] = (h & wbp::HEALTH_BARO) ? 0 : 1;
            p[n++] = (h & wbp::HEALTH_MAG_READY) ? bf_mag(s_.sensors.mag_model) : 1;
            p[n++] = (h & wbp::HEALTH_RANGE) ? kBfOther : 0; p[n++] = (h & wbp::HEALTH_FLOW) ? kBfOther : 0; p[n++] = 0xFF; return true; }
        case GYRO_SENSOR_ACTIVE: {                                         // gyro places, then each one found (NONE 0, DEFAULT 1)
            const uint16_t h = health(); p[n++] = 2; p[n++] = (h & wbp::HEALTH_IMU) ? 1 : 0; p[n++] = (h & wbp::HEALTH_IMU2) ? 1 : 0; return true; }
        case ADVANCED_CONFIG:                                              // the bridge's answer, with this firmware's settings (the real PID loop divider)
            p[n++] = 1; p[n++] = s_.general.pid_denom; p[n++] = 0; p[n++] = bf_motor(s_.motors.protocol);
            u16(p, n, s_.motors.pulse_hz); u16(p, n, (uint16_t)(s_.motors.idle_x10 * 10u));
            p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 0; p[n++] = 48; u16(p, n, 125); u16(p, n, 0); p[n++] = 0;
            p[n++] = 0; p[n++] = 1;                                        // debug mode NONE, debug mode count
            return true;
        case BOARD_ALIGNMENT_CONFIG: { int a[3]; board_bf(a); for (int k = 0; k < 3; k++) u16(p, n, (uint16_t)(int16_t)a[k]); return true; }   // Betaflight's angles (core/board_align.h)
        case SENSOR_ALIGNMENT: {                                           // Betaflight: DEFAULT 0, CW0 1 .. CW270_FLIP 8 (here CW0 0 .. 7)
            p[n++] = 1; p[n++] = 1;                                        // gyro / acc CW0: the chip's orientation is in BOARD_ALIGNMENT's angles
            p[n++] = (uint8_t)(s_.sensors.mag_align + 1);
            p[n++] = (uint8_t)(1u | ((health() & wbp::HEALTH_IMU2) ? 2u : 0u)); p[n++] = (uint8_t)(s_.sensors.gyro_use + 1);   // gyros detected (bit per gyro), enabled (FIRST 1, SECOND 2, BOTH 3)
            u16(p, n, (uint16_t)s_.sensors.mag_trim_roll); u16(p, n, (uint16_t)s_.sensors.mag_trim_pitch); u16(p, n, (uint16_t)s_.sensors.mag_trim_yaw);   // 0.1 deg
            return true; }
        case COMPASS_CONFIG: u16(p, n, (uint16_t)s_.sensors.mag_declination); return true;   // 0.1 deg, east positive
        case MOTOR_3D_CONFIG: u16(p, n, s_.motors.deadband3d_low); u16(p, n, s_.motors.deadband3d_high); u16(p, n, s_.motors.neutral3d); return true;   // 3D, PWM-type ESCs
        case RC_DEADBAND:                                                  // roll / pitch, yaw (us), Position Hold's (none here), 3D throttle (us)
            p[n++] = s_.receiver.deadband; p[n++] = s_.receiver.yaw_deadband; p[n++] = 0; u16(p, n, s_.receiver.deadband3d_throttle); return true;
        case MIXER_CONFIG: p[n++] = bf_mixer(s_.motors.layout); p[n++] = s_.motors.props_out ? 1 : 0; return true;   // the Motors tab's frame, "motor direction reversed"
        // ---- the arming held off while the configurator is there (Betaflight takes it always: armed or not, writes allowed or not;
        //      armed, Betaflight disarms - here it only holds the next arming off, nothing is cut in the air)
        case SET_ARMING_DISABLED:
            if (f.len < 1) { err = true; return true; }
            port_.arming_disabled(f.payload[0] != 0); return true;
        // ---- settings, write
        case SET_NAME: case SET_MODE_RANGE: case SET_RX_MAP: case SET_RC_TUNING: case SET_PID: case SET_PID_ADVANCED: case SET_FILTER_CONFIG: case SET_GPS_CONFIG: case SET_BATTERY_CONFIG: case SET_VOLTAGE_METER_CONFIG: case SET_CURRENT_METER_CONFIG:
        case SET_FAILSAFE_CONFIG: case SET_GPS_RESCUE: case SET_SIMPLIFIED_TUNING: case SET_BLACKBOX_CONFIG: case SET_OSD_CONFIG: case EEPROM_WRITE: case REBOOT:
        case SET_FEATURE_CONFIG: case SET_RX_CONFIG: case SET_ARMING_CONFIG: case SET_BEEPER_CONFIG: case SET_ADVANCED_CONFIG:
        case SET_BOARD_ALIGNMENT_CONFIG: case SET_SENSOR_ALIGNMENT: case SET_COMPASS_CONFIG: case SET_MIXER_CONFIG: case SET_MOTOR_3D_CONFIG: case SET_RC_DEADBAND: case SET_SENSOR_CONFIG:
            if (!s_.msp.writes || port_.armed()) { err = true; refused_++; return true; }
            err = !write(f);
            if (err) refused_++;
            return true;
        default: return false;
        }
    }
    bool reboot_pending() { const bool r = reboot_; reboot_ = false; return r; }   // MSP_REBOOT was answered: restart now
    uint32_t refused() const { return refused_; }

private:
    bool write(const Frame& f) {
        const uint8_t* q = f.payload; const uint16_t len = f.len;
        switch (f.cmd) {
        case SET_NAME: {
            if (len > sizeof(s_.osd.name)) return false;
            memset(s_.osd.name, 0, sizeof(s_.osd.name));
            for (uint16_t i = 0; i < len; i++) { const uint8_t c = q[i]; s_.osd.name[i] = c >= 32 && c < 127 ? c : '?'; }
            port_.commit(wbp::SEC_OSD); return true; }
        case SET_MODE_RANGE: {
            if (len < 5 || q[0] >= kBfModeRanges) return false;
            const int i = q[0];
            if (i >= 16) return q[3] == q[4];                              // (ranges 17 .. 20 do not exist here: only "unused" is accepted)
            if (q[3] >= q[4]) { s_.modes.lo[i] = 0; s_.modes.hi[i] = 0; s_.modes.aux[i] = 0; s_.modes.mode[i] = 0; port_.commit(wbp::SEC_MODES); return true; }   // start = end: the range is cleared
            int mode = -1; for (int m = 0; m < kModeBoxes; m++) if (kBfBox[m] == q[1]) mode = m;
            if (mode < 0 || q[2] > 15 || q[4] > 48) return false;
            s_.modes.mode[i] = (uint8_t)mode; s_.modes.aux[i] = q[2]; s_.modes.lo[i] = (uint16_t)(900 + q[3] * 25); s_.modes.hi[i] = (uint16_t)(900 + q[4] * 25);
            port_.commit(wbp::SEC_MODES); return true; }
        case SET_RX_MAP: {
            if (len < 4) return false;
            for (int k = 0; k < 4; k++) if (q[k] > 15) return false;
            s_.receiver.ch_roll = q[0]; s_.receiver.ch_pitch = q[1]; s_.receiver.ch_yaw = q[2]; s_.receiver.ch_throttle = q[3];
            port_.commit(wbp::SEC_RECEIVER); return true; }
        case SET_RC_TUNING: {
            if (len < 23 || q[22] != kRatesActual) return false;          // (only "Actual" rates: no guessing at a conversion)
            wbp::S_rates& r = s_.rates;
            r.center_sens[0] = (uint16_t)(q[0] * 10u); r.expo[0] = q[1]; r.max_rate[0] = (uint16_t)(q[2] * 10u); r.max_rate[1] = (uint16_t)(q[3] * 10u); r.max_rate[2] = (uint16_t)(q[4] * 10u);
            r.throttle_mid = q[6]; r.throttle_expo = q[7];
            r.expo[2] = q[10]; r.center_sens[2] = (uint16_t)(q[11] * 10u); r.center_sens[1] = (uint16_t)(q[12] * 10u); r.expo[1] = q[13];
            r.throttle_limit = q[14] == 0 ? 100 : q[15];
            port_.commit(wbp::SEC_RATES); return true; }
        case SET_PID: {
            if (len < 12) return false;
            bool changed = false;
            for (int a = 0; a < 3; a++) {
                changed |= s_.pid.p[a] != q[a * 3] || s_.pid.i[a] != q[a * 3 + 1] || s_.pid.d[a] != q[a * 3 + 2];
                s_.pid.p[a] = q[a * 3]; s_.pid.i[a] = q[a * 3 + 1]; s_.pid.d[a] = q[a * 3 + 2];
            }
            s_.pid.angle_strength = q[9];
            if (changed) s_.pid.slider_mode = wbp::SLIDER_MODE_OFF;       // typed gains: the sliders no longer own them
            port_.commit(wbp::SEC_PID); return true; }
        case SET_OSD_CONFIG: {                                             // -1: the general part (not kept); else an item's position, or (screen 0) a statistic
            if (len < 1) return false;
            if (q[0] == 0xFF) { if (len < 9) return false; return true; }     // (video system, units, alarms, warnings, profile: none to set here)
            if (len < 3) return false;
            const uint16_t v = rd16(q, 1); const uint8_t screen = len >= 4 ? q[3] : 1;
            if (screen == 0 && q[0] < kBfOsdStats) { if (osd_stat_shown(q[0])) { s_.osd.stats = v ? 1 : 0; port_.commit(wbp::SEC_OSD); } return true; }
            if (q[0] >= kBfOsdItems) return false;
            for (int e = 0; e < 32; e++) if (kBfOsdItem[e] == q[0]) { s_.osd.pos[e] = from_bf_osd_pos(v); port_.commit(wbp::SEC_OSD); }
            return true; }
        case SET_BLACKBOX_CONFIG: {                                        // device, (rate num / denom), P ratio, sample rate; refused while logging (Betaflight)
            if (len < 3 || q[0] > wbp::BB_DEVICE_SDCARD) return false;
            { wbp::T_blackbox o; if (port_.blackbox(o) && (o.state == wbp::BB_STATE_LOGGING || o.state == wbp::BB_STATE_ERASING)) return false; }
            s_.blackbox.device = q[0];
            if (len >= 6) { const uint8_t sr = q[5] > 4 ? 4 : q[5]; if (sr != sample_rate()) { const uint32_t hz = pid_hz() >> sr; s_.blackbox.rate_hz = (uint16_t)(hz < 10 ? 10 : hz > 2000 ? 2000 : hz); } }   // (read and written back: the rate kept)
            port_.commit(wbp::SEC_BLACKBOX); return true; }
        case SET_SIMPLIFIED_TUNING: {                                      // the sliders; the gains / cutoffs they own follow when the sections are committed (settings_rules.h)
            if (len < 62) return false;
            wbp::S_pid& pd = s_.pid; wbp::S_filters& fl = s_.filters;
            if (q[0] > wbp::SLIDER_MODE_RPY) return false;
            pd.slider_mode = q[0]; pd.slider_master = q[1]; pd.slider_pitch_d = q[2]; pd.slider_i = q[3]; pd.slider_d = q[4]; pd.slider_pi = q[5]; pd.slider_d_max = q[6]; pd.slider_ff = q[8]; pd.slider_pitch_pi = q[9];
            fl.slider_dterm = q[18] ? 1 : 0; fl.slider_dterm_mult = q[19]; fl.slider_gyro = q[40] ? 1 : 0; fl.slider_gyro_mult = q[41];
            if (!fl.slider_dterm) { fl.dterm_lpf1_static_hz = rd16(q, 20); fl.dterm_lpf2_hz = rd16(q, 22); fl.dterm_lpf1_dyn_min_hz = rd16(q, 24); fl.dterm_lpf1_dyn_max_hz = rd16(q, 26); }
            if (!fl.slider_gyro) { fl.gyro_lpf1_static_hz = rd16(q, 42); fl.gyro_lpf2_hz = rd16(q, 44); fl.gyro_lpf1_dyn_min_hz = rd16(q, 46); fl.gyro_lpf1_dyn_max_hz = rd16(q, 48); }
            port_.commit(wbp::SEC_PID); port_.commit(wbp::SEC_FILTERS); return true; }
        case SET_FAILSAFE_CONFIG: {                                        // guard, landing time, throttle, (switch mode, throttle-low delay: not kept), procedure
            if (len < 8) return false;
            int pr = -1;                                                   // (read and written back: HOLD kept)
            if (q[7] != bf_procedure(s_.nav.fs_procedure)) { pr = q[7] == 0 ? wbp::FS_PROCEDURE_DROP : q[7] == 1 ? wbp::FS_PROCEDURE_LAND : q[7] == 2 ? wbp::FS_PROCEDURE_RESCUE : -2; if (pr == -2) return false; }
            s_.general.fs_guard_ms = (uint16_t)(q[0] * 100u); s_.general.fs_max_descent_s = q[1]; s_.general.fs_throttle = rd16(q, 2);   // (the range check clamps)
            if (pr >= 0) s_.nav.fs_procedure = (uint8_t)pr;
            port_.commit(wbp::SEC_GENERAL); port_.commit(wbp::SEC_NAV); return true; }
        case SET_GPS_RESCUE: {                                             // master's layout; the autopilot throttles and sanity checks are not kept
            if (len < 16) return false;
            wbp::S_nav& v = s_.nav;
            int am = -1;
            if (len >= 22 && q[21] != bf_rescue_alt(v.rescue_alt_mode)) { if (q[21] > 2) return false; am = q[21]; }   // (read and written back: CONE kept)
            const auto u8c = [](uint16_t x) { return (uint8_t)(x > 255 ? 255 : x); };
            v.ph_angle_max = u8c(rd16(q, 0)); v.rescue_return_alt = u8c(rd16(q, 2)); v.rescue_descent_dist = u8c(rd16(q, 4)); v.rescue_ground_speed = u8c((uint16_t)((rd16(q, 6) + 5u) / 10u));
            v.rescue_min_sats = q[15];
            if (len >= 22) { v.rescue_ascend_rate = u8c((uint16_t)((rd16(q, 16) + 5u) / 10u)); v.rescue_descend_rate = u8c((uint16_t)((rd16(q, 18) + 5u) / 10u)); v.rescue_allow_no_fix = q[20] ? 1 : 0; if (am >= 0) v.rescue_alt_mode = (uint8_t)am; }
            if (len >= 24) v.rescue_min_start_dist = u8c(rd16(q, 22));
            if (len >= 26) v.rescue_initial_climb = u8c(rd16(q, 24));
            port_.commit(wbp::SEC_NAV); return true; }
        case SET_BATTERY_CONFIG: {                                         // the cell limits (0.1 V, or 0.01 V in the long form), capacity, the meters
            if (len < 7) return false;
            wbp::S_battery& b = s_.battery;
            uint16_t mn = (uint16_t)(q[0] * 10), mx = (uint16_t)(q[1] * 10), wn = (uint16_t)(q[2] * 10);   // (0.01 V)
            if (len >= 13) { mn = rd16(q, 7); mx = rd16(q, 9); wn = rd16(q, 11); }
            if (mn < 100 || mx > 500 || mn > wn || wn > mx) return false;      // Betaflight's check
            int vs = -1, cs = -1;                                          // a meter read and written back keeps AUTO / DroneCAN
            if (q[5] != bf_vmeter(b.source)) { vs = q[5] == 0 ? wbp::BATT_SOURCE_NONE : q[5] == 1 ? wbp::BATT_SOURCE_ADC : q[5] == 2 ? wbp::BATT_SOURCE_ESC : -2; if (vs == -2) return false; }
            if (q[6] != bf_cmeter(b.current_source)) { cs = q[6] == 0 ? wbp::CURR_SOURCE_NONE : q[6] == 1 ? wbp::CURR_SOURCE_ADC : q[6] == 3 ? wbp::CURR_SOURCE_ESC : -2; if (cs == -2) return false; }   // (VIRTUAL 2, MSP 4: none here)
            b.cell_min_mv = (uint16_t)(mn * 10); b.cell_max_mv = (uint16_t)(mx * 10); b.cell_warning_mv = (uint16_t)(wn * 10); b.capacity_mah = rd16(q, 3);
            if (vs >= 0) b.source = (uint8_t)vs;
            if (cs >= 0) b.current_source = (uint8_t)cs;
            port_.commit(wbp::SEC_BATTERY); return true; }
        case SET_VOLTAGE_METER_CONFIG: {                                   // id, scale, divider value, multiplier -> battery.divider (ratio x 100)
            if (len < 4 || q[0] != kBfMeterBattery || q[2] == 0 || q[3] == 0) return false;
            const uint32_t d = (uint32_t)q[1] * 100u / ((uint32_t)q[2] * q[3]);
            s_.battery.divider = (uint16_t)(d > 65535u ? 65535u : d); port_.commit(wbp::SEC_BATTERY); return true; }
        case SET_CURRENT_METER_CONFIG:                                     // id, scale, offset: the ADC sensor only (no virtual meter here)
            if (len < 5 || q[0] != kBfMeterBattery) return false;
            s_.battery.current_scale = (int16_t)rd16(q, 1); s_.battery.current_offset = (int16_t)rd16(q, 3); port_.commit(wbp::SEC_BATTERY); return true;
        case SET_GPS_CONFIG: {                                             // provider, SBAS, auto config, auto baud (+ home once, Galileo); a value out of the tables: refused
            if (len < 4 || q[0] > wbp::GPS_PROVIDER_DRONECAN || q[1] > wbp::SBAS_NONE) return false;
            wbp::S_gps& g = s_.gps;
            g.provider = q[0]; g.sbas = q[1]; g.auto_config = q[2] ? 1 : 0; g.auto_baud = q[3] ? 1 : 0;
            if (len >= 6) { g.set_home_once = q[4] ? 1 : 0; g.use_galileo = q[5] ? 1 : 0; }
            port_.commit(wbp::SEC_GPS); return true; }
        case SET_FILTER_CONFIG: {                                          // master's layout (see FILTER_CONFIG); the range check clamps
            if (len < kFilterConfigLen) return false;
            wbp::S_filters& fl = s_.filters;
            const uint16_t g1 = rd16(q, 20), g2 = rd16(q, 22), gmin = rd16(q, 29), gmax = rd16(q, 31), d1 = rd16(q, 1), d2 = rd16(q, 26), dmin = rd16(q, 33), dmax = rd16(q, 35);
            // typed gyro / D-term lowpass values are no longer the filter sliders' (Betaflight's App turns its sliders off the same way)
            if (g1 != fl.gyro_lpf1_static_hz || g2 != fl.gyro_lpf2_hz || gmin != fl.gyro_lpf1_dyn_min_hz || gmax != fl.gyro_lpf1_dyn_max_hz || q[24] != fl.gyro_lpf1_type || q[25] != fl.gyro_lpf2_type) fl.slider_gyro = 0;
            if (d1 != fl.dterm_lpf1_static_hz || d2 != fl.dterm_lpf2_hz || dmin != fl.dterm_lpf1_dyn_min_hz || dmax != fl.dterm_lpf1_dyn_max_hz || q[17] != fl.dterm_lpf1_type || q[28] != fl.dterm_lpf2_type) fl.slider_dterm = 0;
            fl.gyro_lpf1_static_hz = g1; fl.gyro_lpf2_hz = g2; fl.gyro_lpf1_dyn_min_hz = gmin; fl.gyro_lpf1_dyn_max_hz = gmax; fl.gyro_lpf1_type = q[24]; fl.gyro_lpf2_type = q[25];
            fl.dterm_lpf1_static_hz = d1; fl.dterm_lpf2_hz = d2; fl.dterm_lpf1_dyn_min_hz = dmin; fl.dterm_lpf1_dyn_max_hz = dmax; fl.dterm_lpf1_type = q[17]; fl.dterm_lpf2_type = q[28];
            fl.yaw_lowpass_hz = rd16(q, 3); fl.gyro_notch1_hz = rd16(q, 5); fl.gyro_notch1_cutoff = rd16(q, 7); fl.dterm_notch_hz = rd16(q, 9); fl.dterm_notch_cutoff = rd16(q, 11);
            fl.gyro_notch2_hz = rd16(q, 13); fl.gyro_notch2_cutoff = rd16(q, 15);
            fl.dyn_notch_q = rd16(q, 39); fl.dyn_notch_min_hz = rd16(q, 41); fl.rpm_harmonics = q[43]; fl.rpm_min_hz = q[44]; fl.dyn_notch_max_hz = rd16(q, 45);
            fl.dterm_lpf1_dyn_expo = q[47]; fl.dyn_notch_count = q[48]; fl.rpm_fade_range_hz = rd16(q, 49); fl.rpm_q = rd16(q, 51);
            for (int k = 0; k < 3; k++) fl.rpm_weights[k] = q[53 + k];
            port_.commit(wbp::SEC_FILTERS); return true; }
        case SET_PID_ADVANCED: {                                           // master's layout (see PID_ADVANCED); what this firmware has not is skipped
            if (len < kPidAdvancedLen) return false;
            wbp::S_pid& pd = s_.pid;
            bool changed = false;
            for (int a = 0; a < 3; a++) { const uint16_t f = rd16(q, 32 + 2 * a); changed |= pd.f[a] != f || pd.d_max[a] != q[39 + a]; pd.f[a] = f > 1000 ? 1000 : f; pd.d_max[a] = q[39 + a]; }
            pd.ff_transition = q[8]; s_.rates.angle_limit = q[17];
            const uint16_t ag = rd16(q, 21); pd.anti_gravity = ag != 0; if (ag) pd.anti_gravity_gain = (uint8_t)(ag > 250 ? 250 : ag);   // (0: off)
            pd.iterm_relax = q[27]; pd.iterm_relax_type = q[28]; pd.throttle_boost = q[30]; pd.acro_trainer_angle_limit = q[31];
            pd.d_max_gain = q[42]; pd.d_max_advance = q[43]; pd.iterm_relax_cutoff = q[46];
            s_.motors.output_limit = q[47]; s_.motors.dyn_idle_min_rpm = q[49];
            pd.ff_averaging = q[50]; pd.ff_smooth_factor = q[51]; pd.ff_boost = q[52]; pd.ff_max_rate_limit = q[53]; pd.ff_jitter_factor = q[54];
            pd.tpa_mode = q[57]; pd.tpa_rate = q[58];
            { const uint16_t bp = rd16(q, 59); pd.tpa_breakpoint = (uint8_t)(bp <= 1000 ? 0 : bp >= 2000 ? 100 : (bp - 1000 + 5) / 10); }   // (us -> % of the throttle)
            if (changed) pd.slider_mode = wbp::SLIDER_MODE_OFF;          // (as SET_PID: typed F / D max are no longer the sliders')
            port_.commit(wbp::SEC_PID); port_.commit(wbp::SEC_MOTORS); port_.commit(wbp::SEC_RATES); return true; }
        case TEXT_SET: {                                                   // type, length, text: the craft name; others are accepted and not kept
            if (len < 2 || q[1] > len - 2) return false;
            if (q[0] != 2) return q[0] == 1 || q[0] == 3 || q[0] == 4 || q[0] == 11;
            if (q[1] > sizeof(s_.osd.name)) return false;
            memset(s_.osd.name, 0, sizeof(s_.osd.name));
            for (uint8_t i = 0; i < q[1]; i++) { const uint8_t c = q[2 + i]; s_.osd.name[i] = c >= 32 && c < 127 ? c : '?'; }
            port_.commit(wbp::SEC_OSD); return true; }
        case SET_FEATURE_CONFIG: {                                         // GPS, RANGEFINDER, AIRMODE, 3D, TELEMETRY, LED_STRIP, OSD, ESC_SENSOR, the receiver's kind; other bits not kept
            if (len < 4) return false;
            const uint32_t b = rd32(q, 0);
            s_.sensors.gps_enabled = (b >> 7) & 1; s_.sensors.range_enabled = (b >> 9) & 1; s_.motors.mode3d = (b >> 12) & 1;
            s_.receiver.telemetry = (b >> 10) & 1; s_.motors.esc_sensor = (b >> 27) & 1;
            if (!(b & (1u << 16))) s_.led.count = 0; else if (!s_.led.count) s_.led.count = kLedStripDefault;    // (Betaflight sends no strip length)
            if (!(b & (1u << 18))) s_.osd.type = wbp::OSD_TYPE_NONE; else if (s_.osd.type == wbp::OSD_TYPE_NONE) s_.osd.type = wbp::OSD_TYPE_MSP_DISPLAYPORT;
            port_.commit(wbp::SEC_LED); port_.commit(wbp::SEC_OSD);
            if (!(b & (1u << 22))) s_.general.airmode = wbp::AIRMODE_OFF;
            else if (s_.general.airmode == wbp::AIRMODE_OFF) s_.general.airmode = wbp::AIRMODE_AUTO;      // (ALWAYS stays ALWAYS)
            uint8_t& pr = s_.receiver.protocol;
            if (b & (1u << 0)) pr = wbp::RX_PROTOCOL_PPM;
            else if (b & (1u << 13)) pr = wbp::RX_PROTOCOL_PWM;
            else if (b & (1u << 14)) pr = wbp::RX_PROTOCOL_MSP;
            else if (b & (1u << 25)) pr = wbp::RX_PROTOCOL_NRF24;
            serial_wanted_ = (b & (1u << 3)) && !serial_rx(pr);                // RX_SERIAL: which one comes with RX_CONFIG (the App sends it after)
            port_.commit(wbp::SEC_GENERAL); port_.commit(wbp::SEC_SENSORS); port_.commit(wbp::SEC_RECEIVER); port_.commit(wbp::SEC_MOTORS); return true; }
        case SET_RX_CONFIG: {                                              // the serial receiver, the airmode throttle, the stick smoothing
            if (len < 9 || rd16(q, 5) >= rd16(q, 1)) return false;        // mincheck >= maxcheck: refused (Betaflight)
            if (len >= 12 && rd16(q, 8) >= rd16(q, 10)) return false;     // rx_min_usec >= rx_max_usec
            int pr = -1;
            if (serial_rx(s_.receiver.protocol) || serial_wanted_) { pr = from_bf_provider(q[0]); if (pr < 0) return false; }
            if (pr >= 0) s_.receiver.protocol = (uint8_t)pr;
            serial_wanted_ = false;
            s_.receiver.max_check = rd16(q, 1); s_.receiver.mid_rc = rd16(q, 3); s_.receiver.min_check = rd16(q, 5);   // (the range check clamps them)
            if (len >= 16) { const uint16_t v = rd16(q, 14); s_.general.airmode_start_throttle = (uint8_t)((v < 1000 ? 0 : v > 2000 ? 1000 : v - 1000) / 10); }
            if (len >= 29) s_.receiver.rc_smoothing_throttle = q[27];
            if (len >= 31) s_.receiver.rc_smoothing_factor = q[30];
            if (len >= 32) s_.receiver.rc_smoothing = q[31] ? 1 : 0;
            port_.commit(wbp::SEC_RECEIVER); port_.commit(wbp::SEC_GENERAL); return true; }
        case SET_ARMING_CONFIG:                                            // the small angle; auto disarm (none here) not kept
            if (len < 3) return false;
            s_.general.max_arm_angle = q[2]; port_.commit(wbp::SEC_GENERAL); return true;
        case SET_BEEPER_CONFIG:
            if (len < 4) return false;
            s_.beeper.conditions = from_bf_beeper_off(rd32(q, 0), s_.beeper.conditions);
            if (len >= 5) s_.beeper.dshot_beacon_tone = q[4] < 1 ? 1 : q[4] > 5 ? 5 : q[4];
            if (len >= 9) s_.beeper.motor_conditions = from_bf_beeper_off(rd32(q, 5), s_.beeper.motor_conditions);
            port_.commit(wbp::SEC_BEEPER); return true;
        case SET_ADVANCED_CONFIG: {                                        // PID loop divider, motor protocol, PWM rate, idle; a value read and sent back changes nothing
            static const uint8_t from_bf[] = {wbp::ESC_PROTOCOL_PWM, wbp::ESC_PROTOCOL_ONESHOT125, wbp::ESC_PROTOCOL_ONESHOT42, wbp::ESC_PROTOCOL_MULTISHOT, wbp::ESC_PROTOCOL_BRUSHED,
                                              wbp::ESC_PROTOCOL_DSHOT150, wbp::ESC_PROTOCOL_DSHOT300, wbp::ESC_PROTOCOL_DSHOT600, wbp::ESC_PROTOCOL_PROSHOT1000};
            if (len < 8 || q[3] >= sizeof(from_bf)) return false;          // (9: DISABLED - no such thing here)
            wbp::S_motors& m = s_.motors;
            s_.general.pid_denom = q[1];
            if (q[3] != bf_motor(m.protocol)) m.protocol = from_bf[q[3]];
            m.pulse_hz = rd16(q, 4);
            m.idle_x10 = (uint16_t)((rd16(q, 6) + 5u) / 10u);               // (0.01 % -> 0.1 %: what was read comes back the same)
            port_.commit(wbp::SEC_GENERAL); port_.commit(wbp::SEC_MOTORS); return true; }
        case SET_BOARD_ALIGNMENT_CONFIG: {                                 // Betaflight's angles -> chip orientation + trims (core/board_align.h)
            if (len < 6) return false;
            wbp::S_general& g = s_.general;                                // (copies, not pointers: the settings are packed)
            int now[3]; board_bf(now); const int v[3] = {(int16_t)rd16(q, 0), (int16_t)rd16(q, 2), (int16_t)rd16(q, 4)};
            if (v[0] != now[0] || v[1] != now[1] || (((v[2] - now[2]) % 360) + 360) % 360 != 0) {   // (what was read and sent back keeps its tenths)
                uint8_t a = g.board_align; float t[3];
                if (!fc::balign::from_bf((float)v[0], (float)v[1], (float)v[2], g.board_align, a, t)) return false;   // (a board on its edge: no orientation + trims make it)
                g.board_align = a; g.board_trim_roll = (int16_t)lroundf(t[0] * 10.0f); g.board_trim_pitch = (int16_t)lroundf(t[1] * 10.0f); g.board_trim_yaw = (int16_t)lroundf(t[2] * 10.0f);
            }
            port_.commit(wbp::SEC_GENERAL); return true; }
        case SET_SENSOR_CONFIG: {                                          // the App's switches: baro, compass (and its model), rangefinder, flow; the accelerometer stays on
            if (len < 3) return false;
            wbp::S_sensors& se = s_.sensors;
            se.baro_enabled = q[1] != 1;
            if (q[2] == 1) se.mag_enabled = 0;
            else { se.mag_enabled = 1; if (q[2] != bf_mag(se.mag_model)) { const int m = from_bf_mag(q[2]); if (m >= 0) se.mag_model = (uint8_t)m; } }   // (read and written back: the model kept)
            if (len >= 4) se.range_enabled = q[3] != 0;
            if (len >= 5 && q[4] == 0) s_.flow.type = wbp::FLOW_TYPE_NONE;    // (on: which sensor is not said - set it in the Windburst app)
            port_.commit(wbp::SEC_SENSORS); port_.commit(wbp::SEC_FLOW); return true; }
        case SET_SENSOR_ALIGNMENT: {                                    // the compass's alignment and angles, which gyros; the gyro's alignment is not written (master neither)
            if (len < 4) return false;
            if (q[2] >= 1 && q[2] <= 8) s_.sensors.mag_align = (uint8_t)(q[2] - 1);     // (DEFAULT 0, CUSTOM 9: kept)
            if (q[3] >= 1 && q[3] <= 3) s_.sensors.gyro_use = (uint8_t)(q[3] - 1);
            if (len >= 10) { s_.sensors.mag_trim_roll = clamp1800(rd16(q, 4)); s_.sensors.mag_trim_pitch = clamp1800(rd16(q, 6)); s_.sensors.mag_trim_yaw = clamp1800(rd16(q, 8)); }
            port_.commit(wbp::SEC_SENSORS); return true; }
        case SET_MOTOR_3D_CONFIG:                                          // deadband low, high, neutral (us; the range check clamps them)
            if (len < 6) return false;
            s_.motors.deadband3d_low = rd16(q, 0); s_.motors.deadband3d_high = rd16(q, 2); s_.motors.neutral3d = rd16(q, 4);
            port_.commit(wbp::SEC_MOTORS); return true;
        case SET_RC_DEADBAND:                                              // roll / pitch, yaw, (Position Hold's: not kept), 3D throttle
            if (len < 5) return false;
            s_.receiver.deadband = q[0]; s_.receiver.yaw_deadband = q[1]; s_.receiver.deadband3d_throttle = rd16(q, 3);
            port_.commit(wbp::SEC_RECEIVER); return true;
        case SET_MIXER_CONFIG: {                                           // Betaflight's mixer -> layout (the one read and sent back is kept: HEX_X stays HEX_X)
            if (len < 1) return false;
            if (q[0] != bf_mixer(s_.motors.layout)) {
                int l = -1; for (uint8_t k = 0; k < 32; k++) if (q[0] != 0 && bf_mixer(k) == q[0] && l < 0) l = k;
                if (q[0] == 10) l = wbp::LAYOUT_HEX_X_BF;                  // (HEX6X: Betaflight's motor order)
                if (l < 0) return false;                                   // TRI, bicopter, wings, ...: no such frame here
                s_.motors.layout = (uint8_t)l;
            }
            if (len >= 2) s_.motors.props_out = q[1] ? 1 : 0;
            port_.commit(wbp::SEC_MOTORS); return true; }
        case SET_COMPASS_CONFIG:
            if (len < 2) return false;
            s_.sensors.mag_declination = clamp1800(rd16(q, 0)); port_.commit(wbp::SEC_SENSORS); return true;
        case EEPROM_WRITE: return port_.save();
        case REBOOT: reboot_ = true; return true;
        }
        return false;
    }
    uint32_t pid_hz() const { const uint32_t d = s_.general.pid_denom ? s_.general.pid_denom : 1; return s_.general.loop_hz / d; }
    // Betaflight's blackbox sample_rate: the log takes every 2^n-th PID cycle (n 0 .. 4)
    uint8_t sample_rate() const { const uint32_t r = s_.blackbox.rate_hz ? s_.blackbox.rate_hz : 1; uint8_t k = 0; while (k < 4 && (pid_hz() >> (k + 1)) >= r) k++; return k; }
    uint32_t features() const {                                            // Betaflight's feature bits
        const uint8_t pr = s_.receiver.protocol; uint32_t f = 0;
        f |= pr == wbp::RX_PROTOCOL_PPM ? 1u << 0 : pr == wbp::RX_PROTOCOL_PWM ? 1u << 13 : pr == wbp::RX_PROTOCOL_MSP ? 1u << 14 : pr == wbp::RX_PROTOCOL_NRF24 ? 1u << 25 : 1u << 3;
        if (s_.sensors.gps_enabled) f |= 1u << 7;
        if (s_.sensors.range_enabled) f |= 1u << 9;
        if (s_.general.airmode != wbp::AIRMODE_OFF) f |= 1u << 22;
        if (s_.motors.mode3d) f |= 1u << 12;                               // 3D (reversible motors)
        if (s_.receiver.telemetry) f |= 1u << 10;                          // TELEMETRY: back to the transmitter through the receiver
        if (s_.led.count) f |= 1u << 16;                                   // LED_STRIP
        if (s_.osd.type != wbp::OSD_TYPE_NONE) f |= 1u << 18;              // OSD
        if (s_.motors.esc_sensor) f |= 1u << 27;                           // ESC_SENSOR (the ESCs' telemetry wire)
        return f;
    }
    static bool serial_rx(uint8_t pr) { return pr != wbp::RX_PROTOCOL_PPM && pr != wbp::RX_PROTOCOL_PWM && pr != wbp::RX_PROTOCOL_MSP && pr != wbp::RX_PROTOCOL_NRF24; }
    // Betaflight's SERIALRX_* <-> receiver.protocol
    static uint8_t bf_provider(uint8_t pr) {
        switch (pr) { case wbp::RX_PROTOCOL_SPEKTRUM: return 1; case wbp::RX_PROTOCOL_SBUS: return 2; case wbp::RX_PROTOCOL_IBUS: return 7; case wbp::RX_PROTOCOL_CRSF: return 9;
                      case wbp::RX_PROTOCOL_SPEKTRUM1024: return 15; case wbp::RX_PROTOCOL_MAVLINK: return 16; default: return 0; }
    }
    static int from_bf_provider(uint8_t v) { for (uint8_t pr = 0; pr < 16; pr++) if (v != 0 && serial_rx(pr) && bf_provider(pr) == v) return pr; return -1; }
    // esc_protocol -> Betaflight's (DShot1200: none there, said 600; DroneCAN: DShot300) - the same as msp_info_of
    // layout -> Betaflight's mixerMode (mixer.h); 0 for none
    static uint8_t bf_mixer(uint8_t l) { static const uint8_t m[] = {3, 2, 10, 7, 10, 18, 6, 9, 11, 12, 13, 17, 22, 26}; return l < sizeof(m) ? m[l] : 0; }
    // compass model <-> Betaflight's MAG_* (sensors/compass.h); the ones it has not (AUTO, RM3100, MSP) read DEFAULT
    static uint8_t bf_mag(uint8_t m) {
        switch (m) { case wbp::MAG_MODEL_HMC5883L: return 2; case wbp::MAG_MODEL_AK8963_MPU9250: return 4; case wbp::MAG_MODEL_QMC5883L: return 5;
                     case wbp::MAG_MODEL_QMC5883P: return 11; case wbp::MAG_MODEL_DRONECAN: return 12; default: return 0; }
    }
    static int from_bf_mag(uint8_t v) {
        switch (v) { case 0: return wbp::MAG_MODEL_AUTO; case 2: return wbp::MAG_MODEL_HMC5883L; case 4: case 8: return wbp::MAG_MODEL_AK8963_MPU9250; case 5: return wbp::MAG_MODEL_QMC5883L;
                     case 11: return wbp::MAG_MODEL_QMC5883P; case 12: return wbp::MAG_MODEL_DRONECAN; default: return -1; }
    }
    // battery.source / current_source -> Betaflight's voltage meter (NONE 0, ADC 1, ESC 2) / current meter (NONE 0, ADC 1, VIRTUAL 2, ESC 3, MSP 4)
    // nav.fs_procedure -> Betaflight's (DROP 0, AUTO_LAND 1, GPS_RESCUE 2; HOLD: none there, said AUTO_LAND)
    // a filter slider block (22 bytes): on, multiplier, LPF1 static, LPF2, dynamic min, max (+ 12 reserved, already zero)
    static void filt_out(uint8_t* o, uint8_t on, uint8_t mult, uint16_t lpf1, uint16_t lpf2, uint16_t dmin, uint16_t dmax) {
        o[0] = on; o[1] = mult; o[2] = (uint8_t)lpf1; o[3] = (uint8_t)(lpf1 >> 8); o[4] = (uint8_t)lpf2; o[5] = (uint8_t)(lpf2 >> 8);
        o[6] = (uint8_t)dmin; o[7] = (uint8_t)(dmin >> 8); o[8] = (uint8_t)dmax; o[9] = (uint8_t)(dmax >> 8);
    }
    static fc::PidSliders sliders_of(const wbp::S_pid& pd) {
        fc::PidSliders sl; sl.mode = pd.slider_mode; sl.master = pd.slider_master; sl.d = pd.slider_d; sl.pi = pd.slider_pi; sl.ff = pd.slider_ff;
        sl.d_max = pd.slider_d_max; sl.i = pd.slider_i; sl.pitch_d = pd.slider_pitch_d; sl.pitch_pi = pd.slider_pitch_pi; return sl;
    }
    // osd.pos (column 0-5, row 6-10, shown 11) <-> Betaflight's (column 0-4 + bit 10 for 32..63, row 5-9, shown 11 = profile 1)
    static uint16_t bf_osd_pos(uint16_t w) {
        const uint16_t col = w & 0x3F, row = (w >> 6) & 0x1F;
        return (uint16_t)((col & 0x1F) | ((col & 0x20) << 5) | (row << 5) | (w & 0x0800));
    }
    static uint16_t from_bf_osd_pos(uint16_t b) {
        const uint16_t col = (uint16_t)((b & 0x1F) | ((b >> 5) & 0x20)), row = (b >> 5) & 0x1F;
        return (uint16_t)(col | (row << 6) | ((b & 0x0800) ? 0x0800 : 0));
    }
    // the statistics this firmware's screen shows (osd/osd.h): fly time (timer 2), max speed / distance, min battery, max current, used mAh, max altitude, flight distance
    static bool osd_stat_shown(int i) { return i == 2 || i == 3 || i == 4 || i == 5 || i == 9 || i == 10 || i == 11 || i == 18; }
    static uint8_t bf_procedure(uint8_t p) { return p == wbp::FS_PROCEDURE_DROP ? 0 : p == wbp::FS_PROCEDURE_RESCUE ? 2 : 1; }
    // nav.rescue_alt_mode -> Betaflight's (MAX 0, FIXED 1, CURRENT 2; CONE: none there, said MAX)
    static uint8_t bf_rescue_alt(uint8_t m) { return m <= wbp::RESCUE_ALT_CURRENT ? m : 0; }
    static uint8_t bf_vmeter(uint8_t v) { return v == wbp::BATT_SOURCE_NONE ? 0 : v == wbp::BATT_SOURCE_ESC || v == wbp::BATT_SOURCE_DRONECAN ? 2 : 1; }
    static uint8_t bf_cmeter(uint8_t c) { return c == wbp::CURR_SOURCE_ADC ? 1 : c == wbp::CURR_SOURCE_ESC || c == wbp::CURR_SOURCE_DRONECAN ? 3 : 0; }
    uint16_t health() const { return t_ ? t_->health : 0; }
    static uint8_t bf_motor(uint8_t pr) { static const uint8_t bf[] = {0, 1, 2, 3, 5, 6, 7, 7, 8, 4}; return pr < sizeof(bf) ? bf[pr] : 6; }
    // beeper_cond -> Betaflight's beeper mode (0: none there - kept as it is on a write); Betaflight's off flags: bit = mode - 1
    static uint8_t beep_bf(int i) { static const uint8_t m[19] = {1, 2, 3, 4, 5, 6, 24, 7, 8, 10, 11, 12, 13, 16, 17, 0, 19, 18, 0}; return m[i]; }
    static uint32_t bf_beeper_off(uint32_t on) { uint32_t off = 0x00FFFFFFu; for (int i = 0; i < 19; i++) if (beep_bf(i) && (on & (1u << i))) off &= ~(1u << (beep_bf(i) - 1)); return off; }
    static uint32_t from_bf_beeper_off(uint32_t off, uint32_t on) {
        for (int i = 0; i < 19; i++) if (beep_bf(i)) { if (off & (1u << (beep_bf(i) - 1))) on &= ~(1u << i); else on |= 1u << i; }
        return on;
    }
    static int deg(int16_t tenths) { return (int)lroundf(tenths / 10.0f); }
    // a board alignment angle in degrees -> 0.1 deg within +-lim; the one read before (same whole degrees) keeps its tenths
    // the chip orientation + board trims as Betaflight's board alignment of the whole mounting, whole degrees (yaw clockwise)
    void board_bf(int a[3]) const { float b[3]; fc::balign::bf_of(s_.general.board_align, s_.general.board_trim_roll * 0.1f, s_.general.board_trim_pitch * 0.1f, s_.general.board_trim_yaw * 0.1f, b); for (int k = 0; k < 3; k++) a[k] = (int)lroundf(b[k]);
        if (a[0] <= -180) a[0] += 360;                                     // roll in (-180, 180], yaw 0 .. 359: as the App shows them
        a[2] = ((a[2] % 360) + 360) % 360; }
    static int16_t clamp1800(uint16_t v) { const int x = (int16_t)v; return (int16_t)(x < -1800 ? -1800 : x > 1800 ? 1800 : x); }
    static uint16_t rd16(const uint8_t* q, int at) { return (uint16_t)(q[at] | (q[at + 1] << 8)); }
    static uint32_t rd32(const uint8_t* q, int at) { return (uint32_t)rd16(q, at) | ((uint32_t)rd16(q, at + 2) << 16); }
    bool used(int i) const { return s_.modes.hi[i] > s_.modes.lo[i] && s_.modes.mode[i] < kModeBoxes && kBfBox[s_.modes.mode[i]] != 255; }
    uint32_t modes_set() const { uint32_t m = 0; for (int i = 0; i < 16; i++) if (s_.modes.hi[i] > s_.modes.lo[i] && s_.modes.mode[i] < 32) m |= 1u << s_.modes.mode[i]; return m; }
    static uint8_t step(uint16_t us) { const int v = ((int)us - 900) / 25; return (uint8_t)(v < 0 ? 0 : v > 48 ? 48 : v); }
    static uint8_t c10(uint16_t v) { const unsigned x = (v + 5u) / 10u; return (uint8_t)(x > 255u ? 255u : x); }
    static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
    static void u16(uint8_t* p, uint16_t& n, uint16_t v) { p[n++] = (uint8_t)v; p[n++] = (uint8_t)(v >> 8); }
    static void u32(uint8_t* p, uint16_t& n, uint32_t v) { u16(p, n, (uint16_t)v); u16(p, n, (uint16_t)(v >> 16)); }

    wbp::Settings& s_; ConfigPort& port_; const app::Telemetry* t_ = nullptr; bool reboot_ = false; uint32_t refused_ = 0;
    bool serial_wanted_ = false;                     // SET_FEATURE_CONFIG asked for a serial receiver: SET_RX_CONFIG's provider picks it
};

} // namespace msp
