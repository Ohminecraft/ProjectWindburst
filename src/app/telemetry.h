/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include <string.h>
#include "../proto/wbp_schema.h"

// ============================================================================
// Telemetry: one snapshot of everything the app can display, in engineering units (deg, deg/s, g, m, V ...).
// The flight loop fills it (FlightApp on the ESP32, the simulator on a PC); pack_stream() turns it into the wire structs of the
// schema. All the scaling and saturation lives here, in portable code, so it is unit-tested once for every target.
// Portable: no Arduino includes.
// ============================================================================
namespace app {

struct Telemetry {
    // ---- attitude / rates, pilot convention (roll right +, pitch nose-up -, yaw right +)
    float roll = 0, pitch = 0, yaw = 0;                 // deg
    float gyro_f[3] = {}, sp[3] = {};                   // filtered gyro and rate setpoints, deg/s
    float gyro_raw[3] = {};                             // unfiltered gyro, deg/s
    float acc[3] = {};                                  // board frame, g
    // ---- receiver
    uint16_t rc[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bool rc_link = false; uint8_t rssi = 0; uint16_t rc_rate = 0, rc_bad = 0, rc_ovf = 0, rc_line = 0;      // rc_*: per second
    // ---- status
    bool armed = false; uint32_t blockers = 0; uint8_t failsafe = 0, fs_cause = 0; uint32_t modes = 0; uint16_t health = 0; uint8_t ah_state = 0;
    uint16_t loop_us = 0, loop_max_us = 0, gap_max_us = 0, loop_hz = 0; float throttle = 0;
    uint16_t t_imu_us = 0, t_core_us = 0, t_motor_us = 0, t_rest_us = 0, t_slow_max_us = 0, overruns = 0, loop_target_hz = 0;   // where a control cycle spends its time (averages of the last second)
    uint8_t cpu_pct = 0, ram_pct = 0; uint16_t heap_free_kb = 0, psram_free_kb = 0, psram_total_kb = 0;    // load of the flight-loop core, memory
    // ---- battery (batt::Monitor) and beeper
    uint16_t servo_us[4] = {};                            // pulses sent to the servos (us), 0 = off
    float batt_v = 0, cell_v = 0; uint8_t batt_cells = 0, batt_state = 0, batt_source = 0; uint16_t batt_pin_mv = 0; uint8_t beeping = 0, motor_beeping = 0;
    float batt_amps = 0, batt_mah = 0; uint8_t batt_remaining = 0, curr_source = 0; uint16_t curr_pin_mv = 0;      // current sensor, consumption, what is left (%)
    // ---- motors (logical order)
    float motor_cmd[8] = {}, rpm[8] = {}; uint8_t esc_err_pct[8] = {}, esc_temp[8] = {}; float esc_volt = 0, dyn_idle = 0;
    uint8_t bidir_state = 0, esc_answers = 0, motor_count = 0; uint16_t hw_skipped = 0, hw_late = 0, hw_replies = 0, hw_garbled = 0;                 // output driver: bidirectional DShot state, which ESCs answer, frames skipped / replies lost in the last second
    // ---- sensors
    float mag[3] = {}; float baro_alt = 0, baro_pa = 0, temp_c = 0, range_m = 0;
    // ---- gps (Betaflight units)
    uint8_t gps_fix = 0, gps_sats = 0; int32_t lat = 0, lon = 0, alt_cm = 0; uint16_t gs_cms = 0, course_ddeg = 0, hdop = 0;
    uint32_t hacc_mm = 0, home_dist_m = 0; uint16_t home_dir_ddeg = 0; uint8_t gps_state = 0; uint32_t gps_baud = 0; float gps_rate_hz = 0;
    // ---- estimator
    float z = 0, vz = 0, z_sigma = 0, pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0; bool alt_valid = false, pos_valid = false, yaw_from_mag = false;
    bool vel_valid = false, flow_valid = false, tilt_aided = false, gps_alt_used = false, gps_vz_used = false; uint16_t z_resets = 0;
    // ---- the EKF (core/ekf.h; nav.ekf): heading / position / velocity next to the usual estimate, for comparing
    float ekf_roll = 0, ekf_pitch = 0, ekf_yaw = 0, ekf_pn = 0, ekf_pe = 0, ekf_vn = 0, ekf_ve = 0, ekf_vd = 0, ekf_z = 0, ekf_yaw_sigma = 0, ekf_test_vel = 0, ekf_test_mag = 0;
    uint8_t ekf_flags = 0; uint16_t ekf_us = 0;
    // ---- the app link (link stream): slow_gap_ms from the sensor thread, the rest filled in by the link task itself (LinkHost)
    uint16_t slow_gap_ms = 0;
    uint32_t up_ms = 0, boot_id = 0, link_rx = 0, link_dropped = 0; uint16_t link_bad = 0, link_busy = 0, link_tx_lost = 0, link_kicks = 0, link_cmd_max_ms = 0, link_gap_ms = 0, link_queued = 0;
    // ---- navigation: optical flow, Position Hold, GPS Rescue
    uint8_t nav_flags = 0, flow_quality = 0, flow_reject = wbp::FLOW_REJECT_NO_SAMPLE, flow_rate = 0, ph_state[2] = {}, rescue_phase = 0, rescue_fail = 0;
    float flow_fwd = 0, flow_right = 0, nav_roll = 0, nav_pitch = 0, nav_target_n = 0, nav_target_e = 0, nav_wind_n = 0, nav_wind_e = 0, nav_home_dist = 0, rescue_return_alt = 0;
    // ---- tuning: live state of the filters and the PID helpers
    float gyro_lpf1_hz = 0, dterm_lpf1_hz = 0, dyn_notch_hz[9] = {}, rpm_notch_hz[8] = {}, tpa = 1, anti_gravity = 0, rc_smoothed_hz = 0, rc_cutoff_hz = 0, pid_sum[3] = {}, hover_throttle = 0; uint8_t air_state = false;
    // ---- a section the firmware changed by itself and wants written to flash (in-flight level trim): the link task saves it when seq changes
    uint8_t fw_saved_seq = 0, fw_saved_section = 0; uint8_t trim_progress = 0;      // trim_progress: % of the in-flight level trim measurement
    // ---- calibration in progress
    uint8_t cal_kind = 0, cal_progress = 0, cal_state = 0, cal_mask = 0; float cal_raw[3] = {};
};

// value / scale, rounded, saturated to the integer type; NaN and infinities become 0 (a broken sensor never produces garbage on the wire)
inline int32_t scaled(float v, float scale, int32_t lo, int32_t hi) {
    if (!(v == v) || v > 3.0e38f || v < -3.0e38f) return 0;
    const float x = v / scale;
    if (x >= (float)hi) return hi;
    if (x <= (float)lo) return lo;
    return (int32_t)lroundf(x);
}
inline int16_t s16(float v, float scale) { return (int16_t)scaled(v, scale, -32768, 32767); }
inline uint16_t u16(float v, float scale) { return (uint16_t)scaled(v, scale, 0, 65535); }
inline uint16_t sat_u16(uint32_t v) { return v > 65535u ? (uint16_t)65535u : (uint16_t)v; }
// yaw as a heading: 0 .. 360 deg (0 = north, clockwise), what a compass and Betaflight's MSP_ATTITUDE show. The flight core keeps
// -180 .. 180 inside (its arithmetic does not care); only what leaves the firmware (app, flight log) is converted.
inline float heading360(float yaw_deg) {
    if (!(yaw_deg == yaw_deg)) return 0.0f;
    float h = fmodf(yaw_deg, 360.0f); if (h < 0.0f) h += 360.0f;
    return h >= 359.995f ? 0.0f : h;                       // (rounds to 36000 at a scale of 0.01: that is 0)
}

// Writes the T_<stream> wire struct for `id`. settings_flags = wbp::Server::flags(). Returns the struct size, 0 for an unknown id / too small buffer.
// what the estimators have at the moment, as the schema's est_valid bits (estimate stream, flight log)
inline uint8_t est_valid_bits(const Telemetry& t) {
    uint8_t v = 0;
    if (t.alt_valid) v |= wbp::EST_VALID_ALT;
    if (t.pos_valid) v |= wbp::EST_VALID_POS;
    if (t.yaw_from_mag) v |= wbp::EST_VALID_YAW_FROM_MAG;
    if (t.vel_valid) v |= wbp::EST_VALID_VEL;
    if (t.flow_valid) v |= wbp::EST_VALID_FLOW;
    if (t.tilt_aided) v |= wbp::EST_VALID_TILT_AIDED;
    if (t.gps_alt_used) v |= wbp::EST_VALID_GPS_ALT;
    if (t.gps_vz_used) v |= wbp::EST_VALID_GPS_VZ;
    return v;
}
inline uint16_t pack_stream(uint8_t id, const Telemetry& t, uint8_t settings_flags, uint8_t* out, uint16_t cap) {
    using namespace wbp;
    const StreamInfo* si = find_stream(id);
    if (!si || cap < si->size) return 0;
    switch (id) {
    case STR_ATTITUDE: {
        T_attitude o; o.roll = s16(t.roll, scale::attitude_roll); o.pitch = s16(t.pitch, scale::attitude_pitch); o.yaw = u16(heading360(t.yaw), scale::attitude_yaw);
        for (int i = 0; i < 3; i++) { o.gyro[i] = s16(t.gyro_f[i], scale::attitude_gyro); o.sp[i] = s16(t.sp[i], scale::attitude_sp); }
        memcpy(out, &o, sizeof(o)); break; }
    case STR_RC: {
        T_rc o; for (int i = 0; i < 16; i++) o.ch[i] = t.rc[i];
        o.link = t.rc_link ? 1 : 0; o.rssi = t.rssi; o.rate = t.rc_rate; o.bad_frames = t.rc_bad; o.uart_ovf = t.rc_ovf; o.uart_line = t.rc_line;
        memcpy(out, &o, sizeof(o)); break; }
    case STR_STATUS: {
        T_status o; o.armed = t.armed ? 1 : 0; o.blockers = t.blockers; o.failsafe = t.failsafe; o.modes = t.modes; o.health = t.health; o.ah_state = t.ah_state;
        o.loop_us = t.loop_us; o.loop_max_us = t.loop_max_us; o.gap_max_us = t.gap_max_us; o.loop_hz = t.loop_hz; o.cell_mv = u16(t.cell_v, 0.001f); o.settings_dirty = settings_flags; o.throttle = u16(t.throttle, scale::status_throttle);
        o.cpu = t.cpu_pct; o.ram = t.ram_pct; o.heap_free = t.heap_free_kb; o.psram_free = t.psram_free_kb; o.psram_total = t.psram_total_kb;
        o.t_imu = t.t_imu_us; o.t_core = t.t_core_us; o.t_motor = t.t_motor_us; o.t_rest = t.t_rest_us; o.t_slow_max = t.t_slow_max_us; o.overruns = t.overruns; o.loop_target = t.loop_target_hz; o.fs_cause = t.fs_cause;
        memcpy(out, &o, sizeof(o)); break; }
    case STR_MOTORS: {
        T_motors o;
        for (int i = 0; i < 8; i++) { o.cmd[i] = u16(t.motor_cmd[i], scale::motors_cmd); o.rpm[i] = u16(t.rpm[i], scale::motors_rpm); o.err[i] = t.esc_err_pct[i]; o.temp[i] = t.esc_temp[i]; }
        o.volt[0] = u16(t.esc_volt, scale::motors_volt); o.dyn_idle = u16(t.dyn_idle, scale::motors_dyn_idle);
        o.bidir = t.bidir_state; o.answers = t.esc_answers; o.skipped = t.hw_skipped; o.late = t.hw_late; o.replies = t.hw_replies; o.garbled = t.hw_garbled;
        for (int i = 0; i < 4; i++) o.servo[i] = t.servo_us[i];
        memcpy(out, &o, sizeof(o)); break; }
    case STR_SENSORS: {
        T_sensors o;
        for (int i = 0; i < 3; i++) { o.gyro[i] = s16(t.gyro_raw[i], scale::sensors_gyro); o.acc[i] = s16(t.acc[i], scale::sensors_acc); o.mag[i] = s16(t.mag[i], scale::sensors_mag); }
        o.baro_alt = s16(t.baro_alt, scale::sensors_baro_alt); o.baro_pa = t.baro_pa > 0 ? (uint32_t)lroundf(t.baro_pa) : 0; o.temp = s16(t.temp_c, scale::sensors_temp); o.range = s16(t.range_m, scale::sensors_range);
        memcpy(out, &o, sizeof(o)); break; }
    case STR_GPS: {
        T_gps o; o.fix = t.gps_fix; o.sats = t.gps_sats; o.lat = t.lat; o.lon = t.lon; o.alt = t.alt_cm; o.gs = t.gs_cms; o.course = t.course_ddeg; o.hdop = t.hdop;
        o.hacc = sat_u16(t.hacc_mm); o.home_dist = sat_u16(t.home_dist_m); o.home_dir = t.home_dir_ddeg; o.state = t.gps_state; o.baud = t.gps_baud; o.rate = (uint8_t)scaled(t.gps_rate_hz, scale::gps_rate, 0, 255);
        memcpy(out, &o, sizeof(o)); break; }
    case STR_ESTIMATE: {
        T_estimate o; o.z = s16(t.z, scale::estimate_z); o.vz = s16(t.vz, scale::estimate_vz); o.z_sigma = u16(t.z_sigma, scale::estimate_z_sigma);
        o.valid = est_valid_bits(t);
        o.pos_n = s16(t.pos_n, scale::estimate_pos_n); o.pos_e = s16(t.pos_e, scale::estimate_pos_e); o.vel_n = s16(t.vel_n, scale::estimate_vel_n); o.vel_e = s16(t.vel_e, scale::estimate_vel_e);
        o.z_resets = t.z_resets;
        o.ekf_yaw = u16(heading360(t.ekf_yaw), scale::estimate_ekf_yaw); o.ekf_yaw_sigma = u16(t.ekf_yaw_sigma, scale::estimate_ekf_yaw_sigma);
        o.ekf_pn = s16(t.ekf_pn, scale::estimate_ekf_pn); o.ekf_pe = s16(t.ekf_pe, scale::estimate_ekf_pe); o.ekf_vn = s16(t.ekf_vn, scale::estimate_ekf_vn); o.ekf_ve = s16(t.ekf_ve, scale::estimate_ekf_ve);
        o.ekf_flags = t.ekf_flags; o.ekf_us = t.ekf_us;
        memcpy(out, &o, sizeof(o)); break; }
    case STR_LINK: {
        T_link o; o.uptime = t.up_ms; o.boot_id = t.boot_id; o.rx_frames = t.link_rx; o.rx_bad = t.link_bad; o.busy = t.link_busy; o.tx_lost = t.link_tx_lost; o.dropped = t.link_dropped;
        o.kicks = t.link_kicks; o.cmd_max_ms = t.link_cmd_max_ms; o.link_gap_ms = t.link_gap_ms; o.slow_gap_ms = t.slow_gap_ms; o.queued = t.link_queued;
        memcpy(out, &o, sizeof(o)); break; }
    case STR_GYRO_FAST: {
        T_gyro_fast o; for (int i = 0; i < 3; i++) { o.raw[i] = s16(t.gyro_raw[i], scale::gyro_fast_raw); o.filt[i] = s16(t.gyro_f[i], scale::gyro_fast_filt); }
        memcpy(out, &o, sizeof(o)); break; }
    case STR_CALIBRATION: {
        T_calibration o; o.kind = t.cal_kind; o.progress = t.cal_progress; o.state = t.cal_state; o.mask = t.cal_mask; o.inflight_trim = t.trim_progress;
        for (int i = 0; i < 3; i++) o.raw[i] = s16(t.cal_raw[i], scale::calibration_raw);
        memcpy(out, &o, sizeof(o)); break; }
    case STR_BATTERY: {
        T_battery o; o.voltage = u16(t.batt_v, scale::battery_voltage); o.cell = u16(t.cell_v, scale::battery_cell); o.cells = t.batt_cells; o.state = t.batt_state;
        o.source = t.batt_source; o.pin_mv = t.batt_pin_mv; o.beeping = t.beeping; o.motor_beeping = t.motor_beeping;
        o.current = s16(t.batt_amps, scale::battery_current); o.mah = u16(t.batt_mah, 1.0f); o.remaining = t.batt_remaining; o.current_source = t.curr_source; o.current_mv = t.curr_pin_mv;
        memcpy(out, &o, sizeof(o)); break; }
    case STR_NAV: {
        T_nav o; o.flags = t.nav_flags; o.flow_quality = t.flow_quality; o.flow_reject = t.flow_reject; o.flow_rate = t.flow_rate;
        o.flow_fwd = s16(t.flow_fwd, scale::nav_flow_fwd); o.flow_right = s16(t.flow_right, scale::nav_flow_right);
        o.ph_roll = t.ph_state[0]; o.ph_pitch = t.ph_state[1];
        o.lean_roll = s16(t.nav_roll, scale::nav_lean_roll); o.lean_pitch = s16(t.nav_pitch, scale::nav_lean_pitch);
        o.target_n = s16(t.nav_target_n, scale::nav_target_n); o.target_e = s16(t.nav_target_e, scale::nav_target_e);
        o.wind_n = s16(t.nav_wind_n, scale::nav_wind_n); o.wind_e = s16(t.nav_wind_e, scale::nav_wind_e);
        o.rescue_phase = t.rescue_phase; o.rescue_fail = t.rescue_fail;
        o.home_dist = u16(t.nav_home_dist, scale::nav_home_dist); o.return_alt = u16(t.rescue_return_alt, scale::nav_return_alt);
        memcpy(out, &o, sizeof(o)); break; }
    case STR_TUNING: {
        T_tuning o; o.gyro_lpf1_hz = u16(t.gyro_lpf1_hz, 1.0f); o.dterm_lpf1_hz = u16(t.dterm_lpf1_hz, 1.0f);
        for (int i = 0; i < 9; i++) o.dyn_notch_hz[i] = u16(t.dyn_notch_hz[i], 1.0f);
        for (int i = 0; i < 8; i++) o.rpm_hz[i] = u16(t.rpm_notch_hz[i], 1.0f);
        o.tpa = (uint8_t)scaled(t.tpa, 0.01f, 0, 100); o.anti_gravity = u16(t.anti_gravity, scale::tuning_anti_gravity); o.rc_rate = u16(t.rc_smoothed_hz, 1.0f); o.rc_cutoff = u16(t.rc_cutoff_hz, 1.0f);
        o.air_state = t.air_state; for (int i = 0; i < 3; i++) o.pid_sum[i] = s16(t.pid_sum[i], scale::tuning_pid_sum);
        o.hover = (uint8_t)scaled(t.hover_throttle, 0.01f, 0, 100);
        memcpy(out, &o, sizeof(o)); break; }
    default: return 0;
    }
    return si->size;
}

} // namespace app
