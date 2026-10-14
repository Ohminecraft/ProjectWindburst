#pragma once
#include "wbp_schema.h"
#include "../core/flight_core.h"
#include "../core/simplified_tuning.h"
#include "../hal/motor/motor_output.h"
#include "../hal/gps/gps_types.h"
#include "../hal/mag/mag_calibration.h"
#include "../hal/acc_gyro/acc_calibration.h"
#include "../hal/battery/battery_monitor.h"
#include "../hal/beeper/beeper_core.h"
#include "../blackbox/bb_service.h"

// Settings (wire units: percent, 0.1 deg, mV, x100 rpm ...) -> the typed configuration structs of the core and the drivers.
// Pure functions, unit-tested. The firmware calls them at boot and again for a live change (PID / rates / modes).
namespace wbp {

// The schema's enums and bit names are the app's view of the firmware's own enums: a mismatch is a compile error, not a wrong label.
#define WB_SAME(a, b) static_assert((int)(a) == (int)(b), #a " != " #b)
WB_SAME(LAYOUT_QUAD_X, fc::QUAD_X); WB_SAME(LAYOUT_QUAD_PLUS, fc::QUAD_PLUS); WB_SAME(LAYOUT_HEX_X, fc::HEX_X); WB_SAME(LAYOUT_QUAD_X_1234, fc::QUAD_X_1234); WB_SAME(LAYOUT_OCTO_X8, fc::OCTO_X8); WB_SAME(LAYOUT_ATAIL4, fc::ATAIL4);
WB_SAME(ESC_PROTOCOL_PWM, motor::PROTO_PWM); WB_SAME(ESC_PROTOCOL_ONESHOT125, motor::PROTO_ONESHOT125); WB_SAME(ESC_PROTOCOL_ONESHOT42, motor::PROTO_ONESHOT42);
WB_SAME(ESC_PROTOCOL_MULTISHOT, motor::PROTO_MULTISHOT); WB_SAME(ESC_PROTOCOL_DSHOT150, motor::PROTO_DSHOT150); WB_SAME(ESC_PROTOCOL_DSHOT300, motor::PROTO_DSHOT300);
WB_SAME(ESC_PROTOCOL_DSHOT600, motor::PROTO_DSHOT600); WB_SAME(ESC_PROTOCOL_DSHOT1200, motor::PROTO_DSHOT1200); WB_SAME(ESC_PROTOCOL_PROSHOT1000, motor::PROTO_PROSHOT1000); WB_SAME(ESC_PROTOCOL_BRUSHED, motor::PROTO_BRUSHED);
WB_SAME(GPS_PROVIDER_NMEA, gps::PROV_NMEA); WB_SAME(GPS_PROVIDER_UBLOX, gps::PROV_UBLOX); WB_SAME(GPS_PROVIDER_MSP, gps::PROV_MSP); WB_SAME(GPS_PROVIDER_VIRTUAL, gps::PROV_VIRTUAL);
WB_SAME(SBAS_AUTO, gps::SBAS_AUTO); WB_SAME(SBAS_EGNOS, gps::SBAS_EGNOS); WB_SAME(SBAS_WAAS, gps::SBAS_WAAS); WB_SAME(SBAS_MSAS, gps::SBAS_MSAS); WB_SAME(SBAS_GAGAN, gps::SBAS_GAGAN); WB_SAME(SBAS_NONE, gps::SBAS_NONE);
WB_SAME(GPS_MODEL_PORTABLE, gps::MODEL_PORTABLE); WB_SAME(GPS_MODEL_STATIONARY, gps::MODEL_STATIONARY); WB_SAME(GPS_MODEL_PEDESTRIAN, gps::MODEL_PEDESTRIAN); WB_SAME(GPS_MODEL_AUTOMOTIVE, gps::MODEL_AUTOMOTIVE);
WB_SAME(GPS_MODEL_AT_SEA, gps::MODEL_AT_SEA); WB_SAME(GPS_MODEL_AIRBORNE_1G, gps::MODEL_AIRBORNE_1G); WB_SAME(GPS_MODEL_AIRBORNE_2G, gps::MODEL_AIRBORNE_2G); WB_SAME(GPS_MODEL_AIRBORNE_4G, gps::MODEL_AIRBORNE_4G);
WB_SAME(UTC_AUTO, gps::UTC_AUTO); WB_SAME(UTC_USNO, gps::UTC_USNO); WB_SAME(UTC_EU, gps::UTC_EU); WB_SAME(UTC_SU, gps::UTC_SU); WB_SAME(UTC_NTSC, gps::UTC_NTSC);
WB_SAME(GPS_STATE_UNKNOWN, gps::ST_UNKNOWN); WB_SAME(GPS_STATE_INITIALIZED, gps::ST_INITIALIZED); WB_SAME(GPS_STATE_DETECT_BAUD, gps::ST_DETECT_BAUD); WB_SAME(GPS_STATE_CHANGE_BAUD, gps::ST_CHANGE_BAUD);
WB_SAME(GPS_STATE_CONFIGURE, gps::ST_CONFIGURE); WB_SAME(GPS_STATE_RECEIVING_DATA, gps::ST_RECEIVING_DATA); WB_SAME(GPS_STATE_LOST_COMMUNICATION, gps::ST_LOST_COMMUNICATION);
WB_SAME(MODE_ARM, fc::M_ARM); WB_SAME(MODE_ANGLE, fc::M_ANGLE); WB_SAME(MODE_STABILIZE, fc::M_STABILIZE); WB_SAME(MODE_ALTHOLD, fc::M_ALTHOLD); WB_SAME(MODE_POSHOLD, fc::M_POSHOLD);
WB_SAME(MODE_FAILSAFE, fc::M_FAILSAFE_SW); WB_SAME(MODE_BEEPER, fc::M_BEEPER); WB_SAME(MODE_BLACKBOX, fc::M_BLACKBOX);
WB_SAME(MODE_CALIB, fc::M_CALIB); WB_SAME(MODE_RESCUE, fc::M_RESCUE);
WB_SAME(LPF_TYPE_PT1, fc::LPF_PT1); WB_SAME(LPF_TYPE_BIQUAD, fc::LPF_BIQUAD); WB_SAME(LPF_TYPE_PT2, fc::LPF_PT2); WB_SAME(LPF_TYPE_PT3, fc::LPF_PT3);
WB_SAME(ITERM_RELAX_OFF, fc::RELAX_OFF); WB_SAME(ITERM_RELAX_RP, fc::RELAX_RP); WB_SAME(ITERM_RELAX_RPY, fc::RELAX_RPY); WB_SAME(ITERM_RELAX_RP_INC, fc::RELAX_RP_INC); WB_SAME(ITERM_RELAX_RPY_INC, fc::RELAX_RPY_INC);
WB_SAME(ITERM_RELAX_TYPE_GYRO, fc::RELAX_GYRO); WB_SAME(ITERM_RELAX_TYPE_SETPOINT, fc::RELAX_SETPOINT); WB_SAME(TPA_MODE_PD, fc::TPA_PD); WB_SAME(TPA_MODE_D, fc::TPA_D);
WB_SAME(SLIDER_MODE_OFF, fc::SLIDERS_OFF); WB_SAME(SLIDER_MODE_RP, fc::SLIDERS_RP); WB_SAME(SLIDER_MODE_RPY, fc::SLIDERS_RPY);
WB_SAME(FAILSAFE_NONE, fc::FsStage::NONE); WB_SAME(FAILSAFE_GUARD, fc::FsStage::GUARD); WB_SAME(FAILSAFE_DESCENT, fc::FsStage::DESCENT); WB_SAME(FAILSAFE_LANDED, fc::FsStage::LANDED);
WB_SAME(AH_STATE_STOPPED, fc::AhState::STOPPED); WB_SAME(AH_STATE_LANDED_IDLE, fc::AhState::LANDED_IDLE); WB_SAME(AH_STATE_LANDED_PRE_TAKEOFF, fc::AhState::LANDED_PRE_TAKEOFF);
WB_SAME(AH_STATE_TAKEOFF, fc::AhState::TAKEOFF); WB_SAME(AH_STATE_FLYING, fc::AhState::FLYING);
WB_SAME(ARM_BLOCKER_NO_IMU, fc::AB_NO_IMU); WB_SAME(ARM_BLOCKER_GYRO_NOT_CAL, fc::AB_GYRO_NOT_CAL); WB_SAME(ARM_BLOCKER_RX_LOST, fc::AB_RX_LOST); WB_SAME(ARM_BLOCKER_THROTTLE_HIGH, fc::AB_THROTTLE_HIGH);
WB_SAME(ARM_BLOCKER_ANGLE_TOO_HIGH, fc::AB_ANGLE_TOO_HIGH); WB_SAME(ARM_BLOCKER_FAILSAFE, fc::AB_FAILSAFE); WB_SAME(ARM_BLOCKER_ALTHOLD_NO_BARO, fc::AB_ALTHOLD_NO_BARO); WB_SAME(ARM_BLOCKER_BATT_LOW, fc::AB_BATT_LOW);
WB_SAME(ARM_BLOCKER_ARM_SWITCH_ON, fc::AB_ARM_SWITCH_ON); WB_SAME(ARM_BLOCKER_CONFIG_UNSAVED, fc::AB_CONFIG_UNSAVED); WB_SAME(ARM_BLOCKER_CALIBRATING, fc::AB_CALIBRATING);
WB_SAME(ARM_BLOCKER_POSHOLD_NO_GPS, fc::AB_POSHOLD_NO_GPS); WB_SAME(ARM_BLOCKER_MAG_NOT_CALIBRATED, fc::AB_MAG_NOT_CALIBRATED); WB_SAME(ARM_BLOCKER_PIN_CONFLICT, fc::AB_PIN_CONFLICT); WB_SAME(ARM_BLOCKER_LOOP_RATE, fc::AB_LOOP_RATE); WB_SAME(ARM_BLOCKER_RUNAWAY_TAKEOFF, fc::AB_RUNAWAY_TAKEOFF);
WB_SAME(FS_CAUSE_LINK, fc::FS_LINK); WB_SAME(FS_CAUSE_SWITCH, fc::FS_SWITCH); WB_SAME(FS_CAUSE_BATTERY, fc::FS_BATTERY);
WB_SAME(ARM_BLOCKER_RESCUE_NO_FIX, fc::AB_RESCUE_NO_FIX); WB_SAME(ARM_BLOCKER_RESCUE_NO_COMPASS, fc::AB_RESCUE_NO_COMPASS);
WB_SAME(FS_PROCEDURE_LAND, fc::FSP_LAND); WB_SAME(FS_PROCEDURE_RESCUE, fc::FSP_RESCUE);
WB_SAME(FLOW_REJECT_OK, fc::FLOW_OK); WB_SAME(FLOW_REJECT_QUALITY, fc::FLOW_QUALITY); WB_SAME(FLOW_REJECT_HEIGHT, fc::FLOW_HEIGHT); WB_SAME(FLOW_REJECT_TILT, fc::FLOW_TILT);
WB_SAME(FLOW_REJECT_TURNING, fc::FLOW_TURNING); WB_SAME(FLOW_REJECT_NO_GYRO, fc::FLOW_NO_GYRO); WB_SAME(FLOW_REJECT_NO_SAMPLE, fc::FLOW_NO_SAMPLE);
WB_SAME(PH_AXIS_PILOT, fc::PhAxis::PILOT); WB_SAME(PH_AXIS_BRAKE, fc::PhAxis::BRAKE); WB_SAME(PH_AXIS_BRAKE_READY, fc::PhAxis::BRAKE_READY); WB_SAME(PH_AXIS_TO_LOITER, fc::PhAxis::TO_LOITER);
WB_SAME(PH_AXIS_LOITER, fc::PhAxis::LOITER); WB_SAME(PH_AXIS_TO_PILOT, fc::PhAxis::TO_PILOT);
WB_SAME(RESCUE_PHASE_IDLE, fc::RescuePhase::IDLE); WB_SAME(RESCUE_PHASE_CLIMB, fc::RescuePhase::CLIMB); WB_SAME(RESCUE_PHASE_ROTATE, fc::RescuePhase::ROTATE); WB_SAME(RESCUE_PHASE_FLY_HOME, fc::RescuePhase::FLY_HOME);
WB_SAME(RESCUE_PHASE_DESCENT, fc::RescuePhase::DESCENT); WB_SAME(RESCUE_PHASE_LANDING, fc::RescuePhase::LANDING); WB_SAME(RESCUE_PHASE_FAILED, fc::RescuePhase::FAILED);
WB_SAME(RESCUE_FAIL_NONE, fc::RescueFail::NONE); WB_SAME(RESCUE_FAIL_NO_POSITION, fc::RescueFail::NO_POSITION); WB_SAME(RESCUE_FAIL_NO_HEADING, fc::RescueFail::NO_HEADING); WB_SAME(RESCUE_FAIL_NO_ALTITUDE, fc::RescueFail::NO_ALTITUDE);
WB_SAME(RESCUE_FAIL_GPS_LOST, fc::RescueFail::GPS_LOST); WB_SAME(RESCUE_FAIL_LOW_SATS, fc::RescueFail::LOW_SATS); WB_SAME(RESCUE_FAIL_STALLED, fc::RescueFail::STALLED); WB_SAME(RESCUE_FAIL_FLYAWAY, fc::RescueFail::FLYAWAY);
WB_SAME(RESCUE_ALT_MAX, fc::RESCUE_ALT_MAX); WB_SAME(RESCUE_ALT_FIXED, fc::RESCUE_ALT_FIXED); WB_SAME(RESCUE_ALT_CURRENT, fc::RESCUE_ALT_CURRENT);
WB_SAME(BATT_SOURCE_AUTO, batt::SRC_AUTO); WB_SAME(BATT_SOURCE_ADC, batt::SRC_ADC); WB_SAME(BATT_SOURCE_ESC, batt::SRC_ESC); WB_SAME(BATT_SOURCE_NONE, batt::SRC_NONE);
WB_SAME(BATT_STATE_NOT_PRESENT, batt::State::NOT_PRESENT); WB_SAME(BATT_STATE_OK, batt::State::OK); WB_SAME(BATT_STATE_WARNING, batt::State::WARNING); WB_SAME(BATT_STATE_CRITICAL, batt::State::CRITICAL);
#define WB_BEEP(n) WB_SAME(BEEPER_COND_##n, beep::n)
WB_BEEP(GYRO_CALIBRATED); WB_BEEP(RX_LOST); WB_BEEP(RX_LOST_LANDING); WB_BEEP(DISARMING); WB_BEEP(ARMING); WB_BEEP(ARMING_GPS_FIX); WB_BEEP(ARMING_GPS_NO_FIX); WB_BEEP(BAT_CRIT_LOW); WB_BEEP(BAT_LOW);
WB_BEEP(RX_SET); WB_BEEP(ACC_CALIBRATION); WB_BEEP(ACC_CALIBRATION_FAIL); WB_BEEP(READY_BEEP); WB_BEEP(ARMED); WB_BEEP(SYSTEM_INIT); WB_BEEP(ARM_REFUSED); WB_BEEP(BLACKBOX_ERASE);
WB_SAME(BEEPER_COND_USB, beep::ON_USB);
#undef WB_BEEP
#define WB_BBF(n) WB_SAME(BB_FIELD_##n, bb::G_##n)
WB_BBF(GYRO); WB_BBF(GYRO_RAW); WB_BBF(SETPOINT); WB_BBF(PID); WB_BBF(RC); WB_BBF(MOTORS); WB_BBF(ATTITUDE); WB_BBF(ACC); WB_BBF(RPM); WB_BBF(TIMING); WB_BBF(ALT); WB_BBF(BATTERY); WB_BBF(GPS); WB_BBF(MAG); WB_BBF(NAV);
#undef WB_BBF
WB_SAME(BB_DEVICE_NONE, bb::DEV_NONE); WB_SAME(BB_DEVICE_FLASH, bb::DEV_FLASH); WB_SAME(BB_DEVICE_SDCARD, bb::DEV_SDCARD);
WB_SAME(BB_TRIGGER_SWITCH, bb::TRIG_SWITCH); WB_SAME(BB_TRIGGER_ARMED, bb::TRIG_ARMED); WB_SAME(BB_TRIGGER_ALWAYS, bb::TRIG_ALWAYS);
WB_SAME(BB_STATE_OFF, bb::ST_OFF); WB_SAME(BB_STATE_NO_STORAGE, bb::ST_NO_STORAGE); WB_SAME(BB_STATE_NEEDS_ERASE, bb::ST_NEEDS_ERASE); WB_SAME(BB_STATE_IDLE, bb::ST_IDLE);
WB_SAME(BB_STATE_LOGGING, bb::ST_LOGGING); WB_SAME(BB_STATE_STREAMING, bb::ST_STREAMING); WB_SAME(BB_STATE_ERASING, bb::ST_ERASING); WB_SAME(BB_STATE_FULL, bb::ST_FULL);

inline void apply_modes(const Settings& s, fc::RcInterpreter& rc) {
    rc.modes = fc::ModeMap();
    for (int i = 0; i < 16; i++) {
        const uint8_t m = s.modes.mode[i];
        if (m >= fc::M_COUNT || s.modes.aux[i] >= fc::kAuxCount || s.modes.hi[i] <= s.modes.lo[i]) continue;      // unused / invalid entry
        rc.modes.add(m, s.modes.aux[i], s.modes.lo[i], s.modes.hi[i]);
    }
}

// Rates, the throttle curve, the channel map and what belongs to the receiver's frames (RC smoothing; feedforward repeats).
inline void apply_rates_and_rc(const Settings& s, fc::CoreConfig& c, fc::RcInterpreter& rc) {
    for (int a = 0; a < 3; a++) {
        c.rates.center_sens_dps[a] = s.rates.center_sens[a]; c.rates.max_rate_dps[a] = s.rates.max_rate[a]; c.rates.expo[a] = s.rates.expo[a] * 0.01f;
    }
    c.rc.throttle_mid = s.rates.throttle_mid * 0.01f; c.rc.throttle_expo = s.rates.throttle_expo * 0.01f; c.rc.throttle_limit = s.rates.throttle_limit * 0.01f;
    c.angle.limit_deg = s.rates.angle_limit; c.stab.limit_deg = s.rates.angle_limit;
    c.rc.ch_roll = s.receiver.ch_roll; c.rc.ch_pitch = s.receiver.ch_pitch; c.rc.ch_throttle = s.receiver.ch_throttle; c.rc.ch_yaw = s.receiver.ch_yaw;
    c.rc.ch_aux0 = s.receiver.ch_aux0; c.rc.deadband = s.receiver.deadband * 0.01f;
    c.rc_smoothing.enabled = s.receiver.rc_smoothing != 0; c.rc_smoothing.auto_factor = s.receiver.rc_smoothing_factor; c.rc_smoothing.auto_factor_throttle = s.receiver.rc_smoothing_throttle;
    c.ff.interpolate = s.receiver.protocol != RX_PROTOCOL_CRSF;        // CRSF never repeats a frame; every other link may (Betaflight feedforwardInterpolate)
    rc.cfg = c.rc;
}

// The PID tab: gains (already computed from the sliders by settings_rules.h), PID controller settings, feedforward, Angle and Stabilize.
inline void apply_pid(const Settings& s, fc::CoreConfig& c) {
    const S_pid& p = s.pid;
    for (int a = 0; a < 3; a++) { c.pid.p[a] = p.p[a]; c.pid.i[a] = p.i[a]; c.pid.d[a] = p.d[a]; c.pid.d_max[a] = p.d_max[a]; c.pid.f[a] = p.f[a]; c.stab.kp[a] = p.stab_p[a]; }
    c.pid.iterm_relax = p.iterm_relax; c.pid.iterm_relax_type = p.iterm_relax_type; c.pid.iterm_relax_cutoff = p.iterm_relax_cutoff; c.pid.iterm_windup = p.iterm_windup;
    c.pid.anti_gravity = p.anti_gravity != 0; c.pid.anti_gravity_gain = p.anti_gravity_gain; c.pid.anti_gravity_p_gain = p.anti_gravity_p_gain; c.pid.anti_gravity_cutoff_hz = p.anti_gravity_cutoff;
    c.pid.d_max_gain = p.d_max_gain; c.pid.d_max_advance = p.d_max_advance;
    c.pid.tpa_mode = p.tpa_mode; c.pid.tpa_rate = p.tpa_rate; c.pid.tpa_breakpoint = p.tpa_breakpoint * 0.01f;
    c.pid.tpa_low_rate = p.tpa_low_rate; c.pid.tpa_low_breakpoint = p.tpa_low_breakpoint * 0.01f; c.pid.tpa_low_always = p.tpa_low_always != 0;
    c.ff.transition = p.ff_transition; c.ff.averaging = p.ff_averaging; c.ff.smooth_factor = p.ff_smooth_factor; c.ff.jitter_factor = p.ff_jitter_factor;
    c.ff.boost = p.ff_boost; c.ff.max_rate_limit = p.ff_max_rate_limit; c.ff.yaw_hold_gain = p.ff_yaw_hold_gain; c.ff.yaw_hold_time = p.ff_yaw_hold_time;
    c.throttle_boost = p.throttle_boost; c.throttle_boost_cutoff = p.throttle_boost_cutoff;
    c.angle.strength = p.angle_strength; c.angle.feedforward = p.angle_feedforward; c.angle.ff_smoothing_ms = p.angle_ff_smoothing; c.angle.earth_ref = p.angle_earth_ref;
    c.stab.accel_rp_dss = p.stab_accel_rp; c.stab.accel_yaw_dss = p.stab_accel_yaw; c.stab.input_tc = p.stab_input_tc * 0.01f; c.stab.rate_ff = p.stab_rate_ff != 0;
}

// The Filters tab: the gyro chain and the D-term / yaw filters of the PID loop (cutoffs already computed from the sliders).
inline void apply_filters(const Settings& s, fc::CoreConfig& c) {
    const S_filters& f = s.filters;
    c.gyro.lpf1_type = f.gyro_lpf1_type; c.gyro.lpf1_static_hz = f.gyro_lpf1_static_hz; c.gyro.lpf1_dyn_min_hz = f.gyro_lpf1_dyn_min_hz; c.gyro.lpf1_dyn_max_hz = f.gyro_lpf1_dyn_max_hz;
    c.gyro.lpf1_dyn_expo = f.gyro_lpf1_dyn_expo; c.gyro.lpf2_type = f.gyro_lpf2_type; c.gyro.lpf2_hz = f.gyro_lpf2_hz;
    c.gyro.notch1_hz = f.gyro_notch1_hz; c.gyro.notch1_cutoff = f.gyro_notch1_cutoff; c.gyro.notch2_hz = f.gyro_notch2_hz; c.gyro.notch2_cutoff = f.gyro_notch2_cutoff;
    c.gyro.dyn.count = f.dyn_notch_count; c.gyro.dyn.q = f.dyn_notch_q * 0.01f; c.gyro.dyn.min_hz = f.dyn_notch_min_hz; c.gyro.dyn.max_hz = f.dyn_notch_max_hz;
    c.gyro.rpm.harmonics = f.rpm_harmonics; c.gyro.rpm.min_hz = f.rpm_min_hz; c.gyro.rpm.q = f.rpm_q * 0.01f; c.gyro.rpm.fade_range_hz = f.rpm_fade_range_hz; c.gyro.rpm.lpf_hz = f.rpm_lpf_hz;
    for (int h = 0; h < 3; h++) c.gyro.rpm.weights[h] = f.rpm_weights[h];
    c.pid.dterm_lpf1_type = f.dterm_lpf1_type; c.pid.dterm_lpf1_static_hz = f.dterm_lpf1_static_hz; c.pid.dterm_lpf1_dyn_min_hz = f.dterm_lpf1_dyn_min_hz; c.pid.dterm_lpf1_dyn_max_hz = f.dterm_lpf1_dyn_max_hz;
    c.pid.dterm_lpf1_dyn_expo = f.dterm_lpf1_dyn_expo; c.pid.dterm_lpf2_type = f.dterm_lpf2_type; c.pid.dterm_lpf2_hz = f.dterm_lpf2_hz;
    c.pid.dterm_notch_hz = f.dterm_notch_hz; c.pid.dterm_notch_cutoff = f.dterm_notch_cutoff; c.pid.yaw_lowpass_hz = f.yaw_lowpass_hz;
}

// The "Position" tab: how optical flow is used, Position Hold, GPS Rescue, what the failsafe does. Also needs the mode
// switches: a rescue counts as set up when it has a switch or is the failsafe's procedure.
inline void apply_nav(const Settings& s, fc::CoreConfig& c) {
    const S_nav& n = s.nav;
    c.flow.yaw_deg = 90.0f * (float)(n.flow_align & 3); c.flow.flip = n.flow_flip != 0; c.flow.scale = n.flow_scale * 0.01f;
    c.flow.quality_min = n.flow_quality_min; c.flow.max_height_m = n.flow_max_height * 0.01f;
    c.use_tilt_aid = n.tilt_aid != 0;
    c.poshold.angle_max_deg = n.ph_angle_max; c.poshold.brake_rate_dps = n.ph_brake_rate; c.poshold.brake_angle_max_deg = n.ph_brake_angle;
    c.pos.angle_max_deg = n.ph_angle_max;
    c.pos.pos_p = n.pos_p * 0.1f; c.pos.vel_p = n.vel_p * 0.1f; c.pos.vel_i = n.vel_i * 0.1f; c.pos.vel_d = n.vel_d * 0.01f;
    c.fs.procedure = n.fs_procedure;
    c.rescue.alt_mode = n.rescue_alt_mode; c.rescue.initial_climb_m = n.rescue_initial_climb; c.rescue.return_alt_m = n.rescue_return_alt;
    c.rescue.ascend_mps = n.rescue_ascend_rate * 0.1f; c.rescue.descend_mps = n.rescue_descend_rate * 0.1f; c.rescue.ground_speed_mps = n.rescue_ground_speed * 0.1f;
    c.rescue.descent_dist_m = n.rescue_descent_dist; c.rescue.min_start_dist_m = n.rescue_min_start_dist; c.rescue.landing_alt_m = n.rescue_landing_alt;
    c.rescue.min_sats = n.rescue_min_sats;
    bool on_switch = false;
    for (int i = 0; i < 16; i++) if (s.modes.mode[i] == MODE_RESCUE && s.modes.aux[i] < fc::kAuxCount && s.modes.hi[i] > s.modes.lo[i]) on_switch = true;
    c.rescue_configured = on_switch || n.fs_procedure == FS_PROCEDURE_RESCUE; c.rescue_allow_no_fix = n.rescue_allow_no_fix != 0;
    c.gps_alt = n.gps_alt; c.gps_vel_z = n.gps_vel_z != 0;          // (read by the estimator every step: live)
    c.ph_min_gps_hz = (float)n.ph_min_gps_hz;
    c.ekf_nav = n.ekf == EKF_MODE_NAV;
}

// The althold section: what the throttle stick means in Altitude Hold (live).
inline void apply_althold(const Settings& s, fc::AltHoldConfig& a) {
    const S_althold& h = s.althold;
    a.mid_stick = h.mid_stick * 0.01f; a.deadband = h.deadband * 0.01f; a.low_stick = h.low_stick * 0.01f; a.max_stick = h.max_stick * 0.01f;
    a.max_climb_mps = h.climb_rate * 0.1f; a.max_descend_mps = h.descend_rate * 0.1f;
    if (a.low_stick > a.mid_stick - a.deadband - 0.05f) a.low_stick = fmaxf(0.02f, a.mid_stick - a.deadband - 0.05f);      // (a usable descent travel)
}

inline void apply_core(const Settings& s, fc::CoreConfig& c, fc::RcInterpreter& rc) {
    c.loop_hz = s.general.loop_hz; c.est_hz = s.general.est_hz; c.max_arm_angle_deg = s.general.max_arm_angle;
    c.fs.guard_s = s.general.fs_guard_ms * 0.001f; c.fs.recover_s = s.general.fs_recover_ms * 0.001f; c.fs.max_descent_s = s.general.fs_max_descent_s;
    c.fs.fallback_throttle_scale = s.general.fs_fallback_throttle * 0.01f; c.runaway.enabled = s.general.runaway_takeoff != 0;
    c.airmode = s.general.airmode; c.airmode_threshold = s.general.airmode_start_throttle * 0.01f;
    c.mixer.layout = (fc::Layout)s.motors.layout; c.mixer.props_out = s.motors.props_out != 0;
    c.use_mag = s.sensors.mag_enabled != 0; c.declination_deg = s.sensors.mag_declination * 0.1f;
    apply_pid(s, c); apply_filters(s, c); apply_rates_and_rc(s, c, rc); apply_modes(s, rc); apply_nav(s, c); apply_althold(s, c.alt);
}

inline void apply_motors(const Settings& s, motor::MotorConfig& m) {
    m.protocol = (motor::Protocol)s.motors.protocol; m.count = (uint8_t)fc::mixer_motor_count((fc::Layout)s.motors.layout);
    for (int i = 0; i < motor::kMaxMotors; i++) { m.reversed[i] = ((s.motors.reversed_mask >> i) & 1) != 0; m.order[i] = s.motors.order[i]; }
    m.pulse.disarmed = s.motors.pulse_disarmed; m.pulse.armed = s.motors.pulse_armed; m.pulse.max = s.motors.pulse_max; m.pulse_hz = s.motors.pulse_hz;
    m.bidirectional = s.motors.bidirectional != 0; m.dshot_driver = s.motors.dshot_driver; m.ext_pullup = s.motors.ext_pullup != 0; m.edt = s.motors.edt != 0; m.pole_count = s.motors.pole_count; m.dir_cmd_extended = s.motors.dir_cmd_extended != 0;
    m.idle = s.motors.idle_x10 * 0.001f; m.output_limit = s.motors.output_limit * 0.01f;
    m.dyn.min_rpm_100 = s.motors.dyn_idle_min_rpm; m.dyn.p_gain = s.motors.dyn_idle_p; m.dyn.i_gain = s.motors.dyn_idle_i; m.dyn.d_gain = s.motors.dyn_idle_d; m.dyn.max_increase = s.motors.dyn_idle_max_inc;
}

inline void apply_gps(const Settings& s, gps::Config& g) {
    g.provider = (gps::Provider)s.gps.provider; g.baud = s.gps.baud; g.update_rate_hz = s.gps.rate_hz; g.auto_baud = s.gps.auto_baud != 0; g.auto_config = s.gps.auto_config != 0;
    g.sbas = (gps::SbasMode)s.gps.sbas; g.sbas_integrity = s.gps.sbas_integrity != 0; g.use_galileo = s.gps.use_galileo != 0;
    g.acquire_model = (gps::AcquireModel)s.gps.acquire_model; g.flight_model = (gps::AcquireModel)s.gps.flight_model; g.utc_standard = (gps::UtcStandard)s.gps.utc_standard;
    g.set_home_point_once = s.gps.set_home_once != 0; g.use_3d_speed = s.gps.use_3d_speed != 0; g.min_sats = s.gps.min_sats;
}

// Battery thresholds (mV per cell, 0.1 s) -> the monitor's volts and seconds. The source and the divider go to BatterySensor.
inline void apply_battery(const Settings& s, batt::Config& b) {
    b.cells = s.battery.cells; b.cell_max_v = s.battery.cell_max_mv * 0.001f; b.cell_warning_v = s.battery.cell_warning_mv * 0.001f; b.cell_min_v = s.battery.cell_min_mv * 0.001f;
    b.warning_delay_s = s.battery.warning_delay * 0.1f; b.critical_delay_s = s.battery.critical_delay * 0.1f;
}
inline float battery_divider(const Settings& s) { return s.battery.divider * 0.01f; }
// buzzer: the conditions as set. motors: their own set, minus what a motor can never play; one ESC tone per beep.
inline void apply_beeper(const Settings& s, beep::Beeper& buzzer, beep::Beeper& motors) {
    buzzer.enabled = s.beeper.conditions; buzzer.slot_ms = 0;
    motors.enabled = s.beeper.motor_conditions & ~beep::kMotorNever; motors.slot_ms = (uint16_t)beep::kBeaconIntervalMs;
}
inline bool beeper_motor_default_matches() { S_beeper b; defaults(b); return b.motor_conditions == beep::kMotorDefault; }

inline void apply_blackbox(const Settings& s, bb::Config& c) {
    c.device = s.blackbox.device; c.trigger = s.blackbox.trigger; c.rate_hz = s.blackbox.rate_hz; c.groups = s.blackbox.fields; c.sd_mode = s.blackbox.sd_mode; c.sd_mhz = s.blackbox.sd_mhz;
}

inline void to_magcal(const S_calibration& c, MagCal& m) {
    m.valid = c.mag_valid != 0; m.radius = c.mag_radius;
    for (int i = 0; i < 3; i++) m.offset[i] = c.mag_offset[i];
    for (int i = 0; i < 9; i++) m.m[i / 3][i % 3] = c.mag_matrix[i];
}
inline void from_magcal(const MagCal& m, S_calibration& c) {
    c.mag_valid = m.valid ? 1 : 0; c.mag_radius = m.radius;
    for (int i = 0; i < 3; i++) c.mag_offset[i] = m.offset[i];
    for (int i = 0; i < 9; i++) c.mag_matrix[i] = m.m[i / 3][i % 3];
}

inline void to_acccal(const S_calibration& c, AccCal& a) {
    a.valid = c.acc_valid != 0;
    for (int i = 0; i < 3; i++) { a.offset[i] = c.acc_offset[i]; a.scale[i] = c.acc_scale[i]; }
}
inline void from_acccal(const AccCal& a, S_calibration& c) {
    c.acc_valid = a.valid ? 1 : 0;
    for (int i = 0; i < 3; i++) { c.acc_offset[i] = a.offset[i]; c.acc_scale[i] = a.scale[i]; }
}
// Total fine board alignment = the board trim the user typed + the level trim the firmware measured (degrees).
inline void board_fine_alignment(const Settings& s, float& roll, float& pitch, float& yaw) {
    roll = (s.general.board_trim_roll + s.calibration.acc_trim_roll) * 0.1f;
    pitch = (s.general.board_trim_pitch + s.calibration.acc_trim_pitch) * 0.1f;
    yaw = s.general.board_trim_yaw * 0.1f;
}
WB_SAME(MAG_CAL_RESULT_OK, MagCalResult::OK); WB_SAME(MAG_CAL_RESULT_TOO_FEW_SAMPLES, MagCalResult::TOO_FEW_SAMPLES); WB_SAME(MAG_CAL_RESULT_POOR_COVERAGE, MagCalResult::POOR_COVERAGE);
WB_SAME(MAG_CAL_RESULT_SINGULAR, MagCalResult::SINGULAR); WB_SAME(MAG_CAL_RESULT_NOT_ELLIPSOID, MagCalResult::NOT_ELLIPSOID); WB_SAME(MAG_CAL_RESULT_TOO_DISTORTED, MagCalResult::TOO_DISTORTED);
WB_SAME(MAG_CAL_RESULT_BAD_FIELD_STRENGTH, MagCalResult::BAD_FIELD_STRENGTH); WB_SAME(MAG_CAL_RESULT_BAD_FIT, MagCalResult::BAD_FIT);
WB_SAME(CAL_STATE_IDLE, CalState::IDLE); WB_SAME(CAL_STATE_RUNNING, CalState::RUNNING); WB_SAME(CAL_STATE_DONE, CalState::DONE); WB_SAME(CAL_STATE_FAILED, CalState::FAILED);
#undef WB_SAME

} // namespace wbp
