#pragma once
#include "../config/fast_code.h"
#include "types.h"
#include "filters.h"
#include "ahrs.h"
#include "vertical_est.h"
#include "horizontal_est.h"
#include "rc.h"
#include "gyro_filter.h"
#include "setpoint.h"
#include "attitude.h"
#include "pid.h"
#include "mixer.h"
#include "althold.h"
#include "safety.h"
#include "tilt_aid.h"
#include "flow.h"
#include "poshold.h"
#include "rescue.h"

// ============================================================================
// FlightCore - the whole flight stack behind two entry points, shaped around what the hardware drivers actually deliver:
//
//   SENSOR TASK (whenever a driver returns a NEW sample)          FAST LOOP (gyro rate, e.g. 4 kHz)
//     core.on_baro(BaroSensor::altitudeM(), sigmaM(), ms)           out = core.update(FastInput{ gyro/acc from IMUAccGyro::read(), dt,
//     core.on_range(RangeFinder::heightM(), sigmaM(), ms)                                          rc_us[16] + link, Health, rpm, MotorLimits, now_ms });
//     core.on_gps(GpsMeas{...}, ms)                                  motors.update(micros(), out.motor, out.armed);
//     core.on_mag(MagSensor::board(), readyForFusion, ms)            // and  limits.idle = motors.idleFraction(...)  feeds back next cycle
//
// Inside: gyro filter chain (gyro_filter.h) -> Ahrs (+compass, yaw only) -> estimators at est_hz (vertical Kalman with baro offset +
// range + GPS altitude; horizontal Kalman with GPS) -> arming / failsafe -> sticks: rates, RC smoothing, feedforward (setpoint.h)
// -> modes (Acro; Angle = Betaflight's; Stabilize = ArduPilot's; AltHold; failsafe = self-level + controlled descent) -> rate PID
// (pid.h, Betaflight's) -> mixer -> ABSOLUTE motor values inside [limits.idle, limits.limit].
// Navigation (at the estimators' rate): a velocity over the ground from GPS and / or optical flow (flow.h) goes into the horizontal
// estimate and tells the attitude estimate which way is down while the craft accelerates (tilt_aid.h); Position Hold (poshold.h)
// and GPS Rescue (rescue.h) turn it into a lean for the Angle controller, the rescue also into a climb rate for Altitude Hold.
// Every sensor is optional: a missing / stale one is simply not fused, and arming / the modes ask Health + the estimator validity flags.
// The sensor pushes and update() must run in the same task (or be protected by the caller): they share plain data, no locks.
// ============================================================================
namespace fc {

enum AirmodeSetting : uint8_t { AM_OFF = 0, AM_AUTO = 1, AM_ALWAYS = 2 };

struct Health {                    // straight from the drivers' valid() / usable() / healthy()
    bool imu_ok = false, gyro_calibrated = false, calibrating = false;
    bool batt_low = false, batt_critical = false;       // batt::Monitor: below the warning level (no arming) / below the minimum (failsafe landing)
    bool baro_ok = false, range_ok = false, gps_ok = false;
    bool mag_calibrated = false, mag_ok = false;
    bool flow_ok = false;                               // an optical-flow sensor is there and answers (not: it sees the ground)
    bool pin_conflict = false;                          // the pin map puts two functions on one GPIO (checked at boot)
    bool loop_slow = false;                             // the loop ran less than 80 % of its configured cycles, two seconds in a row
};

struct MotorFeedback { float rpm[kMaxMotors] = {}; bool rpm_valid = false; };   // MotorOutputManager::rpm_array() (logical motor order; 0 = that motor has no telemetry). rpm_valid: there is telemetry at all

struct GpsMeas {                   // metres / m/s relative to the home point, altitude relative to the home altitude
    bool valid = false;
    float north_m = 0, east_m = 0, alt_m = 0, vn = 0, ve = 0, vd = 0;
    float sigma_pos = 2.0f, sigma_vel = 0.3f, sigma_alt = 3.0f;
    uint8_t sats = 0;
};

struct CoreConfig {
    float loop_hz = 4000.0f, est_hz = 500.0f;
    RcConfig rc; RateProfile rates;
    PidConfig pid; GyroFilterConfig gyro; FeedforwardConfig ff; RcSmoothingConfig rc_smoothing;
    AngleConfig angle; StabilizeConfig stab;
    uint8_t throttle_boost = 5, throttle_boost_cutoff = 15;      // Betaflight throttle_boost / throttle_boost_cutoff
    uint8_t airmode = AM_AUTO;                     // AirmodeSetting: stabilised with the throttle stick at the bottom?
    float airmode_threshold = 0.25f;                 // throttle (0..1) that counts as "in the air" at once (Betaflight airmode_start_throttle_percent)
    float airborne_after_s = 1.0f;                   // ... and so does any throttle above "low" held this long (a strong craft hovers below that threshold)
    MixerConfig mixer;
    AltHoldConfig alt;
    FsConfig fs;
    RunawayConfig runaway;
    float max_arm_angle_deg = 25.0f;
    Ahrs::Config ahrs; KfConfig vertical; HorizontalConfig horizontal;
    bool use_mag = false; float declination_deg = 0.0f;
    float baro_stale_ms = 1000.0f, range_stale_ms = 250.0f, gps_stale_ms = 1500.0f, mag_stale_ms = 300.0f;
    float ground_effect_s = 1.5f;                    // the barometer counts for less from arming until the craft has been in the air this long
    AccelGainSchedule::Config ahrs_ground;           // how much more the accelerometer counts while disarmed (standing still it really shows gravity)
    // ---- navigation
    TiltAid::Config tilt_aid; bool use_tilt_aid = true;      // tilt from the measured acceleration (GPS / flow velocity) while armed
    float aid_max_sigma_vel = 0.5f;                  // a GPS velocity less sure than this (m/s) is not used for it
    FlowConfig flow; float flow_stale_ms = 300.0f;
    PosControlConfig pos; PosHoldConfig poshold; RescueConfig rescue;      // poscontrol.h, poshold.h, rescue.h; poshold.angle_max_deg is the lean limit of both
    bool rescue_configured = false, rescue_allow_no_fix = false;          // a RESCUE switch is assigned or the failsafe procedure is RESCUE: arming wants a GPS fix (home)
    CoreConfig() { rates.center_sens_dps[YAW] = 70; rates.max_rate_dps[YAW] = 400; }
};

struct FastInput {
    Vec3 gyro_dps;                 // board frame (FRD) deg/s, bias removed  (IMUAccGyro::read)
    Vec3 acc_g;                    // board frame (FRD) specific force in g
    float dt = 0.00025f;           // seconds since the previous call
    uint16_t rc_us[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bool rc_link = false;
    uint32_t rc_seq = 0;           // changes whenever a new receiver frame arrived (feedforward and RC smoothing work per frame)
    Health health;
    MotorFeedback motors;
    MotorLimits limits;            // idle (static+dynamic) and output limit, owned by the motor layer
    uint32_t now_ms = 0;
};

struct Estimate {
    EulerDeg att;                  // user convention
    float tilt_deg = 0;
    float z = 0, vz = 0, z_sigma = 0; bool alt_valid = false;          // height above the takeoff point (m), climb rate (m/s)
    float az = 0;                  // upward acceleration without gravity (m/s^2), averaged over one estimator step
    float pos_n = 0, pos_e = 0, vel_n = 0, vel_e = 0;
    bool pos_valid = false;        // position relative to the home point (GPS)
    bool vel_valid = false;        // velocity over the ground (GPS or optical flow): enough to hold a position, not to find home
    bool flow_valid = false;       // optical flow is at work: a sample was used within the last flow_stale_ms
    uint8_t flow_reject = FLOW_NO_SAMPLE; float flow_forward = 0, flow_right = 0;      // why the latest sample was not used (FlowReject: OK = it was, NO_SAMPLE = none came for flow_stale_ms), the velocity of the last used one (craft axes)
    bool tilt_aided = false;       // the attitude is corrected with the measured acceleration
    bool yaw_from_mag = false;
};

struct CoreOutput {
    float motor[kMaxMotors] = {};  // absolute 0..1 (0 when disarmed)
    int motor_count = 4;
    bool armed = false;
    uint32_t arm_blockers = 0;
    FsStage failsafe = FsStage::NONE;
    uint8_t fs_cause = 0;          // FsCause bits while the failsafe is active
    uint32_t modes = 0;            // active Mode bits as the pilot selected them
    bool althold_active = false, althold_unavailable = false;
    AhState ah_state = AhState::STOPPED;
    float throttle = 0;
    float hover_throttle = 0;      // the throttle (0..1) at which the craft hovers, as measured in flight (0 before the craft was armed)
    float sp_dps[3] = {}, gyro_f_dps[3] = {}, u[3] = {};               // debug / blackbox; u = PID sum / 1000 (the mixer's input)
    float pid_p[3] = {}, pid_i[3] = {}, pid_d[3] = {}, pid_f[3] = {};  // the terms that make up u[], in PID sum units (x 1000)
    bool airmode_active = false;   // stabilised (full authority, I-terms) although the throttle stick is at the bottom
    bool airborne = false;         // the craft is taken to be in the air (see FlightCore::update, step 6)
    bool ground_idle = false;      // every motor at idle, nothing mixed: armed and standing on the ground
    bool est_stepped = false;      // this cycle also ran the estimators (every loop_hz / est_hz cycles): it takes longer than the others
    float stick[4] = {};           // pilot command: roll, pitch, yaw (-1..1) and the throttle stick (0..1), before modes / failsafe
    bool throttle_low = true;      // throttle stick at the bottom
    uint32_t arm_refused = 0;      // arm attempts rejected by a blocker since boot
    // navigation
    bool poshold_active = false, poshold_unavailable = false;         // Position Hold flies / is switched on but has no velocity over the ground to work with
    uint8_t ph_state[2] = {0, 0};  // PhAxis of roll, pitch
    bool rescue_active = false, rescue_available = false; uint8_t rescue_phase = 0, rescue_fail = 0;      // RescuePhase, RescueFail
    bool nav_lean = false; float nav_roll = 0, nav_pitch = 0;         // an autopilot gives the lean (deg, pilot convention)
    float nav_target_n = 0, nav_target_e = 0, nav_wind_n = 0, nav_wind_e = 0, nav_home_dist = 0;
    Estimate est;
};

class FlightCore {
public:
    CoreConfig cfg;
    RcInterpreter rc;

    void configure() {
        const float dt = 1.0f / cfg.loop_hz;
        ahrs_.cfg = cfg.ahrs; acc_gain_.cfg = cfg.ahrs_ground;
        mixer_.cfg = cfg.mixer; mixer_.configure();
        gf_.configure(cfg.gyro, cfg.loop_hz, mixer_.motor_count());
        configure_control();
        (void)dt;
        vert_.cfg = cfg.vertical; vert_.reset(0.0f);
        horiz_.cfg = cfg.horizontal; horiz_.reset(); tilt_aid_.cfg = cfg.tilt_aid; tilt_aid_.reset(); gyro_hist_.configure(1.0f / cfg.est_hz);
        gyro_sum_ = Vec3(); flow_new_ = false; have_flow_ = false; have_gps_xy_ = false; heading_ok_ = false;
        ph_.cfg = cfg.poshold; ph_.ctl.cfg = cfg.pos; rctl_.cfg = cfg.pos; rctl_.cfg.angle_max_deg = cfg.poshold.angle_max_deg; rescue_.cfg = cfg.rescue;
        was_poshold_ = false; nav_lean_ = false; ph_unavail_ = false; rescue_ok_ = false; rescue_out_ = RescueOutput(); nav_tgt_[0] = nav_tgt_[1] = 0;
        alt_.cfg = cfg.alt; alt_.init();
        fs_.cfg = cfg.fs; runaway_.cfg = cfg.runaway;
        decim_ = (int)(cfg.loop_hz / cfg.est_hz + 0.5f); if (decim_ < 1) decim_ = 1;
        acc_n_ = 0; acc_sum_ = Vec3(); est_dt_ = 0.0f; ground_t_ = 0.0f; have_att_ = false; mag_seeded_ = false; was_althold_ = false; last_thr_ = 0;
        have_rc_ = false; rc_seq_ = 0; raised_ = airborne_ = false; up_t_ = 0.0f; land_.reset(); was_angle_ = was_stab_ = false; ctl_dirty_ = true;
        baro_new_ = range_new_ = gps_new_ = false; baro_ms_ = range_ms_ = gps_ms_ = mag_ms_ = 0; have_baro_ = have_range_ = have_gps_ = false; mag_ready_ = false;
    }

    // Live re-tune from the app while DISARMED (PID / feedforward / angle settings changed in cfg): pushes them into the controllers
    // without touching the attitude or the estimators. Rates and the RC map are read from cfg / rc directly every cycle.
    void apply_tuning() { configure_control(); }
    // The same for the gyro filters (they restart from the current gyro value). Disarmed only.
    void apply_filters() { gf_.configure(cfg.gyro, cfg.loop_hz, mixer_.motor_count()); configure_control(); }

    // The gyro is gone (IMU stopped answering): nothing can be controlled any more. Disarms and requires the arm switch to be cycled.
    void emergency_disarm() { arming_.force_disarm(); armed_ = false; reset_control(); }

    // ---------------- sensor pushes (call when a driver reports a NEW sample)
    void on_baro(float alt_m, float sigma_m, uint32_t now_ms) { baro_alt_ = alt_m; baro_sigma_ = sigma_m; baro_ms_ = now_ms; baro_new_ = true; have_baro_ = true; }
    void on_range(float height_m, float sigma_m, uint32_t now_ms) { range_h_ = height_m; range_sigma_ = sigma_m; range_ms_ = now_ms; range_new_ = true; have_range_ = true; }
    void on_gps(const GpsMeas& g, uint32_t now_ms) { gps_ = g; gps_ms_ = now_ms; gps_new_ = g.valid; have_gps_ = g.valid; }
    void on_mag(const Vec3& board_gauss, bool ready, uint32_t now_ms) { mag_ = board_gauss; mag_ready_ = ready; mag_ms_ = now_ms; }
    void on_flow(const FlowSample& f, uint32_t now_ms) { flow_ = f; flow_ms_ = now_ms; flow_new_ = true; flow_seen_ = true; }
    // Re-zero the horizontal position (e.g. when the home point is set at arming).
    void reset_position() { horiz_.reset(); tilt_aid_.restart(); have_gps_ = false; have_gps_xy_ = false; }
    // Live re-tune of the navigation settings from the app (disarmed)
    void apply_nav() { ph_.cfg = cfg.poshold; ph_.ctl.cfg = cfg.pos; rctl_.cfg = cfg.pos; rctl_.cfg.angle_max_deg = cfg.poshold.angle_max_deg; rescue_.cfg = cfg.rescue; tilt_aid_.cfg = cfg.tilt_aid; fs_.cfg = cfg.fs; }

    WB_FAST CoreOutput update(const FastInput& in) {
        CoreOutput o; const float dt = in.dt; const uint32_t now = in.now_ms;
        o.motor_count = mixer_.motor_count();

        // ---- 1. gyro: FRD deg/s -> user rates (pitch is negated), filtered for the rate loop
        g_f_[0] = in.gyro_dps.x; g_f_[1] = -in.gyro_dps.y; g_f_[2] = in.gyro_dps.z;
        gf_.apply(g_f_, in.motors.rpm_valid ? in.motors.rpm : nullptr);
        for (int a = 0; a < 3; a++) o.gyro_f_dps[a] = g_f_[a];

        // ---- 2. attitude (unfiltered gyro) with the compass when it is trustworthy
        if (!have_att_) {                                      // initial attitude from gravity - only from a plausible reading (never from a zero / 0 g vector)
            const float an = sqrtf(in.acc_g.x * in.acc_g.x + in.acc_g.y * in.acc_g.y + in.acc_g.z * in.acc_g.z);
            if (an > 0.5f && an < 1.5f) { ahrs_.reset_to_accel(in.acc_g); have_att_ = true; }
        }
        const bool mag_fresh = cfg.use_mag && mag_ready_ && age_ms(now, mag_ms_) <= (uint32_t)cfg.mag_stale_ms;
        if (mag_fresh && !mag_seeded_ && !armed_) mag_seeded_ = ahrs_.seed_yaw_from_mag(mag_, cfg.declination_deg);
        const float turning = fmaxf(fabsf(in.gyro_dps.x), fmaxf(fabsf(in.gyro_dps.y), fabsf(in.gyro_dps.z)));
        const float acc_gain = acc_gain_.update(armed_, turning, ahrs_.acc_weight() > 0.0f, dt);
        ahrs_.update({in.gyro_dps.x * kDeg2Rad, in.gyro_dps.y * kDeg2Rad, in.gyro_dps.z * kDeg2Rad}, in.acc_g, dt, mag_fresh ? &mag_ : nullptr, cfg.declination_deg, acc_gain,
                     !armed_ && turning < cfg.ahrs_ground.gyro_limit_dps);
        att_ = ahrs_.euler_user();

        // ---- 3. estimators at est_hz on the averaged accelerometer
        // The cycles are not all the same length: each reading counts for the time it stands for, and the step is made over the
        // time that really passed (not "the last cycle x n").
        acc_sum_.x += in.acc_g.x * dt; acc_sum_.y += in.acc_g.y * dt; acc_sum_.z += in.acc_g.z * dt; est_dt_ += dt;
        gyro_sum_.x += in.gyro_dps.x * dt; gyro_sum_.y += in.gyro_dps.y * dt;
        float dt_est = 0.0f;                                   // > 0: this cycle is an estimator step, over this much time
        if (++acc_n_ >= decim_ && est_dt_ > 0.0f) {
            dt_est = est_dt_;
            gyro_hist_.push(gyro_sum_.x / dt_est * kDeg2Rad, gyro_sum_.y / dt_est * kDeg2Rad);        // (the optical flow asks for the turn rate of its own window)
            estimator_step({acc_sum_.x / dt_est, acc_sum_.y / dt_est, acc_sum_.z / dt_est}, dt_est, now);
            o.est_stepped = true;
            acc_n_ = 0; acc_sum_ = Vec3(); gyro_sum_ = Vec3(); est_dt_ = 0.0f;
            // the tilt from a measured acceleration takes the accelerometer's place while flying (tilt_aid.h)
            const bool aid = cfg.use_tilt_aid && armed_ && tilt_aid_.valid(); const Vec3 e = tilt_aid_.error_world(); ahrs_.set_aid(aid ? &e : nullptr); est_.tilt_aided = aid;
        }

        // ---- 4. pilot command, arming, failsafe
        const RcCommand cmd = rc.process(in.rc_us, in.rc_link);
        o.modes = cmd.modes;
        ArmInput ai;
        ai.imu_ok = in.health.imu_ok; ai.gyro_calibrated = in.health.gyro_calibrated; ai.rx_ok = in.rc_link; ai.throttle_low = cmd.throttle_low;
        ai.tilt_deg = armed_ ? 0.0f : ahrs_.tilt_deg(); ai.max_arm_angle_deg = cfg.max_arm_angle_deg;
        ai.failsafe_active = fs_.get_stage() != FsStage::NONE;
        ai.baro_ok = in.health.baro_ok; ai.gps_fix = in.health.gps_ok;
        // Position Hold needs a velocity over the ground: optical flow gives one by itself, a GPS only together with a compass
        const bool compass = cfg.use_mag && in.health.mag_calibrated;
        ai.pos_source = in.health.flow_ok || (in.health.gps_ok && compass);
        ai.rescue_configured = cfg.rescue_configured; ai.rescue_allow_no_fix = cfg.rescue_allow_no_fix; ai.rescue_no_compass = cfg.rescue_configured && !cfg.use_mag;
        ai.althold_switch = (cmd.modes >> M_ALTHOLD) & 1u; ai.poshold_switch = (cmd.modes >> M_POSHOLD) & 1u;
        ai.calibrating = in.health.calibrating; ai.batt_low = in.health.batt_low;
        ai.use_mag = cfg.use_mag; ai.mag_calibrated = in.health.mag_calibrated; ai.pin_conflict = in.health.pin_conflict; ai.loop_slow = in.health.loop_slow;
        const bool was_armed = armed_;
        armed_ = arming_.update((cmd.modes >> M_ARM) & 1u, in.rc_link, ai);
        if (armed_ && !was_armed) runaway_.on_arm();
        o.arm_blockers = arming_.blockers(); o.arm_refused = arming_.refused();
        o.stick[0] = cmd.roll; o.stick[1] = cmd.pitch; o.stick[2] = cmd.yaw; o.stick[3] = cmd.throttle_raw; o.throttle_low = cmd.throttle_low;

        FsInput fi;
        fi.rx_ok = in.rc_link; fi.batt_critical = in.health.batt_critical; fi.alt_valid = est_.alt_valid;
        fi.landed = alt_.get_state() == AhState::LANDED_IDLE; fi.airborne = airborne_; fi.armed = armed_; fi.switch_on = (cmd.modes >> M_FAILSAFE_SW) & 1u; fi.vz = est_.vz;
        fi.rescue_active = rescue_out_.active; fi.stick_move = in.rc_link ? fmaxf(fabsf(cmd.roll), fmaxf(fabsf(cmd.pitch), fabsf(cmd.yaw))) : 0.0f;
        const FsOutput fo = fs_.update(fi, dt);
        if (fo.disarm) { arming_.force_disarm(); armed_ = false; }
        o.failsafe = fo.stage; o.fs_cause = fo.cause; o.armed = armed_;

        est_.att = att_; est_.tilt_deg = ahrs_.tilt_deg(); est_.yaw_from_mag = mag_fresh && mag_seeded_; heading_ok_ = est_.yaw_from_mag;
        o.est = est_; o.ah_state = alt_.get_state();

        // ---- 4b. the stick path: a new receiver frame -> rates + feedforward; every cycle -> RC smoothing
        if (!have_rc_ || in.rc_seq != rc_seq_) {
            have_rc_ = true; rc_seq_ = in.rc_seq;
            const float st[3] = {cmd.roll, cmd.pitch, cmd.yaw};
            sp_gen_.on_frame(st, cmd.throttle, cfg.rates);
        }
        const bool neutral = fo.neutral_sticks;
        if (neutral) sp_gen_.neutral();                        // failsafe: the pilot's sticks are not to be believed
        const StickSetpoint ss = sp_gen_.update();

        if (!armed_) {                                         // disarmed: nothing accumulates, motors off
            reset_control();
            pid_.update_throttle(0.0f, &gf_);                  // the dynamic low-passes rest at their zero-throttle cutoff
            AhInput idle; idle.armed = false; alt_.update(idle, dt); was_althold_ = false; last_thr_ = 0;
            o.ah_state = alt_.get_state();
            if (dt_est > 0.0f) { RescueInput ri; rescue_out_ = rescue_.update(ri, dt_est); }       // (forgets the flight: its highest point, a failure)
            was_poshold_ = false; nav_lean_ = false; ph_unavail_ = false; rescue_ok_ = false;
            return o;
        }
        ctl_dirty_ = true;

        // ---- 5. throttle: the stick, Altitude Hold, or the failsafe's descent
        const bool althold_sw = (cmd.modes >> M_ALTHOLD) & 1u;
        alt_.set_limits(in.limits.idle, in.limits.limit);
        float thr;
        const bool alt_ok = est_.alt_valid;
        o.althold_unavailable = false;
        bool pilot_throttle = false;
        const bool rescue = rescue_out_.active;                // GPS Rescue flies: it gives the climb rate (it only runs with a valid height)
        bool rescue_landed = false;
        if (rescue || fo.alt_descent || (althold_sw && alt_ok && !fo.throttle_fallback)) {
            // Engaged while flying: carry on from the present throttle. "Flying" is what the craft has been doing (airborne_), not
            // where the stick happens to be: a craft that hovers at 13 % of the stick must not be taken for a parked one.
            if (!was_althold_ && airborne_) alt_.begin_flight(est_.z, last_thr_, ahrs_.up_cos());
            AhInput ah; ah.armed = true; ah.stick01 = cmd.throttle_raw; ah.z = est_.z; ah.v = est_.vz; ah.a_up = est_.az; ah.up_cos = ahrs_.up_cos(); ah.failsafe_descent = fo.alt_descent && !rescue; ah.pilot_absent = neutral || rescue; ah.alt_valid = true;
            ah.ext_climb = rescue; ah.climb_mps = rescue_out_.climb_mps;
            thr = alt_.update(ah, dt); o.althold_active = true; was_althold_ = true;
            rescue_landed = rescue && alt_.get_state() == AhState::LANDED_IDLE;
        } else if (fo.throttle_fallback) {                                            // failsafe without an altitude estimate: gentle fixed descent, never a motor cut
            thr = alt_.hover_throttle(cfg.fs.fallback_throttle_scale); was_althold_ = false;
        } else {
            if (was_althold_) alt_.on_disengage();
            was_althold_ = false; pilot_throttle = true;
            if (althold_sw) {
                // The Altitude Hold switch is on but there is no height (never was, or the barometer just dropped out). The pilot
                // holds the stick where "hold" is - the middle - so the stick keeps that meaning: middle = the measured hover
                // throttle, bottom = 0, top = full (ArduCopter's manual throttle is centred on hover the same way). The throttle
                // does not jump when the height goes, and Altitude Hold takes over again from this throttle when it returns.
                o.althold_unavailable = true;
                thr = hover_centred(cmd.throttle_raw, alt_.hover_throttle());
            } else thr = ss.throttle;                                                 // the stick through its curve, smoothed
        }
        o.ah_state = alt_.get_state();
        last_thr_ = thr; o.throttle = thr;
        // The hover throttle is measured in every mode (Altitude Hold and the failsafe descent start from it) - except while the
        // failsafe flies the open-loop descent: there the throttle is DERIVED from the hover value, and a craft that sinks with
        // air drag carrying part of its weight would "learn" a lower and lower hover, sinking faster and faster.
        if (!fo.throttle_fallback) alt_.learn(thr, ahrs_.up_cos(), est_.az, dt);
        o.hover_throttle = alt_.hover_throttle();

        // ---- 6. in the air, or standing on the ground?
        // airborne_: the throttle has been up (25 % at once, or anything above "low" for a second), or Altitude Hold has lifted the
        // craft off. It ends when the craft is found standing still with the throttle down (LandDetector), when Altitude Hold has
        // landed it, and with the disarm. raised_ is Betaflight's latch: set with it, cleared only by the disarm.
        const float turning_f = fmaxf(fabsf(g_f_[0]), fmaxf(fabsf(g_f_[1]), fabsf(g_f_[2])));
        if (pilot_throttle) {
            up_t_ = cmd.throttle_low ? 0.0f : up_t_ + dt;
            if (cmd.throttle_cmd >= cfg.airmode_threshold || up_t_ >= cfg.airborne_after_s) airborne_ = raised_ = true;
            LandDetector::Input li; li.throttle_down = cmd.throttle_low; li.accel_1g = ahrs_.acc_weight() > 0.0f; li.gyro_dps = turning_f; li.alt_valid = est_.alt_valid; li.vz = est_.vz;
            if (land_.update(li, dt)) airborne_ = false;
        } else {
            up_t_ = 0.0f; land_.reset();
            if (o.althold_active) { if (alt_.lifted()) airborne_ = raised_ = true; else if (alt_.get_state() != AhState::TAKEOFF) airborne_ = false; }
        }
        o.airborne = airborne_;
        // Airmode = the craft stays stabilised (full authority, I-terms) with the throttle stick at the bottom. AUTO: while it is in
        // the air. ALWAYS: from the first time the throttle was up until the disarm, as Betaflight does - on the ground or on the
        // bench the I-terms then wind up against a craft that cannot answer, and the motors run up.
        const bool airmode = pilot_throttle && (cfg.airmode == AM_ALWAYS ? raised_ : cfg.airmode == AM_AUTO ? airborne_ : false);
        o.airmode_active = airmode && cmd.throttle_low;
        // ground_idle: every motor at idle, nothing mixed, nothing integrates (ArduCopter's GROUND_IDLE)
        const bool ground_idle = pilot_throttle ? (cmd.throttle_low && !airmode)
                               : (o.althold_active && alt_.get_state() != AhState::TAKEOFF && alt_.get_state() != AhState::FLYING);
        o.ground_idle = ground_idle;

        // ---- 6b. navigation, at the estimators' rate: GPS Rescue (a switch, or the failsafe's procedure) comes before Position Hold
        if (dt_est > 0.0f) {
            const float yaw = ahrs_.euler_aero().yaw;
            RescueInput ri; ri.armed = true; ri.airborne = airborne_; ri.request = airborne_ && (((cmd.modes >> M_RESCUE) & 1u) || fo.rescue);
            ri.pos_n = est_.pos_n; ri.pos_e = est_.pos_e; ri.vel_n = est_.vel_n; ri.vel_e = est_.vel_e; ri.z = est_.z; ri.yaw_deg = yaw;
            ri.pos_valid = est_.pos_valid; ri.alt_valid = est_.alt_valid; ri.heading_valid = est_.yaw_from_mag; ri.sats = gps_.sats;
            const bool was_rescue = rescue_out_.active;
            rescue_out_ = rescue_.update(ri, dt_est);
            rescue_ok_ = Rescue::available(ri);
            // Position Hold needs a velocity over the ground; one from GPS only means something to the craft with a compass heading
            const bool ph_ok = est_.vel_valid && (est_.flow_valid || est_.yaw_from_mag);
            const bool ph_on = ((cmd.modes >> M_POSHOLD) & 1u) && !neutral && !rescue_out_.active;
            ph_unavail_ = ph_on && !ph_ok;
            if (rescue_out_.active) {
                if (!was_rescue) rctl_.init(est_.pos_n, est_.pos_e, est_.vel_n, est_.vel_e, angle_.target_deg(ROLL), angle_.target_deg(PITCH), yaw);
                rctl_.update(est_.pos_n, est_.pos_e, est_.vel_n, est_.vel_e, dt_est, false, rescue_out_.vel_n, rescue_out_.vel_e);
                rctl_.lean(yaw, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                nav_lean_ = true; was_poshold_ = false;
            } else if (ph_on && ph_ok) {
                PosHoldInput pi; pi.airborne = airborne_; pi.yaw_deg = yaw; pi.pos_n = est_.pos_n; pi.pos_e = est_.pos_e; pi.vel_n = est_.vel_n; pi.vel_e = est_.vel_e;
                pi.pilot_roll_deg = cfg.angle.limit_deg * ss.rate[ROLL] / fmaxf(cfg.rates.max_rate_dps[ROLL], 1.0f); pi.pilot_pitch_deg = cfg.angle.limit_deg * ss.rate[PITCH] / fmaxf(cfg.rates.max_rate_dps[PITCH], 1.0f);
                pi.roll_centred = cmd.roll == 0.0f; pi.pitch_centred = cmd.pitch == 0.0f;
                if (!was_poshold_) ph_.init(pi);
                ph_.update(pi, dt_est, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                nav_lean_ = true; was_poshold_ = true;
            } else { nav_lean_ = false; was_poshold_ = false; }
        }
        o.rescue_active = rescue_out_.active; o.rescue_available = rescue_ok_; o.rescue_phase = (uint8_t)rescue_.phase(); o.rescue_fail = (uint8_t)rescue_.failure();
        o.poshold_active = was_poshold_; o.poshold_unavailable = ph_unavail_; o.ph_state[0] = (uint8_t)ph_.state(0); o.ph_state[1] = (uint8_t)ph_.state(1);
        o.nav_lean = nav_lean_; o.nav_roll = nav_tgt_[ROLL]; o.nav_pitch = nav_tgt_[PITCH];
        o.nav_target_n = ph_.ctl.target_n(); o.nav_target_e = ph_.ctl.target_e(); o.nav_wind_n = ph_.wind_n(); o.nav_wind_e = ph_.wind_e(); o.nav_home_dist = sqrtf(est_.pos_n * est_.pos_n + est_.pos_e * est_.pos_e);

        // ---- 7. modes -> rate setpoints
        const bool angle = ((cmd.modes >> M_ANGLE) & 1u) || neutral;    // the failsafe levels the craft; the hold modes and the rescue switch carry ANGLE with them (ModeMap::resolve)
        const bool stab = !angle && ((cmd.modes >> M_STABILIZE) & 1u);
        float sp_dps[3] = {ss.rate[ROLL], ss.rate[PITCH], ss.rate[YAW]};
        // the rescue turns the nose towards home; a pilot who is there (a rescue from the switch) keeps the last word with his yaw stick
        if (rescue_out_.active && rescue_out_.yaw_control && (neutral || fabsf(cmd.yaw) < 0.05f)) sp_dps[YAW] = rescue_out_.yaw_rate_dps;
        PidInput pin;
        if (angle) {                                           // Betaflight Angle: the stick - or the autopilot - gives a tilt
            if (!was_angle_) angle_.reset();
            const float ffrp[2] = {ss.feedforward[ROLL], ss.feedforward[PITCH]};
            angle_.update(sp_dps, ffrp, att_.roll, att_.pitch, cfg.rates.max_rate_dps, nav_lean_ ? nav_tgt_ : nullptr, cfg.poshold.angle_max_deg);
            pin.feedforward[YAW] = ss.feedforward[YAW];        // roll / pitch already carry the stick's speed in their angle target
        } else if (stab) {                                     // ArduPilot Stabilize: shaped tilt target, heading hold
            if (!was_stab_) stab_.reset(att_.roll, att_.pitch, att_.yaw);
            if (ground_idle) stab_.hold_yaw(att_.yaw);         // standing on the ground: the heading to hold is the one it has (ArduCopter resets the yaw target there)
            const float st[2] = {ss.stick[ROLL], ss.stick[PITCH]}, at[3] = {att_.roll, att_.pitch, att_.yaw};
            stab_.update(st, ss.rate[YAW], at, dt, sp_dps);
            for (int a = 0; a < 3; a++) pin.feedforward[a] = stab_.target_accel(a);   // the shaped target says how the rate is about to change: no waiting for the error
        } else {                                               // Acro
            for (int a = 0; a < 3; a++) pin.feedforward[a] = ss.feedforward[a];
        }
        was_angle_ = angle; was_stab_ = stab;

        // ---- 8. rate loop + mixer
        // TPA and the dynamic low-passes follow the throttle that is flown, whoever commands it. Anti-gravity and the throttle
        // boost answer the PILOT'S hand only: fed with Altitude Hold's output they would amplify barometer noise into the motors
        // (Betaflight computes them from the stick as well, and puts the autopilot's throttle in afterwards).
        pid_.update_throttle(thr, &gf_, pilot_throttle);
        for (int a = 0; a < 3; a++) { pin.setpoint[a] = sp_dps[a]; pin.gyro[a] = g_f_[a]; }
        pin.angle_mode = angle || stab;
        pin.i_enabled = !ground_idle;
        pid_.update(pin, o.u);
        for (int a = 0; a < 3; a++) { o.sp_dps[a] = sp_dps[a]; o.pid_p[a] = pid_.p_term[a]; o.pid_i[a] = pid_.i_term[a]; o.pid_d[a] = pid_.d_term[a]; o.pid_f[a] = pid_.f_term[a]; }
        mixer_.mix(in.limits, boost_.apply(thr, pilot_throttle), o.u[ROLL], o.u[PITCH], o.u[YAW], o.motor, ground_idle);
        if (alt_.get_state() == AhState::STOPPED && o.althold_active) for (auto& m : o.motor) m = 0;

        // ---- 9. runaway take-off prevention (safety.h): the loop fights the craft instead of steering it -> motors off
        RunawayTakeoff::Input ri;
        ri.armed = true; ri.motors_mixed = !ground_idle; ri.throttle = pilot_throttle ? cmd.throttle_cmd : thr;
        ri.stick = fmaxf(fabsf(cmd.roll), fmaxf(fabsf(cmd.pitch), fabsf(cmd.yaw)));
        for (int a = 0; a < 3; a++) { ri.pid_sum[a] = o.pid_p[a] + o.pid_i[a] + o.pid_d[a] + o.pid_f[a]; ri.gyro_dps[a] = g_f_[a]; }
        if (runaway_.update(ri, dt)) {
            arming_.force_disarm(AB_RUNAWAY_TAKEOFF); armed_ = false;
            o.armed = false; o.arm_blockers |= AB_RUNAWAY_TAKEOFF; for (auto& m : o.motor) m = 0;
        }
        // a rescue from the switch has brought the craft down: motors off (after a failsafe the failsafe itself does that)
        if (rescue_landed && !fo.rescue) { arming_.force_disarm(); armed_ = false; o.armed = false; for (auto& m : o.motor) m = 0; }
        return o;
    }

    const Ahrs& ahrs() const { return ahrs_; }
    const VerticalEstimator& vertical() const { return vert_; }
    const HorizontalEstimator& horizontal() const { return horiz_; }
    const AltHold& althold() const { return alt_; }
    const PidController& pid() const { return pid_; }
    const GyroFilter& gyro_filter() const { return gf_; }
    const SetpointGenerator& setpoints() const { return sp_gen_; }
    const AngleController& angle_ctl() const { return angle_; }
    const StabilizeController& stabilize() const { return stab_; }
    const Estimate& estimate() const { return est_; }
    bool armed() const { return armed_; }
    const RunawayTakeoff& runaway() const { return runaway_; }
    const TiltAid& tilt_aid() const { return tilt_aid_; }
    const PosHold& poshold() const { return ph_; }
    const Rescue& rescue() const { return rescue_; }
    Mixer& mixer() { return mixer_; }

    // Throttle stick 0..1 -> throttle with the middle of the stick at `hover` (two straight lines: 0 -> 0, 0.5 -> hover, 1 -> 1).
    static float hover_centred(float stick01, float hover) {
        const float s = clampf(stick01, 0.0f, 1.0f);
        return s <= 0.5f ? hover * (s * 2.0f) : hover + (1.0f - hover) * ((s - 0.5f) * 2.0f);
    }

private:
    void configure_control() {
        pid_.configure(cfg.pid, cfg.loop_hz); sp_gen_.configure(cfg.ff, cfg.rc_smoothing, cfg.loop_hz);
        angle_.configure(cfg.angle, cfg.loop_hz); stab_.configure(cfg.stab); boost_.configure(cfg.throttle_boost, cfg.throttle_boost_cutoff, cfg.loop_hz);
        have_rc_ = false; ctl_dirty_ = true;
    }
    // disarmed: the controllers forget everything, once
    WB_FAST void reset_control() {
        if (!ctl_dirty_) return;
        pid_.reset(); angle_.reset(); boost_.reset();
        raised_ = airborne_ = false; up_t_ = 0.0f; land_.reset(); was_angle_ = was_stab_ = false; ctl_dirty_ = false;
    }
    WB_FAST void estimator_step(Vec3 acc, float dt_est, uint32_t now) {
        float an, ae, aup; ahrs_.world_accel(acc, an, ae, aup);
        est_.az = aup;
        vert_.predict(aup, dt_est);
        const bool baro_fresh = have_baro_ && age_ms(now, baro_ms_) <= (uint32_t)cfg.baro_stale_ms;
        // near the ground with the motors running the air under the craft is pushed about: the barometer counts for less
        // (ArduPilot does the same while a take-off or touch-down is expected)
        ground_t_ = armed_ && airborne_ ? ground_t_ + dt_est : 0.0f;
        if (baro_new_) { vert_.update_baro(baro_alt_, baro_sigma_, armed_ && ground_t_ < cfg.ground_effect_s); baro_new_ = false; }
        if (range_new_) { vert_.update_range(range_h_, range_sigma_, baro_fresh); range_new_ = false; }
        const bool range_fresh = have_range_ && age_ms(now, range_ms_) <= (uint32_t)cfg.range_stale_ms;
        horiz_.predict(an, ae, dt_est);
        tilt_aid_.push_force(an / kG0, ae / kG0, -aup / kG0 - 1.0f, dt_est);
        // The GPS's north means something to the craft only with a heading that does not come from the gyro alone (the compass).
        // Without one: the GPS still shows where the craft is - unless optical flow is at work, whose velocity is measured in
        // the craft's own axes and would not fit together with it. The tilt is never corrected from a GPS without a heading.
        const bool flow_fresh = have_flow_ && age_ms(now, flow_ok_ms_) <= (uint32_t)cfg.flow_stale_ms;
        if (gps_new_) {
            // GPS altitude wanders by metres over a minute; a barometer does not. It is only a height for a craft that has
            // nothing better (ArduPilot's default height source is the barometer, Betaflight lets GPS in with tau 200 s).
            if (!baro_fresh && !range_fresh) vert_.update_gps(gps_.alt_m, gps_.sigma_alt);
            if (heading_ok_ || !flow_fresh) {
                horiz_.fuse_gps(gps_.north_m, gps_.east_m, gps_.vn, gps_.ve, gps_.sigma_pos, gps_.sigma_vel);
                if (heading_ok_ && gps_.sigma_vel <= cfg.aid_max_sigma_vel) tilt_aid_.on_velocity(gps_.vn, gps_.ve, cfg.horizontal.gps_delay_s);
                gps_xy_ms_ = now; have_gps_xy_ = true;
            }
            gps_new_ = false;
        }
        if (!flow_seen_ || age_ms(now, flow_ms_) > (uint32_t)cfg.flow_stale_ms) est_.flow_reject = FLOW_NO_SAMPLE;      // no sensor, or it went silent
        if (flow_new_) {
            flow_new_ = false;
            FlowVelocity fv; float gx = 0, gy = 0;
            if (!range_fresh) fv.reject = FLOW_HEIGHT;
            else if (!gyro_hist_.average(flow_.age_s, flow_.dt, gx, gy)) fv.reject = FLOW_NO_GYRO;
            else fv = flow_velocity(cfg.flow, flow_, gx, gy, range_h_, ahrs_.up_cos());
            est_.flow_reject = fv.reject; if (fv.valid) { est_.flow_forward = fv.forward; est_.flow_right = fv.right; }
            if (fv.valid) {
                const float yaw = ahrs_.euler_aero().yaw * kDeg2Rad, cy = cosf(yaw), sy = sinf(yaw);
                const float vn = fv.forward * cy - fv.right * sy, ve = fv.forward * sy + fv.right * cy, age = flow_.age_s + 0.5f * flow_.dt;
                horiz_.fuse_velocity(vn, ve, fv.sigma, age);
                tilt_aid_.on_velocity(vn, ve, age);
                flow_ok_ms_ = now; have_flow_ = true;
            }
        }
        const VerticalState vs = vert_.state();
        est_.z = vs.z; est_.vz = vs.v; est_.z_sigma = vert_.z_sigma();
        est_.alt_valid = (baro_fresh || range_fresh) && est_.z_sigma < 3.0f;
        est_.pos_n = horiz_.pos_n(); est_.pos_e = horiz_.pos_e(); est_.vel_n = horiz_.vel_n(); est_.vel_e = horiz_.vel_e();
        est_.pos_valid = have_gps_ && have_gps_xy_ && age_ms(now, gps_xy_ms_) <= (uint32_t)cfg.gps_stale_ms && horiz_.initialised();
        const bool flow_now = have_flow_ && age_ms(now, flow_ok_ms_) <= (uint32_t)cfg.flow_stale_ms;
        est_.vel_valid = (est_.pos_valid || flow_now) && horiz_.initialised();
        est_.flow_valid = flow_now;                             // (one sample that was not used - a dark patch - does not end it: flow_stale_ms without any does)
    }

    Ahrs ahrs_; AccelGainSchedule acc_gain_; VerticalEstimator vert_; HorizontalEstimator horiz_; Mixer mixer_; AltHold alt_; Failsafe fs_; Arming arming_; RunawayTakeoff runaway_;
    GyroFilter gf_; SetpointGenerator sp_gen_; PidController pid_; AngleController angle_; StabilizeController stab_; ThrottleBoost boost_;
    float g_f_[3] = {};
    EulerDeg att_; Estimate est_;
    Vec3 acc_sum_, gyro_sum_; int acc_n_ = 0, decim_ = 8; float est_dt_ = 0.0f, ground_t_ = 0.0f;
    TiltAid tilt_aid_; GyroHistory gyro_hist_; FlowSample flow_; bool flow_new_ = false, flow_seen_ = false, have_flow_ = false, have_gps_xy_ = false, heading_ok_ = false; uint32_t flow_ms_ = 0, flow_ok_ms_ = 0, gps_xy_ms_ = 0;
    PosHold ph_; PosControlNE rctl_; Rescue rescue_; RescueOutput rescue_out_; bool was_poshold_ = false, nav_lean_ = false, ph_unavail_ = false, rescue_ok_ = false; float nav_tgt_[2] = {0, 0};
    bool have_att_ = false, mag_seeded_ = false, armed_ = false, was_althold_ = false;
    LandDetector land_; bool airborne_ = false; float up_t_ = 0.0f;
    bool have_rc_ = false, raised_ = false, was_angle_ = false, was_stab_ = false, ctl_dirty_ = true; uint32_t rc_seq_ = 0;
    float last_thr_ = 0;
    float baro_alt_ = 0, baro_sigma_ = 0.3f, range_h_ = 0, range_sigma_ = 0.03f; bool baro_new_ = false, range_new_ = false, gps_new_ = false, have_baro_ = false, have_range_ = false, have_gps_ = false;
    uint32_t baro_ms_ = 0, range_ms_ = 0, gps_ms_ = 0, mag_ms_ = 0; GpsMeas gps_; Vec3 mag_; bool mag_ready_ = false;
};
} // namespace fc
