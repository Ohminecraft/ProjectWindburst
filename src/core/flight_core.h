#pragma once
#include "../config/fast_code.h"
#include "types.h"
#include "filters.h"
#include "ahrs.h"
#include "rc.h"
#include "gyro_filter.h"
#include "setpoint.h"
#include "attitude.h"
#include "pid.h"
#include "mixer.h"
#include "safety.h"
#include "tilt_aid.h"
#include "flow.h"
#include "rescue.h"
#include "nav/mission.h"
#include "acro_trainer.h"
#include "nav/nav_log.h"
#include "nav/alt_estimator.h"
#include "nav/alt_control.h"
#include "nav/pos_estimator.h"
#include "nav/pos_hold.h"

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
// estimate and tells the attitude estimate which way is down while the craft accelerates (tilt_aid.h); Position Hold (nav/pos_hold.h)
// and GPS Rescue (rescue.h) turn it into a lean for the Angle controller, the rescue also into a climb rate for Altitude Hold.
// Every sensor is optional: a missing / stale one is simply not fused, and arming / the modes ask Health + the estimator validity flags.
// The sensor pushes and update() must run in the same task (or be protected by the caller): they share plain data, no locks.
// ============================================================================
namespace fc {

enum AirmodeSetting : uint8_t { AM_OFF = 0, AM_AUTO = 1, AM_ALWAYS = 2 };
// How the GPS height is used for the height estimate (nav/alt_estimator.h, "GPS height"):
//   OFF      never
//   FALLBACK only when there is no barometer and no range finder (none fitted, or one went silent in flight): then it is the
//            height, and Altitude Hold / landing / GPS Rescue work on it
//   BLEND    as FALLBACK, and next to the barometer it slowly takes out the baro's weather drift (long flights)
enum GpsAltMode : uint8_t { GA_OFF = 0, GA_FALLBACK = 1, GA_BLEND = 2 };

struct Health {                    // straight from the drivers' valid() / usable() / healthy()
    bool imu_ok = false, gyro_calibrated = false, calibrating = false;
    bool batt_low = false, batt_critical = false;       // batt::Monitor: below the warning level (no arming) / below the minimum (failsafe landing)
    bool baro_ok = false, range_ok = false, gps_ok = false;
    bool mag_calibrated = false, mag_ok = false;
    bool flow_ok = false;                               // an optical-flow sensor is there and answers (not: it sees the ground)
    bool pin_conflict = false;                          // the pin map puts two functions on one GPIO (checked at boot)
    bool loop_slow = false;                             // the loop ran less than 80 % of its configured cycles, two seconds in a row
};

struct MotorFeedback { float rpm[kMaxMotors] = {}; bool rpm_valid = false; uint8_t valid_mask = 0; };   // valid_mask: bit per motor whose speed is known (bidirectional DShot or the ESC telemetry wire)   // MotorOutputManager::rpm_array() (logical motor order; 0 = that motor has no telemetry). rpm_valid: there is telemetry at all

struct GpsMeas {                   // metres / m/s relative to the home point, altitude relative to the home altitude
    bool valid = false;
    float north_m = 0, east_m = 0, alt_m = 0, vn = 0, ve = 0, vd = 0;
    float sigma_pos = 2.0f, sigma_vel = 0.3f, sigma_alt = 3.0f;
    bool vd_valid = false;         // the module measures vertical velocity (u-blox); NMEA has none, vd is 0 then
    uint8_t sats = 0;
    float rate_hz = 0;             // solutions per second the module sends (0 = not measured yet)
};

// What the EKF (ekf.h, run by the sensor thread) gives navigation: NED from home, heading in deg (aero, -180 .. 180)
struct EkfNav { bool valid = false, gps_aiding = false, heading_ok = false; float pn = 0, pe = 0, vn = 0, ve = 0, yaw_deg = 0; };

struct CoreConfig {
    float loop_hz = 4000.0f, est_hz = 500.0f;      // loop_hz: the gyro's rate (one FastInput per sample)
    int pid_denom = 1;                             // the control part (attitude, modes, PID, mixer) runs on every pid_denom-th gyro sample (Betaflight pid_process_denom)
    float pid_hz() const { return loop_hz / (float)(pid_denom < 1 ? 1 : pid_denom); }
    RcConfig rc; RateProfile rates;
    AcroTrainerConfig trainer;                       // the ACRO_TRAINER switch (acro_trainer.h)
    PidConfig pid; GyroFilterConfig gyro; FeedforwardConfig ff; RcSmoothingConfig rc_smoothing;
    AngleConfig angle; StabilizeConfig stab;
    uint8_t throttle_boost = 5, throttle_boost_cutoff = 15;      // Betaflight throttle_boost / throttle_boost_cutoff
    uint8_t airmode = AM_AUTO;                     // AirmodeSetting: stabilised with the throttle stick at the bottom?
    bool stick_arm = false;                          // arm / disarm with the sticks (safety.h StickArm); an ARM switch then only enables
    float airmode_threshold = 0.25f;                 // throttle (0..1) that counts as "in the air" at once (Betaflight airmode_start_throttle_percent)
    float airborne_after_s = 1.0f;                   // ... and so does any throttle above "low" held this long (a strong craft hovers below that threshold)
    MixerConfig mixer;
    nav::AltConfig alt;                            // Altitude Hold (nav/alt_control.h)
    FsConfig fs;
    RunawayConfig runaway;
    StallConfig stall;                             // a motor that stops: disarm (needs rpm telemetry)
    float max_arm_angle_deg = 25.0f;
    Ahrs::Config ahrs; nav::AltEstConfig alt_est; nav::PosEstConfig pos_est;      // alt_est.gps_delay_s follows pos_est.gps_delay_s
    bool use_mag = false; float declination_deg = 0.0f;
    float baro_stale_ms = 1000.0f, range_stale_ms = 250.0f, gps_stale_ms = 1500.0f, mag_stale_ms = 300.0f;
    uint8_t gps_alt = GA_FALLBACK;                   // GpsAltMode
    bool gps_vel_z = true;                           // fuse the GPS vertical velocity (when the module gives one)
    bool ekf_nav = false;                            // Position Hold / GPS Rescue take position, velocity, heading from the EKF (on_ekf) when it is aiding
    float gps_alt_max_sigma = 8.0f;                  // a GPS height less sure than this (vAcc, m) is not used
    float gps_vz_sigma_scale = 1.5f;                 // GPS vertical velocity sigma = the module's speed accuracy x this (vertical is the worse axis)
    float acc_clip_g = 15.5f;                        // an accelerometer sample this far out counts as clipped (the IMUs run at +-16 g)
    float ground_effect_s = 1.5f;                    // the barometer counts for less from arming until the craft has been in the air this long
    AccelGainSchedule::Config ahrs_ground;           // how much more the accelerometer counts while disarmed (standing still it really shows gravity)
    // ---- navigation
    TiltAid::Config tilt_aid; bool use_tilt_aid = true;      // tilt from the measured acceleration (GPS / flow velocity) while armed
    float aid_max_sigma_vel = 0.5f;                  // a GPS velocity less sure than this (m/s) is not used for it
    FlowConfig flow; float flow_stale_ms = 300.0f;
    nav::PosCtlConfig pos; nav::PosHoldConfig poshold; RescueConfig rescue;      // nav/pos_control.h, nav/pos_hold.h, rescue.h; poshold.angle_max_deg is the lean limit of both
    uint32_t nav_request_timeout_ms = 500;          // a NavRequest older than this is not flown (nav/nav_math.h)
    float ph_min_gps_hz = 5.0f;     // Position Hold on GPS alone wants at least this solution rate (0 = no check); see flight_core.h 6b
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
    bool gps_alt_used = false, gps_vz_used = false;     // the GPS height / vertical velocity went into the height estimate within the last 2 s
};

struct CoreOutput {
    float motor[kMaxMotors] = {};  // absolute 0..1 (0 when disarmed)
    int motor_count = 4;
    bool armed = false;
    uint32_t arm_blockers = 0;
    FsStage failsafe = FsStage::NONE;
    uint8_t fs_cause = 0;          // FsCause bits while the failsafe is active (BATTERY_LOW also as a warning only)
    uint8_t fs_action = 0;         // FsAction: what the failsafe flies (WARN: only warns)
    uint32_t modes = 0;            // active Mode bits as the pilot selected them
    bool althold_active = false, althold_unavailable = false;
    AhState ah_state = AhState::STOPPED;
    float throttle = 0;
    float hover_throttle = 0;      // the throttle (0..1) at which the craft hovers, as measured in flight (0 before the craft was armed)
    float sp_dps[3] = {}, gyro_f_dps[3] = {}, u[3] = {};               // debug / blackbox; u = PID sum / 1000 (the mixer's input)
    float pid_p[3] = {}, pid_i[3] = {}, pid_d[3] = {}, pid_f[3] = {};  // the terms that make up u[], in PID sum units (x 1000)
    bool airmode_active = false;   // stabilised (full authority, I-terms) although the throttle stick is at the bottom
    bool trainer_active = false;   // the acro trainer watches roll / pitch (ACRO_TRAINER switch, Acro)
    bool airborne = false;         // the craft is taken to be in the air (see FlightCore::update, step 6)
    bool ground_idle = false;      // every motor at idle, nothing mixed: armed and standing on the ground
    uint8_t stall_motor = 0;       // the motor (1..8) the stall guard last saw stop (0 = none since boot): safety.h StallGuard
    bool est_stepped = false;      // this cycle also ran the estimators (every pid_hz / est_hz control cycles): it takes longer than the others
    bool pid_stepped = true;       // this gyro sample was also a control cycle (PID + mixer ran, motor[] is new); false: only the gyro filters ran, the rest is the last control cycle's
    float stick[4] = {};           // pilot command: roll, pitch, yaw (-1..1) and the throttle stick (0..1), before modes / failsafe
    bool throttle_low = true;      // throttle stick at the bottom
    uint32_t arm_refused = 0;      // arm attempts rejected by a blocker since boot
    // navigation
    bool poshold_slow_gps = false;  // ... unavailable because the GPS sends too few solutions a second (CoreConfig::ph_min_gps_hz)
    bool poshold_active = false, poshold_unavailable = false;         // Position Hold flies / is switched on but has no velocity over the ground to work with
    uint8_t ph_state[2] = {0, 0};  // PhAxis of roll, pitch
    bool rescue_active = false, rescue_available = false; uint8_t rescue_phase = 0, rescue_fail = 0;      // RescuePhase, RescueFail
    uint8_t rescue_mode = 0; float rescue_left = 0;   // RescueKind; the way still to go home (m)
    bool nav_lean = false; float nav_roll = 0, nav_pitch = 0;         // an autopilot gives the lean (deg, pilot convention)
    bool nav_request = false;      // an outside autopilot flies (FlightCore::set_nav_request)
    bool fence = false;            // the fence acts: Position Hold holds on its circle, or the craft is within a metre of the height ceiling
    float nav_target_n = 0, nav_target_e = 0, nav_wind_n = 0, nav_wind_e = 0, nav_home_dist = 0;
    // the mission (nav/mission.h)
    uint8_t mission_state = 0, mission_index = 0, mission_count = 0, mission_wait = 0;      // MissionState; its command; commands; s left of a wait
    float mission_dist = 0, mission_left = 0;      // m to the present command's point; m the mission still flies
    bool mission_warn = false;                     // the motors are about to be armed by the mission (beeper / LED)
    bool mission_holds_home = false;               // a mid-mission landing: home and the height origin must not move at the next arming
    uint16_t mission_servo_us[4] = {0, 0, 0, 0};   // what DO_SET_SERVO last set (0 = nothing yet): servos with the source MISSION
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
        alt_est_.cfg = cfg.alt_est; alt_est_.cfg.gps_delay_s = cfg.pos_est.gps_delay_s; alt_est_.reset(0.0f);
        pos_est_.cfg = cfg.pos_est; pos_est_.reset(); tilt_aid_.cfg = cfg.tilt_aid; tilt_aid_.reset(); gyro_hist_.configure(1.0f / cfg.est_hz);
        gyro_sum_ = Vec3(); flow_new_ = false; have_flow_ = false; have_gps_xy_ = false; heading_ok_ = false;
        ph_.cfg = cfg.poshold; ph_.ctl.cfg = cfg.pos; rctl_.cfg = cfg.pos; rctl_.cfg.angle_max_deg = cfg.poshold.angle_max_deg; rescue_.cfg = cfg.rescue;
        was_poshold_ = false; nav_lean_ = false; ph_unavail_ = false; ph_slow_gps_ = false; rescue_ok_ = false; rescue_out_ = RescueOutput(); nav_tgt_[0] = nav_tgt_[1] = 0;
        mission_.cfg = cfg.rescue; mission_.stop(); mission_out_ = MissionOutput(); mission_req_ = mission_app_ = mission_sw_ = mission_fly_ = was_mission_h_ = false; for (auto& u : mission_servo_) u = 0;
        alt_.cfg = cfg.alt; alt_.init();
        fs_.cfg = cfg.fs; runaway_.cfg = cfg.runaway; stall_.cfg = cfg.stall; stall_.reset();
        decim_ = (int)(cfg.pid_hz() / cfg.est_hz + 0.5f); if (decim_ < 1) decim_ = 1;
        denom_ = cfg.pid_denom < 1 ? 1 : cfg.pid_denom; pid_n_ = 0; win_dt_ = 0.0f; win_g_ = Vec3(); win_a_ = Vec3(); last_ = CoreOutput();
        acc_n_ = 0; acc_sum_ = Vec3(); est_dt_ = 0.0f; ground_t_ = 0.0f; have_att_ = false; mag_seeded_ = false; was_althold_ = false; last_thr_ = 0;
        have_rc_ = false; rc_seq_ = 0; raised_ = airborne_ = false; up_t_ = 0.0f; land_.reset(); was_angle_ = was_stab_ = false; ctl_dirty_ = true;
        baro_new_ = range_new_ = gps_new_ = false; baro_ms_ = range_ms_ = gps_ms_ = mag_ms_ = 0; have_baro_ = have_range_ = have_gps_ = false; mag_ready_ = false;
    }

    // Live re-tune from the app while DISARMED (PID / feedforward / angle settings changed in cfg): pushes them into the controllers
    // without touching the attitude or the estimators. Rates and the RC map are read from cfg / rc directly every cycle.
    void apply_tuning() { configure_control(); }
    // Live: what the throttle stick means in Altitude Hold (the hover estimate and the state are kept)
    void apply_althold() { alt_.cfg = cfg.alt; alt_.apply_config(); }
    // The same for the gyro filters (they restart from the current gyro value). Disarmed only.
    void apply_filters() { gf_.configure(cfg.gyro, cfg.loop_hz, mixer_.motor_count()); configure_control(); }

    // The gyro is gone (IMU stopped answering): nothing can be controlled any more. Disarms and requires the arm switch to be cycled.
    void emergency_disarm() { arming_.force_disarm(); armed_ = false; reset_control(); }

    // ---------------- sensor pushes (call when a driver reports a NEW sample)
    void on_baro(float alt_m, float sigma_m, uint32_t now_ms) { baro_alt_ = alt_m; baro_sigma_ = sigma_m; baro_ms_ = now_ms; baro_new_ = true; have_baro_ = true; }
    void on_range(float height_m, float sigma_m, uint32_t now_ms) { range_h_ = height_m; range_sigma_ = sigma_m; range_ms_ = now_ms; range_new_ = true; have_range_ = true; }
    void on_gps(const GpsMeas& g, uint32_t now_ms) { gps_ = g; gps_ms_ = now_ms; gps_new_ = g.valid; have_gps_ = g.valid; }
    void on_mag(const Vec3& board_gauss, bool ready, uint32_t now_ms) { mag_ = board_gauss; mag_ready_ = ready; mag_ms_ = now_ms; }
    void on_ekf(const EkfNav& e, uint32_t now_ms) { ekf_ = e; ekf_ms_ = now_ms; }
    // P7 hook: what an outside autopilot wants flown (nav/nav_math.h NavRequest). Call it again at least every nav_request_timeout_ms.
    void set_nav_request(const nav::NavRequest& r, uint32_t now_ms) { req_ = r; req_ms_ = now_ms; have_req_ = true; }
    void clear_nav_request() { have_req_ = false; }
    bool ekf_used() const { return ekf_in_use_; }      // the last navigation step took the EKF's numbers
    void on_flow(const FlowSample& f, uint32_t now_ms) { flow_ = f; flow_ms_ = now_ms; flow_new_ = true; flow_seen_ = true; }
    // Re-zero the horizontal position (e.g. when the home point is set at arming).
    void reset_position() { pos_est_.reset(); tilt_aid_.restart(); alt_est_.restart_gps_ref(); have_gps_ = false; have_gps_xy_ = false; }
    // Live re-tune of the navigation settings from the app (disarmed)
    void apply_nav() { alt_.cfg.ceiling_m = cfg.alt.ceiling_m; ph_.cfg = cfg.poshold; ph_.ctl.cfg = cfg.pos; rctl_.cfg = cfg.pos; rctl_.cfg.angle_max_deg = cfg.poshold.angle_max_deg; rescue_.cfg = cfg.rescue; mission_.cfg = cfg.rescue; tilt_aid_.cfg = cfg.tilt_aid; fs_.cfg = cfg.fs; }
    // The mission (nav/mission.h): the plan, in metres from home (flown from the next start on), and the app's start / stop.
    // A MISSION switch starts it too; either way it starts once per request (a refused or finished mission waits for the next).
    void set_mission(const MissionPlan& p) { mission_plan_ = p; }
    void mission_command(bool start) { mission_app_ = start; }
    const MissionRunner& mission() const { return mission_; }

    // One gyro sample. Every sample goes through the gyro filters; every pid_denom-th one is also a control cycle (control()).
    WB_FAST CoreOutput update(const FastInput& in) {
        // ---- 1. gyro: FRD deg/s -> user rates (pitch is negated), filtered for the rate loop - at the gyro's rate, so that the
        // filters see every sample (nothing above the control rate aliases into it: Betaflight filters at the gyro rate as well)
        g_f_[0] = in.gyro_dps.x; g_f_[1] = -in.gyro_dps.y; g_f_[2] = in.gyro_dps.z;
        gf_.apply(g_f_, in.motors.rpm_valid ? in.motors.rpm : nullptr);
        // the estimators' sums take every sample, each for the time it stands for
        acc_sum_.x += in.acc_g.x * in.dt; acc_sum_.y += in.acc_g.y * in.dt; acc_sum_.z += in.acc_g.z * in.dt; est_dt_ += in.dt;
        if ((fabsf(in.acc_g.x) > cfg.acc_clip_g || fabsf(in.acc_g.y) > cfg.acc_clip_g || fabsf(in.acc_g.z) > cfg.acc_clip_g) && clip_n_ < 255) clip_n_++;
        gyro_sum_.x += in.gyro_dps.x * in.dt; gyro_sum_.y += in.gyro_dps.y * in.dt;
        if (denom_ <= 1) { CoreOutput o = control(in, in.gyro_dps, in.acc_g, in.dt); return o; }
        // the attitude of a control cycle is integrated over the mean rate of its window (the rate times the time that really passed)
        win_dt_ += in.dt;
        win_g_.x += in.gyro_dps.x * in.dt; win_g_.y += in.gyro_dps.y * in.dt; win_g_.z += in.gyro_dps.z * in.dt;
        win_a_.x += in.acc_g.x * in.dt; win_a_.y += in.acc_g.y * in.dt; win_a_.z += in.acc_g.z * in.dt;
        if (++pid_n_ < denom_ || !(win_dt_ > 0.0f)) {          // a gyro-only sample: the last control cycle's output with the new filtered gyro
            CoreOutput o = last_; o.pid_stepped = false; o.est_stepped = false;
            for (int a = 0; a < 3; a++) o.gyro_f_dps[a] = g_f_[a];
            return o;
        }
        const float w = win_dt_, iw = 1.0f / w;
        const Vec3 g = {win_g_.x * iw, win_g_.y * iw, win_g_.z * iw}, acc = {win_a_.x * iw, win_a_.y * iw, win_a_.z * iw};
        pid_n_ = 0; win_dt_ = 0.0f; win_g_ = Vec3(); win_a_ = Vec3();
        last_ = control(in, g, acc, w);
        return last_;
    }

private:
    // The control cycle: gyro_dps / acc_g = the (mean) rate and acceleration of its window, dt = the window's length.
    WB_FAST CoreOutput control(const FastInput& in, const Vec3& gyro_dps, const Vec3& acc_g, float dt) {
        CoreOutput o; const uint32_t now = in.now_ms;
        o.motor_count = mixer_.motor_count();
        o.stall_motor = (uint8_t)stall_.last();
        for (int a = 0; a < 3; a++) o.gyro_f_dps[a] = g_f_[a];

        // ---- 2. attitude (unfiltered gyro) with the compass when it is trustworthy
        if (!have_att_) {                                      // initial attitude from gravity - only from a plausible reading (never from a zero / 0 g vector)
            const float an = sqrtf(acc_g.x * acc_g.x + acc_g.y * acc_g.y + acc_g.z * acc_g.z);
            if (an > 0.5f && an < 1.5f) { ahrs_.reset_to_accel(acc_g); have_att_ = true; }
        }
        const bool mag_fresh = cfg.use_mag && mag_ready_ && age_ms(now, mag_ms_) <= (uint32_t)cfg.mag_stale_ms;
        if (mag_fresh && !mag_seeded_ && !armed_) mag_seeded_ = ahrs_.seed_yaw_from_mag(mag_, cfg.declination_deg);
        const float turning = maxf(fabsf(gyro_dps.x), maxf(fabsf(gyro_dps.y), fabsf(gyro_dps.z)));
        const float acc_gain = acc_gain_.update(armed_, turning, ahrs_.acc_weight() > 0.0f, dt);
        ahrs_.update({gyro_dps.x * kDeg2Rad, gyro_dps.y * kDeg2Rad, gyro_dps.z * kDeg2Rad}, acc_g, dt, mag_fresh ? &mag_ : nullptr, cfg.declination_deg, acc_gain,
                     !armed_ && turning < cfg.ahrs_ground.gyro_limit_dps);
        att_ = ahrs_.euler_user();

        // ---- 3. estimators at est_hz on the averaged accelerometer
        // The cycles are not all the same length: each reading counts for the time it stands for, and the step is made over the
        // time that really passed (not "the last cycle x n").
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
        // (a height for Altitude Hold: the barometer, or - when the GPS height may stand in - a GPS fix; home and with it the GPS height come at arming)
        ai.baro_ok = in.health.baro_ok || (cfg.gps_alt != GA_OFF && in.health.gps_ok); ai.gps_fix = in.health.gps_ok;
        // Position Hold needs a velocity over the ground: optical flow gives one by itself, a GPS only together with a compass
        const bool compass = cfg.use_mag && in.health.mag_calibrated;
        ai.pos_source = in.health.flow_ok || (in.health.gps_ok && compass);
        ai.rescue_configured = cfg.rescue_configured; ai.rescue_allow_no_fix = cfg.rescue_allow_no_fix; ai.rescue_no_compass = cfg.rescue_configured && !cfg.use_mag;
        ai.althold_switch = (cmd.modes >> M_ALTHOLD) & 1u; ai.poshold_switch = (cmd.modes >> M_POSHOLD) & 1u;
        ai.calibrating = in.health.calibrating; ai.batt_low = in.health.batt_low;
        ai.use_mag = cfg.use_mag; ai.mag_calibrated = in.health.mag_calibrated; ai.pin_conflict = in.health.pin_conflict; ai.loop_slow = in.health.loop_slow;
        const bool was_armed = armed_;
        bool arm_sw = (cmd.modes >> M_ARM) & 1u, keep_reason = false;
        if (cfg.stick_arm) {
            const bool g = StickArm::gesture(cmd.roll, cmd.pitch, cmd.yaw, cmd.throttle_low, stick_arm_.cfg.threshold);
            arm_sw = stick_arm_.update(g, in.rc_link, !rc.modes.has(M_ARM) || arm_sw, armed_, !airborne_, dt);
            keep_reason = !g;
        }
        armed_ = arming_.update(arm_sw, in.rc_link, ai, keep_reason);
        if (in.rc_link) mission_sw_ = (cmd.modes >> M_MISSION) & 1u;          // (lost link: the switch as it was)
        // the mission turns the motors off at a delivery stop and on again after the wait (Arming::mission_disarm / mission_arm)
        if (mission_out_.want_disarm && armed_ && mission_out_.active) { arming_.mission_disarm(); armed_ = false; }
        else if (mission_out_.want_arm && !armed_ && mission_.running()) armed_ = arming_.mission_arm(arm_sw, in.rc_link, ai);
        if (was_armed && !armed_ && arming_.last_disarm_reason() != 3) mission_app_ = false;     // (the pilot or a failsafe: the app's start is used up)
        if (armed_ && !was_armed) runaway_.on_arm();
        o.arm_blockers = arming_.blockers(); o.arm_refused = arming_.refused();
        o.stick[0] = cmd.roll; o.stick[1] = cmd.pitch; o.stick[2] = cmd.yaw; o.stick[3] = cmd.throttle_raw; o.throttle_low = cmd.throttle_low;

        FsInput fi;
        fi.rx_ok = in.rc_link || (mission_fly_ && cfg.fs.mission_rc_continue); fi.batt_critical = in.health.batt_critical; fi.batt_low = in.health.batt_low; fi.alt_valid = est_.alt_valid;
        fi.landed = alt_.state() == AhState::LANDED_IDLE; fi.airborne = airborne_; fi.armed = armed_; fi.switch_on = (cmd.modes >> M_FAILSAFE_SW) & 1u; fi.vz = est_.vz;
        fi.rescue_active = rescue_out_.active; fi.stick_move = in.rc_link ? maxf(fabsf(cmd.roll), maxf(fabsf(cmd.pitch), fabsf(cmd.yaw))) : 0.0f;
        const FsOutput fo = fs_.update(fi, dt);
        if (fo.disarm) { arming_.force_disarm(); armed_ = false; }
        // a failsafe that flies something ends the mission (ArduPilot: the failsafe changes the mode, AUTO stops)
        if ((fo.rescue || fo.alt_descent || fo.hold || fo.throttle_fallback || fo.disarm) && mission_.running()) { mission_.stop(); mission_app_ = false; mission_out_ = MissionOutput(); mission_fly_ = false; }
        o.failsafe = fo.stage; o.fs_cause = fo.cause; o.fs_action = fo.action; o.armed = armed_;

        est_.att = att_; est_.tilt_deg = ahrs_.tilt_deg(); est_.yaw_from_mag = mag_fresh && mag_seeded_; heading_ok_ = est_.yaw_from_mag;
        o.est = est_; o.ah_state = alt_.state();

        // ---- 4b. the stick path: a new receiver frame -> rates + feedforward; every cycle -> RC smoothing
        if (!have_rc_ || in.rc_seq != rc_seq_) {
            have_rc_ = true; rc_seq_ = in.rc_seq;
            const float st[3] = {cmd.roll, cmd.pitch, cmd.yaw};
            sp_gen_.on_frame(st, cmd.throttle, cfg.rates);
        }
        const bool neutral = fo.neutral_sticks || (mission_fly_ && !in.rc_link);      // (the mission flies on without a link: the last sticks are not the pilot's)
        if (neutral) sp_gen_.neutral();                        // failsafe: the pilot's sticks are not to be believed
        const StickSetpoint ss = sp_gen_.update();

        if (!armed_) {                                         // disarmed: nothing accumulates, motors off
            reset_control();
            pid_.update_throttle(0.0f, &gf_);                  // the dynamic low-passes rest at their zero-throttle cutoff
            nav::AltInput idle; idle.pos_d = alt_est_.pos_d(); idle.reset_count = alt_est_.reset_count(); alt_.update(idle, dt); was_althold_ = false; last_thr_ = 0;
            o.ah_state = alt_.state();
            if (dt_est > 0.0f) { RescueInput ri; rescue_out_ = rescue_.update(ri, dt_est); mission_step(in, arm_sw, dt_est); }       // (forgets the flight: its highest point, a failure)
            was_poshold_ = false; nav_lean_ = false; ph_unavail_ = false; ph_slow_gps_ = false; rescue_ok_ = false; was_req_ = false; req_climb_on_ = false; mission_fly_ = was_mission_h_ = false;
            mission_fill(o);
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
        const bool mission = mission_fly_;                     // the mission flies (its climb rate, its take-off)
        bool rescue_landed = false;
        if (rescue || mission || fo.alt_descent || fo.hold || (althold_sw && alt_ok && !fo.throttle_fallback)) {
            // Engaged while flying: carry on from the present throttle. "Flying" is what the craft has been doing (airborne_), not
            // where the stick happens to be: a craft that hovers at 13 % of the stick must not be taken for a parked one.
            if (!was_althold_ && airborne_) alt_.begin_flight(alt_est_.pos_d(), alt_est_.vel_d(), -est_.az, last_thr_, ahrs_.up_cos());
            nav::AltInput ah; ah.armed = true; ah.stick01 = cmd.throttle_raw; ah.pos_d = alt_est_.pos_d(); ah.vel_d = alt_est_.vel_d(); ah.acc_d = -est_.az; ah.up_cos = ahrs_.up_cos();
            ah.failsafe_descent = fo.alt_descent && !rescue; ah.pilot_absent = neutral || rescue || mission; ah.ext_climb = rescue || mission || req_climb_on_;
            ah.climb_mps = rescue ? rescue_out_.climb_mps : mission ? mission_out_.climb_mps : req_climb_; ah.auto_takeoff = mission && mission_out_.takeoff;
            ah.reset_count = alt_est_.reset_count(); ah.reset_step_d = alt_est_.reset_step_d();
            thr = alt_.update(ah, dt); o.althold_active = true; was_althold_ = true;
            rescue_landed = rescue && alt_.state() == AhState::LANDED_IDLE;
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
        o.ah_state = alt_.state();
        last_thr_ = thr; o.throttle = thr;
        // The hover throttle is measured in every mode (Altitude Hold and the failsafe descent start from it) - except while the
        // failsafe flies the open-loop descent: there the throttle is DERIVED from the hover value, and a craft that sinks with
        // air drag carrying part of its weight would "learn" a lower and lower hover, sinking faster and faster.
        if (!fo.throttle_fallback) alt_.learn(thr, ahrs_.up_cos(), -est_.az, dt);
        o.hover_throttle = alt_.hover_throttle();

        // ---- 6. in the air, or standing on the ground?
        // airborne_: the throttle has been up (25 % at once, or anything above "low" for a second), or Altitude Hold has lifted the
        // craft off. It ends when the craft is found standing still with the throttle down (LandDetector), when Altitude Hold has
        // landed it, and with the disarm. raised_ is Betaflight's latch: set with it, cleared only by the disarm.
        const float turning_f = maxf(fabsf(g_f_[0]), maxf(fabsf(g_f_[1]), fabsf(g_f_[2])));
        if (pilot_throttle) {
            up_t_ = cmd.throttle_low ? 0.0f : up_t_ + dt;
            if (cmd.throttle_cmd >= cfg.airmode_threshold || up_t_ >= cfg.airborne_after_s) airborne_ = raised_ = true;
            LandDetector::Input li; li.throttle_down = cmd.throttle_low; li.accel_1g = ahrs_.acc_weight() > 0.0f; li.gyro_dps = turning_f; li.alt_valid = est_.alt_valid; li.vz = est_.vz;
            if (land_.update(li, dt)) airborne_ = false;
        } else {
            up_t_ = 0.0f; land_.reset();
            if (o.althold_active) { if (alt_.lifted()) airborne_ = raised_ = true; else if (alt_.state() != AhState::TAKEOFF) airborne_ = false; }
        }
        o.airborne = airborne_;
        // Airmode = the craft stays stabilised (full authority, I-terms) with the throttle stick at the bottom. AUTO: while it is in
        // the air. ALWAYS: from the first time the throttle was up until the disarm, as Betaflight does - on the ground or on the
        // bench the I-terms then wind up against a craft that cannot answer, and the motors run up.
        // An AIRMODE switch, when one is assigned, turns it on and off; on = the kind general.airmode sets (AUTO if that is OFF).
        uint8_t am = cfg.airmode;
        if (rc.modes.has(M_AIRMODE)) am = !((cmd.modes >> M_AIRMODE) & 1u) ? (uint8_t)AM_OFF : cfg.airmode == AM_OFF ? (uint8_t)AM_AUTO : cfg.airmode;
        const bool airmode = pilot_throttle && (am == AM_ALWAYS ? raised_ : am == AM_AUTO ? airborne_ : false);
        o.airmode_active = airmode && cmd.throttle_low;
        // ground_idle: every motor at idle, nothing mixed, nothing integrates (ArduCopter's GROUND_IDLE)
        const bool ground_idle = pilot_throttle ? (cmd.throttle_low && !airmode)
                               : (o.althold_active && alt_.state() != AhState::TAKEOFF && alt_.state() != AhState::FLYING);
        o.ground_idle = ground_idle;

        // ---- 6b. navigation, at the estimators' rate: GPS Rescue (a switch, or the failsafe's procedure) comes before Position Hold
        if (dt_est > 0.0f) {
            // where the craft is, how fast it goes, where its nose points: the usual estimators, or (nav.ekf = NAV) the EKF while it
            // is aiding with GPS. Optical flow keeps the usual path (the EKF does not fuse flow).
            float yaw = ahrs_.euler_aero().yaw, npn = est_.pos_n, npe = est_.pos_e, nvn = est_.vel_n, nve = est_.vel_e;
            bool heading_valid = est_.yaw_from_mag, pos_valid = est_.pos_valid, vel_valid = est_.vel_valid;
            ekf_in_use_ = cfg.ekf_nav && ekf_.valid && ekf_.gps_aiding && !est_.flow_valid && age_ms(now, ekf_ms_) <= 200u;
            if (ekf_in_use_) { yaw = ekf_.yaw_deg; npn = ekf_.pn; npe = ekf_.pe; nvn = ekf_.vn; nve = ekf_.ve; heading_valid = ekf_.heading_ok; pos_valid = vel_valid = have_gps_; }
            // the mission first: it may ask for the rescue (its RTL, or a landing where it is when it lost the position)
            mission_step(in, arm_sw, dt_est, npn, npe, nvn, nve, yaw, pos_valid);
            const bool pilot_rescue = ((cmd.modes >> M_RESCUE) & 1u) || fo.rescue;
            RescueInput ri; ri.armed = true; ri.airborne = airborne_; ri.request = airborne_ && (pilot_rescue || mission_out_.want_rtl || mission_out_.want_land);
            ri.kind = pilot_rescue ? (uint8_t)255 : mission_out_.want_land ? (uint8_t)RESCUE_MODE_LAND : (uint8_t)RESCUE_MODE_RTL;
            ri.pos_n = npn; ri.pos_e = npe; ri.vel_n = nvn; ri.vel_e = nve; ri.z = est_.z; ri.yaw_deg = yaw;
            ri.pos_valid = pos_valid; ri.alt_valid = est_.alt_valid; ri.heading_valid = heading_valid; ri.sats = gps_.sats;
            const bool was_rescue = rescue_out_.active && rescue_out_.horizontal;
            rescue_out_ = rescue_.update(ri, dt_est);
            rescue_ok_ = rescue_.available(ri);
            mission_fly_ = mission_out_.active && !rescue_out_.active;      // (a rescue it asked for flies itself)
            // Position Hold needs a velocity over the ground; one from GPS only means something to the craft with a compass heading,
            // and only if it comes often enough: a 1 Hz GPS (an NMEA module left at its default) moves the position by a metre and the
            // velocity by 1 - 2 m/s at every solution, and the hold answers each step with a lean (0.8.7 log: +-15 deg, 15 m of drift).
            // Betaflight's Position Hold wants a GPS rate too (gps_update_rate_hz, 10 by default); ArduPilot's EKF wants 5 Hz or more.
            const bool gps_rate_ok = !(cfg.ph_min_gps_hz > 0.0f) || gps_.rate_hz >= cfg.ph_min_gps_hz;
            const bool ph_ok = vel_valid && (est_.flow_valid || (heading_valid && gps_rate_ok));
            ph_slow_gps_ = ((cmd.modes >> M_POSHOLD) & 1u) && !est_.flow_valid && heading_valid && vel_valid && !gps_rate_ok;
            // (the failsafe's HOLD: Position Hold with the sticks neutral, if there is a position to hold - else Altitude Hold alone)
            const bool ph_on = ((((cmd.modes >> M_POSHOLD) & 1u) && !neutral) || (fo.hold && ph_ok)) && !rescue_out_.active && !mission_fly_;
            ph_unavail_ = ph_on && !ph_ok;
            // an outside autopilot's request (P7) is flown while Position Hold could fly, it is fresh, and the sticks are centred
            const bool req_on = ph_on && ph_ok && have_req_ && age_ms(now, req_ms_) <= cfg.nav_request_timeout_ms && !neutral && cmd.roll == 0.0f && cmd.pitch == 0.0f;
            if (rescue_out_.active && rescue_out_.horizontal) {
                // the rescue's target point, flown with its velocity as feed-forward (0.36: a path, PX4 / ArduPilot RTL)
                if (!was_rescue) rctl_.init(npn, npe, nvn, nve, angle_.target_deg(ROLL), angle_.target_deg(PITCH), yaw);
                rctl_.set_target(rescue_out_.tgt_n, rescue_out_.tgt_e);
                rctl_.update(npn, npe, nvn, nve, dt_est, true, rescue_out_.vel_n, rescue_out_.vel_e);
                rctl_.lean(yaw, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                nav_lean_ = true; was_poshold_ = false;
            } else if (mission_fly_ && mission_out_.horizontal) {
                // the mission's target point, flown with its velocity as feed-forward - the same controller as the rescue's
                if (!was_mission_h_) rctl_.init(npn, npe, nvn, nve, angle_.target_deg(ROLL), angle_.target_deg(PITCH), yaw);
                rctl_.set_target(mission_out_.tgt_n, mission_out_.tgt_e);
                rctl_.update(npn, npe, nvn, nve, dt_est, true, mission_out_.vel_n, mission_out_.vel_e);
                rctl_.lean(yaw, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                nav_lean_ = true; was_poshold_ = false;
            } else if (req_on) {
                // an outside autopilot flies (P7): to a position at no more than its speed (slowing down as it gets there), or a velocity
                float vn = req_.vel_n_mps, ve = req_.vel_e_mps; const float dn = req_.north_m - npn, de = req_.east_m - npe, d = sqrtf(dn * dn + de * de);
                const bool arrived = req_.position && d < 1.0f;
                if (req_.position) { const float v = d > 1e-3f ? minf(req_.speed_mps, nav::sqrt_controller(d, cfg.pos.pos_p, cfg.pos.accel_corr_mps2)) : 0.0f; vn = d > 1e-3f ? dn / d * v : 0.0f; ve = d > 1e-3f ? de / d * v : 0.0f; }
                if (!was_req_) rctl_.init(npn, npe, nvn, nve, angle_.target_deg(ROLL), angle_.target_deg(PITCH), yaw);
                if (arrived) rctl_.set_target(req_.north_m, req_.east_m);
                rctl_.update(npn, npe, nvn, nve, dt_est, arrived, arrived ? 0.0f : vn, arrived ? 0.0f : ve);
                rctl_.lean(yaw, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                req_climb_on_ = req_.height && est_.alt_valid;
                req_climb_ = clampf(nav::sqrt_controller(req_.height_m - est_.z, cfg.alt.pos_p, cfg.alt.accel_max_mps2), -cfg.alt.descend_max_mps, cfg.alt.climb_max_mps);
                nav_lean_ = true; was_poshold_ = false; was_req_ = true;
            } else if (ph_on && ph_ok) {
                nav::PosHoldInput pi; pi.airborne = airborne_; pi.reset_count = pos_est_.reset_count(); pi.reset_step_n = pos_est_.reset_step_n(); pi.reset_step_e = pos_est_.reset_step_e(); pi.yaw_deg = yaw; pi.pos_n = npn; pi.pos_e = npe; pi.vel_n = nvn; pi.vel_e = nve;
                pi.pilot_roll_deg = cfg.angle.limit_deg * ss.rate[ROLL] / maxf(cfg.rates.max_rate_dps[ROLL], 1.0f); pi.pilot_pitch_deg = cfg.angle.limit_deg * ss.rate[PITCH] / maxf(cfg.rates.max_rate_dps[PITCH], 1.0f);
                pi.roll_centred = neutral || cmd.roll == 0.0f; pi.pitch_centred = neutral || cmd.pitch == 0.0f;      // (the failsafe's hold: the sticks are not the pilot's)
                if (!was_poshold_) ph_.init(pi);
                ph_.update(pi, dt_est, nav_tgt_[ROLL], nav_tgt_[PITCH]);
                nav_lean_ = true; was_poshold_ = true;
            } else { nav_lean_ = false; was_poshold_ = false; }
            if (!req_on || rescue_out_.active) { was_req_ = false; req_climb_on_ = false; }
            was_mission_h_ = mission_fly_ && mission_out_.horizontal && !rescue_out_.active;
        }
        o.rescue_active = rescue_out_.active; o.rescue_available = rescue_ok_; o.rescue_phase = (uint8_t)rescue_.phase(); o.rescue_fail = (uint8_t)rescue_.failure();
        o.rescue_mode = rescue_out_.active ? rescue_.kind() : cfg.rescue.mode; o.rescue_left = rescue_.remaining();
        mission_fill(o);
        o.poshold_active = was_poshold_; o.poshold_unavailable = ph_unavail_; o.poshold_slow_gps = ph_slow_gps_; o.ph_state[0] = (uint8_t)ph_.state(0); o.ph_state[1] = (uint8_t)ph_.state(1);
        o.nav_lean = nav_lean_; o.nav_roll = nav_tgt_[ROLL]; o.nav_pitch = nav_tgt_[PITCH];
        o.nav_request = was_req_;
        o.fence = (was_poshold_ && ph_.fence_breached()) || (o.althold_active && cfg.alt.ceiling_m > 0.0f && est_.z > cfg.alt.ceiling_m - 1.0f);      // (it stops half a metre under)
        navlog_.alt_tgt_pos_d = alt_.controller().pos_target_d(); navlog_.alt_tgt_vel_d = alt_.controller().vel_target_d(); navlog_.hover = alt_.hover();
        const nav::PosController& nctl = rescue_out_.active || mission_fly_ ? rctl_ : ph_.ctl;      // (the rescue and the mission fly their own controller)
        navlog_.pos_tgt[0] = nctl.target_n(); navlog_.pos_tgt[1] = nctl.target_e();
        navlog_.ctl_flags = (uint8_t)((o.althold_active ? 1u : 0u) | (was_poshold_ ? 2u : 0u) | (was_poshold_ && ph_.holding() ? 4u : 0u) | (o.fence ? 8u : 0u));
        o.nav_target_n = nctl.target_n(); o.nav_target_e = nctl.target_e(); o.nav_wind_n = ph_.wind_n(); o.nav_wind_e = ph_.wind_e(); o.nav_home_dist = sqrtf(est_.pos_n * est_.pos_n + est_.pos_e * est_.pos_e);

        // ---- 7. modes -> rate setpoints
        const bool angle = ((cmd.modes >> M_ANGLE) & 1u) || neutral || mission_fly_;    // the failsafe levels the craft; the hold modes and the rescue switch carry ANGLE with them (ModeMap::resolve)
        const bool stab = !angle && ((cmd.modes >> M_STABILIZE) & 1u);
        float sp_dps[3] = {ss.rate[ROLL], ss.rate[PITCH], ss.rate[YAW]};
        // the rescue turns the nose towards home; a pilot who is there (a rescue from the switch) keeps the last word with his yaw stick
        if (rescue_out_.active && rescue_out_.yaw_control && (neutral || fabsf(cmd.yaw) < 0.05f)) sp_dps[YAW] = rescue_out_.yaw_rate_dps;
        else if (mission_fly_ && mission_out_.yaw_control && (neutral || fabsf(cmd.yaw) < 0.05f)) sp_dps[YAW] = mission_out_.yaw_rate_dps;
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
            stab_.update(st, ss.rate[YAW], at, dt, sp_dps, g_f_);
            for (int a = 0; a < 3; a++) pin.feedforward[a] = stab_.target_accel(a);   // the shaped target says how the rate is about to change: no waiting for the error
        } else {                                               // Acro
            for (int a = 0; a < 3; a++) pin.feedforward[a] = ss.feedforward[a];
        }
        // the acro trainer: Acro only (Betaflight: not in Angle / Horizon / the rescue / the hold modes - all of which are Angle here)
        const bool trainer = !angle && !stab && ((cmd.modes >> M_ACRO_TRAINER) & 1u);
        if (trainer && !was_trainer_) trainer_.reset();
        trainer_reset_i_[0] = trainer_reset_i_[1] = false;
        if (trainer) for (int a = ROLL; a <= PITCH; a++) sp_dps[a] = trainer_.apply(a, sp_dps[a], a == ROLL ? att_.roll : att_.pitch, g_f_[a], trainer_reset_i_[a]);
        was_trainer_ = trainer; o.trainer_active = trainer;
        was_angle_ = angle; was_stab_ = stab;

        // ---- 8. rate loop + mixer
        // TPA and the dynamic low-passes follow the throttle that is flown, whoever commands it. Anti-gravity and the throttle
        // boost answer the PILOT'S hand only: fed with Altitude Hold's output they would amplify barometer noise into the motors
        // (Betaflight computes them from the stick as well, and puts the autopilot's throttle in afterwards).
        pid_.set_anti_gravity_switch(rc.modes.has(M_ANTI_GRAVITY) ? (int8_t)((cmd.modes >> M_ANTI_GRAVITY) & 1u) : (int8_t)-1);
        pid_.update_throttle(thr, &gf_, pilot_throttle);
        for (int a = ROLL; a <= PITCH; a++) if (trainer_reset_i_[a]) pid_.reset_i(a);
        for (int a = 0; a < 3; a++) { pin.setpoint[a] = sp_dps[a]; pin.gyro[a] = g_f_[a]; }
        pin.angle_mode = angle || stab;
        pin.i_enabled = !ground_idle;
        pid_.update(pin, o.u);
        for (int a = 0; a < 3; a++) { o.sp_dps[a] = sp_dps[a]; o.pid_p[a] = pid_.p_term[a]; o.pid_i[a] = pid_.i_term[a]; o.pid_d[a] = pid_.d_term[a]; o.pid_f[a] = pid_.f_term[a]; }
        mixer_.mix(in.limits, boost_.apply(thr, pilot_throttle), o.u[ROLL], o.u[PITCH], o.u[YAW], o.motor, ground_idle);
        if (alt_.state() == AhState::STOPPED && o.althold_active) for (auto& m : o.motor) m = 0;

        // ---- 9. runaway take-off prevention (safety.h): the loop fights the craft instead of steering it -> motors off
        RunawayTakeoff::Input ri;
        ri.armed = true; ri.motors_mixed = !ground_idle; ri.throttle = pilot_throttle ? cmd.throttle_cmd : thr;
        ri.stick = maxf(fabsf(cmd.roll), maxf(fabsf(cmd.pitch), fabsf(cmd.yaw)));
        for (int a = 0; a < 3; a++) { ri.pid_sum[a] = o.pid_p[a] + o.pid_i[a] + o.pid_d[a] + o.pid_f[a]; ri.gyro_dps[a] = g_f_[a]; }
        if (runaway_.update(ri, dt)) {
            arming_.force_disarm(AB_RUNAWAY_TAKEOFF); armed_ = false;
            o.armed = false; o.arm_blockers |= AB_RUNAWAY_TAKEOFF; for (auto& m : o.motor) m = 0;
        }
        // ---- 9b. a motor that stopped (a prop strike, a blocked motor, a desync): everything off (safety.h StallGuard)
        { StallGuard::Input si; si.armed = o.armed; si.motors = o.motor_count; si.command = o.motor; si.rpm = in.motors.rpm; si.valid_mask = in.motors.valid_mask;
          if (stall_.update(si, dt)) { arming_.force_disarm(AB_MOTOR_STALL); armed_ = false; o.armed = false; o.arm_blockers |= AB_MOTOR_STALL; for (auto& m : o.motor) m = 0; }
          o.stall_motor = (uint8_t)stall_.last(); }
        // a rescue from the switch has brought the craft down: motors off (after a failsafe the failsafe itself does that)
        if (rescue_landed && !fo.rescue) { arming_.force_disarm(); armed_ = false; o.armed = false; for (auto& m : o.motor) m = 0; }
        return o;
    }

public:
    const Ahrs& ahrs() const { return ahrs_; }
    const nav::AltEstimator& alt_estimator() const { return alt_est_; }
    const nav::Log& navlog() const { return navlog_; }            // the navigation stack's inputs / state for the blackbox (nav/nav_log.h)
    const nav::PosEstimator& pos_estimator() const { return pos_est_; }
    const nav::AltHoldMode& alt_hold() const { return alt_; }
    const PidController& pid() const { return pid_; }
    const GyroFilter& gyro_filter() const { return gf_; }
    const SetpointGenerator& setpoints() const { return sp_gen_; }
    const AngleController& angle_ctl() const { return angle_; }
    const StabilizeController& stabilize() const { return stab_; }
    const Estimate& estimate() const { return est_; }
    bool armed() const { return armed_; }
    const RunawayTakeoff& runaway() const { return runaway_; }
    const StallGuard& stall_guard() const { return stall_; }
    const TiltAid& tilt_aid() const { return tilt_aid_; }
    const nav::PosHoldMode& poshold() const { return ph_; }
    const Rescue& rescue() const { return rescue_; }
    Mixer& mixer() { return mixer_; }

    // Throttle stick 0..1 -> throttle with the middle of the stick at `hover` (two straight lines: 0 -> 0, 0.5 -> hover, 1 -> 1).
    static float hover_centred(float stick01, float hover) {
        const float s = clampf(stick01, 0.0f, 1.0f);
        return s <= 0.5f ? hover * (s * 2.0f) : hover + (1.0f - hover) * ((s - 0.5f) * 2.0f);
    }

private:
    // The mission, once per estimator step (armed or not: it waits on the ground with the motors off): start / stop as the
    // switch or the app ask, then one step. The navigation numbers are the ones 6b flies with (the EKF's when it is in use).
    void mission_step(const FastInput& in, bool arm_sw, float dt_est, float pn = 0, float pe = 0, float vn = 0, float ve = 0, float yaw = 0, bool pos_valid = false) {
        if (!armed_) { pn = est_.pos_n; pe = est_.pos_e; vn = est_.vel_n; ve = est_.vel_e; yaw = att_.yaw; pos_valid = est_.pos_valid; }
        MissionInput mi; mi.armed = armed_; mi.airborne = airborne_; mi.pos_valid = pos_valid; mi.alt_valid = est_.alt_valid;
        mi.landed = armed_ && alt_.state() == AhState::LANDED_IDLE; mi.arm_switch_on = in.rc_link ? arm_sw : true;
        mi.rtl_done = mission_.state() == MissionState::RTL && !armed_;
        mi.pos_n = pn; mi.pos_e = pe; mi.vel_n = vn; mi.vel_e = ve; mi.z = est_.z; mi.vz = est_.vz; mi.yaw_deg = yaw;
        const bool want = mission_sw_ || mission_app_;
        if (want && !mission_req_) { mission_req_ = true; mission_.start(mission_plan_, mi); }      // (REFUSED shows in the state)
        else if (!want && mission_req_) { mission_req_ = false; mission_.stop(); }
        mission_out_ = mission_.update(mi, dt_est);
        if (mission_out_.servo >= 1 && mission_out_.servo <= 4) mission_servo_[mission_out_.servo - 1] = mission_out_.servo_us;
    }
    void mission_fill(CoreOutput& o) const {
        o.mission_state = (uint8_t)mission_.state(); o.mission_index = mission_.index(); o.mission_count = mission_.count(); o.mission_wait = mission_.wait_s();
        o.mission_dist = mission_.dist(); o.mission_left = mission_.left(); o.mission_warn = mission_out_.warn; o.mission_holds_home = mission_.holds_home();
        for (int i = 0; i < 4; i++) o.mission_servo_us[i] = mission_servo_[i];
    }
    void configure_control() {
        const float hz = cfg.pid_hz();                 // the controllers run at the control rate, the gyro filters (gf_) at the gyro's
        pid_.configure(cfg.pid, hz); sp_gen_.configure(cfg.ff, cfg.rc_smoothing, hz);
        angle_.configure(cfg.angle, hz); stab_.configure(cfg.stab); trainer_.cfg = cfg.trainer; boost_.configure(cfg.throttle_boost, cfg.throttle_boost_cutoff, hz);
        have_rc_ = false; ctl_dirty_ = true;
    }
    // disarmed: the controllers forget everything, once
    WB_FAST void reset_control() {
        if (!ctl_dirty_) return;
        pid_.reset(); angle_.reset(); boost_.reset();
        raised_ = airborne_ = false; up_t_ = 0.0f; land_.reset(); was_angle_ = was_stab_ = was_trainer_ = false; ctl_dirty_ = false;
    }
    WB_FAST void estimator_step(Vec3 acc, float dt_est, uint32_t now) {
        float an, ae, aup; ahrs_.world_accel(acc, an, ae, aup);
        est_.az = aup;
        alt_est_.predict(-aup, dt_est);
        const bool baro_fresh = have_baro_ && age_ms(now, baro_ms_) <= (uint32_t)cfg.baro_stale_ms;
        // near the ground with the motors running the air under the craft is pushed about: the barometer counts for less
        // (ArduPilot does the same while a take-off or touch-down is expected)
        ground_t_ = armed_ && airborne_ ? ground_t_ + dt_est : 0.0f;
        if (baro_new_) {
            alt_est_.update_baro(baro_alt_, baro_sigma_, armed_ && ground_t_ < cfg.ground_effect_s); baro_new_ = false;
            if (!armed_) alt_est_.update_still();                  // standing: no climb rate can build up from an accel bias
        }
        if (range_new_) { alt_est_.update_range(range_h_, range_sigma_, baro_fresh); range_new_ = false; }
        const bool range_fresh = have_range_ && age_ms(now, range_ms_) <= (uint32_t)cfg.range_stale_ms;
        pos_est_.predict(an, ae, dt_est);
        tilt_aid_.push_force(an / kG0, ae / kG0, -aup / kG0 - 1.0f, dt_est);
        // The GPS's north means something to the craft only with a heading that does not come from the gyro alone (the compass).
        // Without one: the GPS still shows where the craft is - unless optical flow is at work, whose velocity is measured in
        // the craft's own axes and would not fit together with it. The tilt is never corrected from a GPS without a heading.
        const bool flow_fresh = have_flow_ && age_ms(now, flow_ok_ms_) <= (uint32_t)cfg.flow_stale_ms;
        if (gps_new_) {
            // GPS height wanders by metres over a minute; a barometer does not (ArduPilot's default height source is the barometer,
            // Betaflight lets the GPS in with a 200 s time constant). See GpsAltMode and nav/alt_estimator.h.
            if (cfg.gps_alt != GA_OFF && gps_.sigma_alt <= cfg.gps_alt_max_sigma) {
                const float sa = gps_.sigma_alt;
                if (!baro_fresh && !range_fresh) {                 // the GPS is all there is: as the height (seamless when a baro was there a moment ago)
                    if (alt_est_.gps_ref_ok()) alt_est_.update_gps_rel(gps_.alt_m, sa, true); else alt_est_.update_gps_abs(gps_.alt_m, sa);
                } else if (cfg.gps_alt == GA_BLEND && baro_fresh && !range_fresh) {
                    alt_est_.update_gps_rel(gps_.alt_m, maxf(sa, cfg.alt_est.gps_blend_sigma_min_m), true);
                } else alt_est_.update_gps_rel(gps_.alt_m, sa, false);      // only keeps the reference
            }
            // GPS vertical velocity: compared with the estimate of gps_delay_s ago, when it was measured (nav/alt_estimator.h)
            if (cfg.gps_vel_z && gps_.vd_valid) alt_est_.update_gps_vel(gps_.vd, gps_.sigma_vel * cfg.gps_vz_sigma_scale);
            navlog_.gps_seq++; navlog_.gps_flags = (uint8_t)(1u | (gps_.vd_valid ? 2u : 0u));
            navlog_.gps_pos[0] = gps_.north_m; navlog_.gps_pos[1] = gps_.east_m; navlog_.gps_pos[2] = -gps_.alt_m;
            navlog_.gps_vel[0] = gps_.vn; navlog_.gps_vel[1] = gps_.ve; navlog_.gps_vel[2] = gps_.vd_valid ? gps_.vd : 0.0f;
            navlog_.gps_hacc = gps_.sigma_pos; navlog_.gps_sacc = gps_.sigma_vel; navlog_.gps_sats = gps_.sats;
            if (heading_ok_ || !flow_fresh) {
                navlog_.gps_flags |= 4u;
                pos_est_.fuse_gps(gps_.north_m, gps_.east_m, gps_.vn, gps_.ve, gps_.sigma_pos, gps_.sigma_vel);
                if (heading_ok_ && gps_.sigma_vel <= cfg.aid_max_sigma_vel) tilt_aid_.on_velocity(gps_.vn, gps_.ve, cfg.pos_est.gps_delay_s);
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
                pos_est_.fuse_velocity(vn, ve, fv.sigma, age);
                tilt_aid_.on_velocity(vn, ve, age);
                flow_ok_ms_ = now; have_flow_ = true;
            }
        }
        nav_log_inputs(an, ae, aup, dt_est);
        est_.z = alt_est_.height(); est_.vz = alt_est_.climb_rate(); est_.z_sigma = alt_est_.height_sigma();
        est_.gps_alt_used = alt_est_.gps_used(); est_.gps_vz_used = alt_est_.gps_vel_used();
        // the GPS height counts when it is in use, or ready to take over this moment (its reference is set, the GPS is fresh):
        // a barometer that goes silent in flight then hands over without Altitude Hold dropping out in between
        const bool gps_height = cfg.gps_alt != GA_OFF && (alt_est_.gps_used() || (alt_est_.gps_ref_ok() && have_gps_ && age_ms(now, gps_ms_) <= (uint32_t)cfg.gps_stale_ms));
        est_.alt_valid = (baro_fresh || range_fresh || gps_height) && est_.z_sigma < 3.0f;
        est_.pos_n = pos_est_.pos_n(); est_.pos_e = pos_est_.pos_e(); est_.vel_n = pos_est_.vel_n(); est_.vel_e = pos_est_.vel_e();
        est_.pos_valid = have_gps_ && have_gps_xy_ && age_ms(now, gps_xy_ms_) <= (uint32_t)cfg.gps_stale_ms && pos_est_.initialised();
        const bool flow_now = have_flow_ && age_ms(now, flow_ok_ms_) <= (uint32_t)cfg.flow_stale_ms;
        est_.vel_valid = (est_.pos_valid || flow_now) && pos_est_.initialised();
        est_.flow_valid = flow_now;                             // (one sample that was not used - a dark patch - does not end it: flow_stale_ms without any does)
    }

    Ahrs ahrs_; AccelGainSchedule acc_gain_; nav::AltEstimator alt_est_; nav::PosEstimator pos_est_; Mixer mixer_; nav::AltHoldMode alt_; Failsafe fs_; Arming arming_; RunawayTakeoff runaway_; StallGuard stall_;
    GyroFilter gf_; SetpointGenerator sp_gen_; PidController pid_; AngleController angle_; StabilizeController stab_; ThrottleBoost boost_;
    AcroTrainer trainer_; bool was_trainer_ = false, trainer_reset_i_[2] = {false, false};
    StickArm stick_arm_;
    float g_f_[3] = {};
    EulerDeg att_; Estimate est_;
    int denom_ = 1, pid_n_ = 0; float win_dt_ = 0.0f; Vec3 win_g_, win_a_; CoreOutput last_;     // the gyro samples of the control cycle to come
    Vec3 acc_sum_, gyro_sum_; int acc_n_ = 0, decim_ = 8; float est_dt_ = 0.0f, ground_t_ = 0.0f;
    nav::Log navlog_; Vec3 vib_mean_; uint8_t clip_n_ = 0;
    // The inputs of each estimator step, for the blackbox: the acceleration in NED with gravity taken away, how much it shakes
    // (|a - its 2 Hz mean|, low-passed at 5 Hz - the 'vibration' ArduPilot and PX4 judge an accelerometer by), clipped samples,
    // the barometer. (The GPS part is filled where a fix comes in.)
    void nav_log_inputs(float an, float ae, float aup, float dt) {
        navlog_.acc_ned[0] = an; navlog_.acc_ned[1] = ae; navlog_.acc_ned[2] = -aup;
        const float km = dt / (dt + 1.0f / (2.0f * kPi * 2.0f)), kv = dt / (dt + 1.0f / (2.0f * kPi * 5.0f));
        vib_mean_.x += (an - vib_mean_.x) * km; vib_mean_.y += (ae - vib_mean_.y) * km; vib_mean_.z += (-aup - vib_mean_.z) * km;
        const float dx = an - vib_mean_.x, dy = ae - vib_mean_.y, dz = -aup - vib_mean_.z;
        navlog_.vib += (sqrtf(dx * dx + dy * dy + dz * dz) - navlog_.vib) * kv;
        navlog_.clip = clip_n_; clip_n_ = 0;
        navlog_.baro_alt = baro_alt_;
        navlog_.est_pos_d = alt_est_.pos_d(); navlog_.est_vel_d = alt_est_.vel_d(); navlog_.est_bias_d = alt_est_.accel_bias(); navlog_.est_baro_off = alt_est_.baro_offset();
        navlog_.baro_ratio = alt_est_.baro_innov().ratio; navlog_.gps_ratio = pos_est_.gps_innov().ratio;
        navlog_.alt_resets = alt_est_.reset_count(); navlog_.pos_resets = pos_est_.reset_count();
        navlog_.est_pos[0] = pos_est_.pos_n(); navlog_.est_pos[1] = pos_est_.pos_e(); navlog_.est_vel[0] = pos_est_.vel_n(); navlog_.est_vel[1] = pos_est_.vel_e();
    }
    TiltAid tilt_aid_; GyroHistory gyro_hist_; FlowSample flow_; bool flow_new_ = false, flow_seen_ = false, have_flow_ = false, have_gps_xy_ = false, heading_ok_ = false; uint32_t flow_ms_ = 0, flow_ok_ms_ = 0, gps_xy_ms_ = 0;
    nav::PosHoldMode ph_; nav::PosController rctl_; Rescue rescue_; RescueOutput rescue_out_; bool was_poshold_ = false, nav_lean_ = false, ph_unavail_ = false, ph_slow_gps_ = false, rescue_ok_ = false; float nav_tgt_[2] = {0, 0};
    EkfNav ekf_; uint32_t ekf_ms_ = 0; bool ekf_in_use_ = false;
    nav::NavRequest req_; uint32_t req_ms_ = 0; bool have_req_ = false, was_req_ = false, req_climb_on_ = false; float req_climb_ = 0.0f;
    MissionRunner mission_; MissionPlan mission_plan_; MissionOutput mission_out_; uint16_t mission_servo_[4] = {0, 0, 0, 0};
    bool mission_req_ = false, mission_app_ = false, mission_sw_ = false, mission_fly_ = false, was_mission_h_ = false;
    bool have_att_ = false, mag_seeded_ = false, armed_ = false, was_althold_ = false;
    LandDetector land_; bool airborne_ = false; float up_t_ = 0.0f;
    bool have_rc_ = false, raised_ = false, was_angle_ = false, was_stab_ = false, ctl_dirty_ = true; uint32_t rc_seq_ = 0;
    float last_thr_ = 0;
    float baro_alt_ = 0, baro_sigma_ = 0.3f, range_h_ = 0, range_sigma_ = 0.03f; bool baro_new_ = false, range_new_ = false, gps_new_ = false, have_baro_ = false, have_range_ = false, have_gps_ = false;
    uint32_t baro_ms_ = 0, range_ms_ = 0, gps_ms_ = 0, mag_ms_ = 0; GpsMeas gps_; Vec3 mag_; bool mag_ready_ = false;
};
} // namespace fc
