// Closed-loop tests of the full FlightCore against the independent plant (sim_plant.h): sign conventions, arming, compass, AltHold with
// baro + lidar, failsafe, and the hand-over to the motor layer.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <string>
#include <memory>
#include "sim_plant.h"
#include "../src/core/flight_core.h"
#include "../src/core/simplified_tuning.h"
#include "../src/hal/motor/motor_output.h"
using namespace fc;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(99); static std::normal_distribution<float> g_norm(0, 1); static float gauss(float s) { return g_norm(rng) * s; }
// A test that wants noise of its own (several seeds), or that was added between two older ones: the sensor noise the tests after
// it see stays what it was.
struct KeepNoise { std::mt19937 e = rng; std::normal_distribution<float> d = g_norm; ~KeepNoise() { rng = e; g_norm = d; } };

struct Pilot { float roll = 0, pitch = 0, yaw = 0, thr = 0; bool arm = false; int mode = 0; bool althold = false, fs_sw = false, link = true, poshold = false, rescue = false; uint16_t aux7 = 1000; };      // aux7: free for a test's own switch

// Pilot::thr is the throttle the pilot ASKS FOR in the direct-throttle modes (0 = stick at the bottom, below min_check) and the
// stick's POSITION in Altitude Hold (0.5 = hold).
static uint16_t throttle_us(float thr) { return thr <= 0.0f ? 1000 : (uint16_t)lroundf(1050.0f + 950.0f * thr); }

struct Rig {
    Plant pl; FlightCore core; CoreOutput out; Health h; MotorLimits lim; uint32_t us = 0; float drift = 0, drift_rate = 0;
    bool baro = false, range = false, mag = false; float declination = 0; const float dt = 0.00025f;
    float rx_hz = 150.0f; uint32_t next_frame_us = 0, rc_seq = 0;      // the receiver: one frame every 1 / rx_hz (ELRS 150 Hz), the sticks as they were then
    uint16_t rc_us[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    float gyro_noise = 0.15f, tone_hz = 0, tone_dps = 0, raw_roll = 0;   // sensor noise (dps rms); a vibration on the roll gyro; what the roll gyro read last
    bool esc_rpm = false; float rpm_all = 0;                              // the ESCs report this motor speed (bidirectional DShot)
    bool rpm_per_motor = false; int dead_motor = -1;                      // each ESC reports its own motor's speed (from its thrust); dead_motor: that motor (of the plant) stops - a prop strike
    float gyro_bias[3] = {0, 0, 0};                                       // the gyro reads this much too high (deg/s): what is left after the boot calibration, or grows as the board warms up
    int slow_every = 0, waited = 1; uint32_t slow_us = 1750, cycle_n = 0;             // every slow_every-th control cycle takes slow_us instead of 250 us (the craft flies on meanwhile)
    int motor_map[kMaxMotors] = {0, 1, 2, 3, 4, 5, 6, 7};                 // which output of the core is plugged to which motor of the craft
    bool gps = false; float gps_alt_err = 0;                              // a GPS (10 Hz) whose altitude is this far off
    bool gps_vd = true;                                                   // the GPS reports a vertical velocity (u-blox; NMEA does not)
    float ground_level = 0;                                               // the ground under the craft lies this far above the take-off point (a table): what the range finder sees
    float prop_wash_m = 0;                                                // the barometer reads this much too low while the motors run and the craft is within 0.3 m of the ground
    // navigation sensors. ngps: a fix every 100 ms that shows where the craft was 150 ms ago; the position wanders slowly (multipath), the velocity is good
    uint8_t gps_sats = 14; float gps_rate_said = 10.0f;      // what the GPS layer says its solution rate is (Position Hold wants >= nav.ph_min_gps_hz)
    bool ngps = false; float gps_delay = 0.15f, gps_sigma_p = 0.3f, gps_sigma_v = 0.08f, gps_wander = 0.6f, gps_bias[2] = {0, 0}, gps_sacc = 0.15f; bool gps_valid = true;     // gps_sacc: how sure the receiver says its velocity is
    // optical flow: 50 samples a second, each the mean over 20 ms that ended 10 ms ago, rad/s about the sensor's X / Y (ArduPilot's signs)
    bool flow = false; float flow_noise = 0.03f, flow_scale = 1.0f; uint8_t flow_quality = 90; int flow_window_ms = 20, flow_late_ms = 10;      // each sample: the mean over flow_window_ms that ended flow_late_ms ago
    struct Past { float pn, pe, vn, ve, z, vz, wx, wy, vf, vr, range; }; std::vector<Past> past; size_t past_i = 0;
    const Past& ago(float s) const { const size_t n = past.size(), k = (size_t)(s * 1000.0f + 0.5f); return past[(past_i + n - 1 - (k < n ? k : n - 1)) % n]; }
    explicit Rig(int layout = 0, bool use_mag = false) : pl(layout) {
        core.cfg.mixer.layout = layout == 2 ? HEX_X : layout == 1 ? QUAD_PLUS : QUAD_X; core.cfg.use_mag = use_mag; core.cfg.declination_deg = 0;
        core.rc.modes.add(M_ARM, 0, 1700, 2100); core.rc.modes.add(M_ANGLE, 1, 1300, 1700); core.rc.modes.add(M_STABILIZE, 2, 1300, 1700);
        core.rc.modes.add(M_ALTHOLD, 3, 1300, 1700); core.rc.modes.add(M_FAILSAFE_SW, 4, 1300, 1700); core.rc.modes.add(M_POSHOLD, 5, 1300, 1700); core.rc.modes.add(M_RESCUE, 6, 1300, 1700);
        core.configure(); h.imu_ok = h.gyro_calibrated = true; h.mag_calibrated = use_mag; h.mag_ok = use_mag; mag = use_mag;
    }
    void run(float dur, const Pilot& p) {
        FastInput in; in.dt = dt; in.limits = lim; in.rc_link = p.link; h.baro_ok = baro; in.health = h;
        uint16_t ch[16]; for (int i = 0; i < 16; i++) ch[i] = 1500;
        ch[0] = (uint16_t)(1500 + 500 * p.roll); ch[1] = (uint16_t)(1500 + 500 * p.pitch); ch[2] = p.althold ? (uint16_t)(1000 + 1000 * p.thr) : throttle_us(p.thr); ch[3] = (uint16_t)(1500 + 500 * p.yaw);
        ch[4] = p.arm ? 2000 : 1000; ch[5] = p.mode == 1 ? 1500 : 1000; ch[6] = p.mode == 2 ? 1500 : 1000; ch[7] = p.althold ? 1500 : 1000; ch[8] = p.fs_sw ? 1500 : 1000; ch[9] = p.poshold ? 1500 : 1000; ch[10] = p.rescue ? 1500 : 1000; ch[11] = p.aux7;
        const int steps = (int)(dur / dt + 0.5f);          // (+0.5: 0.001 / 0.00025 is 3.9999 in float)
        for (int i = 0; i < steps;) {
            const uint32_t prev = us; us += 250u * (uint32_t)waited; i += waited; in.dt = dt * (float)waited; in.now_ms = us / 1000;      // the time since the previous cycle
            if (us >= next_frame_us) { next_frame_us += (uint32_t)(1e6f / rx_hz); memcpy(rc_us, ch, sizeof(rc_us)); rc_seq++; }      // the receiver delivers a frame
            memcpy(in.rc_us, rc_us, sizeof(rc_us)); in.rc_seq = rc_seq;
            Vec3 a = pl.acc(); a.x += gauss(0.01f); a.y += gauss(0.01f); a.z += gauss(0.01f);
            Vec3 g = pl.gyro_dps(); g.x += gauss(gyro_noise) + gyro_bias[0]; g.y += gauss(gyro_noise) + gyro_bias[1]; g.z += gauss(gyro_noise) + gyro_bias[2];
            if (tone_dps > 0) g.x += tone_dps * sinf(2 * kPi * tone_hz * (float)(us * 1e-6));
            in.gyro_dps = g; in.acc_g = a; raw_roll = g.x;
            in.motors.rpm_valid = esc_rpm; for (auto& m : in.motors.rpm) m = rpm_all; in.motors.valid_mask = esc_rpm ? 0xFF : 0;
            if (rpm_per_motor) { in.motors.rpm_valid = true; in.motors.valid_mask = 0xFF;
                for (int k = 0; k < pl.n; k++) in.motors.rpm[motor_map[k]] = k == dead_motor ? 0.0f : 24000.0f * sqrtf(fmaxf(pl.T[k], 0.0f) / (pl.Tmax * fmaxf(pl.gain[k], 0.05f))); }
            const float wash = (out.armed && pl.z < 0.3f) ? prop_wash_m : 0.0f;
            if (us / 20000 != prev / 20000) { drift += drift_rate * 0.02f; if (baro) core.on_baro(pl.z + drift + gauss(0.25f) - wash, 0.3f, in.now_ms); }
            if (us / 33000 != prev / 33000 && range && pl.z - ground_level < 3.5f && pl.z >= ground_level) core.on_range(pl.z - ground_level + gauss(0.03f), 0.03f, in.now_ms);
            if (us / 10000 != prev / 10000 && mag) { const double d = declination * M_PI / 180, Bef[3] = {0.30 * cos(d), 0.30 * sin(d), 0.38}; double bf[3]; pl.body_from_world(Bef, bf); core.on_mag({(float)bf[0], (float)bf[1], (float)bf[2]}, true, in.now_ms); }
            if (us / 100000 != prev / 100000 && gps) { GpsMeas gm; gm.valid = true; gm.north_m = pl.pn; gm.east_m = pl.pe; gm.vn = pl.vn; gm.ve = pl.ve; gm.vd = -pl.vz; gm.vd_valid = gps_vd; gm.alt_m = pl.z + gps_alt_err; gm.sigma_alt = 2.0f; gm.rate_hz = gps_rate_said; core.on_gps(gm, in.now_ms); }
            if (us / 1000 != prev / 1000 && (ngps || flow)) {       // what the slow sensors will report later
                if (past.empty()) past.resize(512);
                past[past_i] = {pl.pn, pl.pe, pl.vn, pl.ve, pl.z, pl.vz, pl.w[0], pl.w[1], pl.v_forward(), pl.v_right(), (pl.z - ground_level) / fmaxf(pl.R22(), 0.3f)}; past_i = (past_i + 1) % past.size();
            }
            if (ngps && us / 100000 != prev / 100000 && us >= 600000) {
                const Past& a = ago(gps_delay); GpsMeas g; g.valid = gps_valid;
                for (int k = 0; k < 2; k++) gps_bias[k] += -gps_bias[k] * (0.1f / 20.0f) + gauss(gps_wander * sqrtf(2.0f * 0.1f / 20.0f));     // wanders with a time constant of 20 s
                g.north_m = a.pn + gps_bias[0] + gauss(gps_sigma_p); g.east_m = a.pe + gps_bias[1] + gauss(gps_sigma_p); g.alt_m = a.z + gauss(1.0f);
                g.vn = a.vn + gauss(gps_sigma_v); g.ve = a.ve + gauss(gps_sigma_v); g.vd = -a.vz + gauss(0.1f); g.vd_valid = gps_vd; g.sigma_pos = 1.0f; g.sigma_vel = gps_sacc; g.sigma_alt = 2.0f; g.sats = gps_sats; g.rate_hz = gps_rate_said;
                core.on_gps(g, in.now_ms);
            }
            if (flow && us / 20000 != prev / 20000 && us >= 600000) {
                const int n = flow_window_ms; float fx = 0, fy = 0; for (int k = 0; k < n; k++) { const Past& a = ago(0.001f * (flow_late_ms + k)); fx += a.wx - a.vr / fmaxf(a.range, 0.05f); fy += a.wy + a.vf / fmaxf(a.range, 0.05f); }
                FlowSample f; f.x = fx / n * flow_scale + gauss(flow_noise); f.y = fy / n * flow_scale + gauss(flow_noise); f.dt = n * 0.001f; f.age_s = flow_late_ms * 0.001f; f.quality = flow_quality;
                core.on_flow(f, in.now_ms);
            }
            out = core.update(in);
            float cmd[kMaxMotors]; for (int k = 0; k < kMaxMotors; k++) cmd[k] = out.armed && k != dead_motor ? out.motor[motor_map[k]] : 0.0f;
            waited = slow_every && (cycle_n++ % (uint32_t)slow_every) == 3 ? (int)(slow_us / 250) : 1;      // how long until the next cycle comes: the craft flies on
            for (int k = 0; k < waited; k++) pl.step(cmd, dt);
        }
    }
};

static Pilot armed_pilot(int mode = 1) { Pilot p; p.arm = true; p.mode = mode; p.thr = 0.47f; return p; }
static void arm(Rig& r, int mode = 1) { Pilot p; p.mode = mode; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); }

static void t_arming() {
    printf("[arming via RC]\n");
    { Rig r; Pilot p; r.run(0.3f, p);
      float s = 0; for (float m : r.out.motor) s += m; CHECK(!r.out.armed && s == 0, "disarmed => all motors 0");
      p.arm = true; r.run(0.1f, p); float lo = 1, hi = 0; for (int i = 0; i < 4; i++) { lo = fminf(lo, r.out.motor[i]); hi = fmaxf(hi, r.out.motor[i]); }
      CHECK(r.out.armed && lo >= 0.055f - 1e-4f && hi < 0.2f, "armed at zero throttle: motors idle between %.3f and %.3f", lo, hi);
      p.thr = 0.5f; p.arm = false; r.run(0.1f, p); CHECK(!r.out.armed && r.out.motor[0] == 0, "arm switch off => disarmed immediately"); }
    { Rig r; Pilot p; p.arm = true; r.run(0.4f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_ARM_SWITCH_ON), "arm switch ON at power-up: refused (0x%x)", r.out.arm_blockers);
      p.arm = false; r.run(0.1f, p); p.arm = true; r.run(0.1f, p); CHECK(r.out.armed, "turned off and on: armed"); }
    { Rig r; Pilot p; r.run(0.2f, p); p.arm = true; p.thr = 0.5f; r.run(0.1f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_THROTTLE_HIGH), "throttle high: refused"); }
    { Rig r; r.pl.set_tilt(40, 0); Pilot p; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_ANGLE_TOO_HIGH), "craft tilted 40 deg: refused"); }
    { Rig r; Pilot p; p.althold = true; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_ALTHOLD_NO_BARO), "AltHold switch on without a barometer: refused (spec: warn immediately)"); }
    { Rig r(0, true); r.h.mag_calibrated = false; Pilot p; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_MAG_NOT_CALIBRATED), "compass enabled but not calibrated: refused"); }
    { Rig r; r.h.imu_ok = false; Pilot p; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_NO_IMU), "IMU not healthy: refused"); }
}

// 0.24: arming with the sticks (general.stick_arm)
static void t_stick_arm() {
    printf("[stick arming] throttle and pitch down, yaw and roll to opposite sides\n");
    const float k = 0.8f;
    CHECK(StickArm::gesture(1, -1, -1, true, k) && StickArm::gesture(-1, -1, 1, true, k), "yaw left + roll right, or yaw right + roll left (throttle low, pitch back)");
    CHECK(!StickArm::gesture(1, -1, 1, true, k) && !StickArm::gesture(-1, -1, -1, true, k), "both to the same side: no");
    CHECK(!StickArm::gesture(1, -1, -1, false, k) && !StickArm::gesture(1, 0, -1, true, k) && !StickArm::gesture(1, 1, -1, true, k) && !StickArm::gesture(0.7f, -1, -1, true, k), "throttle up, pitch centred or forward, a stick not far enough (0.7): no");
    // a rig whose mode table has no ARM switch (keep_switch: it has one, on aux 1 of Rig = ch5)
    auto rig = [](bool keep_switch) { Rig* r = new Rig(); ModeMap m; if (keep_switch) m.add(M_ARM, 0, 1700, 2100); m.add(M_ANGLE, 1, 1300, 1700);
        r->core.rc.modes = m; r->core.cfg.stick_arm = true; r->core.configure(); return std::unique_ptr<Rig>(r); };
    Pilot g; g.thr = 0; g.pitch = -1; g.yaw = -1; g.roll = 1;        // the gesture
    {   auto r = rig(false); Pilot p; r->run(0.3f, p);
        r->run(0.5f, g); CHECK(!r->out.armed, "gesture held 0.5 s: not yet");
        r->run(0.6f, g); CHECK(r->out.armed, "held 1 s: armed");
        r->run(0.5f, g); CHECK(r->out.armed, "still held: stays armed (it takes a release before the next gesture)");
        r->run(0.3f, p); CHECK(r->out.armed && r->out.ground_idle, "sticks released: armed, motors at idle on the ground");
        r->run(0.5f, g); CHECK(r->out.armed, "the gesture on the ground 0.5 s: still armed");
        r->run(0.6f, g); CHECK(!r->out.armed && r->out.motor[0] == 0, "1 s: disarmed");
        r->run(1.5f, g); CHECK(!r->out.armed, "held on: stays disarmed");
        r->run(0.2f, p); Pilot y = g; y.roll = -1; y.yaw = 1; r->run(1.1f, y); CHECK(r->out.armed, "the other way round (yaw right + roll left): armed"); }
    {   auto r = rig(false); r->run(2.0f, g); CHECK(!r->out.armed, "the gesture held from power-up: nothing (it must be released first)"); }
    {   auto r = rig(false); Pilot p; r->run(0.3f, p); r->run(1.1f, g); Pilot f = p; f.thr = 0.47f; r->run(1.0f, f);
        CHECK(r->out.armed && r->out.airborne, "armed, taken off");
        r->run(1.3f, g); CHECK(r->out.armed, "the gesture held 1.3 s in the air (throttle cut, sticks out): it is not a disarm"); }
    {   auto r = rig(false); r->h.gyro_calibrated = false; Pilot p; r->run(0.3f, p); r->run(1.2f, g);
        CHECK(!r->out.armed && r->out.arm_refused == 1 && (r->out.arm_blockers & AB_GYRO_NOT_CAL), "a blocker: the gesture is refused (counted)");
        r->h.gyro_calibrated = true; r->run(1.2f, g); CHECK(!r->out.armed, "the blocker gone, the gesture still held: no arming by itself");
        r->run(0.2f, p); CHECK(!(r->out.arm_blockers & AB_ARM_SWITCH_ON), "released: no 'arm switch on' blocker left behind");
        r->run(1.1f, g); CHECK(r->out.armed, "a new gesture: armed"); }
    {   auto r = rig(true); Pilot p; r->run(0.3f, p); r->run(1.2f, g); CHECK(!r->out.armed, "with an ARM switch assigned and off: the gesture does nothing");
        p.arm = true; r->run(0.3f, p); CHECK(!r->out.armed, "switch on alone: not armed");
        g.arm = true; r->run(1.1f, g); CHECK(r->out.armed, "switch on + the gesture: armed");
        p.thr = 0.3f; r->run(0.3f, p); p.arm = false; r->run(0.05f, p); CHECK(!r->out.armed, "switch off: disarmed at once (a kill switch)");
        p.arm = true; p.thr = 0; r->run(0.3f, p); CHECK(!r->out.armed, "switch on again: no arming without the gesture"); g.arm = false; }
    {   Rig r; Pilot p; r.run(0.3f, p); r.run(1.2f, g); CHECK(!r.out.armed, "stick_arm off (default): the gesture does nothing"); }
}

// tilt Betaflight's Angle mode commands for a stick position: the rate curve (centre sensitivity, max rate, expo) shapes it too
static float angle_cmd(const Rig& r, int axis, float stick) { return r.core.cfg.angle.limit_deg * apply_actual_rate(r.core.cfg.rates, axis, stick) / r.core.cfg.rates.max_rate_dps[axis]; }

static void t_attitude_modes() {
    printf("[angle / stabilize / acro] stick -> motion, against the physical plant\n");
    { Rig r; arm(r); Pilot p = armed_pilot(); p.roll = 0.5f; r.run(1.5f, p); const float want = angle_cmd(r, ROLL, 0.5f);
      CHECK(fabsf(want - 12.43f) < 0.05f && fabsf(r.pl.right_down_deg() - want) < 1.0f && fabsf(r.pl.nose_down_deg()) < 1.0f, "angle, roll right 0.5: plant %.2f deg right-down (want %.2f: 45 deg x the rate curve at half stick), nose-down %.2f", r.pl.right_down_deg(), want, r.pl.nose_down_deg());
      CHECK(fabsf(r.out.est.att.roll - want) < 1.5f, "estimate (user convention) %.2f", r.out.est.att.roll);
      p.roll = 1.0f; r.run(1.5f, p); CHECK(fabsf(r.pl.right_down_deg() - 45.0f) < 2.0f, "full stick = the tilt limit: %.2f", r.pl.right_down_deg()); }
    { Rig r; arm(r); Pilot p = armed_pilot(); p.pitch = 0.5f; r.run(1.5f, p); const float want = angle_cmd(r, PITCH, 0.5f);
      CHECK(fabsf(r.pl.nose_down_deg() - want) < 1.0f && r.out.est.att.pitch > want - 2, "pitch FORWARD = nose down %.2f, estimate pitch %.2f (+ = nose down)", r.pl.nose_down_deg(), r.out.est.att.pitch); }
    { Rig r; arm(r); Pilot p = armed_pilot(); p.roll = -0.8f; p.pitch = -0.8f; r.run(1.5f, p);
      CHECK(r.pl.right_down_deg() < -20 && r.pl.nose_down_deg() < -20 && r.out.est.att.roll < -20 && r.out.est.att.pitch < -20, "left + back: roll %.1f, nose %.1f; estimate (%.1f, %.1f) - roll left and nose up are NEGATIVE", r.pl.right_down_deg(), r.pl.nose_down_deg(), r.out.est.att.roll, r.out.est.att.pitch); }
    { Rig r; r.core.cfg.max_arm_angle_deg = 90.0f; r.core.configure(); r.pl.set_tilt(30, -20); arm(r); Pilot p = armed_pilot(); r.run(2.0f, p); CHECK(fabsf(r.pl.right_down_deg()) < 2 && fabsf(r.pl.nose_down_deg()) < 2, "self-level from (30, -20): (%.2f, %.2f)", r.pl.right_down_deg(), r.pl.nose_down_deg()); }
    { Rig r; arm(r, 2); Pilot p = armed_pilot(2); p.roll = 0.5f; r.run(2.5f, p); CHECK(fabsf(r.pl.right_down_deg() - 22.5f) < 1.5f, "stabilize roll 0.5 = half the tilt limit (ArduPilot: linear): %.2f", r.pl.right_down_deg()); }
    { Rig r; arm(r, 0); Pilot p = armed_pilot(0); r.run(0.7f, p); p.roll = 1.0f; r.run(1.5f, p); CHECK(fabsf(r.pl.w[0] * kRad2Deg - 670) < 15, "acro full roll settles at %.0f dps (max 670)", r.pl.w[0] * kRad2Deg); }
    { Rig r; arm(r, 0); Pilot p = armed_pilot(0); p.yaw = 0.5f; r.run(2.0f, p); const float want = apply_actual_rate(r.core.cfg.rates, YAW, 0.5f), got = r.pl.w[2] * kRad2Deg;
      CHECK(got > 0 && fabsf(got - want) < 0.1f * want + 5, "yaw right 0.5: %.1f dps (want ~%.1f)", got, want); }
    { Rig r; arm(r, 0); Pilot p = armed_pilot(0); p.pitch = 0.3f; r.run(1.0f, p); const float want = apply_actual_rate(r.core.cfg.rates, PITCH, 0.3f), got = -r.pl.w[1] * kRad2Deg; CHECK(fabsf(got - want) < 0.1f * want + 3, "pitch forward 0.3: nose-down rate %.1f (want ~%.1f)", got, want); }
    { Rig r; arm(r, 0); Pilot p = armed_pilot(0); r.run(0.5f, p); CHECK(fabsf(r.pl.w[0]) * kRad2Deg < 3 && fabsf(r.pl.w[1]) * kRad2Deg < 3 && fabsf(r.pl.w[2]) * kRad2Deg < 3, "sticks centred: holds still"); }
    { Rig r; r.core.cfg.mixer.props_out = true; for (auto& s : r.pl.spin) s = -s; r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); p.yaw = 0.5f; r.run(2.0f, p); const float want = apply_actual_rate(r.core.cfg.rates, YAW, 0.5f);
      CHECK(fabsf(r.pl.w[2] * kRad2Deg - want) < 0.1f * want + 5, "props-out frame with the flag: yaw follows the stick (%.0f dps, want %.0f)", r.pl.w[2] * kRad2Deg, want); }
    for (int layout = 1; layout <= 2; layout++) { Rig r(layout); arm(r); Pilot p = armed_pilot(); p.roll = 0.5f; p.pitch = 0.3f; r.run(2.0f, p); const float wr = angle_cmd(r, ROLL, 0.5f), wp = angle_cmd(r, PITCH, 0.3f);
      CHECK(fabsf(r.pl.right_down_deg() - wr) < 1.5f && fabsf(r.pl.nose_down_deg() - wp) < 1.5f, "[%s] angle roll %.1f / pitch %.1f -> (%.1f, %.1f)", layout == 1 ? "quad +" : "hex X", wr, wp, r.pl.right_down_deg(), r.pl.nose_down_deg()); }
}

// ---------------------------------------------------------------------------------------------------------------------------------
// How well the rate loop flies. Two crafts: the 5-inch quad Betaflight's default tune is made for, and the heavy low-thrust
// default plant, where the same numbers are soft and the Master multiplier is what tightens them.
// A stick is moved over `ramp` seconds (a thumb is not a step function).
// ---------------------------------------------------------------------------------------------------------------------------------
struct StepResult { float target = 0, peak = 0, t90 = -1, last = 0; float over() const { return peak / target - 1.0f; } };
static float axis_rate(const Rig& r, int axis) { return (axis == PITCH ? -1.0f : 1.0f) * r.pl.w[axis] * kRad2Deg; }      // user convention: + pitch = nose down
static void fly_up(Rig& r, Pilot& p, int mode) { p = Pilot(); p.mode = mode; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); p.thr = r.pl.hover_cmd(); r.run(0.5f, p); }
static StepResult acro_step(Rig& r, int axis, float amp, float ramp, float dur = 0.6f) {
    Pilot p; fly_up(r, p, 0);
    StepResult o; o.target = apply_actual_rate(r.core.cfg.rates, axis, amp);
    for (float t = 0; t < dur; t += 0.001f) {
        const float st = amp * fminf((t + 0.001f) / ramp, 1.0f); if (axis == ROLL) p.roll = st; else if (axis == PITCH) p.pitch = st; else p.yaw = st;
        r.run(0.001f, p); const float w = axis_rate(r, axis);
        o.peak = fmaxf(o.peak, w); if (o.t90 < 0 && w >= 0.9f * o.target) o.t90 = t + 0.001f; o.last = w;
    }
    return o;
}
static void set_master(Rig& r, uint8_t master) { PidSliders s; s.master = master; PidConfig& c = r.core.cfg.pid; sliders_to_pid(s, c.p, c.i, c.d, c.d_max, c.f); r.core.configure(); }
// rms of setpoint - gyro while the stick draws a 2 Hz sine of +-0.5
static float sine_tracking(Rig& r, int axis, int mode = 0, float hz = 2.0f) {
    Pilot p; fly_up(r, p, mode); double e2 = 0; int n = 0;
    for (float t = 0; t < 1.5f; t += 0.001f) {
        const float st = 0.5f * sinf(2 * kPi * hz * t); if (axis == ROLL) p.roll = st; else if (axis == PITCH) p.pitch = st; else p.yaw = st;
        r.run(0.001f, p); if (t > 0.5f) { const float e = r.out.sp_dps[axis] - r.out.gyro_f_dps[axis]; e2 += e * e; n++; }
    }
    return (float)sqrt(e2 / n);
}

static void t_rate_loop() {
    printf("[rate loop] Betaflight's default tune on a 5-inch quad and on a heavy low-thrust craft\n");
    { Rig r; r.pl.make_five_inch(); const StepResult s = acro_step(r, ROLL, 1.0f, 0.06f);
      CHECK(s.over() < 0.08f && s.t90 > 0 && s.t90 < 0.11f && fabsf(s.last - s.target) < 0.02f * s.target, "5-inch, full roll in 60 ms: 90 %% after %.0f ms, overshoot %.1f %%, then %.0f of %.0f dps", s.t90 * 1000, s.over() * 100, s.last, s.target); }
    { Rig r; r.pl.make_five_inch(); const StepResult s = acro_step(r, PITCH, 0.5f, 0.06f);
      CHECK(s.over() < 0.15f && s.t90 > 0 && s.t90 < 0.11f && fabsf(s.last - s.target) < 0.06f * s.target, "5-inch, half pitch: 90 %% after %.0f ms, overshoot %.1f %%, then %.0f of %.0f dps", s.t90 * 1000, s.over() * 100, s.last, s.target); }
    { Rig r; r.pl.make_five_inch(); const StepResult s = acro_step(r, YAW, 0.5f, 0.06f);
      CHECK(s.over() < 0.18f && s.t90 > 0 && s.t90 < 0.15f && fabsf(s.last - s.target) < 0.08f * s.target, "5-inch, half yaw: 90 %% after %.0f ms, overshoot %.1f %%, then %.0f of %.0f dps", s.t90 * 1000, s.over() * 100, s.last, s.target); }
    { Rig a, b; const StepResult s1 = acro_step(a, ROLL, 1.0f, 0.06f, 1.2f); set_master(b, 200); const StepResult s2 = acro_step(b, ROLL, 1.0f, 0.06f, 1.2f);
      CHECK(s1.over() > 0.20f && s1.over() < 0.32f && fabsf(s1.last - s1.target) < 0.03f * s1.target, "heavy craft, default tune: flies, but soft - overshoot %.0f %% (the motors need 140 ms at full differential to reach 670 dps)", s1.over() * 100);
      CHECK(s2.over() < s1.over() - 0.07f && s2.over() < 0.19f && fabsf(s2.last - s2.target) < 0.01f * s2.target, "the same craft with the Master multiplier at 2.0: overshoot %.0f %% (was %.0f %%)", s2.over() * 100, s1.over() * 100); }
    { Rig a, b; a.pl.make_five_inch(); b.pl.make_five_inch(); for (auto& f : b.core.cfg.pid.f) f = 0; b.core.configure();
      const float with = sine_tracking(a, ROLL), without = sine_tracking(b, ROLL);
      CHECK(with < 0.7f * without && with < 25.0f, "feedforward: the craft turns WITH the stick - tracking error of a 2 Hz stick sine %.1f dps rms, %.1f without it", with, without);
      Rig c, d; c.pl.make_five_inch(); d.pl.make_five_inch(); for (auto& f : d.core.cfg.pid.f) f = 0; d.core.configure(); const float yw = sine_tracking(c, YAW), yo = sine_tracking(d, YAW);
      CHECK(yw < 0.9f * yo, "and on yaw: %.1f against %.1f dps rms", yw, yo); }
}

// What the filters do for the loop, inside the whole core: sensor noise, a motor vibration with and without rpm telemetry.
struct Tone { double s = 0, c = 0, e2 = 0; int n = 0; void add(float v, float hz, double t) { s += v * sin(2 * M_PI * hz * t); c += v * cos(2 * M_PI * hz * t); e2 += (double)v * v; n++; }
              float amp() const { return (float)(2 * sqrt(s * s + c * c) / n); } float rms() const { return (float)sqrt(e2 / n); } };
static float tone_left(Rig& r, float hz) {                 // amplitude of that frequency in the filtered roll gyro, over half a second of hover
    Pilot p; fly_up(r, p, 0); r.run(0.5f, p); Tone f;
    for (int k = 0; k < 2000; k++) { r.run(0.00025f, p); f.add(r.out.gyro_f_dps[0], hz, r.us * 1e-6); }
    return f.amp();
}
static void t_filters_in_the_loop() {
    printf("[filters in the loop] noise, a motor vibration, rpm telemetry\n");
    { Rig r; r.pl.make_five_inch(); r.gyro_noise = 3.0f; Pilot p; fly_up(r, p, 0); r.run(0.5f, p); Tone raw, flt;
      for (int k = 0; k < 4000; k++) { r.run(0.00025f, p); raw.add(r.raw_roll - r.pl.w[0] * kRad2Deg, 1, 0); flt.add(r.out.gyro_f_dps[0] - r.pl.w[0] * kRad2Deg, 1, 0); }
      CHECK(raw.rms() > 2.5f && flt.rms() < 0.45f * raw.rms(), "gyro noise of %.1f dps rms reaches the rate loop as %.2f", raw.rms(), flt.rms()); }
    // a 190 Hz vibration of 20 dps (a motor at 11400 rpm): the low-passes alone let most of it through, the notches take it out
    { auto rig = [](int dyn_count, bool rpm) { Rig* r = new Rig; r->pl.make_five_inch(); r->core.cfg.gyro.dyn.count = (uint8_t)dyn_count; r->core.configure(); r->tone_hz = 190; r->tone_dps = 20; r->esc_rpm = rpm; r->rpm_all = 11400; return r; };
      Rig* a = rig(0, false); Rig* b = rig(0, true); Rig* c = rig(3, false);
      const float none = tone_left(*a, 190), with_rpm = tone_left(*b, 190), with_dyn = tone_left(*c, 190);
      CHECK(none > 10.0f && none < 20.0f, "no notch at all: %.1f of 20 dps is left after the low-passes", none);
      CHECK(with_rpm < 0.03f * none && fabsf(b->core.gyro_filter().rpm_filter().notch_hz(0, 0) - 190.0f) < 2.0f, "rpm telemetry: the RPM filter sits on it (%.0f Hz) - %.2f dps left", b->core.gyro_filter().rpm_filter().notch_hz(0, 0), with_rpm);
      CHECK(with_dyn < 0.10f * none, "without rpm telemetry the dynamic notch finds it: %.2f dps left", with_dyn);
      delete a; delete b; delete c; }
}

// Throttle boost: a quick throttle move is exaggerated for a moment, so the motors (which lag) follow the stick sooner.
static void t_throttle_boost() {
    printf("[throttle boost]\n");
    float gained[2], peak[2], settled[2];
    for (int k = 0; k < 2; k++) { Rig r; r.pl.make_five_inch(); r.core.cfg.throttle_boost = k ? 5 : 0; r.core.configure(); Pilot p; fly_up(r, p, 0); const float v0 = r.pl.vz; p.thr = r.pl.hover_cmd() + 0.25f; peak[k] = 0;
      for (int i = 0; i < 100; i++) { r.run(0.001f, p); float m = 0; for (int j = 0; j < 4; j++) m += r.out.motor[j] / 4; peak[k] = fmaxf(peak[k], m); if (i == 59) gained[k] = r.pl.vz - v0; }
      r.run(0.4f, p); settled[k] = 0; for (int j = 0; j < 4; j++) settled[k] += r.out.motor[j] / 4; }
    CHECK(fabsf(peak[0] - settled[0]) < 0.004f, "boost off: the motors go to the new throttle and no further (%.3f, then %.3f)", peak[0], settled[0]);
    CHECK(peak[1] > settled[1] + 0.03f && fabsf(settled[1] - settled[0]) < 0.003f, "boost 5: they overshoot it for a moment (%.3f, then the same %.3f)", peak[1], settled[1]);
    CHECK(gained[1] > 1.08f * gained[0], "and the craft answers sooner: %.2f m/s after 60 ms instead of %.2f", gained[1], gained[0]);
}

// Angle and Stabilize as a pilot feels them: a stick move of 80 ms to 0.8, how the tilt arrives.
// (With an ideal tilt sensor: this is about the shape the CONTROLLER gives the move. What the attitude estimate adds while the
// craft gathers speed - the true tilt creeps past the commanded one - is t_estimator_in_flight's subject.)
static StepResult tilt_step(Rig& r, int mode, float amp, float target) {
    r.pl.true_accel = false;
    Pilot p; fly_up(r, p, mode); StepResult o; o.target = target;
    for (float t = 0; t < 1.5f; t += 0.001f) { p.roll = amp * fminf((t + 0.001f) / 0.08f, 1.0f); r.run(0.001f, p); const float a = r.pl.right_down_deg(); o.peak = fmaxf(o.peak, a); if (o.t90 < 0 && a >= 0.9f * target) o.t90 = t + 0.001f; o.last = a; }
    return o;
}
static void t_level_modes() {
    printf("[angle vs stabilize] the two ways to self-level\n");
    { Rig r; r.pl.make_five_inch(); const StepResult s = tilt_step(r, 1, 0.8f, angle_cmd(r, ROLL, 0.8f));
      CHECK(s.over() < 0.10f && s.t90 > 0 && s.t90 < 0.30f && fabsf(s.last - s.target) < 1.0f, "Angle (Betaflight), 5-inch: %.1f deg after %.0f ms, peak %.1f, then %.1f", 0.9f * s.target, s.t90 * 1000, s.peak, s.last); }
    for (int heavy = 0; heavy < 2; heavy++) { Rig r; if (!heavy) r.pl.make_five_inch(); const StepResult s = tilt_step(r, 2, 0.8f, 36.0f);
      CHECK(s.over() < 0.02f && s.t90 > 0.35f && s.t90 < 0.75f && fabsf(s.last - s.target) < 0.5f, "Stabilize (ArduPilot), %s: a shaped move - 90 %% after %.0f ms (input time constant 0.15 s), peak %.2f of 36: no overshoot", heavy ? "heavy" : "5-inch", s.t90 * 1000, s.peak); }
    // the shaped target knows how its own rate is about to change: that goes to the rate loop as feedforward
    for (int axis : {ROLL, PITCH}) { KeepNoise keep; Rig a, b; a.pl.make_five_inch(); b.pl.make_five_inch(); for (auto& f : b.core.cfg.pid.f) f = 0; b.core.configure();
      const float with = sine_tracking(a, axis, 2, 1.5f), without = sine_tracking(b, axis, 2, 1.5f);
      CHECK(with < 0.75f * without, "Stabilize, stick sine of 1.5 Hz on %s: the rate follows the shaped target with %.1f dps rms of error, %.1f without the feedforward", axis == ROLL ? "roll" : "pitch", with, without); }
    { Rig r; r.pl.make_five_inch(); r.core.cfg.stab.input_tc = 0.08f; r.core.configure(); const StepResult s = tilt_step(r, 2, 0.8f, 36.0f);
      CHECK(s.t90 < 0.42f && s.over() < 0.03f, "a shorter input time constant (0.08 s) is crisper: %.0f ms, peak %.2f", s.t90 * 1000, s.peak); }
    // heading: a gust that turns the craft is turned back
    { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, 2); r.run(0.5f, p); const float y0 = r.pl.yaw_deg(); r.pl.w[2] += 3.0f; r.run(0.05f, p); const float kicked = wrap180(r.pl.yaw_deg() - y0); r.run(1.5f, p); const float moved = wrap180(r.pl.yaw_deg() - y0);
      CHECK(fabsf(kicked) > 2.0f && fabsf(moved) < 1.0f, "Stabilize: knocked %.1f deg round by a gust, it turns back to its heading (%.2f deg off)", kicked, moved); }
    // the yaw stick turns the heading target at the commanded rate, and it stays where the stick is let go
    { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, 2); r.run(0.3f, p); const float y0 = r.pl.yaw_deg(); p.yaw = 0.5f; r.run(1.0f, p); const float rate = r.pl.w[2] * kRad2Deg, want = apply_actual_rate(r.core.cfg.rates, YAW, 0.5f);
      p.yaw = 0; r.run(1.0f, p); const float y1 = r.pl.yaw_deg(); r.run(1.0f, p);
      CHECK(fabsf(rate - want) < 0.08f * want && fabsf(wrap180(y1 - y0)) > 60 && fabsf(wrap180(r.pl.yaw_deg() - y1)) < 1.0f, "Stabilize yaw: %.0f dps at half stick (want %.0f), then it stays at the new heading (%.2f deg of creep)", rate, want, wrap180(r.pl.yaw_deg() - y1)); }
    // tilted and turning: the heading change is shared between the body axes, the tilt does not wobble
    { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, 2); p.roll = 0.6f; r.run(1.5f, p); p.yaw = 0.6f; float lo = 1e9f, hi = -1e9f; for (int k = 0; k < 150; k++) { r.run(0.01f, p); if (k > 50) { lo = fminf(lo, r.out.est.att.roll); hi = fmaxf(hi, r.out.est.att.roll); } }
      CHECK(hi - lo < 2.0f && fabsf(0.5f * (hi + lo) - 27.0f) < 2.0f, "Stabilize, banked 27 deg and yawing: the bank stays (%.1f .. %.1f)", lo, hi); }
}

// What happens at the seams: entering and leaving a mode, arming again, and what each mode hands to the rate loop.
// The attitude estimate in real flight. The accelerometer of a craft that speeds up shows less tilt than there is (it feels the
// thrust, which points along the craft's own axis whatever the tilt; only the drag gives the tilt away, and that needs speed).
// An estimate that believes it gives way - and the controller, holding the ESTIMATE at the commanded angle, leans the craft
// further. How far, with the gains this firmware uses, and what the former gain (2 /s) would have done.
static void t_estimator_in_flight() {
    printf("[attitude estimate in flight] an accelerometer that feels thrust and drag, not gravity\n");
    struct Lean { float max_err = 0, max_tilt = 0, end_true = 0, end_est = 0, speed = 0, after3 = 0, after3_err = 0; };
    // Angle mode, the stick held for a while and let go; the pilot adds throttle in the lean to hold the height
    auto lean = [](float kp, float gate, float stick, float hold_s) { Rig r; r.pl.make_five_inch(); r.core.cfg.ahrs.kp_acc = kp; r.core.cfg.ahrs.accel_gate_g = gate; r.core.configure();
        Pilot p; fly_up(r, p, 1); Lean o;
        auto fly = [&](float dur, float roll, bool watch) { for (float t = 0; t < dur; t += 0.01f) { p.roll = roll; p.thr = fminf((r.pl.hover_cmd() / fmaxf(r.pl.R22(), 0.4f) - r.lim.idle) / (1.0f - r.lim.idle), 1.0f); r.run(0.01f, p);
            if (watch) { o.max_err = fmaxf(o.max_err, fabsf(r.pl.right_down_deg() - r.out.est.att.roll)); o.max_tilt = fmaxf(o.max_tilt, r.pl.right_down_deg()); } } };
        fly(1.0f, 0, false); fly(hold_s, stick, true); o.end_true = r.pl.right_down_deg(); o.end_est = r.out.est.att.roll; o.speed = r.pl.v_right();
        fly(3.0f, 0, false); o.after3 = r.pl.right_down_deg(); o.after3_err = r.pl.right_down_deg() - r.out.est.att.roll;
        return o; };
    const float kp = Ahrs::Config().kp_acc, gate = Ahrs::Config().accel_gate_g;
    { Rig r; const float want = angle_cmd(r, ROLL, 0.5f); const Lean now = lean(kp, gate, 0.5f, 2.0f), old = lean(2.0f, gate, 0.5f, 2.0f);
      CHECK(fabsf(now.end_est - want) < 1.0f && now.max_err > 1.0f && now.max_err < 5.0f && now.speed > 2.0f,
            "half stick for 2 s (commanded %.1f deg, %.1f m/s gained): the estimate holds %.1f, the craft really leans %.1f - up to %.1f deg more than it believes", want, now.speed, now.end_est, now.end_true, now.max_err);
      CHECK(fabsf(now.after3) < 1.5f && fabsf(now.after3_err) < 1.5f, "3 s after the stick is let go it is level again within %.1f deg (estimate off by %.1f)", fabsf(now.after3), fabsf(now.after3_err));
      CHECK(old.max_err > 3.0f * now.max_err && old.max_tilt > want + 12.0f && fabsf(old.after3) > 8.0f,
            "with a gain of 2 /s instead of 0.25 /s the estimate gives way in half a second: the craft leans %.0f deg for %.0f commanded and swings %.0f deg the other way after the stick is let go", old.max_tilt, want, -old.after3); }
    { Rig r; const float want = angle_cmd(r, ROLL, 0.8f); const Lean now = lean(kp, gate, 0.8f, 4.0f), open = lean(kp, 100.0f, 0.8f, 4.0f);
      CHECK(now.max_err < 10.0f && now.max_tilt < want + 10.0f && now.speed > 10.0f, "0.8 stick for 4 s (%.0f deg commanded, %.0f m/s): it leans %.0f deg at most, %.1f more than it believes", want, now.speed, now.max_tilt, now.max_err);
      CHECK(open.max_err > now.max_err + 2.0f, "the lean holds its height with %.2f g of thrust: the accelerometer counts less for that. Counted fully, the error would be %.1f deg instead of %.1f", 1.0f / cosf(now.max_tilt * kDeg2Rad), open.max_err, now.max_err); }
    // a gyro that drifts as the board warms up: 0.4 deg/s in a minute
    { auto warm = [](float ki) { Rig r; r.core.cfg.ahrs.ki_acc = ki; r.core.configure(); r.baro = true; r.range = true; r.h.baro_ok = true;
          Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.6f, p); p.thr = 0.5f; r.run(5.0f, p);
          for (int i = 0; i < 600; i++) { r.gyro_bias[0] = 0.4f * (i + 1) / 600.0f; r.gyro_bias[1] = -0.3f * (i + 1) / 600.0f; r.run(0.1f, p); }
          r.run(30.0f, p); return std::pair<float, float>(hypotf(r.pl.right_down_deg(), r.pl.nose_down_deg()), r.core.ahrs().gyro_bias_dps().x); };
      const auto with = warm(Ahrs::Config().ki_acc), without = warm(0.0f);
      CHECK(with.first < 0.5f && fabsf(with.second - 0.4f) < 0.1f && without.first > 1.3f,
            "the gyro drifts 0.4 / 0.3 deg/s while hovering: the bias is learned (%.2f deg/s), the craft hovers %.2f deg from level; without the integral it would hang %.1f deg off and drift away", with.second, with.first, without.first); }
    // on the ground, disarmed, the reading IS gravity: believed ten times as much
    { Rig r; Pilot p; r.run(1.0f, p); r.pl.set_tilt(12, -7); r.run(1.0f, p);
      CHECK(fabsf(r.out.est.att.roll - 12.0f) < 1.5f && fabsf(r.out.est.att.pitch + 7.0f) < 1.0f, "disarmed and put down on a slope: the horizon follows within a second (%.1f / %.1f for 12 / -7)", r.out.est.att.roll, r.out.est.att.pitch); }
    { Rig r; r.core.cfg.ahrs_ground.disarmed = 1.0f; r.core.configure(); Pilot p; r.run(1.0f, p); r.pl.set_tilt(12, -7); r.run(1.0f, p);
      CHECK(r.out.est.att.roll > 1.5f && r.out.est.att.roll < 4.0f, "(the factor is a setting: with 1 instead of 10 it has covered %.1f of the 12 deg after that second)", r.out.est.att.roll); }
    // ... but only when it lies still. In the hand, carried about, the reading is as wrong as in flight
    { const float s10 = sinf(10 * kDeg2Rad), c10 = cosf(10 * kDeg2Rad);
      struct Hand { FlightCore c; uint32_t us = 0; Hand() { c.configure(); }
          CoreOutput feed(float dur, Vec3 gyro, Vec3 acc) { FastInput in; in.dt = 0.00025f; in.rc_link = true; in.health.imu_ok = in.health.gyro_calibrated = true; for (auto& ch : in.rc_us) ch = 1500; in.rc_us[2] = in.rc_us[4] = 1000;
              CoreOutput o; for (int i = 0; i < (int)(dur / 0.00025f + 0.5f); i++) { us += 250; in.now_ms = us / 1000; in.gyro_dps = gyro; in.acc_g = acc; o = c.update(in); } return o; } };
      Hand still; still.feed(2.0f, {0, 0, 0}, {0, 0, -1}); const float lying = still.feed(0.5f, {0, 0, 0}, {0, -s10, -c10}).est.att.roll;
      Hand carried; carried.feed(2.0f, {0, 0, 0}, {0, 0, -1}); const float turning = carried.feed(0.5f, {0, 0, 30}, {0, -s10, -c10}).est.att.roll;
      CHECK(lying > 6.0f && turning > 0.3f && turning < 2.5f, "a reading 10 deg off for half a second: lying still it is believed (%.1f deg), carried round at 30 deg/s hardly (%.1f deg)", lying, turning);
      Hand bumped; bumped.feed(2.0f, {0, 0, 0}, {0, 0, -1}); bumped.feed(0.05f, {0, 0, 0}, {0, 0, -1.4f}); const float after_bump = bumped.feed(0.2f, {0, 0, 0}, {0, -s10, -c10}).est.att.roll;
      Hand calm; calm.feed(2.0f, {0, 0, 0}, {0, 0, -1}); calm.feed(0.05f, {0, 0, 0}, {0, 0, -1}); const float no_bump = calm.feed(0.2f, {0, 0, 0}, {0, -s10, -c10}).est.att.roll;
      CHECK(no_bump > 3.0f && after_bump < 1.2f, "after a knock (1.4 g for 50 ms) it has to lie still for a quarter second again before it is believed: %.1f deg in the next 0.2 s instead of %.1f", after_bump, no_bump);
      Hand swung; swung.feed(1.0f, {0, 0, 0}, {0, 0, -1}); swung.feed(2.0f, {0, 0, 30}, {0, 0, -1.1f}); Hand flat; flat.feed(1.0f, {0, 0, 0}, {0, 0, -1}); flat.feed(2.0f, {0, 0, 0}, {0, 0, -1.06f});
      CHECK(fabsf(swung.c.ahrs().rest_g() - 1.0f) < 0.005f && fabsf(flat.c.ahrs().rest_g() - 1.06f) < 0.005f, "what the sensor reads \"at rest\" is learned lying still (1.06 g -> %.3f), not while it is swung about (1.1 g -> %.3f)", flat.c.ahrs().rest_g(), swung.c.ahrs().rest_g()); }
}

static void t_mode_details() {
    printf("[modes] entering, leaving, re-arming; what each mode hands to the rate loop\n");
    // Stabilize switched on in the air starts from where the craft IS - also after the heading has changed in another mode
    { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, 0); p.yaw = 0.6f; r.run(1.0f, p); p.yaw = 0; r.run(0.5f, p); const float y0 = r.pl.yaw_deg(); p.mode = 2; r.run(1.0f, p);
      CHECK(fabsf(y0) > 60 && fabsf(wrap180(r.pl.yaw_deg() - y0)) < 2.0f, "turned %.0f deg in Acro, then Stabilize on: it keeps the heading it has now (%.2f deg of movement)", y0, wrap180(r.pl.yaw_deg() - y0)); }
    // armed in Stabilize, standing on the ground with the motors idling: the heading target follows the craft (carried, turned on its gear)
    { Rig r; r.pl.make_five_inch(); Pilot p; p.mode = 2; r.run(0.3f, p); p.arm = true; r.run(0.2f, p); r.pl.w[2] = 1.0f; r.run(0.7f, p); r.pl.w[2] = 0; r.run(0.1f, p); const float y0 = r.pl.yaw_deg();
      p.thr = r.pl.hover_cmd(); r.run(1.0f, p);
      CHECK(r.out.armed && fabsf(y0) > 30 && fabsf(wrap180(r.pl.yaw_deg() - y0)) < 2.0f, "turned %.0f deg by hand before take-off: no swing back to the old heading when the throttle comes up (%.2f deg)", y0, wrap180(r.pl.yaw_deg() - y0)); }
    // back into Angle from Acro: it starts from the attitude of that moment, not from where it left off
    { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, 1); p.roll = 1.0f; r.run(0.05f, p); const float left_with = r.out.sp_dps[0];
      p.mode = 0; p.roll = 0; r.run(0.6f, p); const float tilt = r.out.est.att.roll;
      p.mode = 1; int n = 0; while (!((r.out.modes >> M_ANGLE) & 1u) && n++ < 100) r.run(0.00025f, p);
      CHECK(left_with > 100 && tilt > 5 && n < 100 && fabsf(r.out.sp_dps[0] + 5.0f * tilt) < 0.25f * 5.0f * tilt,
            "Angle left in the middle of a roll (asking %.0f dps), re-entered %.0f deg tilted: the first cycle asks %.0f dps (5 x the tilt, back to level) - nothing of the old move is left in its filters", left_with, tilt, r.out.sp_dps[0]); }
    // Altitude Hold levels the craft whatever the mode switch says
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; Pilot p; p.mode = 0; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.6f, p); p.thr = 0.5f; r.run(6.0f, p);
      p.roll = 0.5f; r.run(1.5f, p); const float want = angle_cmd(r, ROLL, 0.5f);
      CHECK(r.out.ah_state == AhState::FLYING && fabsf(r.out.est.att.roll - want) < 1.5f && r.pl.right_down_deg() > want - 1.5f && r.pl.right_down_deg() < want + 4.0f,
            "Altitude Hold with the mode switch on Acro: the roll stick gives a tilt (%.1f deg, want %.1f), not a roll rate", r.pl.right_down_deg(), want); }
    // ... and so does the failsafe: the link is lost in Acro with the craft banked
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; Pilot p; p.mode = 0; r.run(1.0f, p); p.arm = true; r.run(0.2f, p); p.thr = 0.62f; r.run(1.0f, p); p.thr = 0.47f; p.roll = 0.3f; r.run(0.25f, p); p.roll = 0; r.run(0.3f, p);
      const float tilt = r.pl.right_down_deg(); p.link = false; r.run(2.5f, p);
      CHECK(fabsf(tilt) > 12 && r.out.failsafe == FsStage::DESCENT && fabsf(r.pl.right_down_deg()) < 4 && fabsf(r.pl.nose_down_deg()) < 4, "link lost in Acro while banked %.0f deg: the failsafe levels the craft (%.1f / %.1f deg) before it descends", tilt, r.pl.right_down_deg(), r.pl.nose_down_deg()); }
    // Angle: the stick's speed is already in the angle target - the rate loop gets no stick feedforward for roll, yaw keeps its own
    { auto ff_peaks = [](int mode, float& roll_f, float& yaw_f) { Rig r; r.pl.make_five_inch(); Pilot p; fly_up(r, p, mode); roll_f = yaw_f = 0;
          for (int i = 0; i < 150; i++) { p.roll = p.yaw = 0.5f * fminf(i / 100.0f, 1.0f); r.run(0.001f, p); roll_f = fmaxf(roll_f, fabsf(r.out.pid_f[0])); yaw_f = fmaxf(yaw_f, fabsf(r.out.pid_f[2])); } };
      float ar, ay, cr, cy; ff_peaks(1, ar, ay); ff_peaks(0, cr, cy);
      CHECK(ar == 0 && ay > 3 && cr > 3 && cy > 3, "a stick move in Angle: F term of roll %.1f, of yaw %.1f (in Acro: %.1f / %.1f)", ar, ay, cr, cy); }
    // I-term relax is five times as eager while a mode levels the craft (a craft held in the hand: the setpoint moves, the craft does not)
    { auto wound = [](int mode) { Rig r; r.pl.Ixx = r.pl.Iyy = r.pl.Izz = 1e6f; Pilot p; p.mode = mode; r.run(0.3f, p); p.arm = true; r.run(0.1f, p); p.thr = 0.47f; r.run(0.3f, p);
          for (int i = 0; i < 150; i++) { p.roll = 0.8f * (i + 1) / 150.0f; r.run(0.001f, p); } return fabsf(r.out.pid_i[0]); };
      const float ang = wound(1), acro = wound(0);
      CHECK(ang < 0.25f * acro && acro > 40, "the roll stick pushed over in 150 ms with the craft held still: the I-term winds up to %.0f in Acro, only to %.0f in Angle", acro, ang); }
    // disarm and arm again: nothing of the last flight is left in the rate loop or in the throttle boost
    { auto frozen = [](Rig& r) { r.pl.mass = 1e6f; r.pl.Ixx = r.pl.Iyy = r.pl.Izz = 1e6f; };                // (stands on the ground whatever the motors do)
      auto start = [](Rig& r) { Pilot p; p.arm = true; r.run(0.010f, p); p.thr = 0.5f; r.run(0.015f, p); float m = 0; for (int i = 0; i < 4; i++) m += r.out.motor[i] / 4; return m; };
      Rig a; frozen(a); { Pilot p; a.run(0.4f, p); } const float fresh = start(a);
      Rig b; frozen(b); Pilot p; b.run(0.3f, p); p.arm = true; b.run(0.1f, p); p.thr = 0.6f; p.roll = 0.3f; b.run(0.3f, p); const float i_flown = fabsf(b.out.pid_i[0]), tpa_flown = b.core.pid().tpa_factor();
      p.arm = false; b.run(0.05f, p); const float i_after = fabsf(b.core.pid().i_term[0]), tpa_after = b.core.pid().tpa_factor(); p = Pilot(); b.run(0.05f, p); const float again = start(b);
      CHECK(i_flown > 20 && i_after == 0 && tpa_flown < 0.9f && fabsf(tpa_after - 0.8f) < 1e-4f, "disarmed: the I-term is gone (%.0f -> %.0f) and the low-throttle TPA is armed again (%.2f in flight, %.2f now)", i_flown, i_after, tpa_flown, tpa_after);
      CHECK(fabsf(again - fresh) < 2e-3f && fresh > 0.3f, "armed again with the throttle straight up: the motors get what a freshly booted board gives (%.3f / %.3f) - the boost remembers nothing", again, fresh); }
}

// Airmode: full authority at zero throttle - but only once the throttle has been raised after arming.
static void takeoff_to_hover(Rig& r);
static void t_airmode() {
    printf("[ground / air / airmode] what the motors do with the throttle stick at the bottom\n");
    auto spread = [](const Rig& r) { float lo = 1, hi = 0; for (int i = 0; i < 4; i++) { lo = fminf(lo, r.out.motor[i]); hi = fmaxf(hi, r.out.motor[i]); } return hi - lo; };
    auto all_idle = [](const Rig& r) { bool ok = true; for (int i = 0; i < 4; i++) ok &= r.out.motor[i] == r.lim.idle; return ok; };
    // ---- on the ground
    { Rig r; arm(r, 0); Pilot p; p.arm = true; p.roll = 0.3f; r.run(0.4f, p);
      CHECK(!r.out.airmode_active && !r.out.airborne && r.out.ground_idle && all_idle(r) && r.out.pid_i[0] == 0.0f, "just armed, throttle down, stick moved: every motor exactly at idle, no I-term");
      Rig a; arm(a, 1); p = Pilot(); p.arm = true; p.mode = 1; a.pl.set_tilt(8, 0); a.pl.ground_holds = true; a.run(3.0f, p);
      CHECK(all_idle(a) && a.out.pid_i[0] == 0.0f && fabsf(a.out.sp_dps[0]) > 20.0f, "Angle, standing 8 deg crooked: the levelling asks for %.0f deg/s and nothing happens - the motors stay at idle", a.out.sp_dps[0]); }
    // ---- in the air (default: airmode AUTO)
    { Rig r; arm(r, 0); Pilot p = armed_pilot(0); r.run(0.5f, p); p.thr = 0; p.roll = 0.3f; r.run(0.3f, p);
      const float want = apply_actual_rate(r.core.cfg.rates, ROLL, 0.3f);
      CHECK(r.out.airmode_active && r.out.airborne && !r.out.ground_idle && spread(r) > 0.03f && fabsf(r.pl.w[0] * kRad2Deg - want) < 0.25f * want, "after a flight at 47 %% throttle: stick to zero in the air, the craft still follows the roll stick (%.0f of %.0f dps)", r.pl.w[0] * kRad2Deg, want);
      p.roll = 0; bool kept = true; for (int i = 0; i < 20; i++) { r.run(0.1f, p); kept &= r.out.airmode_active || r.pl.ground; }
      CHECK(kept, "and stays stabilised all the way down (falling is not standing: the accelerometer does not read 1 g)"); }
    // more than a second of falling with the throttle down and no altimeter: only the accelerometer tells that from standing still
    { Rig r; arm(r, 1); Pilot p = armed_pilot(1); p.thr = 0.9f; r.run(1.6f, p); const float z0 = r.pl.z; p.thr = 0;
      bool kept = true; float t_air = 0;
      for (int i = 0; i < 60 && !r.pl.ground; i++) { r.run(0.05f, p); if (!r.pl.ground) { kept &= r.out.airborne && r.out.airmode_active; t_air += 0.05f; } }
      CHECK(z0 > 8.0f && t_air > 1.5f && kept && !r.out.est.alt_valid, "thrown up to %.0f m, throttle down for %.1f s of free flight without an altimeter: never taken for landed", z0, t_air); }
    // sinking steadily with thrust = weight (a strong craft whose idle carries it): 1 g, no turning, throttle down - the altimeter's sink rate tells
    { Rig r; r.baro = true; r.h.baro_ok = true; arm(r, 1); Pilot p = armed_pilot(1); p.thr = 0.9f; r.run(1.3f, p); p.thr = 0;
      while (r.pl.vz > -3.0f && !r.pl.ground) r.run(0.02f, p);                          // let it fall until it sinks with 3 m/s
      r.lim.idle = r.pl.hover_cmd();                                                     // from here on the idle carries the craft
      bool kept = true, one_g = true; const float z1 = r.pl.z;
      for (int i = 0; i < 30 && !r.pl.ground; i++) { r.run(0.05f, p); if (!r.pl.ground) { kept &= r.out.airborne; if (i >= 3) one_g &= fabsf(r.pl.acc().z + 1.0f) < 0.02f; } }      // (the first 0.15 s the motors are still spinning up)
      CHECK(z1 > 6.0f && one_g && kept && fabsf(r.pl.vz + 3.0f) < 0.4f && r.out.est.alt_valid, "sinking with %.1f m/s at exactly 1 g for 1.5 s (from %.0f m): not landed while the altimeter sees it sink", -r.pl.vz, z1); }
    // a strong craft hovers far below the 25 % that count at once: a second of any throttle counts, too
    { Rig r; r.pl.make_five_inch(); arm(r, 1); Pilot p = armed_pilot(1); p.thr = (r.pl.hover_cmd() - r.lim.idle) / (1 - r.lim.idle) * 1.15f; r.run(0.6f, p); const bool early = r.out.airborne; r.run(0.6f, p);
      CHECK(p.thr < 0.2f && !early && r.out.airborne && r.pl.z > 0.1f, "5-inch, taking off at %.0f %% throttle: in the air after 1 s of throttle (not after 0.6 s)", p.thr * 100);
      p.thr = 0; p.roll = 0.4f; r.run(0.25f, p);
      CHECK(r.out.airmode_active && fabsf(r.out.est.att.roll) > 3.0f, "throttle chopped: still flown by the stick (%.1f deg after 0.25 s)", r.out.est.att.roll); }
    // ---- back on the ground / on the bench: the craft cannot answer the rate loop
    { auto bench = [&](uint8_t mode, float& spread_1s, float& spread_30s, bool& idle_30s, float& i_30s) { Rig r; r.core.cfg.airmode = mode; r.core.configure(); r.pl.Ixx = r.pl.Iyy = r.pl.Izz = 1e6f; r.gyro_bias[0] = 0.3f; r.gyro_bias[2] = -0.2f;
          arm(r, 0); Pilot p = armed_pilot(0); p.thr = 0.30f; r.run(1.5f, p); p.thr = 0; r.run(0.9f, p); spread_1s = spread(r); r.run(29.0f, p); spread_30s = spread(r); idle_30s = all_idle(r) && r.out.ground_idle; i_30s = fabsf(r.out.pid_i[0]) + fabsf(r.out.pid_i[2]); return r.pl.z; };
      float s1, s30, i30; bool idle; const float z = bench(AM_AUTO, s1, s30, idle, i30);
      CHECK(z == 0.0f && idle && s30 == 0.0f && i30 == 0.0f, "bench, no props (gyro 0.3 deg/s off): throttle up to 30 %% and back to zero - one second later every motor is at idle and stays there (I-terms %.0f)", i30);
      float b1, b30, bi; bool bidle; bench(AM_ALWAYS, b1, b30, bidle, bi);
      CHECK(!bidle && b30 > 0.3f && b30 > 3.0f * b1 && bi > 200.0f, "the same with airmode ALWAYS (Betaflight's way): 30 s later the motors have run %.0f %% apart by themselves (%.0f %% after 1 s), I-terms at %.0f - only a disarm ends it", b30 * 100, b1 * 100, bi);
      float o1, o30, oi; bool oidle; bench(AM_OFF, o1, o30, oidle, oi);
      CHECK(oidle && o1 == 0.0f && oi == 0.0f, "airmode OFF: at idle the moment the stick is down"); }
    // landed after a flight: the same second, then idle
    { Rig r; r.pl.ground_holds = true; arm(r, 1); Pilot p = armed_pilot(1); p.thr = 0.56f; r.run(1.2f, p); p.thr = 0.40f; for (int i = 0; i < 200 && !r.pl.ground; i++) r.run(0.05f, p);
      p.thr = 0; r.run(0.5f, p); const bool still_air = r.out.airborne; r.run(1.0f, p);
      CHECK(r.pl.ground && still_air && !r.out.airborne && r.out.ground_idle && all_idle(r), "flown, landed, throttle down: taken for standing after a second - motors at idle, nothing winds up on the ground");
      p.thr = 0.3f; r.run(0.05f, p); CHECK(!r.out.ground_idle && r.out.pid_i[0] == r.out.pid_i[0], "throttle up again: the rate loop is back at once"); }
    // ---- airmode OFF in the air
    { Rig r; r.core.cfg.airmode = AM_OFF; r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); r.run(0.5f, p); p.thr = 0; p.roll = 0.3f; r.run(0.3f, p);
      CHECK(!r.out.airmode_active && r.out.airborne && all_idle(r), "airmode OFF: stick at zero in the air = all motors at idle, no control (ArduCopter's Stabilize)");
      p.thr = 0.02f; r.run(0.05f, p); CHECK(spread(r) > 0.03f, "2 %% of throttle is enough to have the rate loop back (%.0f %% apart)", spread(r) * 100); }
    // ---- the throttle that counts as flying at once
    { Rig r; r.core.cfg.airmode_threshold = 0.6f; r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); r.run(0.3f, p); CHECK(!r.out.airborne, "threshold 60 %%: 47 %% throttle for 0.3 s is not yet 'in the air'"); p.thr = 0.65f; r.run(0.05f, p); CHECK(r.out.airborne, "65 %% is, at once");
      p.arm = false; r.run(0.1f, p); p.thr = 0; r.run(0.1f, p); p.arm = true; r.run(0.1f, p); CHECK(r.out.armed && !r.out.airborne && !r.out.airmode_active, "and it is forgotten with the disarm"); }
    // ---- Altitude Hold hands the craft back
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; r.run(0.5f, p);
      p.althold = false; p.thr = 0; p.roll = 0.3f; r.run(0.25f, p);
      CHECK(r.out.airborne && r.out.airmode_active && !r.out.ground_idle && fabsf(r.out.est.att.roll) > 2.0f, "Altitude Hold switched off in a hover with the throttle stick at the bottom: the craft knows it flies (stabilised, follows the stick: %.1f deg)", r.out.est.att.roll);
      Rig g; g.baro = true; g.range = true; g.h.baro_ok = true; takeoff_to_hover(g); p = Pilot(); p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.0f; for (int i = 0; i < 400 && g.out.ah_state != AhState::LANDED_IDLE; i++) g.run(0.05f, p);
      const bool landed = g.out.ah_state == AhState::LANDED_IDLE && !g.out.airborne && g.out.ground_idle; p.althold = false; p.roll = 0.3f; g.run(0.1f, p);
      CHECK(landed && !g.out.airborne && g.out.ground_idle && all_idle(g), "landed by Altitude Hold, then switched off with the stick still down: on the ground at once, motors at idle"); }
    // ---- Altitude Hold / failsafe taking over a craft that flies with the stick low
    { Rig r; r.pl.make_five_inch(); r.baro = true; r.h.baro_ok = true; arm(r, 1); Pilot p = armed_pilot(1); const float hov = (r.pl.hover_cmd() - r.lim.idle) / (1 - r.lim.idle);
      p.thr = hov * 1.4f; r.run(1.5f, p); p.thr = hov; r.run(2.0f, p); const float z0 = r.pl.z, stick_us = 1050 + 950 * hov;
      CHECK(z0 > 1.0f && stick_us < 1150.0f && r.out.airborne, "5-inch hovering at %.1f m with the throttle stick at %.0f us (below 15 %%)", z0, stick_us);
      Rig f = r; Pilot pf = p;
      p.althold = true; p.thr = 0.5f; r.run(0.02f, p); const AhState st = r.out.ah_state; float lo = 1e9f; for (int i = 0; i < 40; i++) { r.run(0.05f, p); lo = fminf(lo, r.pl.z); }
      CHECK(st == AhState::FLYING && r.out.armed && lo > z0 - 0.6f && fabsf(r.pl.vz) < 0.5f, "Altitude Hold switched on: it takes over in flight (not 'parked'), lowest point %.2f m of %.2f", lo, z0);
      pf.link = false; f.run(1.0f, pf); const bool descent = f.out.failsafe == FsStage::DESCENT && f.out.armed && f.out.althold_active && f.out.ah_state == AhState::FLYING; const float z1 = f.pl.z; f.run(1.5f, pf);
      CHECK(descent && f.out.armed && f.pl.z < z1 && f.pl.z > 0.2f && f.pl.vz < -0.1f && f.pl.vz > -2.5f, "link lost in that hover: a controlled descent (%.2f m -> %.2f m, %.1f m/s) - it used to be disarmed in the air", z1, f.pl.z, f.pl.vz); }
}

static void t_compass() {
    printf("[compass] yaw from the magnetometer inside the full core\n");
    { Rig r(0, true); r.pl.set_tilt(0, 0, 70); r.run(1.0f, Pilot());
      double ye = r.out.est.att.yaw - 70; CHECK(fabs(wrap180((float)ye)) < 1.5, "heading 70 deg acquired while still disarmed (seeded from the compass): %.2f", r.out.est.att.yaw);
      CHECK(r.out.est.yaw_from_mag, "reported as compass-assisted"); }
    { Rig r(0, true); r.declination = 3.0f; r.core.cfg.declination_deg = 3.0f; r.pl.set_tilt(10, -5, -40); r.run(2.0f, Pilot());
      CHECK(fabsf(wrap180(r.out.est.att.yaw + 40)) < 1.5f, "true heading -40 with 3 deg east declination configured: %.2f", r.out.est.att.yaw); }
    { Rig r(0, false); r.mag = true; r.h.mag_ok = true; r.pl.set_tilt(0, 0, 70); r.run(2.0f, Pilot()); CHECK(fabsf(r.out.est.att.yaw) < 3.0f && !r.out.est.yaw_from_mag, "compass disabled in the settings: it is ignored even if the sensor keeps reporting (yaw %.2f)", r.out.est.att.yaw); }
    { Rig r(0, false); r.pl.set_tilt(0, 0, 70); r.run(1.0f, Pilot()); CHECK(fabsf(r.out.est.att.yaw) < 3.0f, "compass off: yaw starts at 0 (%.2f) - nothing absolute without it", r.out.est.att.yaw); }
    { Rig r(0, true); r.pl.set_tilt(0, 0, 20); r.run(0.5f, Pilot()); arm(r, 1); Pilot p = armed_pilot(1); p.yaw = 0.5f; r.run(2.0f, p);
      const float y = r.pl.yaw_deg(); CHECK(fabsf(wrap180(r.out.est.att.yaw - y)) < 4.0f, "tracks a yaw spin with the compass active: estimate %.1f vs truth %.1f", r.out.est.att.yaw, y); }
}

static void takeoff_to_hover(Rig& r) {
    r.baro = true; r.range = true; r.h.baro_ok = true;
    Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p);                                 // sensors settle while disarmed
    p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.6f, p); p.thr = 0.5f; r.run(7.0f, p);
}

static void nav_takeoff(Rig& r, Pilot& p, float z) {
    r.baro = true; r.h.baro_ok = true; p = Pilot(); p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p);
    p.thr = 0.8f; for (int i = 0; i < 600 && r.pl.z < z - 0.4f; i++) r.run(0.05f, p);
    p.thr = 0.5f; r.run(3.0f, p);
}
static void t_althold() {
    printf("[althold] baro + lidar through the full core (baro drifting 3 cm/s)\n");
    Rig r; r.drift_rate = 0.03f; Pilot p; p.mode = 1; p.althold = true; r.baro = true; r.range = true; r.h.baro_ok = true;
    r.run(1.0f, p); p.arm = true; r.run(0.4f, p);
    CHECK(r.out.armed && r.out.ah_state == AhState::LANDED_IDLE && r.pl.z == 0 && r.out.althold_active, "armed on the ground in AltHold: idle, no lift, no ghost climb (state %d)", (int)r.out.ah_state);
    p.thr = 0.8f; r.run(0.6f, p); CHECK(r.out.ah_state == AhState::TAKEOFF || r.out.ah_state == AhState::FLYING, "takeoff started (state %d)", (int)r.out.ah_state);
    p.thr = 0.5f; r.run(7.0f, p);
    CHECK(r.out.ah_state == AhState::FLYING && fabsf(r.pl.z - 0.6f) < 0.2f, "takeoff altitude %.2f (want ~0.6), state %d", r.pl.z, (int)r.out.ah_state);
    CHECK(fabsf(r.out.est.z - r.pl.z) < 0.15f && r.out.est.alt_valid, "estimate follows the truth although the baro drifts: est %.3f true %.3f", r.out.est.z, r.pl.z);
    r.run(6.0f, p); const float z0 = r.pl.z; float dev = 0; for (int k = 0; k < 8; k++) { r.run(1.0f, p); dev = fmaxf(dev, fabsf(r.pl.z - z0)); }
    CHECK(dev < 0.12f, "hold: max deviation %.3f m over 8 s with a drifting baro", dev);
    CHECK(fabsf(r.core.alt_hold().hover() - (0.5275f - 0.055f) / 0.945f) < 0.07f, "learned hover throttle %.3f", r.core.alt_hold().hover());
    const float zc = r.pl.z; p.thr = 0.75f; r.run(2.0f, p); CHECK(r.pl.z - zc > 1.0f && r.pl.z - zc < 2.6f, "stick above the deadband climbs: +%.2f m in 2 s", r.pl.z - zc);
    p.thr = 0.5f; r.run(4.0f, p); const float z1 = r.pl.z; r.run(2.0f, p); CHECK(fabsf(r.pl.z - z1) < 0.1f, "holds the new altitude");
    p.thr = 0.0f; r.run(14.0f, p); CHECK(r.out.ah_state == AhState::LANDED_IDLE && r.pl.z < 0.05f, "stick low: descends, lands, goes idle (state %d z %.3f)", (int)r.out.ah_state, r.pl.z);
    // the same Altitude Hold on a craft with seven times its weight in thrust (it hovers at 13 % motor output, not at 50): no setting changed
    { Rig q; q.pl.make_five_inch(); q.baro = true; q.range = true; q.h.baro_ok = true; Pilot s; s.mode = 1; s.althold = true; q.run(1.0f, s); s.arm = true; q.run(0.4f, s); s.thr = 0.8f;
      float peak = 0, vmax = 0; for (int k = 0; k < 100; k++) { if (k == 6) s.thr = 0.5f; q.run(0.1f, s); peak = fmaxf(peak, q.pl.z); vmax = fmaxf(vmax, q.pl.vz); }
      CHECK(q.out.ah_state == AhState::FLYING && fabsf(q.pl.z - 0.6f) < 0.2f && peak < 1.0f && vmax < 1.6f, "a 5-inch quad takes off just as calmly: %.2f m at most (target 0.6), %.2f m/s at most, then %.2f m", peak, vmax, q.pl.z);
      CHECK(fabsf(q.core.alt_hold().hover() - q.pl.hover_cmd()) < 0.012f && fabsf(q.out.hover_throttle - (q.pl.hover_cmd() - 0.055f) / 0.945f) < 0.012f, "its hover output is measured: %.3f (true %.3f) = %.1f %% throttle", q.core.alt_hold().hover(), q.pl.hover_cmd(), 100 * q.out.hover_throttle);
      const float zc = q.pl.z; s.thr = 0.75f; q.run(2.0f, s); CHECK(q.pl.z - zc > 1.0f && q.pl.z - zc < 2.6f, "climbs with the stick: +%.2f m in 2 s", q.pl.z - zc);
      s.thr = 0.5f; q.run(4.0f, s); const float z1 = q.pl.z; q.run(2.0f, s); CHECK(fabsf(q.pl.z - z1) < 0.1f, "holds the new altitude (%.2f -> %.2f)", z1, q.pl.z);
      s.thr = 0.25f; q.run(1.5f, s); CHECK(q.pl.vz < -0.3f && q.pl.vz > -1.3f, "and comes DOWN with the stick: %.2f m/s (a throttle that could not go below the old 40 %% guess would only climb)", q.pl.vz);
      s.thr = 0.0f; q.run(14.0f, s); CHECK(q.out.ah_state == AhState::LANDED_IDLE && q.pl.z < 0.05f, "lands and goes idle (state %d z %.3f)", (int)q.out.ah_state, q.pl.z); }
    // the hover output read at the moment of lift-off allows for how hard the craft is already accelerating
    { Rig q; q.pl.make_five_inch(); q.baro = true; q.range = true; q.h.baro_ok = true; Pilot s; s.mode = 1; s.althold = true; q.run(1.0f, s); s.arm = true; q.run(0.4f, s); s.thr = 0.8f;
      int n = 0; while (!q.core.alt_hold().lifted() && n++ < 4000) q.run(0.001f, s);
      CHECK(n < 4000 && fabsf(q.core.alt_hold().hover() / q.pl.hover_cmd() - 1.0f) < 0.12f, "lift-off seen after %.2f s of the throttle ramp, already climbing with %.1f m/s^2: hover output read as %.3f (true %.3f)", n * 0.001f, q.out.est.az, q.core.alt_hold().hover(), q.pl.hover_cmd()); }
    // the motor layer has raised the idle and limited the output (dynamic idle, output limit): Altitude Hold works in that range
    { Rig q; q.pl.make_five_inch(); q.lim.idle = 0.10f; q.lim.limit = 0.90f; q.baro = true; q.range = true; q.h.baro_ok = true; Pilot s; s.mode = 1; s.althold = true; q.run(1.0f, s); s.arm = true; q.run(0.4f, s); s.thr = 0.8f; q.run(0.6f, s); s.thr = 0.5f; q.run(8.0f, s);
      CHECK(q.out.ah_state == AhState::FLYING && fabsf(q.pl.z - 0.6f) < 0.2f && fabsf(q.core.alt_hold().hover() - q.pl.hover_cmd()) < 0.012f && fabsf(q.out.hover_throttle - (q.pl.hover_cmd() - 0.10f) / 0.80f) < 0.015f,
            "idle 10 %%, limit 90 %%: holds %.2f m, hover output %.3f (true %.3f) = %.1f %% of that throttle range", q.pl.z, q.core.alt_hold().hover(), q.pl.hover_cmd(), 100 * q.out.hover_throttle); }
    // barometer only (no lidar): the estimator must still give a usable altitude
    { Rig b; b.drift_rate = 0.0f; b.baro = true; b.range = false; b.h.baro_ok = true; Pilot q; q.mode = 1; q.althold = true; b.run(1.5f, q); q.arm = true; b.run(0.3f, q); q.thr = 0.8f; b.run(0.7f, q); q.thr = 0.5f; b.run(8.0f, q);
      CHECK(b.out.ah_state == AhState::FLYING && b.pl.z > 0.25f && b.pl.z < 1.0f && b.out.est.alt_valid && fabsf(b.out.est.z - b.pl.z) < 0.35f, "baro only: takes off and holds (z %.2f, estimate %.2f, state %d)", b.pl.z, b.out.est.z, (int)b.out.ah_state); }
    // no sensors at all: the switch is on but the core must not invent an altitude
    Rig n; Pilot q; q.mode = 1; q.althold = true; n.run(0.5f, q); q.arm = true; n.run(0.3f, q);
    CHECK(!n.out.armed && (n.out.arm_blockers & AB_ALTHOLD_NO_BARO), "AltHold requested without any altitude source never arms");
    // the height is lost in flight (the barometer stops answering) with the Altitude Hold switch on. The pilot holds the stick in
    // the middle, where "hold" is: the middle now means the measured hover throttle. (The stick's own curve gives 47 % there -
    // three and a half times what a 5-inch quad hovers at.)
    CHECK(FlightCore::hover_centred(0.0f, 0.2f) == 0.0f && FlightCore::hover_centred(0.5f, 0.2f) == 0.2f && FlightCore::hover_centred(1.0f, 0.2f) == 1.0f && fabsf(FlightCore::hover_centred(0.25f, 0.2f) - 0.1f) < 1e-6f &&
          fabsf(FlightCore::hover_centred(0.75f, 0.2f) - 0.6f) < 1e-6f && FlightCore::hover_centred(-1.0f, 0.2f) == 0.0f && FlightCore::hover_centred(2.0f, 0.2f) == 1.0f, "throttle centred on the hover: bottom 0, middle = hover, top 1, straight lines between");
    for (int racer = 0; racer < 2; racer++) { Rig b; if (racer) b.pl.make_five_inch(); takeoff_to_hover(b); const float z0 = b.pl.z, hov = b.out.hover_throttle; const char* who = racer ? "5-inch" : "heavy";
      b.baro = false; b.range = false; Pilot s; s.mode = 1; s.althold = true; s.arm = true; s.thr = 0.5f;
      float lo = z0, hi = z0, t_lo = 1, t_hi = 0, step = 0, prev = b.out.throttle; int n_un = 0;
      for (int k = 0; k < 600; k++) { b.run(0.005f, s); lo = fminf(lo, b.pl.z); hi = fmaxf(hi, b.pl.z); step = fmaxf(step, fabsf(b.out.throttle - prev)); prev = b.out.throttle;
          if (b.out.althold_unavailable) { n_un++; t_lo = fminf(t_lo, b.out.throttle); t_hi = fmaxf(t_hi, b.out.throttle); } }
      CHECK(b.out.althold_unavailable && !b.out.althold_active && b.out.armed && n_un > 300 && n_un < 420, "[%s] sensors lost in flight: Altitude Hold reports 'unavailable' once the last reading is a second old (%.2f s of the 3), still flying", who, n_un * 0.005f);
      CHECK(t_lo > hov - 0.03f && t_hi < hov + 0.03f && step < 0.03f && hi - z0 < 0.7f && z0 - lo < 0.7f, "[%s] stick in the middle: the throttle is the measured hover throttle (%.3f .. %.3f, hover %.3f) and never steps (%.3f at most in 5 ms) - the craft stays within %.2f / %.2f m without any height sensor",
            who, t_lo, t_hi, hov, step, hi - z0, z0 - lo);
      s.thr = 0.75f; b.run(0.05f, s); const float up = b.out.throttle; s.thr = 0.25f; b.run(0.05f, s); const float down = b.out.throttle; s.thr = 0.5f; b.run(0.3f, s);
      CHECK(fabsf(up - (hov + (1 - hov) * 0.5f)) < 0.03f && fabsf(down - hov * 0.5f) < 0.03f, "[%s] and the stick flies it: three quarters up = %.2f, a quarter = %.2f", who, up, down);
      b.baro = true; b.range = true; const float z1 = b.pl.z; b.run(0.4f, s); const bool again = b.out.althold_active && !b.out.althold_unavailable && b.out.ah_state == AhState::FLYING;
      lo = hi = z1; for (int k = 0; k < 50; k++) { b.run(0.1f, s); lo = fminf(lo, b.pl.z); hi = fmaxf(hi, b.pl.z); }
      // (0.28: taken over the way AC_PosControl does it - the target starts at the craft's climb rate and is stopped with the
      // limited acceleration, so a craft that was still climbing coasts on a little instead of being yanked back)
      CHECK(again && hi - z1 < 1.2f && z1 - lo < 0.8f && lo > 0.05f && fabsf(b.pl.vz) < 0.15f && b.out.ah_state == AhState::FLYING, "[%s] sensors back: Altitude Hold takes over from that throttle and holds (%.2f m then, %.2f .. %.2f after, vz %.2f)", who, z1, lo, hi, b.pl.vz); }
    // the throttle Altitude Hold commands goes to the motors as it is: no throttle boost on it, no anti-gravity from it
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; float ag = 0, off = 0, moved = 0, prev = 0;
      for (int k = 0; k < 4000; k++) { if (k == 700) p.thr = 0.5f; if (k == 2500) p.thr = 0.7f; r.run(0.001f, p); float mean = 0; for (int m = 0; m < 4; m++) mean += r.out.motor[m] * 0.25f;
          ag = fmaxf(ag, r.core.pid().anti_gravity()); off = fmaxf(off, fabsf(mean - (r.lim.idle + r.out.throttle * (r.lim.limit - r.lim.idle)))); moved = fmaxf(moved, fabsf(r.out.throttle - prev)); prev = r.out.throttle; }
      CHECK(r.out.althold_active && moved > 0.002f && ag == 0.0f && off < 0.005f, "a take-off and a climb in Altitude Hold (throttle moving up to %.3f per ms): anti-gravity stays at %.1f, the motors' mean is the commanded throttle to within %.5f", moved, ag, off);
      Rig m; Pilot q; fly_up(m, q, 1); q.thr = m.pl.hover_cmd() * 1.5f; float ag2 = 0, off2 = 0;
      for (int k = 0; k < 100; k++) { m.run(0.001f, q); float mean = 0; for (int i = 0; i < 4; i++) mean += m.out.motor[i] * 0.25f; ag2 = fmaxf(ag2, m.core.pid().anti_gravity()); off2 = fmaxf(off2, mean - (m.lim.idle + m.out.throttle * (m.lim.limit - m.lim.idle))); }
      CHECK(ag2 > 5.0f && off2 > 0.005f, "the pilot's own throttle jump gets both (anti-gravity %.0f, %.3f of boost)", ag2, off2); }
    // a table comes under the craft: the range finder suddenly reads 35 cm less
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; r.run(3.0f, p); const float z0 = r.pl.z; float lo = z0, hi = z0;
      r.ground_level = 0.35f; for (int k = 0; k < 80; k++) { r.run(0.1f, p); lo = fminf(lo, r.pl.z); hi = fmaxf(hi, r.pl.z); }
      CHECK(r.core.alt_estimator().range_reanchors() == 1 && fabsf(r.core.alt_estimator().range_ground() - 0.35f) < 0.08f && hi - z0 < 0.12f && z0 - lo < 0.12f && fabsf(r.out.est.z - r.pl.z) < 0.12f,
            "hovering at %.2f m, a table 35 cm high under it for 8 s: the ground level is moved (%.2f m), the craft is not (%.2f .. %.2f)", z0, r.core.alt_estimator().range_ground(), lo, hi);
      r.ground_level = 0.0f; for (int k = 0; k < 80; k++) { r.run(0.1f, p); lo = fminf(lo, r.pl.z); hi = fmaxf(hi, r.pl.z); }
      CHECK(r.core.alt_estimator().range_reanchors() == 2 && hi - z0 < 0.15f && z0 - lo < 0.15f, "and off the table again (%.2f .. %.2f of %.2f)", lo, hi, z0); }
    // a GPS whose altitude is 3 m off (they wander like that): it has no say while there is a barometer
    for (int lidar = 0; lidar < 2; lidar++) { Rig r; r.baro = true; r.range = lidar; r.h.baro_ok = true; Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.7f, p); p.thr = 0.5f; r.run(8.0f, p);
      const float z0 = r.pl.z; r.gps = true; r.gps_alt_err = 3.0f; float lo = z0, hi = z0; for (int k = 0; k < 300; k++) { r.run(0.1f, p); lo = fminf(lo, r.pl.z); hi = fmaxf(hi, r.pl.z); }
      CHECK(r.out.est.pos_valid && !r.core.alt_estimator().gps_used() && hi - z0 < 0.6f && z0 - lo < 0.45f && lo > 0.15f && r.out.ah_state == AhState::FLYING,
            "%s, GPS altitude 3 m too high for 30 s: not used, the craft holds (%.2f m, then %.2f .. %.2f; fused as a height it would have come down by two metres - into the ground)", lidar ? "baro + lidar" : "baro only", z0, lo, hi); }
    { Rig r; r.range = true; r.gps = true; r.gps_alt_err = 3.0f; Pilot p; fly_up(r, p, 1); r.run(1.0f, p);
      CHECK(r.pl.z > 0.1f && r.pl.z < 3.0f && !r.core.alt_estimator().gps_used() && r.core.alt_estimator().range_used() && r.out.est.alt_valid && fabsf(r.out.est.z - r.pl.z) < 0.15f, "lidar and GPS without a barometer, the GPS 3 m off: the lidar is the height (%.2f, true %.2f)", r.out.est.z, r.pl.z); }
    for (int mode = GA_OFF; mode <= GA_FALLBACK; mode++) { Rig r; r.core.cfg.gps_alt = (uint8_t)mode; r.gps = true; r.gps_alt_err = 0.0f; Pilot p; p.mode = 1; r.run(0.5f, p); p.arm = true; r.run(0.2f, p); p.thr = r.pl.hover_cmd() * 1.2f; r.run(2.5f, p);
      if (mode == GA_FALLBACK) CHECK(r.core.alt_estimator().gps_used() && r.out.est.alt_valid && r.out.est.gps_alt_used && fabsf(r.out.est.z - r.pl.z) < 1.5f, "a craft with GPS and nothing else (FALLBACK): its height is the height (%.2f, true %.2f) and counts as valid - Betaflight's altitude_source DEFAULT does the same", r.out.est.z, r.pl.z);
      else CHECK(!r.core.alt_estimator().gps_used() && !r.out.est.alt_valid && !r.out.est.gps_alt_used, "the same with the GPS height switched off (OFF): no height at all"); }
    // ---- the GPS height in Altitude Hold (nav/alt_estimator.h "GPS height"); the GPS here: 1 m of noise on each height, 150 ms late, vertical velocity 0.1 m/s
    { auto gps_only = [](uint8_t mode, bool& active_all, float& dev, bool& flags) { Rig r; r.core.cfg.gps_alt = mode; r.ngps = true; r.h.gps_ok = true; Pilot p; p.mode = 1; r.run(1.0f, p); p.arm = true; r.run(0.3f, p);
          p.thr = r.pl.hover_cmd() * 1.15f; for (int i = 0; i < 400 && r.pl.z < 5.0f; i++) r.run(0.05f, p);
          p.thr = 0.5f; p.althold = true; r.run(3.0f, p); const float z0 = r.pl.z; active_all = true; dev = 0;
          for (int k = 0; k < 400; k++) { r.run(0.05f, p); active_all &= r.out.althold_active; dev = fmaxf(dev, fabsf(r.pl.z - z0)); }
          flags = r.out.est.gps_alt_used && r.out.est.gps_vz_used; return r.out.ah_state; };
      bool all = false, fl = false; float dev = 0; const AhState st = gps_only(GA_FALLBACK, all, dev, fl);
      CHECK(all && st == AhState::FLYING && dev < 1.5f && fl, "GPS only (no barometer), Altitude Hold at 5 m for 20 s: held within %.2f m on the GPS height and vertical velocity", dev);
      gps_only(GA_OFF, all, dev, fl); CHECK(!all, "the same with the GPS height OFF: Altitude Hold has no height, the pilot keeps the throttle"); }
    // the barometer drifts 1 m a minute for 3 minutes (weather, the board warming up): BLEND takes it out, FALLBACK does not
    { auto drifting = [](uint8_t mode) { Rig r; r.core.cfg.gps_alt = mode; r.ngps = true; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 5.0f);
          const float z0 = r.pl.z; r.drift_rate = 1.0f / 60.0f; r.run(180.0f, p); return r.pl.z - z0; };
      const float fb = drifting(GA_FALLBACK), bl = drifting(GA_BLEND);
      CHECK(fb < -2.4f && fabsf(bl) < 1.0f, "Altitude Hold for 3 min with the barometer drifting 1 m/min: the craft sinks %.1f m with FALLBACK, moves %.2f m with BLEND", -fb, bl); }
    // the barometer goes silent in flight (a loose wire, the I2C bus hung): the GPS height takes over without a step, Altitude Hold carries on
    { Rig r; r.ngps = true; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 5.0f); r.run(10.0f, p); const float z0 = r.pl.z;
      r.baro = false; bool all = true, valid = true; float dev = 0;
      for (int k = 0; k < 400; k++) { r.run(0.05f, p); all &= r.out.althold_active; valid &= r.out.est.alt_valid; dev = fmaxf(dev, fabsf(r.pl.z - z0)); }
      CHECK(all && valid && dev < 1.5f && r.out.est.gps_alt_used && r.out.ah_state == AhState::FLYING, "barometer silent at 5 m: Altitude Hold never drops out, the height stays valid, the craft within %.2f m for 20 s on the GPS", dev); }
    // a NMEA module has no vertical velocity: nothing of it is fused (vd would read 0 - a craft climbing at 2 m/s would be told it hovers)
    { Rig r; r.ngps = true; r.gps_vd = false; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 3.0f);
      CHECK(!r.out.est.gps_vz_used && !r.core.alt_estimator().gps_vel_used(), "GPS without vertical velocity (NMEA): not used for the climb rate"); }
    // near the ground with the motors running the barometer reads wrong (the air is pushed against the ground): it counts for less there
    { auto dip = [](float ground_effect_s, int in_flight) { float sum = 0; for (int rep = 0; rep < 4; rep++) { Rig r; r.core.cfg.ground_effect_s = ground_effect_s; r.core.configure(); r.baro = true; r.h.baro_ok = true; Pilot p; p.mode = 1; r.run(1.5f, p);
              if (in_flight == 2) { r.drift -= 0.8f; r.run(0.25f, p); sum += -r.out.est.z; }               // disarmed: the air itself changed
              else if (!in_flight) { r.prop_wash_m = 0.8f; p.arm = true; r.run(0.25f, p); sum += -r.out.est.z; }
              else { p.arm = true; r.run(0.2f, p); p.thr = r.pl.hover_cmd() * 1.1f; r.run(1.0f, p); p.thr = r.pl.hover_cmd(); r.run(3.0f, p); const float e0 = r.out.est.z - r.pl.z; r.drift -= 0.8f; r.run(0.4f, p); sum += -(r.out.est.z - r.pl.z - e0); } }
          return sum / 4; };
      const float with = dip(1.5f, 0), without = dip(0.0f, 0), disarmed = dip(1.5f, 2);
      CHECK(without > 0.15f && with < 0.75f * without, "armed on the ground, the barometer drops by 0.8 m as the motors start: a quarter of a second later the estimate has followed by %.2f m (%.2f m if the barometer counted in full)", with, without);
      CHECK(disarmed > 1.5f * with, "disarmed there are no motors to disturb it: the same step counts in full (%.2f m)", disarmed);
      const float later = dip(1.5f, 1), always = dip(1000.0f, 1);
      CHECK(later > 0.4f && always < 0.9f * later, "four seconds into the flight it counts in full again: such a step moves the estimate by %.2f m in 0.4 s (%.2f m if it were still held back)", later, always); }
}

static void t_failsafe() {
    printf("[failsafe] link loss in AltHold: self-level + controlled descent + disarm\n");
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.8f; r.run(2.0f, p); p.thr = 0.5f; r.run(3.0f, p); const float z_top = r.pl.z;
      p.link = false; p.roll = 1.0f; p.pitch = -1.0f;
      r.run(0.3f, p); CHECK(r.out.failsafe == FsStage::GUARD && r.out.armed && z_top > 1.5f, "hovering at %.1f m, 0.3 s without link: guard, still armed", z_top);
      r.run(0.5f, p); CHECK(r.out.failsafe == FsStage::DESCENT, "descent after the guard time (stage %d)", (int)r.out.failsafe);
      r.run(0.8f, p); CHECK(fabsf(r.pl.right_down_deg()) < 5 && fabsf(r.pl.nose_down_deg()) < 5, "stale full-stick commands are ignored: craft level (%.1f, %.1f)", r.pl.right_down_deg(), r.pl.nose_down_deg());
      CHECK(r.out.fs_cause == FS_LINK && r.out.althold_active && r.pl.vz < -0.1f, "the cause is shown (LINK), it is coming down (%.2f m/s)", r.pl.vz);
      p.link = true; p.roll = 0; p.pitch = 0; r.run(0.3f, p); CHECK(r.out.failsafe == FsStage::DESCENT && r.out.fs_cause == FS_LINK && r.pl.z > 0.5f && r.pl.z < z_top - 0.2f, "link back for 0.3 s: not believed yet, the descent goes on (stage %d)", (int)r.out.failsafe);
      r.run(0.8f, p); const float z_back = r.pl.z; r.run(2.0f, p);
      CHECK(r.out.failsafe == FsStage::NONE && r.out.fs_cause == 0 && r.out.armed && r.out.ah_state == AhState::FLYING && fabsf(r.pl.z - z_back) < 0.35f && z_back > 0.3f,
            "a second of unbroken link: the pilot has the craft back - Altitude Hold holds where the descent had got to (%.2f m, from %.2f: the descent is stopped with the limited acceleration; stage %d)", r.pl.z, z_back, (int)r.out.failsafe);
      p.roll = 0.5f; r.run(1.0f, p); CHECK(fabsf(r.pl.right_down_deg() - angle_cmd(r, ROLL, 0.5f)) < 3.0f, "and the sticks are his again (roll %.1f deg)", r.pl.right_down_deg()); p.roll = 0;
      p.link = false; r.run(30.0f, p); CHECK(!r.out.armed && r.pl.z < 0.1f && r.out.failsafe == FsStage::LANDED, "lost for good: landed and disarmed (armed %d, z %.2f)", (int)r.out.armed, r.pl.z); }
    // the time limit is for a craft that does not come down - a descent that simply takes longer is never cut short in the air
    { Rig r; r.core.cfg.fs.max_descent_s = 2.0f; r.core.configure(); takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.8f; r.run(2.5f, p); p.thr = 0.5f; r.run(2.0f, p); const float z0 = r.pl.z;
      p.link = false; float z_off = -1, t_off = 0; for (int k = 0; k < 600 && z_off < 0; k++) { r.run(0.05f, p); if (!r.out.armed) { z_off = r.pl.z; t_off = k * 0.05f; } }
      CHECK(z0 > 2.0f && z_off >= 0 && z_off < 0.1f && t_off > 3.0f, "time limit 2 s, the descent from %.1f m takes %.1f s: the motors stay on until it stands (off at %.2f m)", z0, t_off, z_off); }
    // nobody at the stick: what the channel last said is not flown, from the first moment on
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; r.run(2.0f, p); const float z0 = r.pl.z;
      p.thr = 0.95f; p.link = false; r.run(0.45f, p);
      CHECK(r.out.failsafe == FsStage::GUARD && r.pl.z - z0 < 0.1f && fabsf(r.pl.vz) < 0.25f, "the link drops with the throttle channel reading 'full up': during the guard time the craft holds its height (%+.2f m, %.2f m/s) - it does not climb away", r.pl.z - z0, r.pl.vz); }
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.25f, p);
      const bool ramping = r.out.ah_state == AhState::TAKEOFF && r.out.throttle > 0.03f && r.pl.z == 0.0f; p.link = false; r.run(0.1f, p);
      CHECK(ramping && r.out.ah_state == AhState::LANDED_IDLE && r.out.throttle == 0.0f && r.out.armed, "the link drops while Altitude Hold is still raising the throttle for the take-off: idle at once, it stays on the ground");
      r.run(0.6f, p); CHECK(!r.out.armed && r.pl.z == 0.0f, "... and after the guard time the motors stop (z %.2f)", r.pl.z); }
    { Rig r; r.baro = true; r.range = true; r.h.baro_ok = true; Pilot p; p.mode = 1; p.althold = true; r.run(1.0f, p); p.arm = true; r.run(0.3f, p); p.thr = 0.8f; r.run(0.25f, p);
      p.link = false; r.run(0.2f, p); p.link = true; r.run(1.5f, p);
      CHECK(r.out.armed && r.out.failsafe == FsStage::NONE && r.out.ah_state == AhState::LANDED_IDLE && r.pl.z == 0.0f, "a 0.2 s dropout during that ramp: still armed, on the ground - the stick (still at 80 %%) has to come down before a new take-off");
      p.thr = 0.0f; r.run(0.2f, p); p.thr = 0.8f; r.run(0.6f, p); p.thr = 0.5f; r.run(6.0f, p); CHECK(r.out.ah_state == AhState::FLYING && fabsf(r.pl.z - 0.6f) < 0.2f, "down and up again: it takes off (%.2f m)", r.pl.z); }
    // the failsafe switch is a test of the lost link, ended by the pilot's hand
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; p.fs_sw = true; p.roll = 0.6f; r.run(1.5f, p);
      CHECK(r.out.failsafe == FsStage::DESCENT && r.out.fs_cause == FS_SWITCH && fabsf(r.pl.right_down_deg()) < 4.0f, "switch on: descent, cause SWITCH, the roll stick is ignored (%.1f deg)", r.pl.right_down_deg());
      p.fs_sw = false; r.run(0.05f, p); CHECK(r.out.failsafe == FsStage::NONE && r.out.fs_cause == 0, "switch off: over at once"); r.run(1.0f, p);
      CHECK(fabsf(r.pl.right_down_deg() - angle_cmd(r, ROLL, 0.6f)) < 3.0f && r.out.armed, "and the stick counts again (%.1f deg)", r.pl.right_down_deg()); }
    // the battery is at its minimum: the craft comes down, the pilot still chooses where
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; r.run(4.0f, p); const float z0 = r.pl.z;
      r.h.batt_critical = true; r.run(0.3f, p); const bool guard = r.out.failsafe == FsStage::GUARD && r.out.fs_cause == FS_BATTERY; r.run(0.4f, p);
      CHECK(guard && r.out.failsafe == FsStage::DESCENT && r.out.fs_cause == FS_BATTERY, "battery critical: guard time, then the descent (cause BATTERY)");
      p.roll = 0.5f; p.thr = 0.9f; r.run(0.8f, p);
      CHECK(fabsf(r.pl.right_down_deg() - angle_cmd(r, ROLL, 0.5f)) < 3.0f && r.pl.vz < -0.1f && r.pl.z < z0, "roll, pitch and yaw stay with the pilot (%.1f deg) - the throttle does not: stick at 90 %%, sinking %.2f m/s", r.pl.right_down_deg(), r.pl.vz);
      p.roll = 0; r.h.batt_critical = false; r.run(1.0f, p);
      CHECK(r.out.failsafe == FsStage::DESCENT && r.out.fs_cause == FS_BATTERY, "the voltage recovers as the motors are unloaded: the landing is not called off");
      r.run(30.0f, p); CHECK(!r.out.armed && r.pl.z < 0.05f && (r.out.arm_blockers & AB_ARM_SWITCH_ON), "landed, disarmed"); }
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; const float z0 = r.pl.z; p.link = false; r.run(0.3f, p); p.link = true; r.run(2.0f, p);
      CHECK(r.out.failsafe == FsStage::NONE && fabsf(r.pl.z - z0) < 0.2f, "a 0.3 s glitch recovers silently, no altitude change"); }
    { Rig r; takeoff_to_hover(r); Pilot p; p.mode = 1; p.althold = true; p.arm = true; p.thr = 0.5f; p.fs_sw = true; r.run(1.5f, p); CHECK(r.out.failsafe != FsStage::NONE, "failsafe switch triggers it too"); }
    // armed on the ground when the link goes: there is nothing to land - motors off (not a "descent" at 85 % of the hover throttle)
    { Rig r; Pilot p; p.mode = 1; r.run(0.3f, p); p.arm = true; p.thr = 0.0f; r.run(0.2f, p); Pilot q = p; q.link = false; r.run(0.4f, q); const bool guard = r.out.armed && r.out.failsafe == FsStage::GUARD; r.run(0.3f, q);
      float top = 0; for (int i = 0; i < 4; i++) top = fmaxf(top, r.out.motor[i]);
      CHECK(guard && !r.out.armed && top == 0.0f && r.out.failsafe == FsStage::LANDED, "standing on the ground, link lost: the guard time, then disarmed - the motors never left idle");
      q.link = true; r.run(1.5f, q); CHECK(!r.out.armed && (r.out.arm_blockers & AB_ARM_SWITCH_ON), "link back: still disarmed until the switch is cycled");
      q.arm = false; r.run(0.2f, q); q.arm = true; r.run(0.2f, q); CHECK(r.out.armed && r.out.failsafe == FsStage::NONE, "cycled: armed again");
      Rig b; b.baro = true; b.h.baro_ok = true; p = Pilot(); p.mode = 1; p.althold = true; b.run(0.5f, p); p.arm = true; b.run(0.3f, p); p.link = false; b.run(0.8f, p);
      CHECK(!b.out.armed, "the same in Altitude Hold with a barometer"); }
    // in the air without any altitude source: a throttle descent, never a motor cut
    { Rig r; Pilot p; p.mode = 1; r.run(0.3f, p); p.arm = true; r.run(0.2f, p); p.thr = 0.6f; r.run(1.5f, p); Pilot q = p; q.link = false; r.run(1.0f, q);
      CHECK(r.out.failsafe == FsStage::DESCENT && r.out.armed && r.pl.z > 0.3f, "failsafe in the air without any altitude source: descent"); float low = 1; for (int i = 0; i < 100 && !r.pl.ground; i++) { r.run(0.05f, q); for (int k = 0; k < 4; k++) low = fminf(low, r.out.motor[k]); }
      CHECK(low > r.lim.idle + 0.1f, "the fallback is a throttle descent, never a motor cut (lowest motor %.2f on the way down)", low); }
    // ... and that descent is a DESCENT on every craft: the hover throttle it starts from was measured while the pilot flew by hand
    for (int racer = 0; racer < 2; racer++) { Rig r; if (racer) r.pl.make_five_inch(); Pilot p; p.mode = 1; r.run(0.3f, p); p.arm = true; r.run(0.2f, p);
      const float hover_thr = (r.pl.hover_cmd() - 0.055f) / 0.945f;
      p.thr = hover_thr * 1.15f; r.run(0.7f, p); p.thr = hover_thr; r.run(2.5f, p); const float z0 = r.pl.z;                // a manual take-off and a hover: no barometer, no lidar
      CHECK(z0 > 0.2f && fabsf(r.core.alt_hold().hover() - r.pl.hover_cmd()) < 0.015f, "[%s] flown by hand for 3 s: hover output measured as %.3f (true %.3f)", racer ? "5-inch" : "heavy", r.core.alt_hold().hover(), r.pl.hover_cmd());
      p.link = false; r.run(0.6f, p); CHECK(r.out.failsafe == FsStage::DESCENT, "link lost"); r.run(0.2f, p); const float v0 = r.pl.vz, h0 = r.core.alt_hold().hover(); r.run(1.5f, p);
      CHECK(r.core.alt_hold().hover() == h0, "[%s] the hover output is not 'measured' during that descent (%.4f -> %.4f): the craft sinks with the air carrying part of its weight, and a descent derived from a hover value that follows it would get faster and faster", racer ? "5-inch" : "heavy", h0, r.core.alt_hold().hover());
      const float want = r.core.cfg.fs.fallback_throttle_scale * r.pl.hover_cmd(), dv = r.pl.vz - v0, dv_want = -(1 - r.core.cfg.fs.fallback_throttle_scale) * kG0 * 1.5f;
      CHECK(fabsf(0.055f + r.out.throttle * 0.945f - want) < 0.015f && fabsf(dv - dv_want) < 0.5f, "[%s] failsafe without an altitude estimate: the motors go to %.0f %% of the hover output (%.3f) and the craft sinks - %.1f m/s gained downwards in 1.5 s (%.1f expected). It does not climb away",
            racer ? "5-inch" : "heavy", 100 * r.core.cfg.fs.fallback_throttle_scale, 0.055f + r.out.throttle * 0.945f, -dv, -dv_want); }
}

// The control cycles are not all the same length: one in eight takes seven times as long (the flight loop waited for something).
// Everything that integrates over time has to use the time that really passed.
static void t_uneven_cycles() {
    printf("[uneven cycles] one control cycle in eight is 1.75 ms long instead of 0.25\n");
    // (the barometer of this rig has 0.25 m of noise: one flight says little about a climb rate, so eight flights are averaged)
    auto climb = [](bool uneven, float& v_err, float& z_err, float& tilt) { Rig r; r.baro = true; r.h.baro_ok = true; if (uneven) r.slow_every = 8;
      Pilot p; fly_up(r, p, 1); r.run(2.0f, p); p.thr = r.pl.hover_cmd() * 1.35f; float worst = 0;
      for (int k = 0; k < 50; k++) { r.run(0.01f, p); worst = fmaxf(worst, fabsf(r.out.est.vz - r.pl.vz)); }                     // half a second of climbing hard
      v_err = worst; z_err = fabsf(r.out.est.z - r.pl.z); tilt = fmaxf(fabsf(r.pl.right_down_deg()), fabsf(r.pl.nose_down_deg())); };
    float err[2] = {0, 0}, zerr[2] = {0, 0}, tilt[2] = {0, 0};
    { KeepNoise keep; const int kFlights = 8;
      for (int seed = 1; seed <= kFlights; seed++) for (int uneven = 0; uneven < 2; uneven++) { rng.seed(seed); g_norm.reset(); float v, z, t; climb(uneven != 0, v, z, t); err[uneven] += v / kFlights; zerr[uneven] += z / kFlights; tilt[uneven] = fmaxf(tilt[uneven], t); } }
    CHECK(err[0] < 0.20f && zerr[0] < 0.10f, "even cycles, a 0.35 g climb for half a second, 8 flights: the climb rate estimate is off by %.2f m/s at most (average of the flights), the height by %.2f m at the end", err[0], zerr[0]);
    CHECK(err[1] < 0.20f && zerr[1] < 0.10f && tilt[1] < 3.0f, "uneven cycles: the same (%.2f m/s, %.2f m), level (%.1f deg) - with the estimator stepping by 'the last cycle times the number of readings' the acceleration would count for half the time it lasted", err[1], zerr[1], tilt[1]);
    { float v, z, t; climb(false, v, z, t); climb(true, v, z, t); CHECK(t < 3.0f && v < 0.45f, "(one more flight of each, with the noise the tests below have always had)"); }
    // each reading stands for the time since the one before: a reading that covers a long cycle counts for more
    { FlightCore c; c.configure(); FastInput in; in.gyro_dps = {0, 0, 0}; in.health.imu_ok = in.health.gyro_calibrated = true; CoreOutput o; float az = 0; int n = 0;
      for (int k = 0; k < 8 * 400; k++) { const bool slow = k % 8 == 3; in.dt = slow ? 0.00175f : 0.00025f; in.acc_g = {0, 0, slow ? -1.5f : -1.0f}; in.now_ms += slow ? 2 : (k % 4 == 0); o = c.update(in); if (k > 8 * 200) { az += o.est.az; n++; } }
      CHECK(fabsf(az / n - 0.25f * kG0) < 0.05f, "half of the time 0.5 g upwards, seen in one long cycle out of eight: the estimator's acceleration is 0.25 g (%.2f m/s^2), not the 1/16 g a plain mean of the eight readings gives", az / n); }
    { Rig r; r.slow_every = 8; Pilot p; fly_up(r, p, 0); p.roll = 0.3f; const float y0 = r.pl.right_down_deg(); r.run(0.5f, p); const float rate = apply_actual_rate(r.core.cfg.rates, ROLL, 0.3f);
      CHECK(fabsf(wrap180(r.out.est.att.roll - r.pl.right_down_deg())) < 2.0f && r.pl.right_down_deg() - y0 > 0.6f * rate * 0.5f, "a roll at %.0f deg/s through such cycles: the attitude estimate stays with the craft (%.1f vs %.1f deg)", rate, r.out.est.att.roll, r.pl.right_down_deg()); }
}

// The gyro rate and the control rate apart (CoreConfig::pid_denom, Betaflight pid_process_denom): the gyro filters see every
// sample, the attitude, the modes, the PID and the mixer run on every n-th one, and the motors hold their command in between.
static void t_pid_denom() {
    printf("[pid_denom] gyro at 4 kHz, control at 2 / 1 kHz\n");
    { Rig r; r.core.cfg.pid_denom = 2; r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); p.roll = 0.3f; r.run(0.2f, p);
      // step by hand: which samples are control cycles, and what the others give
      FastInput in; in.dt = r.dt; in.limits = r.lim; in.rc_link = true; in.health = r.h; in.now_ms = r.us / 1000; memcpy(in.rc_us, r.rc_us, sizeof(in.rc_us)); in.rc_seq = r.rc_seq; in.acc_g = {0, 0, -1};
      int ctl = 0, held = 0, fresh = 0; CoreOutput prev = r.core.update(in);
      for (int k = 0; k < 40; k++) {
          in.gyro_dps = {10.0f * (float)(k % 5), 0, 0};
          const CoreOutput o = r.core.update(in);
          if (o.pid_stepped) ctl++; else { bool same = true; for (int m = 0; m < 4; m++) same = same && o.motor[m] == prev.motor[m]; held += same; }
          fresh += o.gyro_f_dps[0] != prev.gyro_f_dps[0];
          prev = o;
      }
      CHECK(ctl == 20 && held == 20, "every second sample is a control cycle (%d of 40); on the others the motor command is the last one (%d of 20)", ctl, held);
      CHECK(fresh >= 38, "the filtered gyro is new on every sample, control cycle or not (%d of 40)", fresh); }
    // the estimators keep their own rate whatever the control rate
    for (int d : {1, 2, 4}) { FlightCore c; c.cfg.pid_denom = d; c.configure(); FastInput in; in.dt = 0.00025f; in.acc_g = {0, 0, -1}; in.health.imu_ok = in.health.gyro_calibrated = true; int est = 0, pid = 0;
      for (int k = 0; k < 4000; k++) { in.now_ms = (uint32_t)(k / 4); const CoreOutput o = c.update(in); est += o.est_stepped; pid += o.pid_stepped; }
      CHECK(abs(est - 500) <= 1 && pid == 4000 / d, "pid_denom %d: %d control cycles and %d estimator steps in one second (est_hz 500)", d, pid, est); }
    // the craft flies the same: Angle, Acro steps, the attitude estimate through fast turns
    for (int d : {1, 2, 4}) {
      { Rig r; r.core.cfg.pid_denom = d; r.core.configure(); arm(r); Pilot p = armed_pilot(); p.roll = 0.5f; r.run(1.5f, p); const float want = angle_cmd(r, ROLL, 0.5f);
        CHECK(fabsf(r.pl.right_down_deg() - want) < 1.0f && fabsf(r.out.est.att.roll - want) < 1.5f, "1/%d: angle, roll right 0.5: %.2f deg (want %.2f), estimate %.2f", d, r.pl.right_down_deg(), want, r.out.est.att.roll); }
      { Rig r; r.pl.make_five_inch(); r.core.cfg.pid_denom = d; r.core.configure(); const StepResult s = acro_step(r, ROLL, 1.0f, 0.06f);
        CHECK(s.over() < 0.10f && s.t90 > 0 && s.t90 < 0.12f && fabsf(s.last - s.target) < 0.03f * s.target, "1/%d: 5-inch, full roll in 60 ms: 90 %% after %.0f ms, overshoot %.1f %%, then %.0f of %.0f dps", d, s.t90 * 1000, s.over() * 100, s.last, s.target); }
      { Rig r; r.core.cfg.pid_denom = d; r.core.configure(); Pilot p; fly_up(r, p, 0); p.roll = 0.6f; r.run(0.15f, p); p.roll = 0; r.run(0.3f, p);
        CHECK(fabsf(wrap180(r.out.est.att.roll - r.pl.right_down_deg())) < 2.5f, "1/%d: acro roll at %.0f deg/s and back: the attitude estimate stays with the craft (%.1f vs %.1f deg)", d, apply_actual_rate(r.core.cfg.rates, ROLL, 0.6f), r.out.est.att.roll, r.pl.right_down_deg()); }
      { Rig r; r.core.cfg.pid_denom = d; r.core.configure(); arm(r, 2); Pilot p = armed_pilot(2); p.roll = 0.5f; r.run(2.5f, p); CHECK(fabsf(r.pl.right_down_deg() - 22.5f) < 1.5f, "1/%d: stabilize roll 0.5: %.2f deg", d, r.pl.right_down_deg()); }
    }
}

// A motor that stops in flight (prop strike, safety.h StallGuard) - and no false alarm in hard flying.
static void t_stall() {
    printf("[stall guard] a motor stops in the air: everything off; hard flying never trips it\n");
    { Rig r; r.pl.make_five_inch(); r.rpm_per_motor = true; r.core.cfg.stall.enabled = true; r.core.cfg.runaway.enabled = false; r.core.configure();     // (runaway prevention would trip on flips right after the take-off: not what is tested)
      Pilot p; fly_up(r, p, 0); bool off = false;
      for (int k = 0; k < 6 && !off; k++) {                                  // flips, rolls, throttle punch and chop
          p.roll = 1.0f; r.run(0.55f, p); p.roll = 0; r.run(0.3f, p); off |= !r.out.armed;
          p.pitch = -1.0f; r.run(0.55f, p); p.pitch = 0; r.run(0.3f, p); off |= !r.out.armed;
          p.yaw = 1.0f; r.run(0.4f, p); p.yaw = 0; p.thr = 0.95f; r.run(0.25f, p); p.thr = 0.0f; r.run(0.3f, p); p.thr = r.pl.hover_cmd(); r.run(0.5f, p); off |= !r.out.armed;
      }
      CHECK(!off && r.out.stall_motor == 0, "6 rounds of flips, rolls, yaw spins, full throttle punches and chops: never tripped");
      r.pl.set_tilt(0, 0, 0); r.pl.w[0] = r.pl.w[1] = r.pl.w[2] = 0; r.run(1.0f, p);
      r.dead_motor = 2; float t = 0; while (r.out.armed && t < 1.0f) { r.run(0.001f, p); t += 0.001f; }
      CHECK(!r.out.armed && t < 0.2f && r.out.stall_motor == 3 && (r.out.arm_blockers & AB_MOTOR_STALL), "motor 3 stops (a prop strike): disarmed after %.0f ms, motor %u named, blocker MOTOR_STALL", t * 1000, r.out.stall_motor);
      float s = 0; for (int m = 0; m < 4; m++) s += r.out.motor[m]; CHECK(s == 0, "every motor off");
      const bool held = (r.out.arm_blockers & AB_MOTOR_STALL) != 0; r.dead_motor = -1; p.thr = 0; p.arm = false; r.run(0.2f, p);
      CHECK(held && !(r.out.arm_blockers & AB_MOTOR_STALL), "the blocker stays until the arm switch goes off"); }
    { Rig r; r.pl.make_five_inch(); r.rpm_per_motor = true; r.core.cfg.stall.enabled = false; r.core.cfg.runaway.enabled = false; r.core.configure(); Pilot p; fly_up(r, p, 0); r.run(0.5f, p);
      r.dead_motor = 1; r.run(0.4f, p); CHECK(r.out.armed, "switched off (the default): it flies on (or does not) - nothing disarms"); }
    { Rig r; r.pl.make_five_inch(); r.esc_rpm = false; r.core.cfg.stall.enabled = true; r.core.configure(); Pilot p; fly_up(r, p, 0); r.run(1.0f, p);
      CHECK(r.out.armed, "no rpm telemetry at all: nothing to judge, never trips"); }
}

// Runaway take-off prevention inside the full core (the rules themselves: test_core).
static void t_runaway() {
    printf("[runaway take-off] the rate loop pushes the wrong way: motors off before the craft is on its back\n");
    // two motor plugs swapped (rear right <-> front right): pitch corrections arrive reversed
    for (int on = 1; on >= 0; on--) { Rig r; r.motor_map[0] = 1; r.motor_map[1] = 0; r.core.cfg.runaway.enabled = on; r.core.configure(); r.pl.ground_holds = true;
      r.pl.gain[2] = 0.94f;                // no craft is symmetric: one motor is 6 % weaker (in a perfectly even simulation the reversed loop balances on its point for half a second)
      Pilot p; p.mode = 0; r.run(0.3f, p); p.arm = true; r.run(0.2f, p); const bool idle_ok = r.out.armed; p.thr = r.pl.hover_cmd() * 1.15f;
      float t_off = -1, tilt_off = 0, tilt_max = 0, top = 0;
      for (int k = 0; k < 12000; k++) { r.run(0.00025f, p); const float tilt = acosf(clampf(r.pl.R22(), -1, 1)) * kRad2Deg; tilt_max = fmaxf(tilt_max, tilt);      // cycle by cycle
          if (t_off < 0 && !r.out.armed) { t_off = (k + 1) * 0.00025f; tilt_off = tilt; for (int m = 0; m < 4; m++) top = fmaxf(top, r.out.motor[m]); } }
      if (on) {
        CHECK(idle_ok && t_off > 0 && t_off < 0.7f && tilt_off < 50.0f && top == 0.0f && (r.out.arm_blockers & AB_RUNAWAY_TAKEOFF) && !r.core.runaway().flown(),
              "two motor plugs swapped, throttle up: disarmed %.2f s after the stick moved, tilted %.0f degrees at that moment, the motors at zero in that very cycle (blockers 0x%x)", t_off, tilt_off, r.out.arm_blockers);
        r.run(1.0f, p); CHECK(!r.out.armed && (r.out.arm_blockers & AB_RUNAWAY_TAKEOFF) && (r.out.arm_blockers & AB_ARM_SWITCH_ON), "the reason stays on show while the arm switch is still on");
        p.arm = false; p.thr = 0; r.pl = Plant(0); r.pl.ground_holds = true; r.pl.gain[2] = 0.94f; r.run(0.5f, p); CHECK(r.out.arm_blockers == 0, "switch off (and the craft put back on its feet): cleared (0x%x)", r.out.arm_blockers);
        p.arm = true; r.run(0.2f, p); const bool rearmed = r.out.armed; p.thr = r.pl.hover_cmd() * 1.15f; r.run(1.5f, p);
        CHECK(rearmed && !r.out.armed && (r.out.arm_blockers & AB_RUNAWAY_TAKEOFF), "armed again without fixing it: stopped again");
      } else CHECK(t_off < 0 && tilt_max > 120.0f, "the check switched off in the settings: nothing stops it, the craft goes over (%.0f degrees)", tilt_max);
    }
    // the bench without props: the PID sum is at its limit as well - but nothing turns
    { Rig r; r.pl.Tmax = 0; Pilot p; p.mode = 0; r.run(0.3f, p); p.arm = true; r.run(0.2f, p); p.thr = 0.3f; p.roll = 0.7f; float sum = 0;
      for (int k = 0; k < 60; k++) { r.run(0.05f, p); sum = fmaxf(sum, fabsf(r.out.pid_p[0] + r.out.pid_i[0] + r.out.pid_d[0] + r.out.pid_f[0])); }
      CHECK(r.out.armed && sum > 600.0f && !r.core.runaway().flown(), "no props, throttle at 30 %%, roll stick held over for 3 s: PID sum %.0f, the craft does not turn - not a runaway, still armed", sum); }
    // a normal flight ends the watch
    { Rig r; Pilot p; fly_up(r, p, 0); const bool early = r.core.runaway().flown(); r.run(0.5f, p);
      CHECK(!early && r.core.runaway().flown() && r.out.armed, "a craft that hovers at half throttle: half a second of quiet flight and the check is off for this power-up");
      Rig q; q.pl.make_five_inch(); Pilot s; fly_up(q, s, 0); q.run(2.0f, s); const bool hover_only = q.core.runaway().flown(); s.roll = 0.2f; s.thr = 0.22f; q.run(0.6f, s);
      CHECK(!hover_only && q.core.runaway().flown() && q.out.armed, "a 5-inch quad hovers at 13 %% throttle - below the 20 %% that count as 'flying' (Betaflight's number): the watch goes on until the throttle is at 20 %% with a stick moved"); }
}

static void t_motor_layer() {
    printf("[hand-over to MotorOutputManager]\n");
    motor::MotorOutputManager mgr; mgr.cfg.protocol = motor::PROTO_DSHOT600; mgr.cfg.idle = 0.06f; mgr.cfg.output_limit = 0.9f; mgr.begin();
    Rig r; r.lim.idle = mgr.idle_fraction(0, false); r.lim.limit = mgr.cfg.output_limit;
    CHECK(r.lim.idle == 0.06f && r.lim.limit == 0.9f, "the motor layer is the single owner of idle and limit");
    arm(r); Pilot p = armed_pilot(1); r.run(0.7f, p); p.roll = 1.0f; p.pitch = 1.0f; p.yaw = 1.0f; p.thr = 1.0f; r.run(0.2f, p);
    motor::Frame f; float cmd[8]; for (int i = 0; i < 8; i++) cmd[i] = r.out.motor[i]; mgr.update(1000, cmd, r.out.armed, f);
    const motor::Endpoints e = motor::endpoints(motor::PROTO_DSHOT600, mgr.cfg.pulse, 0.06f, 0.9f);
    { float hi = 0, lo = 1; for (int i = 0; i < 4; i++) { hi = fmaxf(hi, r.out.motor[i]); lo = fminf(lo, r.out.motor[i]); } CHECK(hi <= 0.9f + 1e-4f && lo >= 0.06f - 1e-4f, "the core itself keeps every motor inside [idle 0.06, limit 0.9] (%.3f .. %.3f)", lo, hi); }
    bool inside = true; for (int i = 0; i < 4; i++) { if (f.value[i] < (uint16_t)lroundf(e.low) || f.value[i] > (uint16_t)lroundf(e.high)) inside = false; }
    CHECK(inside, "full-stick output lands inside the DShot endpoints [%.0f, %.0f]: %u %u %u %u", e.low, e.high, f.value[0], f.value[1], f.value[2], f.value[3]);
    Pilot z = armed_pilot(1); z.thr = 0; r.run(0.2f, z); for (int i = 0; i < 8; i++) cmd[i] = r.out.motor[i]; mgr.update(2000, cmd, r.out.armed, f);
    bool at_idle = true; for (int i = 0; i < 4; i++) if (f.value[i] < (uint16_t)lroundf(e.low)) at_idle = false; CHECK(at_idle && f.value[0] >= 48, "zero throttle while armed: never below the idle value, never a stop command");
    Pilot off; r.run(0.2f, off); for (int i = 0; i < 8; i++) cmd[i] = r.out.motor[i]; mgr.update(3000, cmd, r.out.armed, f); CHECK(f.value[0] == 0 && f.value[3] == 0, "disarmed: DShot 0");
    Rig d; d.lim.idle = 0.12f; d.lim.limit = 1.0f; arm(d); Pilot s = armed_pilot(1); s.thr = 0.0f; d.run(0.3f, s); float lo = 1; for (int i = 0; i < 4; i++) lo = fminf(lo, d.out.motor[i]);
    CHECK(lo >= 0.12f - 1e-4f, "a dynamic-idle raise of the floor is honoured at once (min motor %.3f)", lo);
}

// ---------------------------------------------------------------------------------------------------------------------
// Navigation in the closed loop: the craft of sim_plant.h (it moves over the ground, drags, is blown by the wind), a GPS that is
// late and wanders, an optical-flow sensor with a lidar, the whole FlightCore.
// ---------------------------------------------------------------------------------------------------------------------
// take off in Altitude Hold (+ Angle) and hover near z metres
static float nav_dist(const Rig& r, float n, float e) { return hypotf(r.pl.pn - n, r.pl.pe - e); }

static void t_position_hold() {
    printf("[position hold] in the closed loop: GPS + compass, optical flow + lidar\n");
    for (int five = 1; five >= 0; five--) {                                    // GPS + compass, both craft
        Rig r(0, true); if (five) r.pl.make_five_inch(); r.ngps = true; r.gps_wander = 0.3f; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 3.0f);
        const char* who = five ? "5-inch" : "1 kg";
        CHECK(r.out.armed && r.pl.z > 2.0f && r.out.est.pos_valid && r.out.est.vel_valid && r.out.est.yaw_from_mag && !r.out.poshold_active, "[%s] flying at %.1f m with a GPS position and a compass heading", who, r.pl.z);
        if (five) {                                          // 0.8.8: a GPS that sends one solution a second is not enough for a hold
            r.gps_rate_said = 1.0f; p.poshold = true; r.run(0.5f, p);
            CHECK(!r.out.poshold_active && r.out.poshold_unavailable && r.out.poshold_slow_gps && !r.out.nav_lean, "[%s] a 1 Hz GPS: Position Hold refuses (unavailable, 'GPS too slow'), the pilot keeps the sticks", who);
            r.core.cfg.ph_min_gps_hz = 0.0f; r.run(0.1f, p);
            CHECK(r.out.poshold_active && !r.out.poshold_slow_gps, "[%s] ... unless the check is switched off (nav.ph_min_gps_hz 0)", who);
            r.core.cfg.ph_min_gps_hz = 5.0f; p.poshold = false; r.gps_rate_said = 10.0f; r.run(1.0f, p); }
        p.poshold = true; r.run(0.05f, p); const bool on = r.out.poshold_active; r.run(4.0f, p);
        CHECK(on && r.core.poshold().holding() && r.out.nav_lean && hypotf(r.pl.vn, r.pl.ve) < 0.4f, "[%s] Position Hold switched on in a hover: holding after 4 s (%.2f m/s)", who, hypotf(r.pl.vn, r.pl.ve));
        { const float n0 = r.pl.pn, e0 = r.pl.pe; float far = 0, tilt = 0; for (int i = 0; i < 200; i++) { r.run(0.1f, p); far = fmaxf(far, nav_dist(r, n0, e0)); tilt = fmaxf(tilt, fmaxf(fabsf(r.pl.right_down_deg()), fabsf(r.pl.nose_down_deg()))); }
          CHECK(far < 1.5f && tilt < 6.0f, "[%s] 20 s: it stays within %.2f m (the GPS itself wanders 0.3 m), never leaning more than %.1f deg", who, far, tilt); }
        // the pilot flies north, lets go
        p.pitch = 0.4f; r.run(4.0f, p); const float v_fly = r.pl.vn, lean_fly = r.pl.nose_down_deg(), want = angle_cmd(r, PITCH, 0.4f);
        CHECK(v_fly > 2.5f && fabsf(lean_fly - want) < 2.5f && (PhAxis)r.out.ph_state[1] == PhAxis::PILOT, "[%s] stick forward: it flies as in Angle (%.1f deg nose down for %.1f commanded, %.1f m/s)", who, lean_fly, want, v_fly);
        p.pitch = 0; const float n_rel = r.pl.pn; float t_hold = -1, back = 0; for (int i = 0; i < 150; i++) { r.run(0.1f, p); back = fmaxf(back, -r.pl.nose_down_deg()); if (t_hold < 0 && r.core.poshold().holding()) t_hold = 0.1f * (i + 1); }
        const float stop = r.pl.pn - n_rel;
        CHECK(t_hold > 1.5f && t_hold < 12.0f && back > 4.0f && back < 31.0f && stop > 1.0f && stop < 30.0f && hypotf(r.pl.vn, r.pl.ve) < 0.4f,
              "[%s] stick let go at %.1f m/s: it leans back up to %.0f deg, stops %.1f m further on, holds after %.1f s", who, v_fly, back, stop, t_hold);
        // wind
        { const float n0 = r.pl.pn, e0 = r.pl.pe; r.pl.wind_e = 5.0f; float far = 0; for (int i = 0; i < 250; i++) { r.run(0.1f, p); far = fmaxf(far, nav_dist(r, n0, e0)); }
          const float left = nav_dist(r, n0, e0), need = r.pl.drag * 5.0f;
          CHECK(far < 3.5f && left < 1.5f && fabsf(r.out.nav_wind_e + need) < 0.6f && r.pl.right_down_deg() < -6.0f,
                "[%s] a wind of 5 m/s sets in from the west: blown %.1f m at most, %.1f m after 25 s, leaning %.1f deg into it; learned %.2f m/s^2 against it (%.2f needed)", who, far, left, -r.pl.right_down_deg(), -r.out.nav_wind_e, need); }
    }
    // optical flow + lidar: no GPS, no compass - a position relative to where it was
    { Rig r; r.pl.make_five_inch(); r.flow = true; r.range = true; r.h.flow_ok = true; Pilot p; nav_takeoff(r, p, 1.5f);
      CHECK(r.out.est.flow_valid && r.out.est.vel_valid && !r.out.est.pos_valid && !r.out.est.yaw_from_mag && r.pl.z > 0.8f && r.pl.z < 2.5f, "flow sensor and lidar only, %.1f m up: a velocity over the ground, no position on the map", r.pl.z);
      p.poshold = true; r.run(4.0f, p);
      const float n0 = r.pl.pn, e0 = r.pl.pe; float far = 0; for (int i = 0; i < 200; i++) { r.run(0.1f, p); far = fmaxf(far, nav_dist(r, n0, e0)); }
      CHECK(r.core.poshold().holding() && far < 0.5f, "Position Hold on flow: within %.2f m for 20 s", far);
      // turning on the spot: flow is measured in the craft's axes, the hold must not care which way the nose points
      p.yaw = 0.3f; float far_yaw = 0; const float y0 = r.pl.yaw_deg(); for (int i = 0; i < 40; i++) { r.run(0.1f, p); far_yaw = fmaxf(far_yaw, nav_dist(r, n0, e0)); } p.yaw = 0; r.run(2.0f, p);
      CHECK(fabsf(wrap180(r.pl.yaw_deg() - y0)) > 60.0f && far_yaw < 1.0f && nav_dist(r, n0, e0) < 0.7f, "turned %.0f deg on the spot: it stays (%.2f m at most)", wrap180(r.pl.yaw_deg() - y0), far_yaw);
      r.pl.wind_n = -4.0f; float far_w = 0; for (int i = 0; i < 200; i++) { r.run(0.1f, p); far_w = fmaxf(far_w, nav_dist(r, n0, e0)); }
      CHECK(far_w < 3.0f && nav_dist(r, n0, e0) < 1.2f && hypotf(r.pl.vn, r.pl.ve) < 0.3f, "a head wind of 4 m/s: blown %.1f m at most, %.1f m off after 20 s", far_w, nav_dist(r, n0, e0));
      // the surface gives nothing any more (water, darkness)
      r.flow_quality = 0; r.run(0.2f, p); const bool still = r.out.poshold_active; float tilt = 0; for (int i = 0; i < 20; i++) { r.run(0.1f, p); tilt = fmaxf(tilt, fabsf(r.pl.nose_down_deg())); }
      CHECK(still && !r.out.poshold_active && r.out.poshold_unavailable && !r.out.est.vel_valid && !r.out.nav_lean && r.out.est.flow_reject == FLOW_QUALITY,
            "the flow sensor sees nothing any more: 0.3 s later Position Hold is off and says so; the craft is in plain Angle (the pilot flies)");
      r.pl.wind_n = 0; p.roll = 0.4f; r.run(1.5f, p);
      CHECK(fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 1.5f, "the stick gives the lean (%.1f deg)", r.out.est.att.roll);
      p.roll = 0; r.flow_quality = 90; r.run(3.0f, p);
      CHECK(r.out.poshold_active && !r.out.poshold_unavailable, "the picture is back: it takes over again (from the pilot's side: it brakes first)");
      // the lidar goes quiet (out of range, a loose wire): the last distance is not used to scale the flow
      r.baro = true; r.h.baro_ok = true; r.range = false; r.run(0.6f, p);
      CHECK(r.out.est.flow_reject == FLOW_HEIGHT && !r.out.est.flow_valid && !r.out.poshold_active && r.out.poshold_unavailable, "no distance to the ground for 0.6 s: the flow is not used (HEIGHT), Position Hold says it cannot");
      r.range = true; r.run(3.0f, p);
      CHECK(r.out.est.flow_reject == FLOW_OK && r.out.poshold_active, "the lidar is back: flow and Position Hold again"); }
    // flow and a GPS, no compass: the GPS's north cannot be matched with the flow's axes - while the flow works, the GPS waits
    { Rig r; r.pl.make_five_inch(); r.flow = true; r.range = true; r.h.flow_ok = true; r.ngps = true; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 1.5f);
      CHECK(r.out.est.flow_valid && r.out.est.vel_valid && !r.out.est.pos_valid && !r.out.est.yaw_from_mag, "flow + GPS without a compass: the velocity comes from the flow, the GPS is not fused (no position on the map)");
      r.flow_quality = 0; r.run(1.0f, p);
      CHECK(!r.out.est.flow_valid && r.out.est.pos_valid, "the flow sees nothing: now the GPS is used (a position on the map, as without flow)"); }
    // flow + lidar, the nose turned 80 deg to the right: forwards is now east-ish - the flow's velocity is turned into north / east by the heading
    { Rig r; r.pl.make_five_inch(); r.flow = true; r.range = true; r.h.flow_ok = true; Pilot p; nav_takeoff(r, p, 1.5f); const float y0 = r.pl.yaw_deg();
      while (wrap180(r.pl.yaw_deg() - y0) < 75.0f) { p.yaw = 0.3f; r.run(0.02f, p); } p.yaw = 0; r.run(1.0f, p); p.pitch = 0.3f; r.run(2.0f, p);
      const float dot = (r.out.est.vel_n * r.pl.vn + r.out.est.vel_e * r.pl.ve) / fmaxf(hypotf(r.out.est.vel_n, r.out.est.vel_e) * hypotf(r.pl.vn, r.pl.ve), 1e-3f);
      CHECK(r.out.est.flow_valid && hypotf(r.pl.vn, r.pl.ve) > 1.0f && dot > 0.97f && fabsf(hypotf(r.out.est.vel_n, r.out.est.vel_e) - hypotf(r.pl.vn, r.pl.ve)) < 0.3f,
            "nose turned %.0f deg, flying forwards: the estimated velocity (%.2f / %.2f) points the way the craft goes (%.2f / %.2f m/s)", wrap180(r.pl.yaw_deg() - y0), r.out.est.vel_n, r.out.est.vel_e, r.pl.vn, r.pl.ve); }
    // samples as an MSP sensor delivers them (the sum of 90 ms that ended 45 ms ago): the velocity belongs to the middle of that stretch
    { Rig r; r.pl.make_five_inch(); r.flow = true; r.range = true; r.h.flow_ok = true; r.flow_window_ms = 90; r.flow_late_ms = 45; r.flow_noise = 0.01f; Pilot p; nav_takeoff(r, p, 1.5f);
      float worst = 0; p.pitch = 0.6f; for (int i = 0; i < 60; i++) { r.run(0.02f, p); if (i > 15) worst = fmaxf(worst, fabsf(r.out.est.vel_n - r.pl.vn)); }
      CHECK(worst < 0.2f, "speeding up forwards on flow samples of 90 ms that ended 45 ms ago: the estimate is within %.3f m/s of the truth (%.1f m/s reached)", worst, r.pl.vn); }
    // the roll stick in Position Hold: the lean it asks for, as in Angle
    { Rig r; r.pl.make_five_inch(); r.flow = true; r.range = true; r.h.flow_ok = true; Pilot p; nav_takeoff(r, p, 1.5f); p.poshold = true; r.run(4.0f, p); p.roll = 0.4f; r.run(1.5f, p);
      CHECK(r.out.poshold_active && (PhAxis)r.out.ph_state[0] == PhAxis::PILOT && fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 1.5f, "roll stick in Position Hold: %.1f deg for the stick's %.1f (the roll axis is the pilot's)", r.out.est.att.roll, angle_cmd(r, ROLL, 0.4f)); }
    // a flow sample from further back than the gyro record reaches (right after start-up): not used, rather than taken without its turn rate
    { Rig r; r.core.on_range(0.5f, 0.03f, 0); FlowSample f; f.y = 0.5f; f.quality = 90; f.dt = 0.02f; f.age_s = 0.3f; r.core.on_flow(f, 0); Pilot p; r.run(0.004f, p);
      CHECK(r.out.est.flow_reject == FLOW_NO_GYRO && !r.out.est.flow_valid, "a sample that ended 0.3 s ago, 4 ms after start-up: not used (NO_GYRO)"); }
    // switched on without anything to hold with
    { Rig r; r.pl.make_five_inch(); Pilot p; nav_takeoff(r, p, 2.0f); p.poshold = true; r.run(1.0f, p); p.roll = 0.4f; r.run(1.5f, p);
      CHECK(!r.out.poshold_active && r.out.poshold_unavailable && !r.out.nav_lean && fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 1.5f, "no GPS, no flow: Position Hold says it cannot, the craft flies in Angle");
      Rig g; g.pl.make_five_inch(); g.ngps = true; g.h.gps_ok = true; nav_takeoff(g, p, 2.0f); p.poshold = true; g.run(2.0f, p);
      CHECK(g.out.est.pos_valid && !g.out.est.yaw_from_mag && !g.out.poshold_active && g.out.poshold_unavailable && !g.out.est.tilt_aided, "a GPS but no compass: no hold either (the craft would not know which way north is), and the tilt is not corrected from it"); }
    // arming
    { Rig r; Pilot p; p.mode = 1; p.poshold = true; r.run(0.5f, p); p.arm = true; r.run(0.3f, p); const bool refused = !r.out.armed && (r.out.arm_blockers & AB_POSHOLD_NO_GPS);
      p.arm = false; r.run(0.2f, p); r.h.flow_ok = true; p.arm = true; r.run(0.3f, p); const bool with_flow = r.out.armed;
      Rig n; n.h.gps_ok = true; p = Pilot(); p.mode = 1; p.poshold = true; n.run(0.5f, p); p.arm = true; n.run(0.3f, p); const bool gps_alone = n.out.armed;
      Rig g(0, true); g.h.gps_ok = true; p = Pilot(); p.mode = 1; p.poshold = true; g.run(0.5f, p); p.arm = true; g.run(0.3f, p);
      CHECK(refused && with_flow && !gps_alone && g.out.armed, "arming with the Position Hold switch on: refused with nothing to hold with and with a GPS alone; allowed with a flow sensor, or a GPS fix and a calibrated compass");
      // on the ground the sticks pass through and nothing winds up
      p.thr = 0; p.roll = 0.5f; g.run(1.0f, p); float lo = 1, hi = 0; for (int i = 0; i < 4; i++) { lo = fminf(lo, g.out.motor[i]); hi = fmaxf(hi, g.out.motor[i]); }
      CHECK(hi - lo < 0.02f && (PhAxis)g.out.ph_state[0] == PhAxis::PILOT, "on the ground with the throttle down: motors at idle (%.3f .. %.3f)", lo, hi); }
    // the tilt, corrected from the measured acceleration
    { auto lean = [](bool mag, bool gps, bool use = true, float sacc = 0.15f) { Rig r(0, mag); r.pl.make_five_inch(); r.ngps = gps; r.h.gps_ok = gps; r.gps_sacc = sacc; r.core.cfg.use_tilt_aid = use; Pilot p; nav_takeoff(r, p, 3.0f); float worst = 0; bool aided = true, ever = false;
          auto fly = [&](float dur, float roll) { for (float t = 0; t < dur; t += 0.02f) { p.roll = roll; r.run(0.02f, p); worst = fmaxf(worst, fabsf(r.pl.right_down_deg() - r.out.est.att.roll)); aided &= r.out.est.tilt_aided; ever |= r.out.est.tilt_aided; } };
          fly(2.0f, 0.5f); fly(3.0f, 0); return std::pair<float, bool>(worst, use && sacc < 0.3f ? aided : ever); };
      const auto with = lean(true, true), without = lean(true, false), off = lean(true, true, false), poor = lean(true, true, true, 0.8f);
      CHECK(with.second && with.first < 1.0f && !without.second && without.first > 2.0f, "half stick for 2 s and back: with GPS + compass the estimate is within %.2f deg of the truth all the way; without GPS it gives way %.1f deg", with.first, without.first);
      CHECK(!off.second && off.first > 2.0f, "switched off (nav.tilt_aid = 0): not used even with GPS + compass (%.1f deg)", off.first);
      CHECK(!poor.second, "a GPS that says its velocity is only good to 0.8 m/s: not used for the tilt");
      Rig g(0, true); g.ngps = true; g.h.gps_ok = true; g.baro = true; g.h.baro_ok = true; Pilot p; p.mode = 1; g.run(4.0f, p); const bool ground = g.out.est.tilt_aided;
      CHECK(!ground && g.out.est.pos_valid && g.out.est.yaw_from_mag, "disarmed on the ground with GPS + compass: the accelerometer alone says where down is (it is right there)"); }
}

// 0.35.0 (nav rewrite P6): the fence - a circle around home in Position Hold, a height ceiling in Altitude Hold
static void t_nav_fence() {
    printf("[fence] Position Hold stops at the circle, Altitude Hold under the ceiling\n");
    { Rig r(0, true); r.pl.make_five_inch(); r.ngps = true; r.h.gps_ok = true; r.core.cfg.poshold.fence_radius_m = 15.0f; r.core.apply_nav(); Pilot p; nav_takeoff(r, p, 3.0f);
      p.poshold = true; r.run(4.0f, p); p.pitch = 0.6f; float far = 0; bool flag = false;
      for (int i = 0; i < 200; i++) { r.run(0.1f, p); far = fmaxf(far, nav_dist(r, 0, 0)); flag |= r.out.fence; }
      CHECK(flag && far < 18.0f && nav_dist(r, 0, 0) < 16.0f && hypotf(r.pl.vn, r.pl.ve) < 0.5f, "fence 15 m, the stick held forwards for 20 s: %.1f m at most, holding at %.1f m (the fence flag set)", far, nav_dist(r, 0, 0));
      p.pitch = -0.4f; r.run(3.0f, p); p.pitch = 0; r.run(1.0f, p);
      CHECK(!r.out.fence && nav_dist(r, 0, 0) < 13.0f, "the stick back towards home: the pilot flies again (%.1f m)", nav_dist(r, 0, 0)); }
    { Rig r; r.baro = true; r.h.baro_ok = true; r.core.cfg.alt.ceiling_m = 4.0f; r.core.apply_nav(); Pilot p; nav_takeoff(r, p, 1.5f);
      p.thr = 1.0f; float top = 0; for (int i = 0; i < 150; i++) { r.run(0.1f, p); top = fmaxf(top, r.pl.z); }
      CHECK(top < 4.0f && top > 3.0f && r.out.fence, "ceiling 4 m, full stick up for 15 s: %.2f m at most", top);
      p.thr = 0.5f; r.run(2.0f, p); r.core.cfg.alt.ceiling_m = 2.0f; r.core.apply_nav(); r.run(6.0f, p);
      CHECK(r.pl.z < 1.8f && r.pl.z > 1.0f, "the ceiling lowered to 2 m in flight: the craft comes down under it (%.2f m)", r.pl.z); }
    // the height estimate is reset in flight (the barometer jumps 4 m and stays: 1.5 s of rejected readings, then the estimate is SET
    // to it): Altitude Hold moves its target with the step - the craft does not dive 4 m to 'correct' it
    { Rig r; r.baro = true; r.h.baro_ok = true; Pilot p; nav_takeoff(r, p, 5.0f); r.run(3.0f, p); const float z0 = r.pl.z; const uint32_t n0 = r.core.alt_estimator().baro_resets();
      r.drift += 4.0f; float lo = z0, hi = z0; for (int i = 0; i < 80; i++) { r.run(0.1f, p); lo = fminf(lo, r.pl.z); hi = fmaxf(hi, r.pl.z); }
      CHECK(r.core.alt_estimator().baro_resets() == n0 + 1 && z0 - lo < 1.0f && hi - z0 < 1.0f && r.out.ah_state == AhState::FLYING,
            "the barometer jumps 4 m at %.1f m: one height reset, the craft stays between %.2f and %.2f m", z0, lo, hi); }
}

// 0.35.0 (nav rewrite P7): an outside autopilot (a mission, a companion computer) flies the hold modes through set_nav_request
static void t_nav_request() {
    printf("[nav request] the hook for RTH / waypoints: a point and a height, flown in Position Hold while it is refreshed\n");
    Rig r(0, true); r.pl.make_five_inch(); r.ngps = true; r.h.gps_ok = true; Pilot p; nav_takeoff(r, p, 3.0f); p.poshold = true; r.run(4.0f, p);
    nav::NavRequest q; q.north_m = 20.0f; q.east_m = -10.0f; q.speed_mps = 4.0f; q.height = true; q.height_m = 6.0f;
    CHECK(!r.out.nav_request, "nothing asked: Position Hold as always");
    float vmax = 0, t_arrive = -1; for (int i = 0; i < 300; i++) { r.core.set_nav_request(q, r.us / 1000); r.run(0.1f, p); vmax = fmaxf(vmax, hypotf(r.pl.vn, r.pl.ve)); if (t_arrive < 0 && hypotf(r.out.est.pos_n - 20.0f, r.out.est.pos_e + 10.0f) < 1.0f && fabsf(r.pl.vn) < 0.3f) t_arrive = 0.1f * i; }
    const float off = hypotf(r.out.est.pos_n - 20.0f, r.out.est.pos_e + 10.0f);      // (the simulated GPS itself wanders a metre or two)
    CHECK(r.out.nav_request && t_arrive > 3.0f && t_arrive < 20.0f && vmax < 4.6f && off < 0.4f && nav_dist(r, 20, -10) < 2.5f && fabsf(r.pl.z - 6.0f) < 0.4f,
          "fly to 20 m N, 10 m W, 6 m up at 4 m/s: there after %.1f s (%.1f m/s at most), %.2f m off by its estimate (%.2f in truth), %.2f m up", t_arrive, vmax, off, nav_dist(r, 20, -10), r.pl.z);
    p.roll = 0.4f; for (int i = 0; i < 10; i++) { r.core.set_nav_request(q, r.us / 1000); r.run(0.1f, p); }      // (still asked all the while)
    CHECK(!r.out.nav_request && r.pl.right_down_deg() > 5.0f, "the pilot moves the roll stick: the stick has it (%.1f deg)", r.pl.right_down_deg());
    p.roll = 0; const float n0 = r.pl.pn, e0 = r.pl.pe; r.run(8.0f, p);
    CHECK(!r.out.nav_request && r.out.poshold_active && r.core.poshold().holding() && nav_dist(r, n0, e0) < 6.0f, "no more requests: Position Hold brakes and holds (%.1f m from where the stick was let go)", nav_dist(r, n0, e0));
    q.position = false; q.vel_n_mps = -2.0f; q.vel_e_mps = 0.0f; q.height = false;
    for (int i = 0; i < 40; i++) { r.core.set_nav_request(q, r.us / 1000); r.run(0.1f, p); }
    CHECK(r.out.nav_request && fabsf(r.pl.vn + 2.0f) < 0.3f && fabsf(r.pl.ve) < 0.3f, "a velocity asked (2 m/s south): %.2f / %.2f m/s", r.pl.vn, r.pl.ve);
}

static void t_gps_rescue() {
    printf("[gps rescue] in the closed loop: link lost far out, the craft comes home and lands\n");
    auto fly_out = [](Rig& r, Pilot& p, float out_s = 6.0f) { nav_takeoff(r, p, 12.0f); p.pitch = 0.6f; r.run(out_s, p); p.pitch = 0; p.roll = 0; r.run(3.0f, p); };
    auto rig = [](Rig& r, bool rescue_fs) { r.pl.make_five_inch(); r.ngps = true; r.gps_wander = 0.3f; r.h.gps_ok = true; if (rescue_fs) { r.core.cfg.fs.procedure = FSP_RESCUE; r.core.cfg.rescue_configured = true; r.core.configure(); } };
    struct Seen { std::string order; float max_z = 0, t = 0, yaw_fly = 999, worst_tilt = 0; };
    auto watch = [](Rig& r, Pilot& p, float timeout, Seen& s, bool until_disarm = true) { int last = -1;
        static const char* names[] = {"IDLE", "CLIMB", "ROTATE", "FLY_HOME", "DESCENT", "LANDING", "FAILED", "LOITER"};
        for (float t = 0; t < timeout && (!until_disarm || r.out.armed); t += 0.05f) { r.run(0.05f, p); s.t += 0.05f;
            if (r.out.rescue_phase != last) { last = r.out.rescue_phase; s.order += names[last]; s.order += ' '; if (last == (int)RescuePhase::DESCENT) s.yaw_fly = r.pl.yaw_deg(); }
            s.max_z = fmaxf(s.max_z, r.pl.z); s.worst_tilt = fmaxf(s.worst_tilt, fmaxf(fabsf(r.pl.right_down_deg()), fabsf(r.pl.nose_down_deg()))); } };
    // ---- the link is lost 40 m out
    { Rig r(0, true); rig(r, true); r.core.cfg.fs.max_descent_s = 5.0f; r.core.configure(); Pilot p; fly_out(r, p);       // (the failsafe's net for a landing that does not come down: 5 s here)
      const float d0 = hypotf(r.pl.pn, r.pl.pe), z0 = r.pl.z;
      CHECK(d0 > 25.0f && r.pl.pn > 25.0f && z0 > 10.0f && r.out.rescue_available && !r.out.rescue_active, "flown %.0f m north at %.0f m; a rescue would be possible", d0, z0);
      p.link = false; r.run(0.4f, p); const bool guard = r.out.failsafe == FsStage::GUARD && !r.out.rescue_active; r.run(0.3f, p);
      CHECK(guard && r.out.failsafe == FsStage::DESCENT && r.out.rescue_active && (RescuePhase)r.out.rescue_phase == RescuePhase::CLIMB, "link lost: half a second of waiting, then the rescue (not the landing on the spot) - it climbs");
      Seen s; watch(r, p, 180.0f, s);
      const float home = hypotf(r.pl.pn, r.pl.pe);
      CHECK(s.order == "CLIMB FLY_HOME DESCENT LOITER LANDING IDLE " || s.order == "CLIMB FLY_HOME DESCENT LOITER LANDING ", "the phases (PX4 RTL): %s", s.order.c_str());
      CHECK(s.max_z > 29.0f && s.max_z < 31.5f, "it climbed to the return height (the cone: %.0f m out is beyond RTL_MIN_DIST, so the full 30 m; %.1f m at most, was at %.1f)", d0, s.max_z, z0);
      CHECK(fabsf(wrap180(s.yaw_fly - 180.0f)) < 35.0f, "the nose turned along the way home while flying (heading %.0f over home; home lay to the south)", s.yaw_fly);
      CHECK(!r.out.armed && r.pl.z < 0.05f && home < 3.0f && s.t < 120.0f && s.worst_tilt < 35.0f, "landed %.1f m from where it was armed and disarmed, %.0f s after the link was lost (never leaning more than %.0f deg; the failsafe's 5 s limit for not coming down does not count while the rescue flies)", home, s.t, s.worst_tilt);
      r.run(0.1f, p); CHECK(r.core.rescue().max_height() == 0.0f, "disarmed: the highest point of this flight is forgotten (the next flight's return height starts afresh)"); }
    // ---- the procedure is "land": a lost link brings the craft down where it is, even with everything a rescue would need
    { Rig r(0, true); rig(r, false); Pilot p; fly_out(r, p); const float n0 = r.pl.pn; p.poshold = true; r.run(2.0f, p); const bool held = r.out.poshold_active;
      p.link = false; r.run(1.0f, p); const bool ph_in_fs = r.out.poshold_active; Seen s; watch(r, p, 90.0f, s);
      CHECK(held && !ph_in_fs && s.order == "IDLE " && !r.out.armed && r.pl.z < 0.05f && fabsf(r.pl.pn - n0) < 10.0f, "failsafe procedure LAND (Position Hold was on): no rescue (phases: %s), no Position Hold either - level, down, %.0f m from where the link was lost", s.order.c_str(), fabsf(r.pl.pn - n0)); }
    // ---- the battery runs flat on the way home: down where it is
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p, 12.0f); p.link = false; Seen s; watch(r, p, 12.0f, s, false); const bool flying = (RescuePhase)r.out.rescue_phase == RescuePhase::FLY_HOME;
      const float d0 = hypotf(r.pl.pn, r.pl.pe); r.h.batt_critical = true; r.run(0.5f, p); const bool stopped = !r.out.rescue_active && (r.out.fs_cause & FS_BATTERY); Seen s2; watch(r, p, 90.0f, s2);
      CHECK(flying && stopped && !r.out.armed && r.pl.z < 0.05f && hypotf(r.pl.pn, r.pl.pe) > 10.0f, "the battery is critical while flying home (%.0f m out): the rescue stops, the craft coasts to a stop and lands there (%.0f m from home)", d0, hypotf(r.pl.pn, r.pl.pe)); }
    // ---- too few satellites on the way: it gives up after 15 s and lands
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p, 12.0f); p.link = false; Seen s; watch(r, p, 12.0f, s, false); r.gps_sats = 3; Seen s2; watch(r, p, 17.0f, s2, false);
      CHECK(!r.out.rescue_active && (RescueFail)r.out.rescue_fail == RescueFail::LOW_SATS && r.out.armed, "3 satellites for 15 s on the way home: the rescue gives up (LOW_SATS) and the failsafe lands"); }
    // ---- a rescue from the switch all the way: it lands at home and turns the motors off by itself
    { Rig r(0, true); rig(r, false); Pilot p; fly_out(r, p); p.althold = false; p.thr = 0.45f; p.rescue = true; Seen s; watch(r, p, 120.0f, s);
      CHECK(!r.out.armed && r.pl.z < 0.05f && hypotf(r.pl.pn, r.pl.pe) < 3.0f && r.out.failsafe == FsStage::NONE && s.max_z > 20.0f, "RESCUE switch left on (Altitude Hold switch off: the rescue flies the height itself): climbed to %.0f m, home, landed, motors off (%.1f m from home, %.0f s)", s.max_z, hypotf(r.pl.pn, r.pl.pe), s.t); }
    { Rig r(0, true); rig(r, false); Pilot p; fly_out(r, p); r.mag = false; r.h.mag_ok = false; p.mode = 0; p.althold = false; p.thr = 0.47f; r.run(1.0f, p); p.rescue = true; r.run(0.5f, p);       // (in Acro, without Altitude Hold: the RESCUE switch alone brings Angle)
      const bool failed = !r.out.rescue_active && (RescueFail)r.out.rescue_fail == RescueFail::NO_HEADING; p.roll = 0.4f; r.run(1.5f, p);
      CHECK(failed && r.out.armed && fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 2.0f, "RESCUE switch without a compass: refused (NO_HEADING); the pilot flies on in Angle (%.1f deg for the stick's %.1f)", r.out.est.att.roll, angle_cmd(r, ROLL, 0.4f)); }
    // ---- the switch on the ground does nothing; flow without GPS is no position to fly home to
    { Rig r(0, true); rig(r, false); r.baro = true; r.h.baro_ok = true; Pilot p; p.mode = 1; p.rescue = true; r.run(6.0f, p); p.arm = true; r.run(3.0f, p);
      CHECK(r.out.armed && !r.out.rescue_active && r.pl.z < 0.05f, "armed on the ground with the RESCUE switch on: nothing happens (it starts only in the air)");
      Rig f(0, true); f.pl.make_five_inch(); f.flow = true; f.range = true; f.h.flow_ok = true; Pilot q; nav_takeoff(f, q, 1.5f); q.rescue = true; f.run(0.5f, q);
      CHECK(f.out.est.vel_valid && !f.out.est.pos_valid && !f.out.rescue_active && (RescueFail)f.out.rescue_fail == RescueFail::NO_POSITION, "flow + compass, no GPS: a velocity but no position - the rescue refuses (NO_POSITION)"); }
    // ---- on a switch; the pilot takes the craft back by switching it off
    { Rig r(0, true); rig(r, false); Pilot p; fly_out(r, p, 12.0f); const float d0 = hypotf(r.pl.pn, r.pl.pe);
      p.rescue = true; Seen s; watch(r, p, 9.0f, s, false);
      const float d1 = hypotf(r.pl.pn, r.pl.pe);
      CHECK(r.out.rescue_active && (RescuePhase)r.out.rescue_phase == RescuePhase::FLY_HOME && d1 < d0 - 5.0f && r.out.failsafe == FsStage::NONE, "RESCUE switch on: after 9 s it is on the way home (%.0f m of %.0f left), no failsafe involved", d1, d0);
      const float y0 = r.pl.yaw_deg(); p.yaw = 0.5f; r.run(1.0f, p); const float turning = r.pl.gyro_dps().z, turned = wrap180(r.pl.yaw_deg() - y0);
      CHECK(turning > 60.0f && turned > 45.0f && r.out.rescue_active, "the yaw stick still turns the craft its way (%.0f deg/s to the right, %.0f deg in a second)", turning, turned);
      p.yaw = 0; p.rescue = false; r.run(0.1f, p); const bool off = !r.out.rescue_active && !r.out.nav_lean; p.roll = 0.4f; r.run(1.5f, p);
      CHECK(off && fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 2.0f && r.out.armed, "switch off: the pilot has the craft back at once (the stick gives %.1f deg)", r.out.est.att.roll); }
    // ---- switched on while Position Hold leans into a side wind: the rescue starts from that lean, the craft is not blown off first
    { Rig r(0, true); rig(r, false); Pilot p; fly_out(r, p); r.pl.wind_e = 6.0f; p.poshold = true; r.run(15.0f, p); const float lean0 = r.pl.right_down_deg();
      const float n0 = r.pl.pn, e0 = r.pl.pe; p.rescue = true; float far = 0; for (int i = 0; i < 150; i++) { r.run(0.02f, p); far = fmaxf(far, nav_dist(r, n0, e0)); }
      CHECK(r.out.rescue_active && lean0 < -5.0f && far < 0.4f, "Position Hold in a 6 m/s side wind (%.0f deg into it), RESCUE switched on: the climb starts where the craft was (%.2f m off at most in 3 s)", -lean0, far); }
    // ---- the link comes back on the way: the pilot must show he is there
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p); p.link = false; Seen s; watch(r, p, 14.0f, s, false);
      p.link = true; r.run(3.0f, p); const bool still = r.out.rescue_active && r.out.failsafe == FsStage::DESCENT;
      p.roll = 0.2f; r.run(1.5f, p); const bool small = r.out.rescue_active;
      p.roll = 0.4f; r.run(0.3f, p); const bool back = !r.out.rescue_active && r.out.failsafe == FsStage::NONE; r.run(1.5f, p);
      CHECK(still && small && back && r.out.armed && fabsf(r.out.est.att.roll - angle_cmd(r, ROLL, 0.4f)) < 2.0f,
            "link back, sticks centred: the rescue carries on; a stick moved 20 %%: still; 40 %%: the pilot has the craft"); }
    // ---- what it needs is not there: the landing on the spot, as before
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p); const float n0 = r.pl.pn;
      r.mag = false; r.h.mag_ok = false; r.run(1.0f, p);              // the compass stops answering in the air
      CHECK(r.out.armed && r.pl.z > 8.0f && !r.out.rescue_available && !r.out.est.yaw_from_mag, "the compass has failed in flight: a rescue is not possible any more, and the craft says so beforehand");
      p.link = false; r.run(1.0f, p);
      CHECK(r.out.failsafe == FsStage::DESCENT && !r.out.rescue_active && (RescueFail)r.out.rescue_fail == RescueFail::NO_HEADING, "link lost: the rescue fails at once (no heading)");
      Seen s; watch(r, p, 90.0f, s);
      CHECK(!r.out.armed && r.pl.z < 0.05f && s.max_z < 14.5f && r.pl.pn > n0 - 15.0f && s.t < 61.0f, "and the craft lands where it is, level, as it always did (%.0f m from home, %.0f s)", hypotf(r.pl.pn, r.pl.pe), s.t); }
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p); p.link = false; Seen s; watch(r, p, 9.0f, s, false);
      const bool flying = (RescuePhase)r.out.rescue_phase == RescuePhase::FLY_HOME; r.ngps = false; r.run(2.0f, p);
      const bool failed = !r.out.rescue_active && (RescueFail)r.out.rescue_fail == RescueFail::GPS_LOST && r.out.failsafe == FsStage::DESCENT;
      Seen s2; watch(r, p, 90.0f, s2);
      CHECK(flying && failed && !r.out.armed && r.pl.z < 0.05f && s2.t < 61.0f && s2.worst_tilt < 25.0f, "the GPS dies on the way home: the rescue gives up (GPS_LOST) and the craft comes down where it is (%.0f s later, %.0f m from home)", s2.t, hypotf(r.pl.pn, r.pl.pe)); }
    // ---- 0.36 SMART_RTL: out north, then east; the link is lost - back the way it came, round the corner (ArduPilot SmartRTL)
    { Rig r(0, true); rig(r, true); r.core.cfg.rescue.mode = RESCUE_MODE_SMART_RTL; r.core.configure(); Pilot p; nav_takeoff(r, p, 12.0f);
      p.pitch = 0.6f; r.run(5.0f, p); p.pitch = 0; r.run(3.0f, p); const float cn = r.pl.pn, ce = r.pl.pe; p.roll = 0.6f; r.run(5.0f, p); p.roll = 0; r.run(3.0f, p);
      const float d0 = hypotf(r.pl.pn, r.pl.pe), leg = hypotf(r.pl.pn - cn, r.pl.pe - ce);
      p.link = false; float corner = 1e9f, top = 0, rem0 = -1; bool smart = false;
      for (float t = 0; t < 180.0f && r.out.armed; t += 0.05f) { r.run(0.05f, p); corner = fminf(corner, nav_dist(r, cn, ce)); top = fmaxf(top, r.pl.z); smart |= r.core.rescue().smart(); if (rem0 < 0 && r.out.rescue_active) rem0 = r.out.rescue_left; }
      CHECK(cn > 20.0f && leg > 20.0f && smart && corner < 4.0f && top < 14.5f, "out %.0f m north, %.0f m east, link lost: back the way it came, %.1f m from the corner at most, no climb (%.1f m at most)", cn, leg, corner, top);
      CHECK(rem0 > d0 + 5.0f && rem0 < cn + leg + 8.0f, "the way home it reported at the start (the OSD): %.0f m along the path (straight home would be %.0f)", rem0, d0);
      CHECK(!r.out.armed && r.pl.z < 0.05f && hypotf(r.pl.pn, r.pl.pe) < 3.0f, "landed %.1f m from home, disarmed", hypotf(r.pl.pn, r.pl.pe)); }
    // ---- 0.36 RTL in a 5 m/s side wind: the craft stays on the line home (a target point it is pulled back to, not a velocity alone)
    { Rig r(0, true); rig(r, true); Pilot p; fly_out(r, p); p.poshold = true; r.pl.wind_e = 5.0f; r.run(10.0f, p); const float n0 = r.pl.pn, e0 = r.pl.pe, l0 = hypotf(n0, e0);
      p.link = false; float off = 0; bool homing = false;
      for (float t = 0; t < 120.0f && r.out.armed; t += 0.05f) { r.run(0.05f, p); const bool fh = r.out.rescue_phase == (uint8_t)RescuePhase::FLY_HOME; homing |= fh;
        if (fh) off = fmaxf(off, fabsf(r.pl.pn * e0 - r.pl.pe * n0) / l0); }
      CHECK(homing && l0 > 25.0f && off < 2.0f && !r.out.armed && hypotf(r.pl.pn, r.pl.pe) < 1.5f, "link lost %.0f m out in a 5 m/s side wind: at most %.1f m off the straight line home, landed %.1f m from home", l0, off, hypotf(r.pl.pn, r.pl.pe)); }
    // ---- 0.36 LAND: the switch brings the craft down where it is, holding that point (ArduPilot LAND)
    { Rig r(0, true); rig(r, false); r.core.cfg.rescue.mode = RESCUE_MODE_LAND; r.core.configure(); Pilot p; fly_out(r, p); const float n0 = r.pl.pn, e0 = r.pl.pe, z0 = r.pl.z;
      r.pl.wind_e = 4.0f; p.althold = false; p.thr = 0.45f; p.rescue = true; Seen s; float far = 0;
      for (float t = 0; t < 90.0f && r.out.armed; t += 0.05f) { r.run(0.05f, p); s.t += 0.05f; far = fmaxf(far, nav_dist(r, n0, e0)); if (r.out.rescue_active && s.order.empty()) s.order = r.out.rescue_phase == (uint8_t)RescuePhase::LANDING ? "LANDING" : "?"; }
      CHECK(s.order == "LANDING" && !r.out.armed && r.pl.z < 0.05f && far < 4.0f && hypotf(n0, e0) > 25.0f, "RESCUE switch with rescue_mode LAND, %.0f m from home at %.0f m, a 4 m/s side wind: straight down, %.1f m from where it was at most, motors off (%.0f s)", hypotf(n0, e0), z0, far, s.t); }
    // ---- arming with a rescue set up
    { Rig r(0, true); r.core.cfg.rescue_configured = true; r.core.configure(); Pilot p; p.mode = 1; r.run(0.5f, p); p.arm = true; r.run(0.3f, p);
      const bool refused = !r.out.armed && (r.out.arm_blockers & AB_RESCUE_NO_FIX); p.arm = false; r.run(0.2f, p); r.h.gps_ok = true; p.arm = true; r.run(0.3f, p); const bool fix = r.out.armed;
      Rig a(0, true); a.core.cfg.rescue_configured = true; a.core.cfg.rescue_allow_no_fix = true; a.core.configure(); p = Pilot(); p.mode = 1; a.run(0.5f, p); p.arm = true; a.run(0.3f, p);
      CHECK(refused && fix && a.out.armed, "a rescue is set up: no arming without a GPS fix (home would be unknown) - unless the pilot allows it in the settings");
      Rig n(0, false); n.core.cfg.rescue_configured = true; n.core.configure(); n.h.gps_ok = true; p = Pilot(); p.mode = 1; n.run(0.5f, p); p.arm = true; n.run(0.3f, p);
      CHECK(!n.out.armed && (n.out.arm_blockers & AB_RESCUE_NO_COMPASS) && !(n.out.arm_blockers & AB_RESCUE_NO_FIX), "a rescue is set up but the compass is switched off: no arming (it could never fly home)"); }
}

// 0.23: the switches ACRO_TRAINER, AIRMODE, ANTI_GRAVITY (on aux 8 here)
static void t_switch_modes() {
    printf("[switch modes] acro trainer, airmode and anti-gravity on a switch\n");
    auto all_idle = [](const Rig& r) { bool ok = true; for (int i = 0; i < 4; i++) ok &= r.out.motor[i] == r.lim.idle; return ok; };
    // ---- acro trainer: full roll stick in Acro, the craft leans to the limit (20 deg) and stays there instead of flipping
    {   auto lean = [](bool trainer, int axis, float& peak, float& end, bool& active) {
            Rig r; r.core.rc.modes.add(M_ACRO_TRAINER, 7, 1700, 2100); r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); r.run(0.6f, p);
            p.aux7 = trainer ? 2000 : 1000; (axis == ROLL ? p.roll : p.pitch) = 1.0f; peak = 0; active = false;
            for (int i = 0; i < 30; i++) { r.run(0.05f, p); const float a = axis == ROLL ? r.out.est.att.roll : r.out.est.att.pitch; peak = fmaxf(peak, fabsf(a)); active |= r.out.trainer_active; }
            end = axis == ROLL ? r.out.est.att.roll : r.out.est.att.pitch; };
        float pk, end, pk0, end0; bool on, on0;
        lean(true, ROLL, pk, end, on); lean(false, ROLL, pk0, end0, on0);
        CHECK(on && !on0 && pk < 40.0f && end > 14.0f && end < 24.0f && pk0 > 90.0f, "roll stick held right for 1.5 s: %.1f deg at the end, %.1f at most - full stick (670 deg/s) overshoots a little (without the trainer: %.0f deg - it rolls over)", end, pk, pk0);
        lean(true, PITCH, pk, end, on);
        CHECK(on && pk < 40.0f && end > 14.0f && end < 24.0f, "pitch the same: %.1f deg at the end, %.1f at most", end, pk);
        Rig r; r.core.rc.modes.add(M_ACRO_TRAINER, 7, 1700, 2100); r.core.configure(); arm(r, 0); Pilot p = armed_pilot(0); r.run(0.6f, p); p.aux7 = 2000; p.roll = 1.0f; r.run(1.0f, p);
        p.roll = -0.4f; r.run(0.4f, p); CHECK(r.out.est.att.roll < 10.0f, "the stick reversed: the craft rolls back at once (%.1f deg)", r.out.est.att.roll);
        Rig a; a.core.rc.modes.add(M_ACRO_TRAINER, 7, 1700, 2100); a.core.configure(); arm(a, 1); p = armed_pilot(1); p.aux7 = 2000; p.roll = 1.0f; a.run(0.5f, p);
        CHECK(!a.out.trainer_active, "in Angle the trainer does nothing (Angle has its own limit)");
        // switched off while it held the craft at the limit, flown back level, switched on again: it starts from nothing
        Rig z; z.core.rc.modes.add(M_ACRO_TRAINER, 7, 1700, 2100); z.core.configure(); arm(z, 0); p = armed_pilot(0); z.run(0.6f, p); p.aux7 = 2000; p.roll = 1.0f; z.run(1.0f, p);
        p.aux7 = 1000; p.roll = -0.2f; for (int i = 0; i < 400 && z.out.est.att.roll > 1.0f; i++) z.run(0.005f, p);
        p.roll = 0; z.run(0.2f, p); const float lvl = z.out.est.att.roll; p.aux7 = 2000; p.roll = 0.03f; z.run(0.3f, p);
        CHECK(fabsf(lvl) < 4.0f && fabsf(z.out.est.att.roll - lvl) < 6.0f, "switched on again near level with a touch of right stick: not pulled to the old limit (%.1f -> %.1f deg)", lvl, z.out.est.att.roll);
        Rig k; k.core.cfg.trainer.angle_limit_deg = 45; k.core.rc.modes.add(M_ACRO_TRAINER, 7, 1700, 2100); k.core.configure(); arm(k, 0); p = armed_pilot(0); k.run(0.6f, p); p.aux7 = 2000; p.roll = 1.0f; k.run(1.5f, p);
        CHECK(k.out.est.att.roll > 38.0f && k.out.est.att.roll < 52.0f, "acro_trainer_angle_limit 45: %.1f deg", k.out.est.att.roll); }
    // ---- airmode on a switch: off = the stick at the bottom idles the motors, on = the kind general.airmode sets
    {   auto chop = [&](uint8_t setting, bool sw, bool& air) { Rig r; r.core.cfg.airmode = setting; r.core.rc.modes.add(M_AIRMODE, 7, 1700, 2100); r.core.configure(); arm(r, 0);
            Pilot p = armed_pilot(0); p.aux7 = sw ? 2000 : 1000; r.run(0.5f, p); p.thr = 0; p.roll = 0.3f; r.run(0.3f, p); air = r.out.airmode_active; return all_idle(r); };
        bool a1, a2, a3; const bool i1 = chop(AM_AUTO, false, a1), i2 = chop(AM_AUTO, true, a2), i3 = chop(AM_OFF, true, a3);
        CHECK(i1 && !a1, "AIRMODE switch off: throttle chopped in the air = motors at idle (general.airmode AUTO is overruled)");
        CHECK(!i2 && a2, "switch on: airmode as general.airmode says (AUTO)");
        CHECK(!i3 && a3, "switch on with general.airmode OFF: airmode AUTO"); }
    // ---- anti-gravity on a switch
    {   auto punch = [](bool setting, int sw) { Rig r; r.core.cfg.pid.anti_gravity = setting; if (sw >= 0) r.core.rc.modes.add(M_ANTI_GRAVITY, 7, 1700, 2100); r.core.configure(); arm(r, 0);
            Pilot p = armed_pilot(0); p.aux7 = sw > 0 ? 2000 : 1000; r.run(0.6f, p); p.thr = 0.1f; float pk = 0; for (int i = 0; i < 20; i++) { r.run(0.005f, p); pk = fmaxf(pk, r.core.pid().anti_gravity()); } return pk; };
        const float none_on = punch(true, -1), sw_off = punch(true, 0), sw_on = punch(false, 1), none_off = punch(false, -1);
        CHECK(none_on > 5.0f && none_off == 0.0f, "no switch: pid.anti_gravity decides (%.1f / %.1f)", none_on, none_off);
        CHECK(sw_off == 0.0f && sw_on > 5.0f, "ANTI_GRAVITY switch: off = none although the setting is on, on = boost although it is off (%.1f)", sw_on); }
}

int main() {
    t_arming(); t_attitude_modes(); t_rate_loop(); t_filters_in_the_loop(); t_throttle_boost(); t_level_modes(); t_mode_details(); t_airmode(); t_compass(); t_althold(); t_failsafe(); t_uneven_cycles(); t_pid_denom(); t_stall(); t_runaway(); t_motor_layer(); t_estimator_in_flight();
    t_position_hold(); t_nav_fence(); t_nav_request(); t_gps_rescue(); t_switch_modes(); t_stick_arm();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
