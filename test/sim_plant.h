// Independent rigid-body + vertical plant for the flight-core tests. Built from frame GEOMETRY (motor angle, prop spin) and physics,
// NOT from the mixer: if the mixer / gyro / attitude signs disagree with physics the closed-loop tests fail.
// Frame FRD, quaternion body->world (NED). Motor angle = clockwise from the nose; spin +1 = CCW prop (reaction torque turns the frame clockwise).
#pragma once
#include <math.h>
#include <random>
#include <vector>
#include "../src/core/types.h"
using fc::Vec3;

struct Plant {
    int n = 4;
    std::vector<float> ang, spin, rad;                             // rad: arm length of each motor as a fraction of `arm` (empty = all 1); coaxial motors share an angle
    float q0 = 1, q1 = 0, q2 = 0, q3 = 0, w[3] = {0, 0, 0}, T[8] = {};
    float Tmax = 4.9f, arm = 0.12f, kyaw = 0.02f, Ixx = 0.010f, Iyy = 0.010f, Izz = 0.020f, lag = 0.02f, mass = 1.0f;
    float gain[8] = {1, 1, 1, 1, 1, 1, 1, 1};                     // thrust of each motor relative to the others (a tired motor, a chipped prop)
    float rotor_j = 0, rotor_kt = 1.9e-6f;                        // rotor inertia (kg m^2), thrust = kt * speed^2. 0 = no reaction torque from accelerating rotors
    float z = 0, vz = 0; bool ground = true;                      // height above the ground (m), climb rate
    // Horizontal motion (world frame, north / east, metres): thrust direction + linear drag against the air, which moves with the wind.
    // drag is 1/s: deceleration = drag x airspeed (0.35 lets a quad leaning 30 degrees settle at 16 m/s). 0 = no drag (the craft never stops by itself).
    float pn = 0, pe = 0, vn = 0, ve = 0, wind_n = 0, wind_e = 0, drag = 0.35f;
    // The accelerometer of a flying craft feels thrust and drag, NOT gravity: accelerating sideways without drag it reads "level".
    // false = a quasi-static model (the reading always shows the tilt): physically wrong for anything that accelerates, kept for the
    // few tests that want an ideal tilt sensor.
    bool true_accel = true;
    // Landing gear: with this on, a craft standing on the ground keeps its attitude until the motors can really lift or tip it
    // (one motor at 10 % in a motor test does not roll a real quad over). Off by default: the closed-loop tests were written for the free body.
    bool ground_holds = false;
    // Somebody carries the craft: the height (z) and the turn rates (w) are whatever the hand does, the speed over the ground
    // goes to hold_vn / hold_ve within about a third of a second. The motors move nothing; the accelerometer reads gravity and
    // the hand's acceleration.
    bool held = false; float hold_vn = 0, hold_ve = 0, hold_an = 0, hold_ae = 0;
    float hold_vz = 0;                                            // ... and lifts it at this rate (m/s, up +)
    float total_thrust = 0;
    float delay_s = 0.0f;                                         // a pure delay of the commands (filters, ESC, motor electronics); 0 = none (most tests)
    std::vector<std::vector<float>> cmd_pipe; size_t pipe_i = 0;
    explicit Plant(int layout = 0) {                                // 0 quad X (BF numbering), 1 quad +, 2 hex X
        // a quad built the Betaflight way ("props in"): rear-right CW, front-right CCW, rear-left CCW, front-left CW
        if (layout == 0) { n = 4; ang = {135, 45, 225, 315}; spin = {-1, +1, +1, -1}; Tmax = 4.9f; }
        else if (layout == 1) { n = 4; ang = {180, 90, 270, 0}; spin = {-1, +1, +1, -1}; Tmax = 4.9f; }
        else { n = 6; ang = {30, 90, 150, 210, 270, 330}; spin = {+1, -1, +1, -1, +1, -1}; Tmax = 3.27f; Izz = 0.03f; }
    }
    // any frame from its geometry: motor angles (clockwise from the nose), prop spins (+1 CCW), arm fractions
    Plant(std::vector<float> a, std::vector<float> sp, std::vector<float> r = {}) : n((int)a.size()), ang(a), spin(sp), rad(r) { Tmax = 4.9f * 4.0f / (float)n; }
    // The craft Betaflight's default tune is made for: a 5-inch quad, 650 g, 1.2 kg of thrust per motor (thrust : weight = 7),
    // light and stiff. The default Plant is the opposite end: 1 kg with thrust : weight = 2 (a camera craft with small motors).
    void make_five_inch() { n = 4; Tmax = 12.0f; arm = 0.11f; Ixx = Iyy = 0.004f; Izz = 0.007f; mass = 0.65f; lag = 0.025f; kyaw = 0.012f; rotor_j = 5e-6f; rotor_kt = 1.9e-6f; }
    float hover_cmd() const { return mass * fc::kG0 / (n * Tmax); }
    // two ESC plugs in each other's place: command i now drives the motor at j's position, and the other way round
    void motor_swap(int i, int j) { std::swap(ang[i], ang[j]); std::swap(spin[i], spin[j]); }
    void step(const float* cmd_in, float dt) {
        const float* cmd = cmd_in; float delayed[8];
        if (delay_s > 0.0f) {                                     // a ring of past commands, delay_s / dt deep
            const size_t depth = (size_t)(delay_s / dt + 0.5f) + 1;
            if (cmd_pipe.size() != depth) { cmd_pipe.assign(depth, std::vector<float>(8, 0.0f)); pipe_i = 0; }
            for (int i = 0; i < n && i < 8; i++) cmd_pipe[pipe_i][i] = cmd_in[i];
            const size_t out = (pipe_i + 1) % depth;
            for (int i = 0; i < n && i < 8; i++) delayed[i] = cmd_pipe[out][i];
            pipe_i = out; cmd = delayed;
        }
        float tq[3] = {0, 0, 0}; total_thrust = 0;
        for (int i = 0; i < n; i++) {
            const float T0 = T[i];
            T[i] += (Tmax * gain[i] * cmd[i] - T[i]) * (dt / lag);
            // a rotor that speeds up pushes the frame the other way: on a small fast quad this is as strong as the drag torque itself
            if (rotor_j > 0) tq[2] += spin[i] * rotor_j * (sqrtf(fmaxf(T[i], 0.0f) / rotor_kt) - sqrtf(fmaxf(T0, 0.0f) / rotor_kt)) / dt;
            const float ri = (size_t)i < rad.size() ? rad[i] : 1.0f;
            const float x = arm * ri * cosf(ang[i] * fc::kDeg2Rad), y = arm * ri * sinf(ang[i] * fc::kDeg2Rad);
            tq[0] += -y * T[i];                                     // r x F with F = (0,0,-T)
            tq[1] += +x * T[i];
            tq[2] += spin[i] * kyaw * T[i];
            total_thrust += T[i];
        }
        const float wd[3] = {(tq[0] - (w[1] * Izz * w[2] - w[2] * Iyy * w[1])) / Ixx, (tq[1] - (w[2] * Ixx * w[0] - w[0] * Izz * w[2])) / Iyy, (tq[2] - (w[0] * Iyy * w[1] - w[1] * Ixx * w[0])) / Izz};
        if (!held) for (int k = 0; k < 3; k++) w[k] += wd[k] * dt;
        if (ground_holds && ground && total_thrust < mass * fc::kG0) {
            const float hold = (mass * fc::kG0 - total_thrust) * arm * 0.7f;        // weight left on the gear x half the gear width
            if (fabsf(tq[0]) < hold && fabsf(tq[1]) < hold) w[0] = w[1] = w[2] = 0;
        }
        const float dq0 = 0.5f * (-q1 * w[0] - q2 * w[1] - q3 * w[2]), dq1 = 0.5f * (q0 * w[0] + q2 * w[2] - q3 * w[1]);
        const float dq2 = 0.5f * (q0 * w[1] - q1 * w[2] + q3 * w[0]), dq3 = 0.5f * (q0 * w[2] + q1 * w[1] - q2 * w[0]);
        q0 += dq0 * dt; q1 += dq1 * dt; q2 += dq2 * dt; q3 += dq3 * dt;
        const float nn = 1 / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); q0 *= nn; q1 *= nn; q2 *= nn; q3 *= nn;
        if (held) {
            hold_an = (hold_vn - vn) / 0.3f; hold_ae = (hold_ve - ve) / 0.3f; vn += hold_an * dt; ve += hold_ae * dt; pn += vn * dt; pe += ve * dt;
            vz = hold_vz; z += hold_vz * dt; ground = false; drag_n = drag_e = 0; return;
        }
        // vertical: thrust along body -z, gravity down
        const float a_up = (total_thrust * R22()) / mass - fc::kG0;
        vz += a_up * dt; z += vz * dt;
        ground = false;
        if (z <= 0) { z = 0; if (vz < 0) vz = 0; ground = true; if (a_up < 0) { /* standing on the ground */ } }
        // horizontal: thrust along body -z seen in the world, drag against the airspeed; standing on the ground it does not slide
        if (ground) { vn = ve = 0; drag_n = drag_e = 0; }
        else {
            drag_n = -drag * (vn - wind_n); drag_e = -drag * (ve - wind_e);
            const float an = -R02() * total_thrust / mass + drag_n, ae = -R12() * total_thrust / mass + drag_e;
            vn += an * dt; ve += ae * dt; pn += vn * dt; pe += ve * dt;
        }
    }
    float R02() const { return 2 * (q1 * q3 + q0 * q2); }
    float R12() const { return 2 * (q2 * q3 - q0 * q1); }
    float drag_n = 0, drag_e = 0;                                   // drag deceleration of the last step (m/s^2, world)
    // velocity over the ground in body axes (forward, right)
    float v_forward() const { const float y = yaw_deg() * fc::kDeg2Rad; return vn * cosf(y) + ve * sinf(y); }
    float v_right() const { const float y = yaw_deg() * fc::kDeg2Rad; return -vn * sinf(y) + ve * cosf(y); }
    // velocity over the ground in the board's own axes (FRD, x / y): what an optical flow sensor sees - the tilt and the climb rate included
    float v_board_x() const { const double v[3] = {vn, ve, -vz}; double b[3]; body_from_world(v, b); return (float)b[0]; }
    float v_board_y() const { const double v[3] = {vn, ve, -vz}; double b[3]; body_from_world(v, b); return (float)b[1]; }
    float R20() const { return 2 * (q1 * q3 - q0 * q2); }
    float R21() const { return 2 * (q2 * q3 + q0 * q1); }
    float R22() const { return 1 - 2 * (q1 * q1 + q2 * q2); }
    float nose_down_deg() const { return asinf(fc::clampf(R20(), -1, 1)) * fc::kRad2Deg; }
    float right_down_deg() const { return atan2f(R21(), R22()) * fc::kRad2Deg; }
    float yaw_deg() const { return atan2f(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * fc::kRad2Deg; }
    // Accelerometer: direction = gravity direction in body axes (what a steady-flight IMU reports), magnitude = thrust/(m g); on the ground = 1 g.
    Vec3 acc() const {
        if (held) { const double a[3] = {hold_an, hold_ae, 0}; double b[3]; body_from_world(a, b); return {-R20() + (float)b[0] / fc::kG0, -R21() + (float)b[1] / fc::kG0, -R22() + (float)b[2] / fc::kG0}; }
        if (true_accel && !ground) {                              // specific force = (thrust + drag) / m, in body axes, in g
            const double d[3] = {drag_n, drag_e, 0}; double b[3]; body_from_world(d, b);
            return {(float)b[0] / fc::kG0, (float)b[1] / fc::kG0, (float)b[2] / fc::kG0 - total_thrust / (mass * fc::kG0)};
        }
        const float mag = ground ? 1.0f : fmaxf(total_thrust / (mass * fc::kG0), 0.0f);
        return {-R20() * mag, -R21() * mag, -R22() * mag};
    }
    Vec3 gyro_dps() const { return {w[0] * fc::kRad2Deg, w[1] * fc::kRad2Deg, w[2] * fc::kRad2Deg}; }
    void set_tilt(float roll_right_deg, float nose_down_deg_, float yaw_deg_ = 0) {
        const float r = roll_right_deg * fc::kDeg2Rad / 2, p = -nose_down_deg_ * fc::kDeg2Rad / 2, y = yaw_deg_ * fc::kDeg2Rad / 2;
        const float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
        q0 = cr * cp * cy + sr * sp * sy; q1 = sr * cp * cy - cr * sp * sy; q2 = cr * sp * cy + sr * cp * sy; q3 = cr * cp * sy - sr * sp * cy;
    }
    void body_from_world(const double v[3], double b[3]) const {   // b = R^T v
        const double a0 = q0, a1 = q1, a2 = q2, a3 = q3;
        const double R[3][3] = {{1 - 2 * (a2 * a2 + a3 * a3), 2 * (a1 * a2 - a0 * a3), 2 * (a1 * a3 + a0 * a2)},
                                {2 * (a1 * a2 + a0 * a3), 1 - 2 * (a1 * a1 + a3 * a3), 2 * (a2 * a3 - a0 * a1)},
                                {2 * (a1 * a3 - a0 * a2), 2 * (a2 * a3 + a0 * a1), 1 - 2 * (a1 * a1 + a2 * a2)}};
        for (int i = 0; i < 3; i++) b[i] = R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2];
    }
};
