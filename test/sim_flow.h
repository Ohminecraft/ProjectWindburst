// Simulated optical-flow sensors for the firmware-in-the-loop tests: what a camera that looks straight down sees of the plant's
// motion, delivered the way the real parts deliver it.
//   Pmw3901Sim   register level, on the second SPI bus: identity, the start-up writes (stored, read back), the 12-byte motion burst
//   MspFlowSim   a MicoAir MTF-01 on a UART: MSP v2 frames with the flow of a 40 ms window that ended 45 ms ago, and its lidar
// Written from the parts' documented behaviour (ArduPilot AP_OpticalFlow_Pixart, Betaflight rangefinder_lidarmt.c), not from
// the firmware's drivers.
#pragma once
#include <stdint.h>
#include <math.h>
#include <deque>
#include <vector>
#include <string.h>

// The picture's angular rate about the sensor's X / Y axis (rad/s; a right-handed rotation of the sensor reads positive):
//   x = roll rate - v_right / range        y = pitch rate (nose up +) + v_forward / range        range = height / cos(tilt)
// v_forward / v_right: the velocity over the ground in the BOARD's axes (FRD x / y) - a tilted craft that climbs sees part of its
// climb there (ArduPilot EKF3 / PX4 EKF2 predict the flow from the full 3-D velocity turned into the sensor's axes)
// How the sensor sits on the craft: its X axis points `mount_cw_deg` to the right of the nose (0 = the normal direction); a
// mirrored picture (a sensor that looks through a mirror, or one whose maker counts the other way) reads X with the other sign.
struct FlowTruth {
    static constexpr int kMs = 256;
    float fx[kMs] = {}, fy[kMs] = {}, range[kMs] = {}; int head = 0, count = 0;
    float mount_cw_deg = 0; bool mirrored = false;
    void push(float wx, float wy, float v_fwd, float v_right, float height, float up_cos) {     // once per millisecond
        const float r = fmaxf(height, 0.02f) / fmaxf(up_cos, 0.3f);
        const float bx = wx - v_right / r, by = wy + v_fwd / r;                                 // about the craft's own X / Y
        const float a = mount_cw_deg * 0.017453292f, sx = cosf(a) * bx + sinf(a) * by, sy = -sinf(a) * bx + cosf(a) * by;
        fx[head] = mirrored ? -sx : sx; fy[head] = sy; range[head] = r; head = (head + 1) % kMs; if (count < kMs) count++;
    }
    // mean over the `len_ms` milliseconds that ended `ago_ms` ago
    void mean(int ago_ms, int len_ms, float& x, float& y) const {
        x = y = 0; int n = 0;
        for (int i = 0; i < len_ms; i++) { const int k = ago_ms + i; if (k >= count) break; const int idx = (head - 1 - k + 2 * kMs) % kMs; x += fx[idx]; y += fy[idx]; n++; }
        if (n) { x /= n; y /= n; }
    }
    float range_now() const { return count ? range[(head - 1 + kMs) % kMs] : 0.0f; }
};

struct Pmw3901Sim {
    static constexpr float kRadPerCount = 1.26e-3f;
    bool present = true; uint8_t id = 0x49; int motion_reg = -1;     // motion_reg >= 0: the motion register reads this (bits the driver must not care about)
    uint8_t squal = 80, shutter_upper = 0x05;          // what the chip says about the surface (dark: shutter_upper 0x1F)
    float scale = 1.0f;                                // a lens that makes the picture move this much faster
    uint8_t reg[128] = {}; double acc[2] = {0, 0}; uint32_t bursts = 0, writes = 0, resets = 0;
    Pmw3901Sim() { power_on(); }
    void power_on() { memset(reg, 0, sizeof(reg)); reg[0x00] = id; reg[0x5F] = (uint8_t)~id; }
    void integrate(float fx, float fy, float dt) { acc[0] += (double)fx * scale * dt / kRadPerCount; acc[1] += (double)fy * scale * dt / kRadPerCount; }
    // ---- SPI: chip select edges and bytes
    void cs(bool low) { active = low; idx = 0; }
    uint8_t xfer(uint8_t v) {
        if (!present || !active) return 0xFF;
        uint8_t out = 0;
        if (idx == 0) {
            wr = (v & 0x80) != 0; addr = v & 0x7F;
            if (!wr && addr == 0x16) {                 // motion burst: latch the counts since the previous burst
                bursts++;
                long c[2]; for (int k = 0; k < 2; k++) { c[k] = lround(acc[k]); if (c[k] > 32767) c[k] = 32767; if (c[k] < -32768) c[k] = -32768; acc[k] -= (double)c[k]; }
                const int16_t dx = (int16_t)c[0], dy = (int16_t)c[1];
                burst[0] = motion_reg >= 0 ? (uint8_t)motion_reg : (dx || dy) ? 0x80 : 0x00; burst[1] = 0; burst[2] = (uint8_t)dx; burst[3] = (uint8_t)((uint16_t)dx >> 8); burst[4] = (uint8_t)dy; burst[5] = (uint8_t)((uint16_t)dy >> 8);
                burst[6] = squal; burst[7] = 0x40; burst[8] = 0x80; burst[9] = 0x10; burst[10] = shutter_upper; burst[11] = 0x00;
            }
        } else if (wr) {
            if (idx == 1) { writes++; if (addr == 0x3A && v == 0x5A) { power_on(); resets++; } else reg[addr] = v; }
        } else out = addr == 0x16 ? (idx - 1 < 12 ? burst[idx - 1] : 0) : reg[addr];
        idx++;
        return out;
    }
    bool active = false, wr = false; int idx = 0; uint8_t addr = 0, burst[12] = {};
};

struct MspFlowSim {
    bool present = true; uint32_t sensor_baud = 115200, host_baud = 0;
    uint8_t quality = 200;                             // 0..255 as the sensor sends it
    float max_range_m = 8.0f; bool send_range = true;
    int flow_every_ms = 10, range_every_ms = 20;
    float scale = 1.0f; bool corrupt_next = false;
    std::deque<uint8_t> q; double rem[2] = {0, 0}; uint32_t flow_frames = 0, range_frames = 0;

    static uint8_t crc8(uint8_t crc, uint8_t a) { crc ^= a; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1); return crc; }
    void frame(uint16_t cmd, const uint8_t* payload, uint16_t n) {
        std::vector<uint8_t> f = {'$', 'X', '<', 0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)n, (uint8_t)(n >> 8)};
        for (uint16_t i = 0; i < n; i++) f.push_back(payload[i]);
        uint8_t c = 0; for (size_t i = 3; i < f.size(); i++) c = crc8(c, f[i]);
        if (corrupt_next) { c ^= 0x55; corrupt_next = false; }
        f.push_back(c);
        if (host_baud == sensor_baud) for (uint8_t b : f) q.push_back(b);          // at the wrong baud rate nothing readable arrives
    }
    // once per millisecond
    void tick(uint32_t now_ms, const FlowTruth& truth) {
        if (!present) return;
        if (now_ms % (uint32_t)flow_every_ms == 0) {
            float fx, fy; truth.mean(45, 40, fx, fy);                              // the 40 ms that ended 45 ms ago
            const double dt = flow_every_ms * 1e-3;
            int32_t m[2]; const double want[2] = {fx * scale * 200.0 * dt + rem[0], fy * scale * 200.0 * dt + rem[1]};
            for (int k = 0; k < 2; k++) { m[k] = (int32_t)lround(want[k]); rem[k] = want[k] - m[k]; }
            uint8_t p[9]; p[0] = quality; memcpy(p + 1, &m[0], 4); memcpy(p + 5, &m[1], 4);
            frame(0x1F02, p, 9); flow_frames++;
        }
        if (send_range && now_ms % (uint32_t)range_every_ms == 0) {
            const float r = truth.range_now(); const int32_t mm = (r > max_range_m || truth.count == 0) ? -1 : (int32_t)lroundf(r * 1000.0f);
            uint8_t p[5]; p[0] = 255; memcpy(p + 1, &mm, 4);
            frame(0x1F01, p, 5); range_frames++;
        }
    }
    int read() { if (q.empty()) return -1; const uint8_t c = q.front(); q.pop_front(); return c; }
};
