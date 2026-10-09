// Range finder: height processing (tilt, gating, ground clearance, jumps, staleness), closed loop with the REAL VerticalKf and a
// drifting baro, and the RangeFinder manager over a fake driver. The Pololu adapters are compile-checked for ESP32 (not host-testable).
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <random>
#include <vector>
#include "../src/hal/range/range_base.h"
#include "../src/core/vertical_est.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(5); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }

// Stubs for the Arduino headers the manager pulls in (no chip behind them: the manager is exercised through useDriver()).
int g_spi_begin_pins[4];
void digitalWrite(uint8_t, uint8_t) {}
uint8_t stub_spi_transfer(uint8_t) { return 0xFF; }
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t, const uint8_t*, size_t, bool) { return 2; }
int stub_i2c_rx(uint8_t, uint8_t*, size_t) { return 0; }

struct FakeRange : RangeDriver {
    bool has = false, ok = true; RangeReading next; float maxr = 3.0f, minr = 0.04f; bool inited = true;
    const char* name() const override { return "FAKE"; }
    bool init() override { return inited; }
    bool update(uint32_t, RangeReading& o) override { if (!has) return false; has = false; o = next; return true; }
    float max_range_m() const override { return maxr; } float min_range_m() const override { return minr; }
    float nominal_rate_hz() const override { return 30; } bool healthy() const override { return ok; }
    void push(float d, uint8_t q = 90) { next.distance_m = d; next.quality_pct = q; has = true; }
};

static RangeReading rd(float d, uint8_t q = 90) { RangeReading r; r.distance_m = d; r.quality_pct = q; return r; }

static void t_height() {
    printf("[height] tilt correction + gating\n");
    RangeHeight h; h.begin(0.04f, 4.0f); h.cfg.ground_clearance_m = 0.0f; h.reset(); uint32_t ms = 1000;
    CHECK(h.update(rd(1.0f), 1.0f, ms += 33, true) && fabsf(h.height_m() - 1.0f) < 1e-5f, "level: 1.000");
    CHECK(h.update(rd(1.0f), 0.9f, ms += 33, true) && fabsf(h.height_m() - 0.9f) < 1e-5f, "25 deg tilt (cos .9): 1.0 m along the beam = 0.9 m up");
    uint32_t rej = h.rejected();
    CHECK(!h.update(rd(1.0f), 0.8f, ms += 33, true) && h.rejected() == rej + 1 && fabsf(h.height_m() - 0.9f) < 1e-5f, "37 deg tilt rejected, height unchanged");
    CHECK(!h.update(rd(0.02f), 1.0f, ms += 33, true), "below the chip's minimum range");
    { RangeHeight u; u.begin(0.04f, 4.0f); u.cfg.ground_clearance_m = 0.0f; u.reset();
      CHECK(!u.update(rd(3.8f), 1.0f, 100, true), "3.8 m is above 90 %% of the 4 m max (edge readings are noisy)"); }
    { RangeHeight u; u.begin(0.04f, 4.0f); u.cfg.ground_clearance_m = 0.0f; u.reset();
      CHECK(u.update(rd(3.5f), 1.0f, 100, true), "3.5 m is inside the usable range (3.6 m)"); }
    CHECK(!h.update(rd(1.0f, 10), 1.0f, ms += 33, true), "weak signal rejected");
    printf("[height] jumps, staleness\n");
    RangeHeight j; j.begin(0.04f, 4.0f); j.cfg.ground_clearance_m = 0.0f; j.reset(); ms = 0;
    for (int i = 0; i < 10; i++) j.update(rd(1.0f), 1.0f, ms += 33, true);
    CHECK(!j.update(rd(2.5f), 1.0f, ms += 33, true) && fabsf(j.height_m() - 1.0f) < 1e-5f, "hand under the drone / 1.5 m spike rejected");
    CHECK(j.update(rd(1.02f), 1.0f, ms += 33, true), "normal sample right after the spike accepted");
    CHECK(!j.update(rd(2.5f), 1.0f, ms += 33, true) && !j.update(rd(2.52f), 1.0f, ms += 33, true) && j.update(rd(2.51f), 1.0f, ms += 33, true) && fabsf(j.height_m() - 2.51f) < 1e-4f, "...but 3 consistent samples at a new level are accepted (rooftop / ledge)");
    CHECK(j.valid(ms + 200) && !j.valid(ms + 300), "valid for 250 ms after the last accepted sample");
    float sg0 = j.sigma_m(); RangeHeight k; k.begin(0.04f, 4.0f); k.cfg.ground_clearance_m = 0; k.reset(); k.update(rd(0.3f), 1, 100, true);
    CHECK(k.sigma_m() < sg0 && k.sigma_m() > 0.02f, "sigma grows with height (%.3f < %.3f)", k.sigma_m(), sg0);
}

static void t_clearance() {
    printf("[height] ground clearance\n");
    RangeHeight h; h.begin(0.04f, 4.0f); uint32_t ms = 0;
    for (int i = 0; i < 60; i++) h.update(rd(0.062f + gauss(0.002f)), 1.0f, ms += 33, false);
    CHECK(h.clearance_known() && fabsf(h.clearance_m() - 0.062f) < 0.004f, "learned the 6.2 cm mounting height: %.4f", h.clearance_m());
    CHECK(fabsf(h.height_m()) < 0.01f, "on the ground reads 0 (%.4f)", h.height_m());
    for (int i = 0; i < 20; i++) h.update(rd(1.062f), 1.0f, ms += 33, true);
    CHECK(fabsf(h.height_m() - 1.0f) < 0.01f, "1 m up reads 1.000 (%.4f)", h.height_m());
    for (int i = 0; i < 100; i++) h.update(rd(0.4f), 1.0f, ms += 33, true);
    CHECK(fabsf(h.clearance_m() - 0.062f) < 0.004f, "armed: clearance frozen even if the drone sits low (%.4f)", h.clearance_m());
    RangeHeight a; a.begin(0.04f, 4.0f); ms = 0;
    for (int i = 0; i < 40; i++) a.update(rd(1.6f), 1.0f, ms += 33, false);
    CHECK(!a.clearance_known(), "carried in the air at power-up (1.6 m): nothing learned");
    RangeHeight b; b.cfg.ground_clearance_m = 0.05f; b.begin(0.04f, 4.0f); b.update(rd(0.45f), 1.0f, 33, true);
    CHECK(fabsf(b.height_m() - 0.40f) < 1e-5f, "manual clearance 5 cm");
    RangeHeight c; c.begin(0.04f, 4.0f); ms = 0; for (int i = 0; i < 40; i++) c.update(rd(0.1f), 0.8f, ms += 33, false);
    CHECK(!c.clearance_known(), "tilted on the ground (e.g. on a slope): not used for learning");
}

// 1-D plant + the real Kalman filter: baro drifts, lidar available only below 3 m
static void t_closed_loop() {
    printf("[kalman] baro drift corrected by the lidar; velocity must not ramp\n");
    fc::VerticalEstimator kf; kf.reset(0); RangeHeight rh; rh.begin(0.04f, 4.0f); rh.cfg.ground_clearance_m = 0.06f; rh.reset();
    const float dt = 0.005f; float z = 2.0f, drift = 0, maxv = 0, maxe = 0; kf.reset(2.0f); uint32_t ms = 0;
    for (int i = 0; i < 24000; i++) {                                       // 2 min hovering at 2 m
        const float t = i * dt; ms = (uint32_t)(t * 1000);
        drift = 0.10f * t * 0.2f + 0.3f * sinf(t * 0.05f);                  // 2 cm/s walk + thermal swing
        kf.predict(0.25f + gauss(0.3f), dt);                                // accel bias 0.25 m/s^2
        if (i % 4 == 0) kf.update_baro(z + drift + gauss(0.3f));
        if (i % 6 == 0) {
            RangeReading r; r.distance_m = (z + 0.06f) + gauss(0.02f); r.quality_pct = 90;
            if (rh.update(r, 1.0f, ms, true)) { kf.update_range(rh.height_m(), rh.sigma_m()); }
        }
        if (t > 20) { maxv = fmaxf(maxv, fabsf(kf.state().v)); maxe = fmaxf(maxe, fabsf(kf.state().z - z)); }
    }
    CHECK(maxe < 0.12f, "height error with a drifting baro: %.3f m", maxe);
    CHECK(maxv < 0.25f, "no velocity ramp while baro and lidar disagree: %.3f m/s", maxv);
    // climb above the lidar range: estimate continues on baro with the learned offset, no jump
    float z_before = kf.state().z; float jump = 0, last = z_before; const float drift_at_loss = drift;
    for (int i = 0; i < 4000; i++) {
        const float t = 120 + i * dt; ms = (uint32_t)(t * 1000);
        z = 2.0f + fminf(i * dt * 0.8f, 5.0f);                               // up to 7 m (beyond the 3.6 m usable lidar range)
        drift = 0.10f * t * 0.2f + 0.3f * sinf(t * 0.05f);
        kf.predict(gauss(0.3f) + 0.25f, dt);
        if (i % 4 == 0) kf.update_baro(z + drift + gauss(0.3f));
        if (i % 6 == 0) { RangeReading r; r.distance_m = z + 0.06f; r.quality_pct = 90; if (rh.update(r, 1.0f, ms, true)) kf.update_range(rh.height_m()); }
        jump = fmaxf(jump, fabsf(kf.state().z - last - kf.state().v * dt) ); last = kf.state().z;
    }
    // with no drift-free source the remaining error is exactly the baro drift accrued since the lidar dropped out - nothing can observe it
    CHECK(fabsf(kf.state().z - z) < fabsf(drift - drift_at_loss) + 0.3f, "above lidar range: error %.2f m vs drift accrued %.2f m (est %.2f true %.2f)", fabsf(kf.state().z - z), fabsf(drift - drift_at_loss), kf.state().z, z);
    CHECK(jump < 0.15f, "no jump when the lidar drops out: %.3f", jump);
}

static void t_manager() {
    printf("[manager] enable switch, failures, health\n");
    FakeRange fr; RangeFinder rf; I2CBus bus(I2C_DEVICE_0); rf.attachI2C(bus);
    CHECK(rf.useDriver(&fr) && rf.available() && strcmp(rf.name(), "FAKE") == 0, "driver attached");
    rf.height().cfg.ground_clearance_m = 0.0f; rf.useDriver(&fr);
    uint32_t us = 0, ms = 0; bool got;
    fr.push(1.0f); got = rf.update(us += 33000, ms += 33, 1.0f, true);
    CHECK(got && fabsf(rf.heightM() - 1.0f) < 1e-5f && rf.valid(ms) && rf.rawM() == 1.0f, "sample flows");
    fr.push(1.0f); got = rf.update(us += 33000, ms += 33, 0.7f, true);
    CHECK(!got && rf.valid(ms), "tilted beyond 30 deg: rejected but previous still valid");
    CHECK(!rf.update(us += 33000, ms += 33, 1.0f, true), "no new sample => false");
    fr.ok = false; CHECK(!rf.valid(ms), "unhealthy driver => invalid (arm check)"); fr.ok = true;
    CHECK(!rf.valid(ms + 400), "stale => invalid");
    RangeFinder off; off.setEnabled(false); CHECK(!off.RangeInit() && strstr(off.failReason(), "disabled"), "switch off: '%s'", off.failReason());
    RangeFinder nopin; CHECK(!nopin.RangeInit() && strstr(nopin.failReason(), "pins not set"), "no pins: '%s'", nopin.failReason());
    RangeFinder nolib; nolib.attachI2C(bus); bool ok = nolib.RangeInit();
    CHECK(!ok && nolib.failReason()[0] != 0, "no chip / no library reports why: '%s'", nolib.failReason());
    CHECK(strcmp(nolib.name(), "none") == 0 && !nolib.available() && nolib.maxRangeM() == 0, "not available");
}

int main() {
    t_height(); t_clearance(); t_closed_loop(); t_manager();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
