// 3D mode (0.44, Betaflight's feature 3D): the throttle stick -> direction + magnitude (src/core/throttle3d.h, Betaflight rc.c /
// mixer.c calculateThrottleAndCurrentMotorEndpoints), and the motor values for reversible ESCs (src/hal/motor/motor_output.h,
// Betaflight dshot.c dshotInitEndpoints / pwm_output.c analogInitEndpoints).
// Build: g++ -std=c++17 -O2 test/test_mode3d.cpp -o build/test_mode3d
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../src/core/throttle3d.h"
#include "../src/hal/motor/motor_output.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static bool near(float a, float b, float e = 1e-4f) { return fabsf(a - b) <= e; }

static void t_stick() {
    printf("[stick] the throttle stick around its centre (3d_deadband_throttle 50 us)\n");
    fc::Mode3d m; m.cfg.enabled = true; m.cfg.deadband_us = 50;
    fc::Throttle3d o = m.update(1500, false, true, 0.001f);
    CHECK(o.low && !o.reversed && o.thr == 0.0f, "centre: stopped, forward, throttle low (arming allowed)");
    o = m.update(1600, false, true, 0.001f); CHECK(!o.reversed && near(o.thr, 50.0f / 450.0f) && !o.low, "1600: forward %.3f", o.thr);
    o = m.update(2000, false, true, 0.001f); CHECK(!o.reversed && near(o.thr, 1.0f), "2000: full forward");
    o = m.update(1400, false, true, 0.001f); CHECK(o.reversed && near(o.thr, 50.0f / 450.0f) && !o.low, "1400: reversed %.3f", o.thr);
    o = m.update(1000, false, true, 0.001f); CHECK(o.reversed && near(o.thr, 1.0f), "1000: full reverse");
    o = m.update(1500, false, true, 0.001f); CHECK(o.reversed && o.thr == 0.0f && o.low, "back to the centre from below: still reversed, at zero (Betaflight INVERTED_TO_DEADBAND)");
    m.update(1700, false, true, 0.001f); o = m.update(1520, false, true, 0.001f);
    CHECK(!o.reversed && o.thr == 0.0f && o.low, "from above: forward at zero (NORMAL_TO_DEADBAND)");
    CHECK(m.update(1451, false, true, 0.001f).low && !m.update(1449, false, true, 0.001f).low, "throttle low only inside the deadband");
    m.update(1200, false, true, 0.001f); m.update(1500, false, false, 0.001f); o = m.update(1500, false, true, 0.001f);
    CHECK(!o.reversed, "disarmed: the next arming starts forward (Betaflight: mid_rc while disarmed)");
    fc::Mode3d w; w.cfg.enabled = true; w.cfg.deadband_us = 100; o = w.update(1420, false, true, 0.001f);
    CHECK(o.low && o.thr == 0.0f, "deadband 100 us: 1420 is still the centre");
    fc::Mode3d c; c.cfg.enabled = true; c.cfg.deadband_us = 50; c.cfg.mid_us = 1550;   // Betaflight's 3D centre is mid_rc
    CHECK(c.update(1550, false, true, 0.001f).low && c.update(1580, false, true, 0.001f).low && !c.update(1610, false, true, 0.001f).reversed && c.update(1490, false, true, 0.001f).reversed,
          "mid_rc 1550: the stop band 1500 .. 1600, forward above, reversed below");
    fc::Mode3d s; s.cfg.enabled = true; s.cfg.switched = true; s.cfg.min_check_us = 1100;
    CHECK(s.update(1090, false, true, 0.001f).low && !s.update(1110, false, true, 0.001f).low && near(s.update(2000, false, true, 0.001f).thr, 1.0f), "switched: low below min_check 1100");
}

static void t_iterm() {
    printf("[iterm] the I-terms held at zero for 250 ms after a reversal (Betaflight)\n");
    fc::Mode3d m; m.cfg.enabled = true;
    m.update(1700, false, true, 0.001f); for (int i = 0; i < 300; i++) m.update(1700, false, true, 0.001f);
    CHECK(!m.iterm_hold(), "flying forward: I-terms free");
    m.update(1300, false, true, 0.001f); CHECK(m.iterm_hold(), "reversed: held");
    for (int i = 0; i < 240; i++) m.update(1300, false, true, 0.001f); CHECK(m.iterm_hold(), "240 ms later: still held");
    for (int i = 0; i < 20; i++) m.update(1300, false, true, 0.001f); CHECK(!m.iterm_hold(), "260 ms: free again");
    m.update(1500, false, true, 0.001f); for (int i = 0; i < 10; i++) m.update(1500, false, true, 0.001f);
    CHECK(!m.iterm_hold(), "into the deadband from below: no reversal (it stays reversed)");
}

static void t_switch() {
    printf("[switch] the 3D switch: 3d_switched_mode, or (off) the switch disables 3D\n");
    fc::Mode3d m; m.cfg.enabled = true; m.cfg.switched = true;
    fc::Throttle3d o = m.update(1000, false, true, 0.001f); CHECK(o.low && !o.reversed && o.thr == 0.0f, "switched, switch off: stick bottom = stopped, low");
    o = m.update(2000, false, true, 0.001f); CHECK(!o.reversed && near(o.thr, 1.0f) && !o.low, "full stick: full forward");
    o = m.update(2000, true, true, 0.001f); CHECK(o.reversed && near(o.thr, 1.0f), "switch on: the same stick, reversed");
    o = m.update(1500, true, true, 0.001f); CHECK(o.reversed && near(o.thr, 450.0f / 950.0f) && !o.low, "half stick reversed (%.3f)", o.thr);
    fc::Mode3d d; d.cfg.enabled = true;
    o = d.update(1000, true, true, 0.001f); CHECK(o.low && !o.reversed, "not switched, the switch on = 3D off: stick bottom = stopped");
    o = d.update(1500, true, true, 0.001f); CHECK(!o.reversed && near(o.thr, 450.0f / 950.0f), "... the whole stick forward (%.3f)", o.thr);
}

static void t_output() {
    printf("[output] DShot 3D: 48..1047 reversed, 1048..2047 forward; PWM: neutral / deadband / limits\n");
    motor::MotorOutputManager mo; mo.cfg.protocol = motor::PROTO_DSHOT600; mo.cfg.count = 4; mo.cfg.mode3d = true; mo.begin();
    float cmd[8] = {0.5f, 0.5f, 0.055f, 1.0f}; motor::Frame f;
    mo.update(1000, cmd, true, f, false, false);
    CHECK(f.value[0] == 1548 && f.value[2] == 1103 && f.value[3] == 2047, "forward: 0.5 -> %u, idle 0.055 -> %u, full -> %u", f.value[0], f.value[2], f.value[3]);
    mo.update(2000, cmd, true, f, false, true);
    CHECK(f.value[0] == 548 && f.value[2] == 103 && f.value[3] == 1047, "reversed: 0.5 -> %u, idle -> %u, full -> %u", f.value[0], f.value[2], f.value[3]);
    mo.update(3000, cmd, false, f); CHECK(f.value[0] == 0 && f.value[3] == 0, "disarmed: 0 (motor stop)");
    mo.test_motor(1, 0.2f, 4000); mo.update(5000, cmd, false, f); CHECK(f.value[1] == 1248 && f.value[0] == 0, "motor test: the forward range (%u)", f.value[1]);
    motor::MotorOutputManager pw; pw.cfg.protocol = motor::PROTO_PWM; pw.cfg.count = 4; pw.cfg.mode3d = true; pw.begin();
    pw.update(1000, cmd, true, f, false, false); CHECK(f.value[0] == 1757 && f.value[3] == 2000, "PWM forward: 1514 + 0.5 x 486 = %u, full %u", f.value[0], f.value[3]);
    pw.update(2000, cmd, true, f, false, true); CHECK(f.value[0] == 1203 && f.value[3] == 1000, "PWM reversed: 1406 - 0.5 x 406 = %u, full %u", f.value[0], f.value[3]);
    pw.update(3000, cmd, false, f); CHECK(f.value[0] == 1460, "PWM disarmed: neutral 1460 (%u)", f.value[0]);
    pw.cfg.neutral3d = 1500; pw.cfg.db3d_low = 1450; pw.cfg.db3d_high = 1550; pw.cfg.limit3d_low = 1100; pw.cfg.limit3d_high = 1900;
    pw.update(4000, cmd, true, f, false, false); CHECK(f.value[3] == 1900, "limit3d_high: %u", f.value[3]);
    pw.update(5000, cmd, true, f, false, true); CHECK(f.value[3] == 1100 && f.value[0] == 1275, "limit3d_low %u, half %u", f.value[3], f.value[0]);
    motor::MotorOutputManager n; n.cfg.protocol = motor::PROTO_DSHOT600; n.cfg.count = 4; n.begin();
    n.update(1000, cmd, true, f, false, true); CHECK(f.value[0] == 1048, "3D off: the reversed flag changes nothing (%u)", f.value[0]);
    motor::MotorOutputManager c3, c0; c3.cfg.protocol = c0.cfg.protocol = motor::PROTO_DRONECAN; c3.cfg.count = c0.cfg.count = 4; c3.cfg.mode3d = true; c3.begin(); c0.begin();
    motor::Frame f0; c3.update(1000, cmd, true, f, false, true); c0.update(1000, cmd, true, f0, false, false);
    CHECK(f.value[0] == f0.value[0] && f.value[3] == f0.value[3], "DroneCAN ESCs: no 3D range (%u = %u)", f.value[0], f0.value[0]);
}

int main() {
    t_stick();
    t_iterm();
    t_switch();
    t_output();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
