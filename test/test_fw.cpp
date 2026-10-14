// Firmware-in-the-loop: the real RX firmware body (FlightApp + drivers + flight core + settings + WBP server) on a virtual board.
// Build: g++ -std=c++17 -O2 -DARDUINO=1 -Itest/arduino_stub test/test_fw.cpp -o build/test_fw
//
// What is REAL here: every line of src/ except the ESP32 peripheral drivers (SPI/I2C/UART/RMT/LEDC/NVS/FreeRTOS).
// What is SIMULATED: the chips (register level), the airframe, the pilot's transmitter, the app (a frame-level client).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include "wblog_reader.h"      // before the Arduino stub: its macros (bit, word ...) would break <sstream>
#include "fw_rig.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace wbp;

static bool has_line(Rig& r, const char* needle) { for (int i = 0; i < r.app().report_lines(); i++) if (strstr(r.app().report()[i], needle)) return true; return false; }
static void arm_switch_setup(Rig& r) {                     // ARM on aux0 (ch5) high, ANGLE on aux1 high, ALTHOLD on aux2 high
    S_modes m; r.get_section(SEC_MODES, &m, sizeof(m));
    m.mode[0] = MODE_ARM; m.aux[0] = 0; m.lo[0] = 1700; m.hi[0] = 2100;
    m.mode[1] = MODE_ANGLE; m.aux[1] = 1; m.lo[1] = 1700; m.hi[1] = 2100;
    m.mode[2] = MODE_ALTHOLD; m.aux[2] = 2; m.lo[2] = 1700; m.hi[2] = 2100;
    r.set_section(SEC_MODES, &m, sizeof(m));
}
static std::unique_ptr<Rig> fresh(bool provision = true) {
    g_stub_us = 0; g_stub_delay_advances = true;
    std::unique_ptr<Rig> r(new Rig());
    if (provision) r->provision();
    r->boot();
    return r;
}

// ------------------------------------------------------------------------------------------------------------------------------
static bool status_now(Rig& r, T_status& st) { r.clear_inbox(); r.stream(STR_STATUS, 50); r.run(0.05); const bool ok = r.latest(STR_STATUS, st); r.stream(STR_STATUS, 0); return ok; }
static void t_loop() {
    printf("[loop] rate, lost cycles, where the time goes, output driver state\n");
    { auto r = fresh(); r->run(2.5); T_status st;
      CHECK(status_now(*r, st) && st.loop_hz == 4000 && st.loop_target == 4000 && st.overruns == 0 && !(st.blockers & ARM_BLOCKER_LOOP_RATE), "a loop that gets its time: %u Hz of %u, no cycle lost", st.loop_hz, st.loop_target);
      CHECK(st.t_imu == 0 && st.t_core == 0 && st.t_motor == 0 && st.t_rest == 0 && st.loop_us == 0, "(nothing takes time on the simulator unless told to)");
      // where the time goes: the IMU bus takes 30 us, the step after it 5 us, the motor output 12 us
      r->board.cost_imu_us = 30; g_stub_millis_cost_us = 5; static_cast<HostMotorHw*>(r->app().motors.hw())->on_write = [] { g_stub_us += 12; };
      r->run(2.2);
      const bool got = status_now(*r, st); g_stub_millis_cost_us = 0;
      CHECK(got && st.t_imu == 30 && st.t_core == 5 && st.t_motor == 12 && st.t_rest == 0, "time per stage: IMU %u, core %u, motors %u, log + telemetry %u us", st.t_imu, st.t_core, st.t_motor, st.t_rest);
      CHECK(st.loop_us == 47 && st.loop_max_us == 47 && st.cpu >= 13 && st.cpu <= 17, "cycle time %u us (max %u), load %u %%", st.loop_us, st.loop_max_us, st.cpu);
      // the sensor thread: an I2C transaction of the barometer takes 300 us - that is its time, not the control cycle's
      r->board.cost_imu_us = 0; static_cast<HostMotorHw*>(r->app().motors.hw())->on_write = nullptr; r->board.cost_i2c_us = 300; r->run(2.2);
      const bool got2 = status_now(*r, st); r->board.cost_i2c_us = 0;
      CHECK(got2 && st.t_slow_max >= 300 && st.t_slow_max <= 1500 && st.t_imu == 0 && st.t_core == 0 && st.t_motor == 0 && st.loop_us == 0, "a slow bus shows up as the sensor thread's time (%u us), not in the control cycle", st.t_slow_max); }
    { // one IMU sample = one burst on the SPI bus (not 15 single-byte transfers)
      auto r = fresh(); r->run(2.0); const uint32_t b0 = g_stub_spi_bursts, n0 = r->app().loops(); r->run(1.0);
      CHECK(r->app().loops() - n0 == 4000 && g_stub_spi_bursts - b0 == 4000, "IMU read in one SPI burst per cycle (%u bursts for %u cycles)", (unsigned)(g_stub_spi_bursts - b0), (unsigned)(r->app().loops() - n0)); }
    { // a pause: the cycles that did not happen are counted, and they are not run afterwards in a burst
      auto r = fresh(); r->run(2.3);
      const uint32_t n0 = r->app().loops();
      r->board.stall(2000);
      int ran = 0; for (int i = 0; i < 10; i++) if (r->app().fast_update()) ran++;
      CHECK(ran == 1 && r->app().loops() - n0 == 1, "after 2 ms without the CPU exactly one cycle runs, not eight in a row (%d)", ran);
      uint16_t seen = 0; for (int i = 0; i < 25; i++) { r->run(0.1); if (r->tel().overruns > seen) seen = r->tel().overruns; }
      CHECK(seen >= 6 && seen <= 8, "the 7 cycles that were lost are reported (%u; the one-second window may cut one cycle more or less)", seen);
      CHECK(!(r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "a pause is not 'the loop is too slow'"); }
    { // the gyro is integrated over the time that really passed, also across pauses
      auto r = fresh(); r->run(2.0); VirtualBoard& b = r->board;
      const float y0 = r->tel().yaw, t0 = b.plant.yaw_deg();
      b.plant.w[2] = 60.0f * fc::kDeg2Rad;
      for (int i = 0; i < 100; i++) { r->run(0.02); b.stall(1500); }                 // 2 s of turning; every 20 ms the CPU is away for 1.5 ms
      b.plant.w[2] = 0; r->run(0.2);
      float est = r->tel().yaw - y0, truth = b.plant.yaw_deg() - t0;
      if (est < 0) est += 360; if (truth < 0) truth += 360;
      CHECK(fabsf(truth - 129.0f) < 1.0f && fabsf(est - truth) < 1.0f, "turned %.1f deg by hand with pauses: the estimate says %.1f", truth, est); }
    { // a loop that cannot hold the configured rate: arming is refused (every filter and gain assumes that rate)
      auto r = fresh(); r->run(1.5); arm_switch_setup(*r);
      r->step_us = 303; r->run(3.0);                                                    // 3300 Hz of 4000: 82 %
      CHECK(r->tel().loop_hz >= 3290 && r->tel().loop_hz <= 3310 && !(r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "82 %% of the rate (%u Hz): tolerated", r->tel().loop_hz);
      r->step_us = 625; r->run(3.0);                                                    // 1600 Hz: what the board did with firmware 0.5
      CHECK(r->tel().loop_hz == 1600 && (r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "1600 Hz of 4000: arming blocker LOOP_RATE (0x%x)", r->tel().blockers);
      CHECK(r->tel().overruns >= 2300 && r->tel().overruns <= 2500, "and the lost cycles are shown (%u per second)", r->tel().overruns);
      r->board.rc[4] = 2000; r->run(1.0); CHECK(!r->tel().armed, "does not arm");
      r->board.rc[4] = 1000; r->step_us = 250; r->run(2.5);
      CHECK(r->tel().loop_hz == 4000 && !(r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "the rate is back: the blocker goes");
      r->board.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "and it arms");
      r->step_us = 625; r->run(4.0);
      CHECK(r->tel().armed && (r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "a loop that gets slow in flight never disarms the craft (the blocker only shows)");
      r->step_us = 250; }
    { // one slow moment (300 ms without the CPU) is not judged
      auto r = fresh(); r->run(2.2); bool blocked = false;
      r->board.stall(300000);
      for (int i = 0; i < 40; i++) { r->run(0.1); if (r->tel().blockers & ARM_BLOCKER_LOOP_RATE) blocked = true; }
      CHECK(!blocked, "a single pause of 300 ms does not block arming"); }
    { // a second during which the app did something with the craft is not judged - whenever in that second it was
      auto r = fresh(); r->run(1.5);
      r->step_us = 303; int waited = 0; while (r->tel().loop_hz == 4000 && waited++ < 400) r->run(0.005);       // (3300 Hz is tolerated; the number changes when a second of statistics ends)
      r->step_us = 625;                                                                                         // from the start of this second on: 1600 Hz
      CHECK(waited < 400 && r->command(CMD_MOTOR_TEST, {0, 100, 0}).err == ERR_OK, "a motor test of half a second at the start of a slow second");
      r->run(1.02); CHECK(r->board.motor_frac[0] == 0 && r->tel().loop_hz >= 1600 && r->tel().loop_hz < 1700, "the test is over, the second was slow (%u Hz)", r->tel().loop_hz);
      r->run(1.0); CHECK(r->tel().loop_hz == 1600 && !(r->tel().blockers & ARM_BLOCKER_LOOP_RATE), "the next second is slow as well: that is ONE slow second so far, the one with the motor test in it does not count");
      r->run(1.0); CHECK(r->tel().blockers & ARM_BLOCKER_LOOP_RATE, "a third: now two in a row were the loop's own doing - blocked");
      r->step_us = 250; }
    { // bidirectional DShot as the output driver really runs it
      auto r = fresh(); r->run(1.0); T_motors mo; r->stream(STR_MOTORS, 50); r->run(0.2);
      CHECK(r->latest(STR_MOTORS, mo) && mo.bidir == BIDIR_STATE_OFF && mo.answers == 0, "one-way DShot: bidirectional OFF, nobody answers");
      S_motors m; r->get_section(SEC_MOTORS, &m, sizeof(m)); m.bidirectional = 1; r->set_section(SEC_MOTORS, &m, sizeof(m));
      r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(2.0); r->stream(STR_MOTORS, 50); r->run(0.2);
      CHECK(r->latest(STR_MOTORS, mo) && mo.bidir == BIDIR_STATE_ACTIVE && mo.answers == 0x0F && (r->tel().health & HEALTH_ESC_TELEMETRY), "bidirectional on: ACTIVE, all four ESCs answer (0x%x)", mo.answers);
      r->board.esc_silent[2] = true; r->run(0.5); r->clear_inbox(); r->run(0.1);
      CHECK(r->latest(STR_MOTORS, mo) && mo.answers == 0x0B && !(r->tel().health & HEALTH_ESC_TELEMETRY), "the ESC of motor 3 stops answering: shown per motor (0x%x)", mo.answers);
      r->board.esc_silent[2] = false; r->run(0.2); r->clear_inbox(); r->run(0.1);
      CHECK(r->latest(STR_MOTORS, mo) && mo.answers == 0x0F, "and when it is back");
      // the IDF driver cannot listen
      r->get_section(SEC_MOTORS, &m, sizeof(m)); m.dshot_driver = DSHOT_DRIVER_IDF; r->set_section(SEC_MOTORS, &m, sizeof(m));
      r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(2.0); r->stream(STR_MOTORS, 50); r->run(0.2);
      CHECK(r->latest(STR_MOTORS, mo) && mo.bidir == BIDIR_STATE_UNAVAILABLE && mo.answers == 0, "bidirectional with the IDF driver: UNAVAILABLE, shown instead of silently doing nothing");
      CHECK(!(r->tel().health & HEALTH_ESC_TELEMETRY) && r->tel().rpm[0] == 0, "no rpm: the RPM filter and dynamic idle stay off");
      arm_switch_setup(*r); r->board.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed && r->board.motor_frac[0] > 0.03f, "the motors still run"); }
}

// ------------------------------------------------------------------------------------------------------------------------------
static void t_boot() {
    printf("[boot] detection, boot report, wiring\n");
    { auto r = fresh();
      CHECK(r->fw->began, "firmware starts on the virtual board");
      CHECK(has_line(*r, "imu: SPI ICM42688P") && has_line(*r, "baro: BMP280") && has_line(*r, "rx: CRSF on UART2") && has_line(*r, "motors: host x4"), "boot report names what was found");
      CHECK(has_line(*r, "range: FAILED"), "a missing sensor is reported with its reason, and nothing else breaks");
      CHECK(has_line(*r, "mag: FAILED - magnetometer disabled") || has_line(*r, "mag: FAILED"), "compass is off by default");
      CHECK(g_spi_begin_pins[0] == VirtualBoard::kSck && g_spi_begin_pins[1] == VirtualBoard::kMiso && g_spi_begin_pins[2] == VirtualBoard::kMosi && g_spi_begin_pins[3] == VirtualBoard::kCs,
            "IMU SPI pins come from the settings (sck %d miso %d mosi %d cs %d)", g_spi_begin_pins[0], g_spi_begin_pins[1], g_spi_begin_pins[2], g_spi_begin_pins[3]);
      CHECK(g_stub_spi_bus_refused == 0 && g_stub_spi_bus_started == FSPI, "the IMU's SPI bus has a number this chip knows (started bus %d, refused %d)", g_stub_spi_bus_started, g_stub_spi_bus_refused);
      CHECK(r->board.rc_uart_baud() == 420000 && Serial2.rx_pin == VirtualBoard::kRcRx && !Serial2.inverted, "CRSF: 420000 baud, not inverted, on the configured pin (%u, pin %d)", r->board.rc_uart_baud(), Serial2.rx_pin);
      CHECK(Serial1.rx_pin == VirtualBoard::kGpsRx && Serial1.tx_pin == VirtualBoard::kGpsTx, "GPS UART pins from the settings");
      r->run(2.0);
      const app::Telemetry& t = r->tel();
      CHECK((t.health & HEALTH_IMU) && (t.health & HEALTH_GYRO_CAL) && (t.health & HEALTH_BARO) && (t.health & HEALTH_RX) && (t.health & HEALTH_MOTORS), "health after 2 s: 0x%04x", t.health);
      CHECK(!(t.health & HEALTH_RANGE) && !(t.health & HEALTH_MAG_READY), "absent sensors are not healthy");
      CHECK(t.rc_link && t.rc_rate > 120 && t.rc_rate < 160 && t.rssi == 100, "receiver: link, %u frames/s, LQ %u", t.rc_rate, t.rssi);
      CHECK(t.blockers == 0, "nothing blocks arming on a healthy, calibrated, level craft (0x%x)", t.blockers);
      CHECK(fabsf(t.roll) < 0.3f && fabsf(t.pitch) < 0.3f, "level craft reads level: %.2f %.2f", t.roll, t.pitch);
      { // the pins the app stored (settings section "pins" in flash) are the pins the peripherals were really started on
        typedef VirtualBoard B; HostMotorHw* mh = static_cast<HostMotorHw*>(r->app().motors.hw());
        CHECK(g_spi_begin_pins[0] == B::kSck && g_spi_begin_pins[1] == B::kMiso && g_spi_begin_pins[2] == B::kMosi, "IMU SPI started on the stored pins (sck %d miso %d mosi %d)", g_spi_begin_pins[0], g_spi_begin_pins[1], g_spi_begin_pins[2]);
        CHECK(g_i2c_begin_pins[0] == B::kSda && g_i2c_begin_pins[1] == B::kScl, "sensor I2C bus started on the stored pins (%d %d)", g_i2c_begin_pins[0], g_i2c_begin_pins[1]);
        CHECK(Serial1.rx_pin == B::kGpsRx && Serial1.tx_pin == B::kGpsTx && Serial2.rx_pin == B::kRcRx, "GPS and receiver UARTs on the stored pins (%d %d %d)", Serial1.rx_pin, Serial1.tx_pin, Serial2.rx_pin);
        CHECK(mh && mh->pins[0] == B::kMotorPin[0] && mh->pins[1] == B::kMotorPin[1] && mh->pins[2] == B::kMotorPin[2] && mh->pins[3] == B::kMotorPin[3], "motor outputs on the stored pins");
        CHECK(HardwareConfig::appPins().imu_cs == B::kCs && HardwareConfig::appPins().motor[4] == GPIO_UNUSED, "HardwareConfig holds the app's pin table"); }
      r->run(4.0);
      CHECK(r->tel().gps_fix == 3 && r->tel().gps_sats == 14 && (r->tel().health & HEALTH_GPS) && r->tel().gps_state == GPS_STATE_RECEIVING_DATA, "GPS: auto-baud, configuration, 3D fix (fix %u sats %u state %u)", r->tel().gps_fix, r->tel().gps_sats, r->tel().gps_state);
      CHECK(r->tel().gps_baud == 115200 && r->board.gps.module_baud == 115200 && fabs(r->tel().lat * 1e-7 - 47.3977419) < 1e-6, "GPS at the configured baud rate (115200 by default) and reports the position"); }

    { auto r = fresh(false);                                // factory-fresh flash: no pins anywhere
      CHECK(!r->fw->began, "without pins the firmware does not pretend to work");
      CHECK(has_line(*r, "imu: FAILED") && has_line(*r, "motors: FAILED") && has_line(*r, "rx: FAILED - receiver RX pin not set"), "every missing pin is named in the boot report");
      CHECK(HardwareConfig::appPins().imu_cs == GPIO_UNUSED && HardwareConfig::appPins().motor[0] == GPIO_UNUSED, "pins of the previous boot do not survive in HardwareConfig");
      r->run(1.0);
      CHECK(r->tel().blockers & ARM_BLOCKER_NO_IMU, "no IMU => cannot arm");
      CHECK(r->request(MSG_HELLO).type == MSG_DEVICE_INFO, "the app link still works, so the pins can be configured"); }

    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision(); r.board.has_imu = false; r.boot();
      CHECK(!r.fw->began && has_line(r, "imu: FAILED - no known IMU answered on SPI"), "IMU chip missing: clear reason"); }
    { g_stub_us = 0; Rig r; r.provision(); r.board.has_baro = false; r.boot(); r.run(2.0);
      CHECK(r.fw->began && has_line(r, "baro: FAILED") && !(r.tel().health & HEALTH_BARO), "no barometer: firmware runs, baro unhealthy");
      CHECK(!r.tel().alt_valid, "no altitude source => altitude estimate is not valid"); }
}

static void t_gyro_cal() {
    printf("[gyro calibration] at boot, retry while moved, on request\n");
    { auto r = fresh(); r->run(1.5);
      CHECK(r->tel().cal_kind == CAL_KIND_GYRO && r->tel().cal_state == CAL_STATE_DONE, "boot calibration finished");
      float m[3] = {0, 0, 0}; for (int i = 0; i < 400; i++) { r->run(0.001); for (int k = 0; k < 3; k++) m[k] += r->tel().gyro_raw[k] / 400; }
      CHECK(fabsf(m[0]) < 0.03f && fabsf(m[1]) < 0.03f && fabsf(m[2]) < 0.03f, "the chip's 0.8/-0.5/0.3 deg/s bias is gone: %.3f %.3f %.3f", m[0], m[1], m[2]); }
    { g_stub_us = 0; Rig r; r.provision(); r.board.shaking = true; r.boot(); r.run(3.0);
      CHECK(!(r.tel().health & HEALTH_GYRO_CAL) && (r.tel().blockers & ARM_BLOCKER_GYRO_NOT_CAL), "craft moved during power-up: not calibrated, arming blocked");
      r.board.shaking = false; r.run(2.5);
      CHECK((r.tel().health & HEALTH_GYRO_CAL) && !(r.tel().blockers & ARM_BLOCKER_GYRO_NOT_CAL), "it retries by itself and succeeds once the craft is still"); }
    { auto r = fresh(); r->run(1.5);
      r->board.gyro_bias[0] = 3.0f;                         // temperature drift after boot
      r->run(0.2); CHECK(fabsf(r->tel().gyro_raw[0]) > 1.5f, "drifted gyro visible");
      const auto res = r->command(CMD_GYRO_CAL);
      CHECK(res.answered && res.err == ERR_OK, "GYRO_CAL accepted");
      r->run(0.3); CHECK(r->tel().cal_state == CAL_STATE_RUNNING && r->tel().cal_progress > 10 && r->tel().cal_progress < 60 && (r->tel().blockers & ARM_BLOCKER_CALIBRATING), "progress %u %%, arming blocked meanwhile", r->tel().cal_progress);
      r->run(1.0); CHECK(r->tel().cal_state == CAL_STATE_DONE && fabsf(r->tel().gyro_raw[0]) < 0.2f && !(r->tel().blockers & ARM_BLOCKER_CALIBRATING), "recalibrated on request"); }
}

static void t_flight() {
    printf("[flight] arm over CRSF, take off in Angle + AltHold, stick directions, failsafe on a lost transmitter\n");
    auto r = fresh(); r->run(1.5); arm_switch_setup(*r);
    VirtualBoard& b = r->board;
    b.rc[5] = 2000;                                         // ANGLE
    r->run(0.3);
    CHECK(!r->tel().armed && r->board.motor_frac[0] == 0, "disarmed: motors stopped");
    b.rc[4] = 2000; r->run(0.5);
    CHECK(r->tel().armed, "arm switch arms (blockers 0x%x)", r->tel().blockers);
    CHECK(b.motor_frac[0] > 0.03f && b.motor_frac[0] < 0.10f && b.plant.z == 0, "armed at zero throttle: motors idle (%.3f), craft stays on the ground", b.motor_frac[0]);
    // settings are locked while armed
    { S_pid p; r->get_section(SEC_PID, &p, sizeof(p)); p.p[0] *= 2; CHECK(r->ack_of(r->set_section(SEC_PID, &p, sizeof(p))) == ERR_ARMED, "settings cannot be changed while armed");
      CHECK(r->command(CMD_MOTOR_TEST, {0, 100, 0}).err == ERR_ARMED, "motor test refused while armed");
      CHECK(r->ack_of(r->request(MSG_REBOOT)) == ERR_ARMED && r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_ARMED, "no reboot / flash write while armed"); }
    b.rc[6] = 2000; r->run(0.2);                            // ALTHOLD
    b.rc[2] = 1700; r->run(4.0);                            // climb
    CHECK(b.plant.z > 1.0f, "throttle above mid in AltHold: it takes off and climbs (z = %.2f m)", b.plant.z);
    b.rc[2] = 1500; r->run(4.0);
    const float z_hold = b.plant.z; r->run(4.0);
    CHECK(fabsf(b.plant.z - z_hold) < 0.35f && fabsf(b.plant.vz) < 0.25f, "mid stick: holds its height (%.2f -> %.2f m, vz %.2f)", z_hold, b.plant.z, b.plant.vz);
    CHECK(fabsf(r->tel().z - b.plant.z) < 0.5f && r->tel().alt_valid, "height estimate follows the truth (est %.2f, true %.2f)", r->tel().z, b.plant.z);
    CHECK(fabsf(b.plant.right_down_deg()) < 2.0f && fabsf(b.plant.nose_down_deg()) < 2.0f, "hovers level (%.2f %.2f)", b.plant.right_down_deg(), b.plant.nose_down_deg());
    // Angle (Betaflight): the tilt is the tilt limit x the rate curve at that stick position - 0.4 of the travel (minus the deadband) gives 7.9 deg
    const float tilt_04 = r->app().core.cfg.angle.limit_deg * fc::apply_actual_rate(r->app().core.cfg.rates, fc::ROLL, (0.4f - 0.02f) / 0.98f) / r->app().core.cfg.rates.max_rate_dps[fc::ROLL];
    b.rc[0] = 1700; r->run(1.5);                            // roll stick right
    // (the craft is still gathering speed after 1.5 s: its accelerometer shows less tilt than there is, the estimate has given way a little - core/ahrs.h)
    CHECK(fabsf(tilt_04 - 7.9f) < 0.1f && b.plant.right_down_deg() > tilt_04 - 1.0f && b.plant.right_down_deg() < tilt_04 + 2.5f && fabsf(r->tel().roll - tilt_04) < 1.0f, "roll stick right => right side down, shown as + roll (true %.1f, shown %.1f, commanded %.1f)", b.plant.right_down_deg(), r->tel().roll, tilt_04);
    b.rc[0] = 1500; b.rc[1] = 1700;                         // pitch stick forward
    { float peak_true = 0, peak_shown = 0, peak_filt = 0;
      for (int i = 0; i < 300; i++) { r->run(0.001); const float w = -b.plant.gyro_dps().y; if (fabsf(w) > fabsf(peak_true)) { peak_true = w; peak_shown = r->tel().gyro_raw[1]; peak_filt = r->tel().gyro_f[1]; } }
      CHECK(peak_true > 20.0f && fabsf(peak_shown - peak_true) < 0.15f * peak_true + 3.0f && peak_filt > 0.3f * peak_true,
            "nose going down is a POSITIVE pitch rate in both gyro fields of the telemetry (true %.1f, raw %.1f, filtered %.1f deg/s)", peak_true, peak_shown, peak_filt); }
    r->run(1.2);
    CHECK(b.plant.nose_down_deg() > tilt_04 - 1.0f && b.plant.nose_down_deg() < tilt_04 + 2.5f && fabsf(r->tel().pitch - tilt_04) < 1.0f, "pitch stick forward => nose down, shown as + pitch (pilot convention; true %.1f, shown %.1f)", b.plant.nose_down_deg(), r->tel().pitch);
    b.rc[1] = 1500; r->run(2.0);
    // transmitter off
    const float z0 = b.plant.z;
    b.tx_on = false; r->run(0.45);
    CHECK(r->tel().rc_link, "link not declared lost before rx_timeout_ms (500 ms)");
    r->run(0.3);
    CHECK(!r->tel().rc_link && !(r->tel().health & HEALTH_RX), "no frames for 500 ms => link lost");
    r->run(2.0);
    CHECK(r->tel().failsafe == FAILSAFE_DESCENT && r->tel().armed, "failsafe: controlled descent, still armed (stage %u)", r->tel().failsafe);
    { T_status st{}; CHECK(status_now(*r, st) && st.failsafe == FAILSAFE_DESCENT && st.fs_cause == FS_CAUSE_LINK, "the status stream says why: cause LINK (0x%x)", st.fs_cause);
      // the transmitter comes back before the craft is down
      const float z_lost = b.plant.z; b.tx_on = true; r->run(0.6);
      CHECK(r->tel().rc_link && r->tel().failsafe == FAILSAFE_DESCENT, "transmitter back for 0.6 s: not believed yet, still descending (%.2f m)", b.plant.z);
      r->run(1.0); const float z_back = b.plant.z; r->run(2.0);
      CHECK(r->tel().failsafe == FAILSAFE_NONE && r->tel().armed && z_back > 0.2f && z_back < z_lost && fabsf(b.plant.z - z_back) < 0.25f && status_now(*r, st) && st.fs_cause == 0,
            "after a second of unbroken link the pilot has it back: Altitude Hold holds at %.2f m (the descent started at %.2f)", b.plant.z, z0);
      b.rc[0] = 1700; r->run(1.5); CHECK(b.plant.right_down_deg() > 5.0f, "and the sticks work (roll %.1f deg)", b.plant.right_down_deg()); b.rc[0] = 1500; r->run(1.5);
      b.tx_on = false; r->run(2.0); CHECK(r->tel().failsafe == FAILSAFE_DESCENT, "transmitter off for good"); }
    r->run(25.0);
    CHECK(b.plant.z < 0.05f && !r->tel().armed, "lands and disarms by itself (z %.2f from %.2f)", b.plant.z, z0);
    CHECK(r->tel().motor_cmd[0] == 0 && b.motor_frac[0] == 0, "motors off after the landing");
    b.tx_on = true; r->run(1.5);
    CHECK(!r->tel().armed && (r->tel().blockers & ARM_BLOCKER_ARM_SWITCH_ON || r->tel().blockers), "link back with the arm switch still on: does NOT re-arm (blockers 0x%x)", r->tel().blockers);
    b.rc[4] = 1000; b.rc[2] = 1000; r->run(0.5); b.rc[4] = 2000; r->run(0.5);
    CHECK(r->tel().armed, "after cycling the switch it arms again");
}

// The ground: arming only by the pilot's hand on a live link; an armed craft that stands still has its motors at idle.
static void t_ground() {
    printf("[ground] arm switch already on at power-up / when the link returns; motors with the throttle at the bottom (bench without props)\n");
    // ---- the craft is powered with the transmitter's arm switch on (modes are in the flash)
    { auto r = fresh(); r->run(1.5); arm_switch_setup(*r); CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK, "arm switch on aux 1, saved");
      VirtualBoard& b = r->board; b.rc[4] = 2000; b.rc[2] = 1000; b.tx_on = false;
      r->request(MSG_REBOOT); r->run(3.0);
      CHECK(r->boots == 2 && !r->tel().armed && !r->tel().rc_link, "rebooted, transmitter still off: disarmed, no link");
      b.tx_on = true; r->run(1.0);
      CHECK(r->tel().rc_link && !r->tel().armed && b.motor_frac[0] == 0 && (r->tel().blockers & fc::AB_ARM_SWITCH_ON), "transmitter on, arm switch ON, throttle low: the first frames do NOT arm it (blocker: arm switch)");
      r->run(3.0); CHECK(!r->tel().armed, "... however long it is left like that");
      b.rc[4] = 1000; r->run(0.3); CHECK(!r->tel().armed && r->tel().blockers == 0, "switch off: ready to arm");
      b.rc[4] = 2000; r->run(0.3); CHECK(r->tel().armed, "on again: armed by the pilot's hand");
      b.rc[4] = 1000; r->run(0.3);
      // out of range while disarmed, the pilot flips the switch, comes back
      b.tx_on = false; r->run(1.0); b.rc[4] = 2000; b.tx_on = true; r->run(1.5);
      CHECK(r->tel().rc_link && !r->tel().armed && (r->tel().blockers & fc::AB_ARM_SWITCH_ON), "link lost while disarmed, back with the arm switch on: not armed");
      b.rc[4] = 1000; r->run(0.2); b.rc[4] = 2000; r->run(0.3); CHECK(r->tel().armed, "cycled: armed"); b.rc[4] = 1000; r->run(0.3); }
    // ---- armed on the bench without props: the craft cannot answer the rate loop, and its gyro is a little off
    { auto spread = [](VirtualBoard& b) { float lo = 1, hi = 0; for (int i = 0; i < 4; i++) { lo = fminf(lo, b.motor_frac[i]); hi = fmaxf(hi, b.motor_frac[i]); } return hi - lo; };
      for (int always = 0; always < 2; always++) {
        auto r = fresh(); r->run(1.5); arm_switch_setup(*r); VirtualBoard& b = r->board;
        if (always) { S_general g; r->get_section(SEC_GENERAL, &g, sizeof(g)); CHECK(g.airmode == AIRMODE_AUTO, "default: airmode AUTO"); g.airmode = AIRMODE_ALWAYS; r->set_section(SEC_GENERAL, &g, sizeof(g)); r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(3.0); }
        r->stream(STR_TUNING, 20);
        b.plant.Ixx = b.plant.Iyy = b.plant.Izz = 1e6f; b.gyro_bias[0] += 0.3f; b.gyro_bias[2] -= 0.2f;                 // held down; the gyro has drifted since its calibration
        b.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "armed (Acro)");
        T_tuning tu{}; r->latest(STR_TUNING, tu);
        CHECK(tu.air_state == AIR_STATE_GROUND && spread(b) == 0.0f && fabsf(b.motor_frac[0] - 0.055f) < 0.002f, "armed, throttle down: every motor at idle (%.3f), state GROUND", b.motor_frac[0]);
        b.rc[0] = 1900; b.rc[1] = 1150; b.rc[3] = 1800; r->run(1.0);
        CHECK(spread(b) == 0.0f, "the pilot stirs roll, pitch and yaw with the throttle down: the motors do not answer");
        b.rc[0] = b.rc[1] = b.rc[3] = 1500; b.rc[2] = 1350; r->run(1.5); r->latest(STR_TUNING, tu);
        CHECK(tu.air_state == AIR_STATE_AIR && b.plant.z == 0.0f && spread(b) > 0.0f, "throttle to 32 %% for 1.5 s (the craft stays down): the rate loop runs (state %u, z %.3f, spread %.4f)", tu.air_state, b.plant.z, spread(b));
        b.rc[2] = 1000; r->run(0.5); r->latest(STR_TUNING, tu); const uint8_t at_05 = tu.air_state; const float s_05 = spread(b);
        r->run(1.5); r->latest(STR_TUNING, tu); const uint8_t at_2 = tu.air_state; const float s_2 = spread(b);
        r->run(28.0); r->latest(STR_TUNING, tu); const float s_30 = spread(b); const fc::CoreOutput& co = r->app().output();
        if (!always) CHECK(at_05 == AIR_STATE_AIRMODE && at_2 == AIR_STATE_GROUND && s_2 == 0.0f && s_30 == 0.0f && tu.air_state == AIR_STATE_GROUND && co.pid_i[0] == 0 && co.pid_i[2] == 0 && r->tel().armed,
                           "throttle back to the bottom: stabilised for one more second (AIRMODE), then GROUND - 2 s and 30 s later every motor is at idle, I-terms 0, still armed");
        else CHECK(at_2 == AIR_STATE_AIRMODE && tu.air_state == AIR_STATE_AIRMODE && s_30 > 0.3f && s_30 > 2.0f * s_05, "airmode ALWAYS: it never ends - 30 s later the motors have run %.0f %% apart (%.0f %% after 0.5 s)", s_30 * 100, s_05 * 100);
        b.rc[4] = 1000; r->run(0.3); CHECK(!r->tel().armed && b.motor_frac[0] == 0, "disarm stops them");
      } }
}

static void t_link_streams() {
    printf("[link] hello, streams decode to the plant's truth, rates, section round trip\n");
    auto r = fresh(); r->run(1.5);
    { const Rig::Rx h = r->request(MSG_HELLO);
      uint32_t hash = 0; if (h.p.size() >= 9) memcpy(&hash, h.p.data() + 5, 4);
      CHECK(h.type == MSG_DEVICE_INFO && h.p[0] == kProtocolVersion && h.p[1] == ROLE_RX && hash == kSchemaHash, "DEVICE_INFO carries the schema hash 0x%08X", hash); }
    r->board.rest(20.0f, -10.0f); r->run(3.0);              // right side down 20, nose UP 10
    r->clear_inbox();
    CHECK(r->stream(STR_ATTITUDE, 50) && r->stream(STR_STATUS, 10) && r->stream(STR_SENSORS, 100) && r->stream(STR_RC, 20), "streams subscribed");
    r->run(2.0);
    const int na = r->count(MSG_STREAM, STR_ATTITUDE), ns = r->count(MSG_STREAM, STR_STATUS), nx = r->count(MSG_STREAM, STR_SENSORS);
    CHECK(na >= 97 && na <= 101 && ns >= 19 && ns <= 21 && nx >= 195 && nx <= 201, "frame counts over 2 s match the requested rates: %d %d %d", na, ns, nx);
    T_attitude a{}; uint32_t t_ms = 0;
    CHECK(r->latest(STR_ATTITUDE, a, &t_ms) && fabs(a.roll * scale::attitude_roll - 20.0) < 0.5 && fabs(a.pitch * scale::attitude_pitch + 10.0) < 0.5, "attitude on the wire: roll %.2f pitch %.2f (true +20 / -10: nose up is negative)", a.roll * 0.01, a.pitch * 0.01);
    CHECK(millis() - t_ms < 30, "stream time stamp is current (%u vs %u)", t_ms, millis());
    T_sensors s{}; CHECK(r->latest(STR_SENSORS, s), "sensors frame");
    { const Vec3 acc = r->board.plant.acc();
      CHECK(fabs(s.acc[0] * scale::sensors_acc - acc.x) < 0.02 && fabs(s.acc[1] * scale::sensors_acc - acc.y) < 0.02 && fabs(s.acc[2] * scale::sensors_acc - acc.z) < 0.02, "accelerometer on the wire = plant (%.3f %.3f %.3f)", s.acc[0] * 0.001, s.acc[1] * 0.001, s.acc[2] * 0.001);
      CHECK(s.baro_pa > 101200 && s.baro_pa < 101450 && fabs(s.temp * scale::sensors_temp - 25.0) < 0.5, "pressure %u Pa, temperature %.2f C", s.baro_pa, s.temp * 0.01); }
    T_rc rc{}; r->board.rc[3] = 1234; r->run(0.2);
    CHECK(r->latest(STR_RC, rc) && abs((int)rc.ch[3] - 1234) <= 1 && rc.link == 1 && rc.rssi == 100, "rc on the wire: ch4 = %u", rc.ch[3]);
    T_status st{}; CHECK(r->latest(STR_STATUS, st) && st.armed == 0 && (st.health & HEALTH_IMU) && st.settings_dirty == 0, "status frame");
    CHECK(st.blockers & ARM_BLOCKER_ANGLE_TOO_HIGH || st.blockers == 0, "status blockers present");
    r->request(MSG_STREAM_CFG, {0xFF, 0, 0}); r->clear_inbox(); r->run(0.5);
    CHECK(r->count(MSG_STREAM) == 0, "all streams off");
    // gyro_fast at 1 kHz really delivers ~1000 frames/s (the snapshot is published at 1 kHz)
    r->stream(STR_GYRO_FAST, 1000); r->clear_inbox(); r->run(1.0);
    const int ng = r->count(MSG_STREAM, STR_GYRO_FAST);
    CHECK(ng >= 990 && ng <= 1001, "gyro_fast: %d frames in 1 s", ng);
    r->request(MSG_STREAM_CFG, {0xFF, 0, 0});
}

static void t_settings() {
    printf("[settings] live apply, reboot-required, save, power cycle, defaults\n");
    auto r = fresh(); r->run(1.5);
    S_pid p; CHECK(r->get_section(SEC_PID, &p, sizeof(p)), "read pid");
    CHECK(p.p[0] == 45 && r->app().core.cfg.pid.p[0] == 45 && p.slider_mode == SLIDER_MODE_RPY, "the default tune is in use");
    p.slider_pi = 150; p.p[0] = 9; p.tpa_rate = 40; p.stab_p[1] = 6.0f;                    // Tracking slider to 1.5; the 9 typed into a box the sliders own must not count
    Rig::Rx rep = r->set_section(SEC_PID, &p, sizeof(p)); S_pid stored; memcpy(&stored, rep.p.data() + 3, sizeof(stored));
    CHECK(rep.type == MSG_SECTION && stored.p[0] == 67 && stored.i[1] == 126 && stored.d[0] == 30, "pid set answered with the values in effect: the slider computed P 67 / I 126, the typed 9 is gone");
    CHECK(r->app().core.cfg.pid.p[0] == 67 && r->app().core.cfg.pid.tpa_rate == 40 && r->app().core.cfg.stab.kp[1] == 6.0f, "PID change is live in the flight core (P %u)", r->app().core.cfg.pid.p[0]);
    CHECK(r->app().core.pid().config().p[0] == 67 && r->app().core.pid().config().tpa_rate == 40, "...and in the rate controller itself, not only in the configuration");
    { S_filters f; CHECK(r->get_section(SEC_FILTERS, &f, sizeof(f)) && f.gyro_lpf1_dyn_min_hz == 250, "read filters"); f.slider_gyro_mult = 80; f.dyn_notch_count = 5; f.yaw_lowpass_hz = 60;
      rep = r->set_section(SEC_FILTERS, &f, sizeof(f)); S_filters fs; memcpy(&fs, rep.p.data() + 3, sizeof(fs));
      CHECK(fs.gyro_lpf1_dyn_min_hz == 200 && fs.gyro_lpf2_hz == 400 && r->app().core.cfg.gyro.lpf1_dyn_min_hz == 200 && r->app().core.cfg.pid.yaw_lowpass_hz == 60, "filters: the slider at 0.8 gives 200 / 400 Hz, live in the core's configuration");
      CHECK(r->app().core.gyro_filter().lpf1_cutoff_hz() == 200.0f && r->app().core.gyro_filter().dyn_notch().count() == 5 && r->app().core.pid().config().yaw_lowpass_hz == 60 && r->app().core.pid().config().p[0] == 67,
            "...and in the running filters (gyro low-pass 1 at %.0f Hz, %d dynamic notches); the PID gains are still the new ones", r->app().core.gyro_filter().lpf1_cutoff_hz(), r->app().core.gyro_filter().dyn_notch().count()); }
    r->stream(STR_STATUS, 50); r->run(0.1); T_status st{}; r->latest(STR_STATUS, st);
    CHECK(st.settings_dirty == SETTINGS_FLAGS_DIRTY, "unsaved, but no reboot needed for a live section (flags %u)", st.settings_dirty);
    { S_rates rt; r->get_section(SEC_RATES, &rt, sizeof(rt)); rt.max_rate[0] = 900; r->set_section(SEC_RATES, &rt, sizeof(rt));
      CHECK(r->app().core.cfg.rates.max_rate_dps[0] == 900, "rates are live");
      rt.angle_limit = 30; r->set_section(SEC_RATES, &rt, sizeof(rt)); CHECK(r->app().core.cfg.angle.limit_deg == 30 && r->app().core.cfg.stab.limit_deg == 30 && r->app().core.angle_ctl().config().limit_deg == 30 && r->app().core.stabilize().config().limit_deg == 30,
            "and the tilt limit of Angle / Stabilize with them - in the running controllers, not only in the configuration"); }
    { arm_switch_setup(*r); r->board.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "modes are live: the new arm switch works without a reboot"); r->board.rc[4] = 1000; r->run(0.3); }
    S_motors m; r->get_section(SEC_MOTORS, &m, sizeof(m)); m.protocol = ESC_PROTOCOL_DSHOT600; m.idle_x10 = 70;
    r->set_section(SEC_MOTORS, &m, sizeof(m)); r->run(0.1); r->latest(STR_STATUS, st);
    CHECK(st.settings_dirty == (SETTINGS_FLAGS_DIRTY | SETTINGS_FLAGS_REBOOT_REQUIRED) && r->app().motors.cfg().protocol != motor::PROTO_DSHOT600, "motor protocol: stored, flagged 'reboot required', NOT applied to the running outputs");
    { S_pid bad = stored; float nan = NAN; bad.stab_p[0] = nan; bad.stab_p[2] = 1e9f; bad.iterm_windup = 0; bad.ff_smooth_factor = 255; rep = r->set_section(SEC_PID, &bad, sizeof(bad)); S_pid back; memcpy(&back, rep.p.data() + 3, sizeof(back));
      CHECK(back.stab_p[0] == 4.5f && back.stab_p[2] == 12.0f && back.iterm_windup == 30 && back.ff_smooth_factor == 95 && r->app().core.cfg.stab.kp[0] == 4.5f && r->app().core.cfg.pid.iterm_windup == 30,
            "NaN / absurd values from a buggy app never reach the controller (stab P %.1f %.1f, windup %u)", back.stab_p[0], back.stab_p[2], back.iterm_windup); }
    CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK && r->flash.writes > 0, "saved to flash");
    r->run(0.1); r->latest(STR_STATUS, st); CHECK(!(st.settings_dirty & SETTINGS_FLAGS_DIRTY), "no longer unsaved");
    CHECK(r->ack_of(r->request(MSG_REBOOT)) == ERR_OK, "reboot acknowledged first");
    const int boots = r->boots; r->run(0.2); CHECK(r->boots == boots, "...then the firmware restarts ~300 ms later, not immediately");
    r->run(0.3); CHECK(r->boots == boots + 1, "restarted");
    r->run(1.5);
    CHECK(r->app().core.cfg.pid.p[0] == 67 && r->app().core.pid().config().p[0] == 67 && r->app().core.gyro_filter().dyn_notch().count() == 5 && r->app().core.cfg.gyro.lpf1_dyn_min_hz == 200 && r->app().motors.cfg().protocol == motor::PROTO_DSHOT600 &&
          fabsf(r->app().motors.cfg().idle - 0.07f) < 1e-6f, "after the restart everything saved is in use (P from the slider, filters, protocol, idle)");
    CHECK(has_line(*r, "imu: SPI ICM42688P"), "pins survived the restart");
    r->board.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "saved modes survive"); r->board.rc[4] = 1000; r->run(0.3);
    // a flash write that fails is reported
    r->flash.fail_write = true; CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_STORAGE, "flash failure is reported, not ignored"); r->flash.fail_write = false;
    // defaults of one section
    CHECK(r->ack_of(r->request(MSG_SETTINGS_DEFAULTS, {SEC_PID})) == ERR_OK && r->app().core.cfg.pid.p[0] == 45 && r->app().core.pid().config().p[0] == 45 && r->app().core.cfg.gyro.lpf1_dyn_min_hz == 200, "defaults for one section, applied live (the filters keep theirs)");
    // unused mode slots stay unused through a save / load cycle
    { S_modes mo; r->get_section(SEC_MODES, &mo, sizeof(mo)); CHECK(mo.mode[3] == 255 && mo.mode[15] == 255, "unused mode slots stay 255 (not clamped into a real mode): %u", mo.mode[3]); }
}

static void t_motor_test() {
    printf("[motor test] one motor, watchdog, arming interlock, direction / beacon commands\n");
    auto r = fresh(); r->run(1.5); arm_switch_setup(*r);
    VirtualBoard& b = r->board;
    CHECK(r->command(CMD_MOTOR_TEST, {2, 200, 0}).err == ERR_OK, "MOTOR_TEST motor 2 at 20 %%");
    r->run(0.05);
    CHECK(fabsf(b.motor_frac[2] - 0.2f) < 0.01f && b.motor_frac[0] == 0 && b.motor_frac[1] == 0 && b.motor_frac[3] == 0, "only motor 2 spins, at 20 %% (%.3f %.3f %.3f %.3f)", b.motor_frac[0], b.motor_frac[1], b.motor_frac[2], b.motor_frac[3]);
    CHECK(fabsf(r->tel().motor_cmd[2] - 0.2f) < 1e-3f && r->tel().motor_cmd[0] == 0, "the motors stream shows the test output (%.3f)", r->tel().motor_cmd[2]);
    b.rc[4] = 2000; r->run(0.2);
    CHECK(!r->tel().armed && (r->tel().blockers & ARM_BLOCKER_CALIBRATING), "cannot arm while the app is testing motors");
    b.rc[4] = 1000;
    r->run(0.6);
    CHECK(b.motor_frac[2] == 0, "not refreshed for 0.5 s => the motor stops by itself (app crash / cable pulled)");
    for (int i = 0; i < 10; i++) { r->command(CMD_MOTOR_TEST, {255, 150, 0}); r->run(0.1); }
    CHECK(fabsf(b.motor_frac[0] - 0.15f) < 0.01f && fabsf(b.motor_frac[3] - 0.15f) < 0.01f, "motor 255 = all motors, kept alive by refreshing");
    CHECK(r->command(CMD_MOTOR_STOP).err == ERR_OK, "MOTOR_STOP"); r->run(0.02); CHECK(b.motor_frac[0] == 0 && b.motor_frac[3] == 0, "stopped at once");
    CHECK(r->command(CMD_MOTOR_TEST, {4, 100, 0}).err == ERR_BAD_ARG && r->command(CMD_MOTOR_TEST, {0, 0xE9, 0x03}).err == ERR_BAD_ARG, "motor index / value out of range refused");
    CHECK(r->command(CMD_MOTOR_TEST, {0, 100}).err == ERR_BAD_ARG, "wrong argument size refused");
    CHECK(r->command(CMD_MOTOR_DIRECTION, {1, 1}).err == ERR_OK && r->app().motors.manager().commands_pending(), "direction command queued for the ESC (DShot)");
    r->run(1.0);
    CHECK(r->command(CMD_MOTOR_BEACON, {3}).err == ERR_OK && r->command(CMD_MOTOR_BEACON, {7}).err == ERR_BAD_ARG, "beacon");
    CHECK(r->command(200).err == ERR_UNKNOWN, "unknown command");
}

static void t_acc_cal() {
    printf("[accelerometer] 6-face calibration through the app link, level trim\n");
    auto r = fresh(); VirtualBoard& b = r->board;
    b.acc_offset[0] = 0.045f; b.acc_offset[1] = -0.030f; b.acc_offset[2] = 0.080f; b.acc_scale[0] = 1.012f; b.acc_scale[1] = 0.991f; b.acc_scale[2] = 1.020f;
    r->run(1.5); r->stream(STR_CALIBRATION, 20);
    b.rest(35, 20); r->run(2.0);
    { const float* a = r->tel().acc; const float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); CHECK(fabsf(n - 1.0f) > 0.02f, "uncalibrated chip: |acc| = %.3f g at an oblique attitude", n); }
    CHECK(r->command(CMD_ACC_CAL_FINISH).err == ERR_BAD_STATE, "FINISH before any face: refused");
    // faces are a few degrees off square on purpose, and in arbitrary order
    const float faces[6][2] = {{4, -3}, {176, 5}, {93, 2}, {-87, -4}, {3, 86}, {-5, -94}};
    uint8_t mask = 0;
    for (int f = 0; f < 6; f++) {
        b.rest(faces[f][0], faces[f][1]); r->run(0.3);
        CHECK(r->command(CMD_ACC_CAL_POINT).err == ERR_OK, "face %d: capture started", f);
        if (f == 0) { r->run(0.4); CHECK(r->command(CMD_ACC_CAL_POINT).err == ERR_BUSY && r->tel().cal_state == CAL_STATE_RUNNING && r->tel().cal_progress > 20, "a second POINT while averaging is refused (progress %u)", r->tel().cal_progress); }
        r->run(1.3);
        if (f == 3) CHECK(r->command(CMD_ACC_CAL_FINISH).err == ERR_BAD_STATE && r->tel().cal_mask != 0, "FINISH with missing faces: refused as 'not ready' (not as a failed fit), session kept");
        T_calibration c{}; r->latest(STR_CALIBRATION, c);
        CHECK(c.kind == CAL_KIND_ACCEL && c.state == CAL_STATE_DONE && c.mask != mask && __builtin_popcount(c.mask) == f + 1, "face %d captured: mask 0x%02x", f, c.mask);
        mask = c.mask;
    }
    CHECK(mask == 0x3F, "all six faces");
    { b.rest(2, 1); b.shaking = true; r->run(0.2); r->command(CMD_ACC_CAL_POINT); r->run(1.4);
      CHECK(r->tel().cal_state == CAL_STATE_FAILED && r->tel().cal_mask == 0x3F, "a capture while the craft is moved is rejected and changes nothing"); b.shaking = false; }
    { b.rest(45, 0); r->run(0.3); r->command(CMD_ACC_CAL_POINT); r->run(1.4); CHECK(r->tel().cal_state == CAL_STATE_FAILED, "a craft standing on an edge (45 deg) is not accepted as a face"); }
    b.rc[4] = 2000; r->run(0.1); CHECK(r->tel().blockers & ARM_BLOCKER_CALIBRATING, "arming blocked during the calibration session"); b.rc[4] = 1000;
    const auto fin = r->command(CMD_ACC_CAL_FINISH);
    CHECK(fin.err == ERR_OK, "FINISH: fit accepted (err %d)", fin.err);
    S_calibration cal; r->get_section(SEC_CALIBRATION, &cal, sizeof(cal));
    CHECK(cal.acc_valid == 1, "result is in the calibration section");
    for (int k = 0; k < 3; k++) CHECK(fabsf(cal.acc_offset[k] - b.acc_offset[k]) < 0.004f && fabsf(cal.acc_scale[k] - b.acc_scale[k]) < 0.004f, "axis %d: offset %.4f (true %.4f) scale %.4f (true %.4f)", k, cal.acc_offset[k], b.acc_offset[k], cal.acc_scale[k], b.acc_scale[k]);
    r->stream(STR_STATUS, 50); r->run(0.1); T_status st{}; r->latest(STR_STATUS, st);
    CHECK((st.settings_dirty & SETTINGS_FLAGS_DIRTY) && (st.health & HEALTH_ACC_CAL), "the app sees 'unsaved changes' and 'accelerometer calibrated'");
    b.rest(35, 20); r->run(2.0);
    { const float* a = r->tel().acc; const float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); CHECK(fabsf(n - 1.0f) < 0.006f, "calibrated: |acc| = %.4f g at the same oblique attitude", n);
      CHECK(fabsf(r->tel().roll - 35) < 0.5f && fabsf(r->tel().pitch - 20) < 0.5f, "and the attitude is right: %.2f %.2f (true 35 / 20)", r->tel().roll, r->tel().pitch); }
    // survives save + power cycle
    r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(2.5);
    CHECK(r->boots == 2 && (r->tel().health & HEALTH_ACC_CAL), "calibration is loaded at the next boot");
    { const float* a = r->tel().acc; const float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); CHECK(fabsf(n - 1.0f) < 0.006f, "and applied (|acc| = %.4f)", n); }
    // cancel discards a session
    b.rest(0, 0); r->run(0.3); r->command(CMD_ACC_CAL_POINT); r->run(1.4); CHECK(r->tel().cal_mask != 0, "new session started");
    CHECK(r->command(CMD_ACC_CAL_CANCEL).err == ERR_OK, "cancel"); r->run(0.1);
    CHECK(r->tel().cal_mask == 0 && !(r->tel().blockers & ARM_BLOCKER_CALIBRATING) && r->command(CMD_ACC_CAL_FINISH).err == ERR_BAD_STATE, "session gone, arming allowed again");
}

static void t_level_trim() {
    printf("[level trim] a crooked board is trimmed to level\n");
    auto r = fresh(); VirtualBoard& b = r->board;
    b.mount_roll_deg = 3.2f; b.mount_pitch_deg = -2.1f;
    r->run(4.0);
    CHECK(fabsf(r->tel().roll) > 1.5f && fabsf(r->tel().pitch) > 1.0f, "crooked board: a level craft reads %.2f / %.2f", r->tel().roll, r->tel().pitch);
    CHECK(r->command(CMD_LEVEL_TRIM).err == ERR_OK, "LEVEL_TRIM accepted");
    r->run(4.0);
    CHECK(fabsf(r->tel().roll) < 0.15f && fabsf(r->tel().pitch) < 0.15f, "after the trim it reads level: %.3f / %.3f", r->tel().roll, r->tel().pitch);
    { float a[3] = {0, 0, 0}; for (int i = 0; i < 500; i++) { r->run(0.001); for (int k = 0; k < 3; k++) a[k] += r->tel().acc[k] / 500; }
      CHECK(fabsf(a[0]) < 0.003f && fabsf(a[1]) < 0.003f && fabsf(a[2] + 1.0f) < 0.003f, "gravity in the board frame is (0,0,-1): %.4f %.4f %.4f", a[0], a[1], a[2]); }
    S_calibration cal; r->get_section(SEC_CALIBRATION, &cal, sizeof(cal));
    CHECK(abs(cal.acc_trim_roll) > 20 && abs(cal.acc_trim_pitch) > 10 && abs(cal.acc_trim_roll) < 45 && abs(cal.acc_trim_pitch) < 35, "trim stored: %.1f / %.1f deg", cal.acc_trim_roll * 0.1, cal.acc_trim_pitch * 0.1);
    CHECK(r->command(CMD_LEVEL_TRIM).err == ERR_OK, "trimming again is harmless"); r->run(3.0);
    CHECK(fabsf(r->tel().roll) < 0.15f && fabsf(r->tel().pitch) < 0.15f, "still level: %.3f / %.3f", r->tel().roll, r->tel().pitch);
    // tilted in flight direction: trimmed attitude must follow the craft, not the board
    b.rest(15, 10); r->run(3.0);
    CHECK(fabsf(r->tel().roll - 15) < 0.4f && fabsf(r->tel().pitch - 10) < 0.4f, "trimmed craft tilted 15 / 10 reads %.2f / %.2f", r->tel().roll, r->tel().pitch);
    // persisted
    r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); b.rest(0, 0); r->run(5.0);
    CHECK(r->boots == 2 && fabsf(r->tel().roll) < 0.15f && fabsf(r->tel().pitch) < 0.15f, "trim survives a power cycle: %.3f / %.3f", r->tel().roll, r->tel().pitch);
    // not a trim: 30 degrees
    b.rest(30, 0); r->run(3.0); CHECK(r->command(CMD_LEVEL_TRIM).err == ERR_FAILED, "30 degrees is refused (that is a board alignment, not a trim)");
    b.rest(0, 0); r->run(2.0); const float flat = fabsf(r->tel().roll);              // lying flat again, the horizon has followed: only the movement is in the way now
    b.shaking = true; r->run(0.5); CHECK(flat < 1.0f && r->command(CMD_LEVEL_TRIM).err == ERR_FAILED, "refused while the craft is being moved (it lies flat: %.2f deg)", flat); b.shaking = false;
}

static void rotate_everywhere(Rig& r, double seconds_per_pose = 0.03) {
    for (int yaw = 0; yaw < 360; yaw += 30) for (int roll = -180; roll < 180; roll += 30) for (int pitch = -60; pitch <= 60; pitch += 60) {
        r.board.plant.set_tilt((float)roll, (float)pitch, (float)yaw); r.board.plant.ground = true; r.board.plant.z = 0;
        r.run(seconds_per_pose);
    }
}
static void t_mag_cal() {
    printf("[compass] calibration through the app link, heading, arming interlock\n");
    g_stub_us = 0; g_stub_delay_advances = true;
    Rig r; r.provision();
    { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.sensors.mag_enabled = 1; st.save(s); }
    r.board.mag_hard[0] = 0.12; r.board.mag_hard[1] = -0.08; r.board.mag_hard[2] = 0.05;
    r.boot(); r.run(2.0); r.stream(STR_CALIBRATION, 20);
    CHECK(has_line(r, "mag: QMC5883L (not calibrated)"), "compass detected, flagged as not calibrated");
    CHECK(r.tel().blockers & ARM_BLOCKER_MAG_NOT_CALIBRATED, "compass on but not calibrated: arming blocked");
    CHECK(r.command(CMD_MAG_CAL_FINISH).err == ERR_BAD_STATE, "FINISH without START refused");
    CHECK(r.command(CMD_MAG_CAL_START).err == ERR_OK, "calibration started");
    r.run(0.5);
    { const auto early = r.command(CMD_MAG_CAL_FINISH);
      CHECK(early.err == ERR_FAILED && early.data.size() == 1 && early.data[0] != MAG_CAL_RESULT_OK, "finishing without rotating fails and says why (reason %d)", early.data.empty() ? -1 : early.data[0]);
      r.run(0.2); CHECK(r.tel().cal_kind == CAL_KIND_MAG && r.tel().cal_state == CAL_STATE_RUNNING, "...and keeps collecting"); }
    rotate_everywhere(r);
    T_calibration c{}; r.latest(STR_CALIBRATION, c);
    CHECK(c.kind == CAL_KIND_MAG && c.progress >= 80, "progress after rotating through all attitudes: %u %%", c.progress);
    const auto fin = r.command(CMD_MAG_CAL_FINISH);
    CHECK(fin.err == ERR_OK && fin.data.size() == 1 && fin.data[0] == MAG_CAL_RESULT_OK, "fit accepted (err %d reason %d)", fin.err, fin.data.empty() ? -1 : fin.data[0]);
    S_calibration cal; r.get_section(SEC_CALIBRATION, &cal, sizeof(cal));
    CHECK(cal.mag_valid == 1 && fabs(cal.mag_offset[0] - 0.12) < 0.02 && fabs(cal.mag_offset[1] + 0.08) < 0.02 && fabs(cal.mag_offset[2] - 0.05) < 0.02, "hard iron found: %.3f %.3f %.3f (true 0.12 -0.08 0.05)", cal.mag_offset[0], cal.mag_offset[1], cal.mag_offset[2]);
    r.board.rest(0, 0); r.board.plant.set_tilt(0, 0, 135); r.board.plant.ground = true; r.run(8.0);
    CHECK((r.tel().health & HEALTH_MAG_CAL) && (r.tel().health & HEALTH_MAG_READY) && !(r.tel().blockers & ARM_BLOCKER_MAG_NOT_CALIBRATED), "calibrated compass is healthy and no longer blocks arming");
    { float e = r.tel().yaw - 135.0f; while (e > 180) e -= 360; while (e < -180) e += 360; CHECK(fabsf(e) < 3.0f && r.tel().yaw_from_mag, "heading from the compass: %.1f (true 135)", r.tel().yaw); }
    r.request(MSG_SETTINGS_SAVE); r.request(MSG_REBOOT); r.run(6.0);
    CHECK(r.boots == 2 && has_line(r, "mag: QMC5883L") && !has_line(r, "not calibrated") && (r.tel().health & HEALTH_MAG_CAL), "compass calibration survives a power cycle");
    CHECK(r.command(CMD_MAG_CAL_START).err == ERR_OK && r.command(CMD_MAG_CAL_CANCEL).err == ERR_OK, "start + cancel"); r.run(0.1);
    CHECK((r.tel().health & HEALTH_MAG_CAL) && !(r.tel().blockers & ARM_BLOCKER_CALIBRATING), "cancel keeps the old calibration");
    { auto n = fresh(); n->run(1.0); CHECK(n->command(CMD_MAG_CAL_START).err == ERR_NO_SENSOR, "no compass => NO_SENSOR"); }
}

static void t_misc() {
    printf("[misc] report, set home, clock wrap, idle re-sync of a broken frame\n");
    { // the control loop runs at the configured rate whatever the rate loop() is called at (here: 20 kHz)
      auto r = fresh(); r->run(1.0); const uint32_t n0 = r->app().loops();
      for (int i = 0; i < 40000; i++) r->board.step(r->app(), 50);
      const uint32_t n = r->app().loops() - n0;
      CHECK(n >= 7990 && n <= 8010, "4 kHz control loop while loop() spins at 20 kHz: %u cycles in 2 s", n);
      r->run(1.5); CHECK(r->tel().loop_hz >= 3990 && r->tel().loop_hz <= 4010, "status.loop_hz reports the real rate: %u", r->tel().loop_hz); }
    { // compass switched on in the settings but no chip on the board: it must not block arming for ever
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.sensors.mag_enabled = 1; st.save(s); }
      r.board.has_mag = false; r.boot(); r.run(2.0); arm_switch_setup(r);
      CHECK(has_line(r, "mag: FAILED") && !r.app().core.cfg.use_mag && !(r.tel().blockers & ARM_BLOCKER_MAG_NOT_CALIBRATED), "compass enabled but absent: reported, not used, does not block arming (0x%x)", r.tel().blockers);
      r.board.rc[4] = 2000; r.run(0.5); CHECK(r.tel().armed, "and the craft arms"); }
    { auto r = fresh(); r->run(1.0); r->clear_inbox();
      CHECK(r->command(CMD_REPORT).err == ERR_OK, "REPORT");
      const auto lg = r->logs(); bool imu = false, range = false; for (const auto& l : lg) { if (l.find("imu: SPI ICM42688P") != std::string::npos) imu = true; if (l.find("range: FAILED") != std::string::npos) range = true; }
      for (const auto& l : lg) if (lg.size() != 12) printf("    %s\n", l.c_str());
      CHECK(lg.size() == 12 && imu && range, "boot report re-sent as %zu LOG lines", lg.size());
      CHECK(r->command(CMD_SET_HOME).err == ERR_NO_SENSOR, "SET_HOME before a fix: refused");
      r->run(6.0);
      CHECK(r->command(CMD_SET_HOME).err == ERR_OK && r->app().gnss.home().has_home(), "SET_HOME with a 3D fix"); }
    { // micros() wraps after 71.6 minutes: boot 2 s before the wrap and fly through it
      g_stub_us = 0xFFFFFFFFull - 2000000ull; g_stub_delay_advances = true;
      Rig r; r.provision(); r.boot(); r.run(1.5); arm_switch_setup(r);
      r.board.rc[5] = 2000; r.board.rc[6] = 2000; r.board.rc[4] = 2000; r.run(0.3); r.board.rc[2] = 1650;
      CHECK(r.tel().armed, "armed just before the wrap");
      r.run(6.0);
      CHECK(g_stub_us > 0x100000000ull && r.tel().armed && r.board.plant.z > 0.5f && r.tel().alt_valid && (r.tel().health & HEALTH_BARO) && r.tel().rc_link,
            "flew through the micros() wrap: armed %d z %.2f alt_valid %d health 0x%x", r.tel().armed, r.board.plant.z, r.tel().alt_valid, r.tel().health);
      CHECK(fabsf(r.board.plant.right_down_deg()) < 3 && fabsf(r.board.plant.nose_down_deg()) < 3, "and stayed level"); }
    { // the IMU stops answering in flight (broken wire): motors are stopped explicitly and it stays disarmed
      auto r = fresh(); r->run(1.5); arm_switch_setup(*r);
      r->board.rc[5] = 2000; r->board.rc[4] = 2000; r->run(0.4); r->board.rc[2] = 1600; r->run(1.0);
      CHECK(r->tel().armed && r->board.motor_frac[0] > 0.2f, "flying");
      r->board.has_imu = false; r->run(0.003);
      CHECK(r->tel().armed, "a few missed samples (3 ms) are ridden through");
      r->run(0.02);
      CHECK(!r->tel().armed && r->board.motor_frac[0] == 0 && (r->tel().blockers & ARM_BLOCKER_NO_IMU), "IMU silent for ~13 ms (32 frozen samples + 20 failed cycles): motors stopped, blocker NO_IMU shown to the app");
      r->board.has_imu = true; r->board.rest(0, 0); r->run(1.0);
      CHECK(!r->tel().armed && r->board.motor_frac[0] == 0, "IMU answers again with the arm switch still on: stays disarmed");
      r->board.rc[2] = 1000; r->board.rc[4] = 1000; r->run(0.3); r->board.rc[4] = 2000; r->run(0.5);
      CHECK(r->tel().armed, "arming again needs the switch to be cycled"); }
    { auto r = fresh(); r->run(1.0);
      // a frame cut in half by a cable glitch, then silence, then a good frame: the server must answer the good one
      uint8_t buf[64]; const uint8_t pl[1] = {SEC_PID}; const size_t n = wbp::encode(buf, sizeof(buf), MSG_SECTION_GET, 9, pl, 1);
      r->fw->server.on_bytes(buf, n - 3); r->fw->server.on_idle();
      CHECK(r->request(MSG_HELLO).type == MSG_DEVICE_INFO, "half a frame + silence does not block the next request"); }
}

// ---- buzzer pin as the board saw it: lengths (ms) of the on / off runs, starting at the first "on" after t0
static std::vector<int> buzz_runs(const VirtualBoard& b, uint32_t t0, uint32_t t1) {
    std::vector<int> r; size_t i = 0; bool on = false;
    while (i < b.buzz.size() && b.buzz[i].ms <= t0) on = b.buzz[i++].on;
    uint32_t start = t0;
    if (!on) { while (i < b.buzz.size() && !b.buzz[i].on) i++; if (i >= b.buzz.size() || b.buzz[i].ms >= t1) return r; start = b.buzz[i].ms; on = true; i++; }
    uint32_t last = start;
    for (; i < b.buzz.size() && b.buzz[i].ms < t1; i++) { if (b.buzz[i].on == on) continue; r.push_back((int)(b.buzz[i].ms - last)); last = b.buzz[i].ms; on = b.buzz[i].on; }
    r.push_back((int)(t1 - last));
    return r;
}
static bool runs_are(const std::vector<int>& runs, std::initializer_list<int> want, int tol = 3) {
    size_t i = 0; for (int w : want) { if (i >= runs.size() || abs(runs[i] - w) > tol) return false; i++; } return true;
}
static std::string show(const std::vector<int>& runs) { std::string o; for (size_t i = 0; i < runs.size() && i < 8; i++) o += std::to_string(runs[i]) + " "; return o; }

static void t_battery() {
    printf("[battery] ADC voltage, cell count, arming, failsafe landing, live calibration\n");
    { auto r = fresh(); VirtualBoard& b = r->board;
      CHECK(has_line(*r, "battery: ADC") && r->app().status.battery, "boot report: battery on the ADC pin");
      r->run(2.0);
      const app::Telemetry& t = r->tel();
      CHECK(fabsf(t.batt_v - 16.0f) < 0.05f && t.batt_cells == 4 && fabsf(t.cell_v - 4.0f) < 0.02f && t.batt_state == BATT_STATE_OK && t.batt_source == BATT_SOURCE_ADC,
            "16.0 V pack behind an 11:1 divider: %.2f V, %u cells, %.3f V per cell", t.batt_v, t.batt_cells, t.cell_v);
      CHECK(abs((int)t.batt_pin_mv - 1455) <= 3 && (t.health & HEALTH_BATTERY), "pin voltage as the app shows it for the calibration: %u mV", t.batt_pin_mv);
      CHECK(g_stub_adc_reads >= 90 && g_stub_adc_reads <= 110, "the ADC is read 50 times a second, not every loop (%u reads in 2 s)", g_stub_adc_reads);
      r->stream(STR_BATTERY, 10); r->stream(STR_STATUS, 10); r->run(0.5);
      T_battery tb{}; T_status ts{};
      CHECK(r->latest(STR_BATTERY, tb) && abs((int)tb.voltage - 16000) < 50 && abs((int)tb.cell - 4000) < 15 && tb.cells == 4 && tb.state == BATT_STATE_OK && tb.source == BATT_SOURCE_ADC,
            "battery stream: %u mV, %u mV per cell", tb.voltage, tb.cell);
      CHECK(r->latest(STR_STATUS, ts) && abs((int)ts.cell_mv - 4000) < 15, "status stream carries the cell voltage: %u mV", ts.cell_mv);

      // ---- the divider is calibrated from the app while running: the board really has 11.00, the user had typed 10.00
      S_battery sb; r->get_section(SEC_BATTERY, &sb, sizeof(sb)); sb.divider = 1000; r->set_section(SEC_BATTERY, &sb, sizeof(sb)); r->run(3.0);
      CHECK(fabsf(r->tel().batt_v - 16.0f * 10.0f / 11.0f) < 0.05f, "wrong divider (10.00): reads %.2f V", r->tel().batt_v);
      sb.divider = 1100; CHECK(r->set_section(SEC_BATTERY, &sb, sizeof(sb)).type == MSG_SECTION, "divider corrected without a reboot"); r->run(3.0);
      CHECK(fabsf(r->tel().batt_v - 16.0f) < 0.05f && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "reads %.2f V again, no reboot asked for", r->tel().batt_v);

      // ---- a low pack does not take off
      arm_switch_setup(*r);
      b.batt_v = 13.8; r->run(3.0);
      CHECK(r->tel().batt_state == BATT_STATE_WARNING && (r->tel().blockers & ARM_BLOCKER_BATT_LOW), "13.8 V (3.45 V per cell): warning, arming blocked (state %u, blockers 0x%x)", r->tel().batt_state, r->tel().blockers);
      b.rc[4] = 2000; r->run(0.5);
      CHECK(!r->tel().armed && b.motor_frac[0] == 0, "arm switch on a low pack: stays disarmed");
      b.rc[4] = 1000; b.batt_v = 16.0; r->run(3.0);
      CHECK(r->tel().batt_state == BATT_STATE_OK && !(r->tel().blockers & ARM_BLOCKER_BATT_LOW), "fresh pack: blocker gone");
      b.rc[4] = 2000; r->run(0.5);
      CHECK(r->tel().armed, "and it arms"); }

    { // critical in flight: controlled landing. The pack sags under load, which must not trigger it early.
      auto r = fresh(); VirtualBoard& b = r->board; arm_switch_setup(*r); r->run(2.0);
      b.rc[5] = 2000; b.rc[4] = 2000; r->run(0.5); b.rc[6] = 2000; r->run(0.2); b.rc[2] = 1700; r->run(4.0); b.rc[2] = 1500; r->run(4.0);
      CHECK(r->tel().armed && b.plant.z > 1.0f && r->tel().batt_state == BATT_STATE_OK && r->tel().batt_v < 15.9f && r->tel().batt_v > 15.3f,
            "hovering on a 16.0 V pack: sags to %.2f V under load, state OK", r->tel().batt_v);
      CHECK(b.buzz_ms(millis() - 3000, millis()) == 0, "in flight with a good pack the buzzer is silent (no 'armed, throttle down' reminder with the stick up)");
      b.batt_v = 14.3; r->run(4.0);                         // resting 3.575 V per cell, about 3.49 under load
      CHECK(r->tel().batt_state == BATT_STATE_WARNING && r->tel().failsafe == FAILSAFE_NONE && r->tel().armed, "pack at the warning level in flight: warning only, keeps flying (%.2f V)", r->tel().batt_v);
      b.batt_v = 13.3; r->run(1.5);
      CHECK(r->tel().failsafe == FAILSAFE_NONE, "below the minimum for 1.5 s: not yet (2 s delay + filter)");
      r->run(3.0);
      CHECK(r->tel().batt_state == BATT_STATE_CRITICAL && r->tel().failsafe == FAILSAFE_DESCENT && r->tel().armed, "critical: failsafe descent starts (state %u, failsafe %u)", r->tel().batt_state, r->tel().failsafe);
      { T_status st{}; const float z_now = b.plant.z; b.rc[0] = 1700; b.rc[2] = 1900; r->run(1.2);
        CHECK(status_now(*r, st) && st.fs_cause == FS_CAUSE_BATTERY && b.plant.right_down_deg() > 5.0f && b.plant.z < z_now,
              "cause BATTERY in the status stream; the link is fine, so the pilot still steers (roll %.1f deg) - but full throttle does not stop the descent (%.2f -> %.2f m)", b.plant.right_down_deg(), z_now, b.plant.z);
        b.rc[0] = 1500; b.batt_v = 15.5; r->run(3.0);
        CHECK(r->tel().failsafe == FAILSAFE_DESCENT && r->tel().armed, "the pack recovers once the motors are unloaded: the landing goes on"); }
      r->run(25.0);
      CHECK(b.plant.z < 0.05f && !r->tel().armed, "it lands and disarms (z %.2f)", b.plant.z); }

    { // the same with land_on_critical switched off: beeps, but the pilot keeps control
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.battery.land_on_critical = 0; st.save(s); }
      r.boot(); VirtualBoard& b = r.board; arm_switch_setup(r); r.run(2.0);
      b.rc[5] = 2000; b.rc[4] = 2000; r.run(0.5); b.rc[6] = 2000; r.run(0.2); b.rc[2] = 1700; r.run(4.0); b.rc[2] = 1500; r.run(2.0);
      b.batt_v = 13.3; r.run(6.0);
      CHECK(r.tel().batt_state == BATT_STATE_CRITICAL && r.tel().failsafe == FAILSAFE_NONE && r.tel().armed && b.plant.z > 1.0f, "land_on_critical off: critical battery, still flying under pilot control");
      const uint32_t now = millis();
      CHECK(b.buzz_ms(now - 2000, now) > 1800, "but the buzzer screams (on %u of the last 2000 ms)", b.buzz_ms(now - 2000, now)); }

    { // no ADC pin: AUTO falls back to the ESCs; nothing reports -> "no battery", which blocks nothing
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.batt_adc = 255; st.save(s); }
      r.boot(); r.run(2.0);
      CHECK(has_line(r, "battery: ESC telemetry") && r.tel().batt_source == BATT_SOURCE_ESC && r.tel().batt_state == BATT_STATE_NOT_PRESENT && r.tel().cell_v == 0.0f && !(r.tel().blockers & ARM_BLOCKER_BATT_LOW),
            "no ADC pin: source = ESC telemetry, no reading, no battery, no blocker"); }
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.batt_adc = 40; s.battery.source = BATT_SOURCE_ADC; st.save(s); }
      r.boot(); r.run(1.0);
      CHECK(has_line(r, "battery: FAILED - battery pin is not an ADC pin") && !r.app().status.battery, "GPIO40 cannot measure: said so in the boot report"); }
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.battery.source = BATT_SOURCE_NONE; st.save(s); }
      r.boot(); r.board.batt_v = 12.0; r.run(4.0);
      CHECK(r.tel().batt_state == BATT_STATE_NOT_PRESENT && r.tel().batt_source == BATT_SOURCE_NONE && !(r.tel().blockers & ARM_BLOCKER_BATT_LOW), "monitoring switched off: a flat pack is not noticed and blocks nothing"); }
    { // running from USB only (0 V on the divider)
      auto r = fresh(); r->board.batt_v = 0.0; r->run(5.0);
      CHECK(r->tel().batt_state == BATT_STATE_NOT_PRESENT && r->tel().batt_cells == 0 && !(r->tel().health & HEALTH_BATTERY) && r->tel().blockers == 0, "USB power, no pack: no battery, arming not blocked (the motors have no power anyway)"); }
}

static void t_beeper() {
    printf("[beeper] the buzzer pin: boot chirp, arming, lost signal, battery, bench silence, test button, DShot beacon\n");
    { auto r = fresh(); VirtualBoard& b = r->board;
      CHECK(has_line(*r, "beeper: active buzzer") && r->app().status.buzzer, "boot report: buzzer");
      r->run(2.5);
      const std::vector<int> boot = buzz_runs(b, 0, 600);
      CHECK(runs_are(boot, {30, 50, 30}), "power-on chirp: two 30 ms beeps (%s)", show(boot).c_str());
      // gyro calibration finished about one second after boot
      const std::vector<int> cal = buzz_runs(b, 700, 2500);
      CHECK(runs_are(cal, {200, 100, 200, 100, 200}), "gyro calibrated: three beeps (%s)", show(cal).c_str());
      r->run(4.0);                                          // GPS fix arrives (3 s) -> ready tune; let it finish
      arm_switch_setup(*r); r->run(3.5);                    // > 3 s without app traffic: "the app is gone"
      uint32_t t0 = millis(); b.rc[4] = 2000; r->run(1.5);
      const std::vector<int> arm = buzz_runs(b, t0, t0 + 1500);
      CHECK(r->tel().armed && runs_are(arm, {50, 50, 150, 50, 50, 50, 150}), "arming with a GPS fix: the GPS-fix tune (%s)", show(arm).c_str());
      b.rc[2] = 1300; r->run(0.5);
      t0 = millis(); b.rc[4] = 1000; b.rc[2] = 1000; r->run(1.0);
      const std::vector<int> dis = buzz_runs(b, t0, t0 + 1000);
      CHECK(!r->tel().armed && runs_are(dis, {150, 50, 150}), "disarming: two beeps (%s)", show(dis).c_str());

      // ---- transmitter off on the field: lost-signal beep; in front of the app: quiet
      b.tx_on = false; r->run(1.0); t0 = millis(); r->run(4.0);
      const std::vector<int> lost = buzz_runs(b, t0, t0 + 4000);
      CHECK(!r->tel().rc_link && lost.size() >= 6 && abs((int)b.buzz_ms(t0, t0 + 4000) - 2000) < 60, "receiver signal lost: 500 ms on / 500 ms off (%u ms on in 4 s)", b.buzz_ms(t0, t0 + 4000));
      r->request(MSG_HELLO); r->run(0.2); t0 = millis();
      for (int i = 0; i < 6; i++) { r->run(0.5); r->request(MSG_HELLO); }
      CHECK(b.buzz_ms(t0, t0 + 3000) == 0, "the app is talking to the board: no lost-signal noise on the desk (%u ms)", b.buzz_ms(t0, t0 + 3000));
      r->run(3.5); t0 = millis(); r->run(2.0);
      CHECK(b.buzz_ms(t0, t0 + 2000) > 900, "app silent for 3 s: the board is alone again and beeps");
      b.tx_on = true; r->run(1.5); t0 = millis(); r->run(1.0);
      CHECK(b.buzz_ms(t0, t0 + 1000) == 0, "signal back: quiet");

      // ---- arming refused
      b.rc[2] = 1600; r->run(0.3); t0 = millis(); b.rc[4] = 2000; r->run(1.0);
      const std::vector<int> ref = buzz_runs(b, t0, t0 + 1000);
      CHECK(!r->tel().armed && runs_are(ref, {400, 100, 100, 100, 100}), "arm switch with the throttle up: refused, one long + two short (%s)", show(ref).c_str());
      b.rc[4] = 1000; b.rc[2] = 1000; r->run(0.5);

      // ---- low battery
      b.batt_v = 13.8; r->run(3.5); t0 = millis(); r->run(3.0);
      CHECK(r->tel().batt_state == BATT_STATE_WARNING && abs((int)b.buzz_ms(t0, t0 + 3000) - 1000) < 60 && r->tel().beeping == beep::BAT_LOW + 1, "battery warning: 250 ms beep every 750 ms (%u ms in 3 s), telemetry names the sound", b.buzz_ms(t0, t0 + 3000));
      b.batt_v = 16.0; r->run(3.0);

      // ---- one condition switched off in the settings (live)
      S_beeper sb; r->get_section(SEC_BEEPER, &sb, sizeof(sb));
      CHECK(sb.conditions == 0x1FFFFu && sb.type == BUZZER_TYPE_ACTIVE && sb.volume == 100 && sb.frequency == 2700, "default beeper settings");
      sb.conditions &= ~(1u << BEEPER_COND_DISARMING); sb.conditions |= 1u << BEEPER_COND_USB; r->set_section(SEC_BEEPER, &sb, sizeof(sb));
      b.rc[4] = 2000; r->run(1.5); CHECK(r->tel().armed, "armed");
      b.rc[2] = 1300; r->run(0.3); t0 = millis(); b.rc[4] = 1000; b.rc[2] = 1000; r->run(1.0);
      CHECK(!r->tel().armed && b.buzz_ms(t0, t0 + 1000) == 0, "DISARMING switched off: disarms silently");

      // ---- the app's test button
      t0 = millis(); CHECK(r->command(CMD_BEEP, {beep::DISARMING}).err == ERR_OK, "BEEP DISARMING"); r->run(0.8);
      const std::vector<int> tst = buzz_runs(b, t0, t0 + 800);
      CHECK(runs_are(tst, {150, 50, 150}), "the test button plays a sound that is switched off (%s)", show(tst).c_str());
      t0 = millis(); r->command(CMD_BEEP, {beep::RX_LOST_LANDING}); r->run(0.25); CHECK(b.buzzer_on() || b.buzz_ms(t0, t0 + 250) > 100, "SOS started");
      CHECK(r->command(CMD_BEEP, {255}).err == ERR_OK, "BEEP 255 = silence"); r->run(0.05); t0 = millis(); r->run(1.0);
      CHECK(b.buzz_ms(t0, t0 + 1000) == 0 && r->tel().beeping == 0, "silenced");
      CHECK(r->command(CMD_BEEP, {beep::ON_USB}).err == ERR_BAD_ARG && r->command(CMD_BEEP, {beep::MODE_COUNT}).err == ERR_BAD_ARG && r->command(CMD_BEEP, {}).err == ERR_BAD_ARG, "BEEP with something that is not a sound: refused");
      b.rc[4] = 2000; r->run(1.0);
      CHECK(r->tel().armed && r->command(CMD_BEEP, {beep::ARMING}).err == ERR_ARMED, "no test sounds while armed");
      b.rc[4] = 1000; r->run(1.5);
      // ---- one accelerometer calibration point finished: two short beeps (heard while holding the craft, eyes off the screen)
      t0 = millis(); CHECK(r->command(CMD_ACC_CAL_POINT).err == ERR_OK, "ACC_CAL_POINT"); r->run(1.6);
      const std::vector<int> acc = buzz_runs(b, t0, t0 + 1600);
      CHECK(r->tel().cal_state == CAL_STATE_DONE && runs_are(acc, {50, 50, 50}), "calibration point captured: two short beeps (%s)", show(acc).c_str());
      r->command(CMD_ACC_CAL_CANCEL); }

    { // on the bench: USB power, app connected -> silent by default
      auto r = fresh(); VirtualBoard& b = r->board; b.batt_v = 0.0; b.buzz.clear();
      r->boot(); arm_switch_setup(*r);
      uint32_t t0 = millis();
      for (int i = 0; i < 12; i++) { r->run(0.5); r->request(MSG_HELLO); if (i == 6) b.rc[4] = 2000; if (i == 9) b.rc[4] = 1000; }
      r->command(CMD_ACC_CAL_POINT); for (int i = 0; i < 4; i++) { r->run(0.4); r->request(MSG_HELLO); }
      CHECK(r->tel().cal_state == CAL_STATE_DONE && r->command(CMD_ACC_CAL_CANCEL).err == ERR_OK, "a calibration point was captured meanwhile");
      CHECK(b.buzz_ms(t0 + 3000, millis()) == 0, "USB power + app connected: gyro calibration, arming, disarming, calibration point all silent (%u ms)", b.buzz_ms(t0 + 3000, millis()));
      t0 = millis(); r->command(CMD_BEEP, {beep::ARMING}); r->run(0.6);
      CHECK(runs_are(buzz_runs(b, t0, t0 + 600), {300, 50, 50}), "the test button still works there"); }

    { // passive buzzer: a tone, volume = duty; inverted output
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.beeper.type = BUZZER_TYPE_PASSIVE; s.beeper.frequency = 3100; s.beeper.volume = 40; st.save(s); }
      r.boot(); const StubPwm& pw = g_stub_pwm[VirtualBoard::kBuzzer];
      CHECK(has_line(r, "beeper: passive buzzer") && pw.attached && pw.freq == 3100 && pw.bits == 10, "passive buzzer: PWM at the configured tone (%u Hz)", pw.freq);
      uint32_t peak = 0; for (int i = 0; i < 200; i++) { r.run(0.001); if (pw.duty > peak) peak = pw.duty; }
      CHECK(peak == 1023u * 40u / 200u, "volume 40 %%: duty %u of 1023 (100 %% would be a 50 %% square wave)", peak);
      r.run(3.0); CHECK(pw.duty == 0, "silent: no PWM output");
      S_beeper sb; r.get_section(SEC_BEEPER, &sb, sizeof(sb)); sb.inverted = 1; sb.volume = 100; r.set_section(SEC_BEEPER, &sb, sizeof(sb)); r.run(0.01);
      CHECK(pw.attached && pw.duty == 1023, "inverted, live: the silent level is now HIGH (duty %u)", pw.duty);
      r.command(CMD_BEEP, {beep::RX_LOST}); r.run(0.1);
      CHECK(pw.duty == 1023 - 511, "inverted and sounding: duty %u", pw.duty);
      sb.type = BUZZER_TYPE_ACTIVE; sb.inverted = 0; r.set_section(SEC_BEEPER, &sb, sizeof(sb)); r.command(CMD_BEEP, {beep::RX_LOST}); r.run(0.1);
      CHECK(!pw.attached && g_stub_gpio[VirtualBoard::kBuzzer] == 1 && g_stub_pwm_free == 8, "switched to an active buzzer without a reboot: PWM channel given back, plain pin HIGH");
      r.command(CMD_BEEP, {255}); sb.inverted = 1; r.set_section(SEC_BEEPER, &sb, sizeof(sb)); r.run(0.05);
      CHECK(g_stub_gpio[VirtualBoard::kBuzzer] == 1, "active + inverted: silent = pin HIGH");
      r.command(CMD_BEEP, {beep::RX_LOST}); r.run(0.1);
      CHECK(g_stub_gpio[VirtualBoard::kBuzzer] == 0, "active + inverted: sounding = pin LOW"); }

    { // BEEPER switch on an aux channel
      auto r = fresh(); VirtualBoard& b = r->board; r->run(8.0);
      S_modes m; r->get_section(SEC_MODES, &m, sizeof(m));
      m.mode[0] = MODE_ARM; m.aux[0] = 0; m.lo[0] = 1700; m.hi[0] = 2100; m.mode[1] = MODE_BEEPER; m.aux[1] = 3; m.lo[1] = 1700; m.hi[1] = 2100;
      r->set_section(SEC_MODES, &m, sizeof(m)); r->run(3.5);
      uint32_t t0 = millis(); b.rc[7] = 2000; r->run(2.0);
      CHECK(abs((int)b.buzz_ms(t0, t0 + 2000) - 1000) < 120, "BEEPER switch: 100 ms beeps (%u ms in 2 s)", b.buzz_ms(t0, t0 + 2000)); }
}

// The motors as the sounder: a craft WITHOUT a buzzer. What is checked is what leaves the DShot outputs.
static void t_motor_beeper() {
    printf("[motor beeper] no buzzer pin: the ESCs beep the motors (DShot beacon) while disarmed\n");
    auto no_buzzer = [](Rig& r, void (*more)(wbp::Settings&) = nullptr) {
        g_stub_us = 0; g_stub_delay_advances = true; r.provision();
        r.board.gps.module_baud = 230400;                 // (a module that has to be switched to 115200 first: its fix, and the "GPS ready" tune, come after the 4 s looked at below)
        wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.buzzer = 255; if (more) more(s); st.save(s);
        r.boot();
    };
    { Rig r; no_buzzer(r); VirtualBoard& b = r.board;
      CHECK(has_line(r, "beeper: motors (DShot beacon") && !r.app().status.buzzer && r.app().status.motor_beep && r.tel().blockers == 0, "boot report: the motors are the beeper; nothing blocks arming");
      r.run(4.0);
      // gyro calibration ends about 1 s after boot -> 3 tones, 550 ms apart, tone 1 (default), all four motors in the same frame
      std::vector<uint32_t> t = b.tones(0, 4000, 1);
      CHECK(t.size() == 3 && t[1] - t[0] >= 549 && t[1] - t[0] <= 551 && t[2] - t[1] >= 549 && t[2] - t[1] <= 551 && t[0] > 900 && t[0] < 1400,
            "gyro calibrated: 3 beacon tones on all four motors, 550 ms apart (%zu, first at %u ms)", t.size(), t.empty() ? 0u : (unsigned)t[0]);
      CHECK(b.esc_cmds.size() == 3 && r.app().beaconsSent() == 3, "and no other DShot command went out (%zu commands)", b.esc_cmds.size());
      CHECK(b.buzz.empty() && g_stub_gpio[VirtualBoard::kBuzzer] == 0, "no pin was touched for a buzzer");
      r.run(4.0);                                           // GPS fix -> ready tune on the motors; let it finish
      const size_t n_ready = b.esc_cmds.size();
      CHECK(n_ready == 3 + 7, "GPS ready: the 7 beeps of the READY tune as 7 tones (%zu commands so far)", n_ready);

      // ---- the stream tells the app what the motors are playing
      r.stream(STR_BATTERY, 20); r.request(MSG_HELLO);
      S_beeper sb; r.get_section(SEC_BEEPER, &sb, sizeof(sb));
      CHECK(sb.motor_conditions == 0x19F8Bu && sb.dshot_beacon_tone == 1, "default motor conditions 0x%x, tone %u", (unsigned)sb.motor_conditions, sb.dshot_beacon_tone);
      sb.dshot_beacon_tone = 3; r.set_section(SEC_BEEPER, &sb, sizeof(sb));
      uint32_t t0 = millis(); CHECK(r.command(CMD_BEEP, {beep::DISARMING}).err == ERR_OK, "BEEP works without a buzzer"); r.run(0.3);
      T_battery tb{}; r.latest(STR_BATTERY, tb);
      CHECK(tb.motor_beeping == beep::DISARMING + 1 && tb.beeping == beep::DISARMING + 1, "battery stream: motor_beeping names the condition (%u)", tb.motor_beeping);
      r.run(1.2); t = b.tones(t0, millis(), 3);
      CHECK(t.size() == 2 && t[1] - t[0] == 500 && b.tones(t0, millis(), 1).empty(), "the test button: 2 tones with the newly chosen tone 3, 500 ms apart (%zu)", t.size());
      r.run(0.2); r.latest(STR_BATTERY, tb); CHECK(tb.motor_beeping == 0, "finished: 0");
      t0 = millis(); r.command(CMD_BEEP, {beep::RX_LOST_LANDING}); r.run(0.6); CHECK(b.tones(t0, millis(), 3).size() >= 1, "a forced sound plays even if a motor could never play it by itself");
      CHECK(r.command(CMD_BEEP, {255}).err == ERR_OK, "silence"); t0 = millis(); r.run(1.5); CHECK(b.tones(t0, millis(), 3).empty(), "silenced: no more tones");

      // ---- the pilot arms: the line carries throttle only; after the disarm the motors say "disarmed" once they have stopped
      arm_switch_setup(r); r.run(3.5);
      size_t n0 = b.esc_cmds.size(); b.rc[4] = 2000; r.run(1.0); b.rc[2] = 1300; r.run(1.5);
      CHECK(r.tel().armed && b.esc_cmds.size() == n0 && b.motor_frac[0] > 0.1f, "armed and flying: not one command frame (%zu), motors at %.2f", b.esc_cmds.size() - n0, b.motor_frac[0]);
      b.rc[2] = 1000; r.run(1.0); t0 = millis(); b.rc[4] = 1000; r.run(3.0);
      t = b.tones(t0, millis(), 3);
      CHECK(!r.tel().armed && t.size() == 2 && t[0] - t0 >= 1200 && t[0] - t0 <= 1225 && t[1] - t[0] == 500, "disarmed: 2 tones, the first one 1.2 s after the disarm (%zu, +%u ms after the switch moved)", t.size(), t.empty() ? 0u : (unsigned)(t[0] - t0));

      // ---- transmitter off: the lost-model finder
      b.tx_on = false; r.run(1.5); t0 = millis(); r.run(5.0); t = b.tones(t0, millis(), 3);
      bool rhythm = t.size() >= 5; for (size_t i = 1; i < t.size(); i++) if (t[i] - t[i - 1] < 949 || t[i] - t[i - 1] > 953) rhythm = false;
      CHECK(!r.tel().rc_link && rhythm, "receiver signal lost: a tone every 951 ms for as long as it takes to find the craft (%zu in 5 s)", t.size());
      r.request(MSG_HELLO); r.run(0.6); t0 = millis(); for (int i = 0; i < 6; i++) { r.run(0.5); r.request(MSG_HELLO); }
      CHECK(b.tones(t0, millis(), 3).empty(), "with the app connected the motors keep quiet on the desk");
      b.tx_on = true; r.run(4.0);

      // ---- arming refused, battery low
      t0 = millis(); b.rc[2] = 1600; r.run(0.3); b.rc[4] = 2000; r.run(2.5); t = b.tones(t0, millis(), 3);
      CHECK(!r.tel().armed && t.size() == 3, "arm switch with the throttle up: refused, 3 tones (%zu)", t.size());
      b.rc[4] = 1000; b.rc[2] = 1000; r.run(1.0);
      b.batt_v = 13.8; r.run(3.5); t0 = millis(); r.run(4.0); t = b.tones(t0, millis(), 3);
      CHECK(r.tel().batt_state == BATT_STATE_WARNING && t.size() >= 4 && t.size() <= 5, "battery warning on the ground: a tone every 951 ms (%zu in 4 s)", t.size());
      // only RX_LOST left switched on for the motors (live)
      sb.motor_conditions = 1u << BEEPER_COND_RX_LOST; r.set_section(SEC_BEEPER, &sb, sizeof(sb)); r.run(4.0); t0 = millis(); r.run(3.0);
      CHECK(b.tones(t0, millis(), 3).empty(), "BAT_LOW switched off for the motors (live): quiet (app silent for more than 3 s, so it is not the bench rule)");
      b.batt_v = 16.0; r.run(3.0);

      // ---- the app turns a motor with the test slider: no tone in between (the beep would take the line away from the test)
      sb.motor_conditions = 0x19F8Bu; r.set_section(SEC_BEEPER, &sb, sizeof(sb));
      uint8_t claimed = 0;
      t0 = millis(); for (int i = 0; i < 20; i++) { r.command(CMD_MOTOR_TEST, {0, 100, 0}); if (i == 2) r.command(CMD_BEEP, {beep::GYRO_CALIBRATED}); r.run(0.1); if (i == 8) { r.latest(STR_BATTERY, tb); claimed = tb.motor_beeping; } }
      CHECK(b.tones(t0, millis(), 3).empty() && b.motor_frac[0] > 0.05f, "motor test running: no beacon tone interrupts it (motor at %.2f)", b.motor_frac[0]);
      CHECK(claimed == 0, "and the app is not told that the motors are beeping while they are being tested (motor_beeping %u)", claimed);
      r.command(CMD_MOTOR_STOP); r.run(1.0); }

    { // no battery measured (no ADC pin, or USB only) + app connected: the bench rule mutes the buzzer, never the motors (an ESC
      // that can make a sound has power, so a battery is there even when nothing measures it)
      Rig r; no_buzzer(r); VirtualBoard& b = r.board; b.batt_v = 0.0;
      for (int i = 0; i < 20; i++) { r.run(0.5); r.request(MSG_HELLO); }
      CHECK(b.tones(0, millis(), 1).size() == 3 + 7, "app connected, battery not measured: gyro calibration (3) and GPS ready (7) still on the motors (%zu tones)", b.tones(0, millis(), 1).size());
      const uint32_t t0 = millis(); CHECK(r.command(CMD_BEEP, {beep::GYRO_CALIBRATED}).err == ERR_OK, "BEEP on the bench"); r.run(2.0);
      CHECK(b.tones(t0, millis(), 1).size() == 3, "the test button is played by the motors there as well (%zu tones)", b.tones(t0, millis(), 1).size()); }

    { // ProShot1000 carries DShot packets: the motors are the sounder, a beacon goes out as a command value
      Rig r; no_buzzer(r, [](wbp::Settings& s) { s.motors.protocol = ESC_PROTOCOL_PROSHOT1000; s.motors.bidirectional = 1; }); VirtualBoard& b = r.board; r.run(3.0);
      CHECK(r.app().status.motor_beep && has_line(r, "beeper: motors (DShot beacon") && r.tel().bidir_state == BIDIR_STATE_OFF, "ProShot1000: the motors beep; bidirectional asked for but not possible (state %u)", r.tel().bidir_state);
      const size_t n0 = b.esc_cmds.size(); CHECK(r.command(CMD_MOTOR_BEACON, {1}).err == ERR_OK, "beacon button"); r.run(0.5);
      CHECK(b.esc_cmds.size() == n0 + 1 && b.esc_cmds.back().cmd == 2, "one beacon command (tone 2) went out to the ESC"); }
    { // brushed motors: duty cycle on LEDC, no DShot, nothing to beep with
      Rig r; no_buzzer(r, [](wbp::Settings& s) { s.motors.protocol = ESC_PROTOCOL_BRUSHED; }); r.run(2.0);
      CHECK(!r.app().status.motor_beep && r.app().status.motors && r.command(CMD_MOTOR_BEACON, {0}).err == ERR_NOT_SUPPORTED, "brushed: motors run, no beacon (NOT_SUPPORTED)"); }

    { // PWM-type ESCs cannot beep: said in the report, the test button says there is no sounder
      Rig r; no_buzzer(r, [](wbp::Settings& s) { s.motors.protocol = ESC_PROTOCOL_ONESHOT125; }); r.run(3.0);
      CHECK(has_line(r, "beeper: FAILED - no buzzer pin set, and the motors can only beep with a DShot protocol") && !r.app().status.motor_beep, "Oneshot125 and no buzzer: no sounder, said in the boot report");
      CHECK(r.command(CMD_BEEP, {beep::ARMING}).err == ERR_NO_SENSOR && r.tel().blockers == 0 && r.app().beaconsSent() == 0, "BEEP: no sounder; arming unaffected"); }

    { // buzzer AND DShot motors: both play, the report names both
      auto r = fresh(); r->run(3.0);
      CHECK(has_line(*r, "beeper: active buzzer + motors (DShot beacon") && r->board.tones(0, 3000, 1).size() == 3 && r->board.buzz_ms(700, 2500) == 600, "buzzer + motors: the gyro-calibrated signal on both (%zu tones, %u ms of buzzer)",
            r->board.tones(0, 3000, 1).size(), r->board.buzz_ms(700, 2500));
      S_beeper sb; r->get_section(SEC_BEEPER, &sb, sizeof(sb)); sb.motor_conditions = 0; r->set_section(SEC_BEEPER, &sb, sizeof(sb)); r->run(3.5);
      const size_t n0 = r->board.esc_cmds.size(); const uint32_t t0 = millis(); r->board.tx_on = false; r->run(4.0);
      CHECK(r->board.esc_cmds.size() == n0 && r->board.buzz_ms(t0 + 1000, t0 + 4000) > 1200, "motor conditions all off: only the buzzer beeps for a lost signal"); }
}

// ---- blackbox helpers
static void bb_config(Rig& r, uint8_t device, uint8_t trigger, uint16_t rate_hz, uint32_t fields = 0) {
    S_blackbox b; r.get_section(SEC_BLACKBOX, &b, sizeof(b)); b.device = device; b.trigger = trigger; b.rate_hz = rate_hz; if (fields) b.fields = fields;
    r.set_section(SEC_BLACKBOX, &b, sizeof(b));
}
static T_blackbox bb_status(Rig& r) { T_blackbox t{}; r.stream(STR_BLACKBOX, 10); r.run(0.15); r.latest(STR_BLACKBOX, t); return t; }
// every frame of a blackbox type that arrived on the link, as a byte stream the reader understands
static std::vector<uint8_t> bb_frames(Rig& r, uint8_t type_lo, uint8_t type_hi) {
    r.collect(); std::vector<uint8_t> out; uint8_t buf[wbp::kMaxPayload + wbp::kOverhead];
    for (const Rig::Rx& x : r.inbox) if (x.type >= type_lo && x.type <= type_hi) { const size_t n = wbp::encode(buf, sizeof(buf), x.type, x.seq, x.p.data(), (uint16_t)x.p.size()); out.insert(out.end(), buf, buf + n); }
    return out;
}
// downloads [start, start + size) of the log memory through the app link
static std::vector<uint8_t> bb_download(Rig& r, uint32_t start, uint32_t size) {
    r.clear_inbox();
    std::vector<uint8_t> a(8); memcpy(a.data(), &start, 4); memcpy(a.data() + 4, &size, 4);
    std::vector<uint8_t> out(size, 0xEE); uint32_t got = 0;
    if (r.command(CMD_BLACKBOX_READ, a).err != ERR_OK) return {};
    for (int i = 0; i < 4000 && got < size; i++) {
        r.run(0.001); r.collect();
        for (const Rig::Rx& x : r.inbox) if (x.type == bb::kMsgBlob && x.p.size() > 4) { uint32_t off; memcpy(&off, x.p.data(), 4); if (off >= start && off - start + x.p.size() - 4 <= size) { memcpy(out.data() + (off - start), x.p.data() + 4, x.p.size() - 4); got += (uint32_t)x.p.size() - 4; } }
        r.inbox.clear();
    }
    return got == size ? out : std::vector<uint8_t>();
}
struct Truth { uint32_t us; float gyro[3], roll, z, motor[4], pid_p = 0; };

static void t_blackbox() {
    printf("[blackbox] a flight logged to flash (with the stalls flash writes cause), downloaded, decoded and compared with what really happened\n");
    { auto r = fresh(); VirtualBoard& b = r->board; arm_switch_setup(*r); r->run(2.0);
      b.plant.true_accel = false;        // an ideal tilt sensor: the log is compared with the truth, and what the estimator itself gets wrong in a manoeuvre (test_flight) is not the recorder's
      CHECK(has_line(*r, "blackbox: partition 'sim' 1024 kB, 0 sessions"), "boot report: log partition found");
      T_blackbox st = bb_status(*r);
      CHECK(st.state == BB_STATE_OFF && st.capacity == 1024 * 1024 && st.used == 0 && st.sessions == 0, "default: blackbox off (state %u)", st.state);
      b.rc[4] = 2000; r->run(1.0); b.rc[4] = 1000; r->run(0.5);
      CHECK(r->logmem.writes == 0, "off: arming writes nothing to flash");
      bb_config(*r, BB_DEVICE_FLASH, BB_TRIGGER_ARMED, 500);
      CHECK(!(r->fw->server.flags() & SF_REBOOT_REQUIRED), "blackbox settings take effect without a reboot");
      st = bb_status(*r);
      CHECK(st.state == BB_STATE_IDLE && st.record_bytes == 74, "switched to flash: idle, a fast record is %u bytes with the default fields", st.record_bytes);

      // ---- fly: take off in AltHold, hover, a roll input; note the truth every millisecond
      std::vector<Truth> truth; truth.reserve(12000);
      auto fly = [&](double seconds) { const int n = (int)(seconds * 1000); for (int i = 0; i < n; i++) { r->run(0.001); Truth t; t.us = micros(); const Vec3 g = b.plant.gyro_dps(); t.gyro[0] = g.x; t.gyro[1] = -g.y; t.gyro[2] = g.z; t.roll = b.plant.right_down_deg(); t.z = b.plant.z; for (int k = 0; k < 4; k++) t.motor[k] = b.motor_frac[k]; t.pid_p = r->app().output().pid_p[0]; truth.push_back(t); } };
      b.rc[5] = 2000; b.rc[6] = 2000; r->run(0.3);
      const uint32_t t_arm = micros(); b.rc[4] = 2000; fly(0.5);
      CHECK(r->tel().armed && r->fw->blackbox.state() == bb::ST_LOGGING, "armed: the session starts");
      b.rc[2] = 1700; fly(3.5); b.rc[2] = 1500; fly(2.0); b.rc[0] = 1800; fly(1.0); b.rc[0] = 1500; fly(2.0);
      const float z_hover = b.plant.z;
      // (with true_accel off the height estimate has the barometer only: the half metre lost in the roll comes back slowly)
      CHECK(z_hover > 1.2f && z_hover < 1.9f && fabsf(b.plant.right_down_deg()) < 3.0f && fabsf(b.plant.nose_down_deg()) < 3.0f && fabsf(b.plant.vz) < 0.4f, "it flies normally while logging: z %.2f m, level, vz %.2f", z_hover, b.plant.vz);
      CHECK(b.stalled_us > 100000 && b.longest_stall_us == 700 && r->logmem.bad_writes == 0 && r->logmem.longest_write <= 256, "the flash stalled the flight loop %.0f ms in total, 0.7 ms at a time (one page), clean NOR writes", b.stalled_us / 1000.0);
      CHECK(r->tel().loop_hz > 3600 && r->tel().gap_max_us >= 900 && r->tel().gap_max_us <= 1300, "the loop lost a few cycles to it: %u Hz, longest gap between cycles %u us (status stream shows it)", r->tel().loop_hz, r->tel().gap_max_us);
      st = bb_status(*r); CHECK(st.state == BB_STATE_LOGGING && st.rate_hz >= 440 && st.rate_hz <= 505 && st.dropped == 0 && st.byte_rate > 30000 && st.byte_rate < 42000, "status while logging: %u records/s (500 asked; the stalled cycles are missing), %u bytes/s, none dropped", st.rate_hz, st.byte_rate);
      CHECK(r->command(CMD_BLACKBOX_ERASE).err == ERR_ARMED && r->command(CMD_BLACKBOX_READ, {0, 0, 0, 0, 16, 0, 0, 0}).err == ERR_ARMED, "no erase, no download while armed");
      b.rc[2] = 1300; fly(6.0); b.rc[2] = 1000; fly(1.0);
      const uint32_t t_disarm = micros(); b.rc[4] = 1000; r->run(1.0);
      st = bb_status(*r);
      CHECK(!r->tel().armed && st.state == BB_STATE_IDLE && st.sessions == 1 && st.used > 400000 && st.used < 700000, "disarmed: session closed, %u bytes used", st.used);

      // ---- download it through the link and read it
      const Rig::CmdResult ses = r->command(CMD_BLACKBOX_SESSION, {0, 0}); uint32_t s_start = 1, s_size = 0;
      if (ses.data.size() == 8) { memcpy(&s_start, ses.data.data(), 4); memcpy(&s_size, ses.data.data() + 4, 4); }
      CHECK(ses.err == ERR_OK && s_start == 0 && s_size == st.used, "session table: one session from 0, %u bytes", s_size);
      const std::vector<uint8_t> raw = bb_download(*r, s_start, s_size);
      CHECK(raw.size() == s_size && memcmp(raw.data(), r->logmem.mem.data(), s_size) == 0, "download = the bytes in flash");
      const std::vector<RSession> ss = read_log(raw.data(), raw.size());
      CHECK(ss.size() == 1 && ss[0].bad_records == 0 && ss[0].seq_gaps == 0, "it parses as one session, no gap");
      if (ss.size() == 1) {
        const RSession& s = ss[0];
        char want[16]; snprintf(want, sizeof(want), "0x%08lX", (unsigned long)kSchemaHash);
        CHECK(s.meta.at("firmware") == "Windburst RX (sim) 0.11.0" && s.meta.at("schema") == want && s.meta.at("loop_hz") == "4000" && s.meta.at("rate_hz") == "500" && s.meta.at("motors") == "4" && s.meta.at("to") == "storage", "header: firmware, schema, rates");
        { const std::string& hex = s.meta.at("set.rates"); S_rates live; r->get_section(SEC_RATES, &live, sizeof(live)); bool same = hex.size() == 2 * sizeof(live);
          for (size_t i = 0; same && i < sizeof(live); i++) if ((uint8_t)strtol(hex.substr(2 * i, 2).c_str(), nullptr, 16) != ((const uint8_t*)&live)[i]) same = false;
          CHECK(same, "header: the settings the flight was flown with (section rates matches the firmware's)"); }
        const double span = (s.F.back()[0] - s.F.front()[0]) * 1e-6, flown = (t_disarm - t_arm) * 1e-6;
        CHECK(fabs(span - flown) < 0.05 && s.F.size() > 7600 && s.F.size() < 8100, "the log covers the armed time: %.2f s of %.2f s, %zu fast records", span, flown, s.F.size());
        // timing: records every 8th cycle = 2000 us, later when the flash held the loop up; the gap is in the record itself
        // (the first 100 records are left out: the header's 14 pages are written back to back right after arming)
        int exact = 0, late = 0; double worst = 0, max_gap = 0, min_gap = 1e9;
        for (size_t i = 100; i < s.F.size(); i++) { const double d = s.F[i][0] - s.F[i - 1][0]; if (d == 2000) exact++; else late++; if (d > worst) worst = d; const double g = s.fv(i, "gap_us"); if (g > max_gap) max_gap = g; if (g < min_gap) min_gap = g; }
        CHECK(exact > late && worst <= 3500 && min_gap == 250 && max_gap >= 900 && max_gap <= 1300, "record spacing: 2000 us nominal (%d), stretched by flash stalls (%d, worst %.0f us); gap_us shows 250 .. %.0f us", exact, late, worst, max_gap);
        // values against the truth at the same instant
        auto at = [&](double us) -> const Truth& { size_t lo = 0, hi = truth.size() - 1; while (lo < hi) { const size_t mid = (lo + hi) / 2; if (truth[mid].us < us) lo = mid + 1; else hi = mid; } return truth[lo]; };
        auto near = [&](double us) -> const Truth& { const Truth* t0 = &at(us); return t0 > truth.data() ? t0[-1] : *t0; };      // the truth sample just before: a step in the motors may fall between the two
        double e_gyro = 0, e_roll = 0, e_motor = 0, peak_roll = 0, peak_sp = 0, peak_stick = 0; int n = 0;
        for (size_t i = 0; i < s.F.size(); i += 5) {
            const double t = s.F[i][0]; if (t < truth.front().us + 2000 || t > truth.back().us) continue;
            const Truth& tr = at(t); n++;
            for (int a = 0; a < 3; a++) e_gyro = fmax(e_gyro, fabs(s.fv(i, std::string("gyro[") + char('0' + a) + "]") - tr.gyro[a]));
            e_roll = fmax(e_roll, fabs(s.fv(i, "roll") - tr.roll));
            for (int k = 0; k < 4; k++) { const double m = s.fv(i, std::string("motor[") + char('0' + k) + "]"); e_motor = fmax(e_motor, fmin(fabs(m - tr.motor[k]), fabs(m - near(t).motor[k]))); }
            peak_roll = fmax(peak_roll, s.fv(i, "roll")); peak_sp = fmax(peak_sp, s.fv(i, "setpoint[0]")); peak_stick = fmax(peak_stick, s.fv(i, "stick[0]"));
        }
        CHECK(n > 1000 && e_gyro < 12.0 && e_roll < 1.5, "gyro and roll angle follow the real motion: worst difference %.1f deg/s, %.2f deg over %d samples", e_gyro, e_roll, n);
        CHECK(e_motor < 0.03, "motor outputs are what the ESCs received (worst difference %.3f)", e_motor);
        CHECK(peak_roll > 8.0 && peak_sp > 20.0 && fabs(peak_stick - (0.6 - 0.02) / 0.98) < 0.03, "the roll input is in the log: stick %.2f (0.6 of the travel, minus the deadband), setpoint up to %.0f deg/s, roll up to %.1f deg", peak_stick, peak_sp, peak_roll);
        // the setpoint is the COMMAND: at the stick step it jumps while the craft has not started to turn yet
        int lead = 0; for (size_t i = 0; i < s.F.size(); i++) if (s.fv(i, "setpoint[0]") > 40.0 && fabs(s.fv(i, "gyro[0]")) < 10.0) lead++;
        CHECK(lead >= 2, "setpoint leads the gyro at the start of the roll (%d records with setpoint > 40 deg/s while the gyro is still below 10)", lead);
        // the PID terms as the core computed them (PID sum units: tens to hundreds in a roll), not clipped by the record's number format
        double p_log = 0, p_true = 0; for (size_t i = 0; i < s.F.size(); i++) p_log = fmax(p_log, fabs(s.fv(i, "pid_p[0]"))); for (const Truth& t : truth) p_true = fmax(p_true, fabs(t.pid_p));
        CHECK(p_true > 30.0 && p_log > 0.8 * p_true && p_log < 1.05 * p_true, "the roll P term reached %.1f in the flight; the log shows %.1f (one record in 8 cycles)", p_true, p_log);
        CHECK(s.fv(s.F.size() / 2, "throttle") > 0.2 && s.fv(s.F.size() / 2, "loop_us") < 250, "throttle and loop time are recorded");
        // slow records
        CHECK(fabs((double)s.S.size() - span * 10.0) <= 3.0, "slow records at 10 Hz: %zu in %.1f s", s.S.size(), span);
        size_t mid = s.S.size() / 2; const Truth& tm = at(s.S[mid][0]);
        CHECK(s.sv(mid, "armed") == 1 && fabs(s.sv(mid, "baro_alt") - tm.z) < 0.6 && fabs(s.sv(mid, "est_z") - tm.z) < 0.6 && fabs(s.sv(mid, "vbat") - 15.6) < 0.3 && s.sv(mid, "batt_state") == BATT_STATE_OK,
              "mid-flight: armed, altitude %.2f m (true %.2f), battery %.2f V", s.sv(mid, "est_z"), tm.z, s.sv(mid, "vbat"));
        CHECK(fabs(s.sv(mid, "lat") - 47.3977419) < 2e-5 && s.sv(mid, "gps_fix") == 3 && s.sv(mid, "gps_sats") == 14 && ((uint32_t)s.sv(mid, "modes") & (1u << MODE_ALTHOLD)) && s.sv(mid, "gap_max_us") >= 900, "GPS position, mode bits and the loop's longest gap are in the slow record");
        CHECK(s.count_events(bb::EV_ARM) == 1 && s.count_events(bb::EV_DISARM) == 1 && s.count_events(bb::EV_DROPPED) == 0 && s.events.back().code == bb::EV_END, "events: armed at the start, disarm, END; nothing dropped");
      }

      // ---- the log survives a power cycle; the next flight is a second session
      CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK, "settings saved");
      r->boot(); r->run(2.5);
      CHECK(has_line(*r, "1 sessions") && r->fw->blackbox.log().sessions() == 1 && r->fw->blackbox.log().used() == st.used, "after a power cycle the session is still there");
      b.rc[4] = 2000; r->run(1.5); b.rc[4] = 1000; r->run(1.0);
      st = bb_status(*r);
      CHECK(st.sessions == 2 && r->logmem.bad_writes == 0, "second flight: second session (bad NOR writes: %u)", r->logmem.bad_writes);

      // ---- erase: seconds during which the flight loop is held up again and again -> the craft must stay on the ground
      r->run(3.5);                                          // no app traffic for 3 s: beeps are not muted
      const uint32_t t0 = millis();
      CHECK(r->command(CMD_BLACKBOX_ERASE).err == ERR_OK, "ERASE accepted");
      b.rc[4] = 2000; r->run(0.002);                       // a few polls = a few blocks of 64 kB, each holding everything up for 150 ms
      CHECK(r->fw->blackbox.state() == bb::ST_ERASING && (r->tel().blockers & ARM_BLOCKER_CALIBRATING) && !r->tel().armed && r->fw->blackbox.log().erase_percent() >= 6 && r->fw->blackbox.log().erase_percent() <= 80,
            "erasing (%u %%): arming is blocked, the arm switch does nothing (state %d blockers 0x%x armed %d)", r->fw->blackbox.log().erase_percent(), (int)r->fw->blackbox.state(), r->tel().blockers, (int)r->tel().armed);
      b.rc[4] = 1000;
      uint32_t t_done = 0; for (int i = 0; i < 40 && !t_done; i++) { r->run(0.001); if (r->fw->blackbox.state() != bb::ST_ERASING) t_done = millis(); }
      r->run(1.5); st = bb_status(*r);
      CHECK(t_done && st.state == BB_STATE_IDLE && st.sessions == 0 && st.used == 0 && !(r->tel().blockers & ARM_BLOCKER_CALIBRATING), "erased after %.1f s of flash time: empty, arming free again", (t_done - t0) / 1000.0);
      bool blank = true; for (uint8_t x : r->logmem.mem) if (x != 0xFF) { blank = false; break; }
      CHECK(blank && r->logmem.erases == 16, "the whole region is 0xFF (16 blocks of 64 kB)");
      const std::vector<int> beep = buzz_runs(b, t_done - 2, t_done + 1000);
      CHECK(runs_are(beep, {50, 50, 50}), "two short beeps when the erase is done (%s)", show(beep).c_str()); }

    printf("[blackbox] live stream to the app, switch trigger, storage problems\n");
    { auto r = fresh(); VirtualBoard& b = r->board; arm_switch_setup(*r); r->run(2.0); r->clear_inbox();
      // the app records on the bench: no storage configured at all
      CHECK(r->command(CMD_BLACKBOX_STREAM, {1, 200, 0}).err == ERR_OK, "STREAM on at 200 records/s");
      b.plant.set_tilt(12, -5, 0); b.plant.ground = true;
      for (int i = 0; i < 4; i++) { r->run(0.5); r->request(MSG_HELLO); }
      CHECK(r->fw->blackbox.state() == bb::ST_STREAMING && r->logmem.writes == 0, "streaming, nothing written to flash");
      CHECK(r->command(CMD_BLACKBOX_STREAM, {0, 0, 0}).err == ERR_OK, "STREAM off"); r->run(0.2);
      const std::vector<uint8_t> cap = bb_frames(*r, bb::kMsgHeader, bb::kMsgData);
      const std::vector<RSession> ss = read_log(cap.data(), cap.size());
      CHECK(ss.size() == 1 && ss[0].meta.at("to") == "stream" && ss[0].meta.at("rate_hz") == "200" && ss[0].bad_records == 0 && ss[0].seq_gaps == 0, "what arrived is a complete log");
      if (ss.size() == 1) {
        const RSession& s = ss[0]; const size_t last = s.F.size() - 1;
        CHECK(s.F.size() > 380 && s.F.size() < 410 && fabs(s.fv(last, "roll") - 12.0) < 1.0 && fabs(s.fv(last, "pitch") + 5.0) < 1.0 && fabs(s.fv(last, "gyro[0]")) < 2.0 && s.fv(last, "motor[0]") == 0, "on the ground, disarmed: attitude 12 / -5 deg, gyro still, motors off (%zu records)", s.F.size());
        CHECK(!s.S.empty() && s.sv(s.S.size() - 1, "armed") == 0 && s.events.back().code == bb::EV_END, "slow records and END"); }
      // the app stops talking without saying goodbye
      r->clear_inbox(); r->command(CMD_BLACKBOX_STREAM, {1, 0, 0}); r->run(1.0);
      CHECK(r->fw->blackbox.state() == bb::ST_STREAMING, "streaming again");
      r->run(3.5); CHECK(r->fw->blackbox.state() == bb::ST_OFF, "3 s of silence from the app: the stream stops by itself"); }

    { // BLACKBOX mode on an aux switch
      auto r = fresh(); VirtualBoard& b = r->board; r->run(2.0);
      S_modes m; r->get_section(SEC_MODES, &m, sizeof(m)); m.mode[0] = MODE_ARM; m.aux[0] = 0; m.lo[0] = 1700; m.hi[0] = 2100; m.mode[1] = MODE_BLACKBOX; m.aux[1] = 3; m.lo[1] = 1700; m.hi[1] = 2100; r->set_section(SEC_MODES, &m, sizeof(m));
      bb_config(*r, BB_DEVICE_FLASH, BB_TRIGGER_SWITCH, 250, 1u << BB_FIELD_GYRO);
      b.rc[4] = 2000; r->run(1.0); CHECK(r->fw->blackbox.state() == bb::ST_IDLE && r->logmem.writes == 0, "trigger SWITCH: armed but switch off -> no log");
      b.rc[7] = 2000; r->run(1.0); CHECK(r->fw->blackbox.state() == bb::ST_LOGGING, "BLACKBOX switch on: logging");
      b.batt_v = 13.6; r->run(3.0);                           // the pack sags to the warning level while the log runs
      b.rc[7] = 1000; r->run(0.5); b.rc[4] = 1000; r->run(0.5);
      const std::vector<RSession> ss = read_log(r->logmem.mem.data(), r->fw->blackbox.log().used());
      CHECK(ss.size() == 1 && ss[0].fast_size == 10 && ss[0].F.size() > 980 && ss[0].F.size() < 1020 && ss[0].meta.at("rate_div") == "16", "4 s at 250 records/s with only the gyro: %zu records of 10 bytes", ss.empty() ? 0 : ss[0].F.size());
      int batt_events = 0; uint8_t last_state = 255; if (ss.size() == 1) for (const RSession::Ev& e : ss[0].events) if (e.code == bb::EV_BATTERY && e.data.size() == 1) { batt_events++; last_state = e.data[0]; }
      CHECK(batt_events == 2 && last_state == BATT_STATE_WARNING, "the battery state is in the log: OK at the start, WARNING when the pack sagged (%d events, last %u)", batt_events, last_state); }

    { // a board whose partition table has no log partition
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision(); r.has_logmem = false; r.boot(); r.run(2.0);
      bb_config(r, BB_DEVICE_FLASH, BB_TRIGGER_ARMED, 500);
      const T_blackbox st = bb_status(r);
      CHECK(has_line(r, "blackbox: FAILED - no log partition") && st.state == BB_STATE_NO_STORAGE && st.capacity == 0 && r.command(CMD_BLACKBOX_ERASE).err == ERR_NO_SENSOR, "no log partition: reported at boot, state NO_STORAGE");
      arm_switch_setup(r); r.board.rc[4] = 2000; r.run(1.0); CHECK(r.tel().armed, "flying is not affected"); }
    { // the partition holds something else (first use on a board that had a file system there)
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision(); r.logmem.fill_garbage(0, 20000); r.boot(); r.run(2.0);
      bb_config(r, BB_DEVICE_FLASH, BB_TRIGGER_ARMED, 500); arm_switch_setup(r);
      CHECK(has_line(r, "holds other data") && bb_status(r).state == BB_STATE_NEEDS_ERASE, "foreign data in the partition: 'needs erase'");
      r.board.rc[4] = 2000; r.run(1.0); r.board.rc[4] = 1000; r.run(0.5);
      CHECK(r.logmem.writes == 0, "arming does not write over it");
      CHECK(r.command(CMD_BLACKBOX_ERASE).err == ERR_OK, "erase"); r.run(1.0);
      r.board.rc[4] = 2000; r.run(1.0); CHECK(r.fw->blackbox.state() == bb::ST_LOGGING && r.logmem.bad_writes == 0, "then it logs"); }
}

// The log says where the time of a cycle goes: the longest of each stage since the previous record.
static void t_log_stage_times() {
    printf("[blackbox] the stages of a control cycle in the log: lock, IMU, core (with and without the estimators), motors\n");
    auto r = fresh(); VirtualBoard& b = r->board; arm_switch_setup(*r); r->run(2.0);
    bb_config(*r, BB_DEVICE_FLASH, BB_TRIGGER_ARMED, 1000);                    // one record per 4 cycles; the estimators step in every 8th
    r->run(3.0);                                                               // (the free space of the log memory is checked before a flight, not during it)
    struct SlowLock : app::Sync { void lock() override { g_stub_us += 40; } } lk;   // the sensor thread happens to hold the lock for 40 us whenever it is asked for
    r->app().set_sync(&lk);
    b.cost_imu_us = 32; g_stub_millis_cost_us = 8;
    static_cast<HostMotorHw*>(r->app().motors.hw())->on_write = [] { static uint32_t n = 0; if (++n % 3 == 0) g_stub_us += 12; };    // one write in three is slow: the LONGEST of an interval is logged
    b.rc[4] = 2000; r->run(2.2);
    { T_status st; const bool got = status_now(*r, st);
      CHECK(got && st.t_imu == 32 && st.t_core == 8 && st.t_motor == 4 && st.t_rest == 40, "the status stream's averages: IMU %u us (the bus alone), core %u, motors %u (one write in three), the rest %u us - the wait for the lock is counted there", st.t_imu, st.t_core, st.t_motor, st.t_rest); }
    b.cost_imu_us = 0; g_stub_millis_cost_us = 0; static_cast<HostMotorHw*>(r->app().motors.hw())->on_write = nullptr; r->app().set_sync(nullptr);
    r->run(0.3); b.rc[4] = 1000; r->run(0.5);
    const Rig::CmdResult ses = r->command(CMD_BLACKBOX_SESSION, {0, 0}); uint32_t start = 1, size = 0;
    if (ses.data.size() == 8) { memcpy(&start, ses.data.data(), 4); memcpy(&size, ses.data.data() + 4, 4); }
    const std::vector<uint8_t> raw = bb_download(*r, start, size);
    const std::vector<RSession> ss = read_log(raw.data(), raw.size());
    CHECK(ss.size() == 1 && ss[0].F.size() > 2000, "the flight at 1000 records/s is in the log (%zu records)", ss.empty() ? (size_t)0 : ss[0].F.size());
    if (ss.size() == 1 && ss[0].F.size() > 2000) {
        const RSession& s = ss[0]; size_t same = 0, with_est = 0, without = 0, other = 0; const size_t n = s.F.size() - 330;      // (the last 0.3 s were flown without the costs)
        for (size_t i = 2; i < n; i++) {
            if (s.fv(i, "t_lock") == 40 && s.fv(i, "t_imu") == 32 && s.fv(i, "t_core") == 8 && s.fv(i, "t_motor") == 12) same++;
            const double e = s.fv(i, "t_est"); if (e == 8) with_est++; else if (e == 0) without++; else other++;
        }
        CHECK(same == n - 2, "waiting for the lock 40 us, IMU 32 us, core 8 us, motors 12 us (the slowest of the four cycles): in every record (%zu of %zu)", same, n - 2);
        CHECK(other == 0 && with_est > 0 && with_est + 1 >= without && without + 1 >= with_est, "the cycle that steps the estimators is timed apart: it is in every second record (%zu with, %zu without)", with_est, without);
        CHECK(s.fv(s.F.size() - 5, "t_lock") == 0 && s.fv(s.F.size() - 5, "t_imu") == 0 && s.fv(s.F.size() - 5, "t_motor") == 0, "the numbers are those of the interval, not the largest ever seen");
    }
}

static void t_blackbox_card() {
    printf("[blackbox] a flight logged to an SD card: the flight loop never waits, file = download = what happened; card problems\n");
    { auto r = fresh(); VirtualBoard& b = r->board; arm_switch_setup(*r); r->run(2.0);
      b.plant.true_accel = false;        // an ideal tilt sensor, as in the flash test: the log is compared with the truth
      CHECK(!has_line(*r, "SD card") && r->sdcard.mounts == 0, "blackbox off: the card socket is never touched");
      // the card is selected in the app; no reboot
      { S_blackbox s; r->get_section(SEC_BLACKBOX, &s, sizeof(s)); CHECK(s.sd_mode == SD_MODE_SPI && s.sd_mhz == 20, "default card settings: SPI, 20 MHz");
        s.device = BB_DEVICE_SDCARD; s.trigger = BB_TRIGGER_ARMED; s.rate_hz = 1000; r->set_section(SEC_BLACKBOX, &s, sizeof(s)); }
      CHECK(!(r->fw->server.flags() & SF_REBOOT_REQUIRED), "no reboot asked");
      r->run(0.5); T_blackbox st = bb_status(*r);
      CHECK(st.state == BB_STATE_IDLE && st.device == BB_DEVICE_SDCARD && st.capacity_kb == 65536 && st.used_kb == 0 && st.sessions == 0 && r->sdcard.mounts == 1, "card mounted by the writer task: idle, 64 MB (state %u, %d mounts)", st.state, r->sdcard.mounts);
      r->sdcard.stall_every = 25;                            // every 25th write the card takes 180 ms instead of 3
      std::vector<Truth> truth; truth.reserve(12000);
      auto fly = [&](double seconds) { const int n = (int)(seconds * 1000); for (int i = 0; i < n; i++) { r->run(0.001); Truth t; t.us = micros(); const Vec3 g = b.plant.gyro_dps(); t.gyro[0] = g.x; t.gyro[1] = -g.y; t.gyro[2] = g.z; t.roll = b.plant.right_down_deg(); t.z = b.plant.z; for (int k = 0; k < 4; k++) t.motor[k] = b.motor_frac[k]; t.pid_p = r->app().output().pid_p[0]; truth.push_back(t); } };
      b.rc[5] = 2000; b.rc[6] = 2000; r->run(0.3);
      const uint32_t t_arm = micros(); b.rc[4] = 2000; fly(0.5);
      CHECK(r->tel().armed && r->fw->blackbox.state() == bb::ST_LOGGING && r->sdcard.open_number == 1, "armed: logging to wb_00001.wblog");
      b.rc[2] = 1700; fly(3.5); b.rc[2] = 1500; fly(2.0); b.rc[0] = 1800; fly(1.0); b.rc[0] = 1500; fly(2.0);
      CHECK(b.plant.z > 1.0f && fabsf(b.plant.vz) < 0.4f, "it flies: z %.2f m", b.plant.z);
      st = bb_status(*r);
      CHECK(b.stalled_us == 0 && r->tel().gap_max_us == 250 && r->tel().loop_hz == 4000, "the flight loop never stood still: %u Hz, longest gap %u us (the flash would cost 0.7 ms per page)", r->tel().loop_hz, r->tel().gap_max_us);
      CHECK(r->card_longest_busy_us == 180000 && r->card_busy_total_us > 500000, "although the card kept its writer waiting %.1f s in total, up to %u ms at a time", r->card_busy_total_us * 1e-6, r->card_longest_busy_us / 1000u);
      CHECK(st.state == BB_STATE_LOGGING && st.rate_hz >= 998 && st.rate_hz <= 1002 && st.dropped == 0 && st.buffer_pct >= 1 && st.buffer_pct <= 10 && st.byte_rate > 65000 && st.byte_rate < 80000, "status: %u records/s, %u bytes/s, none dropped, buffer peak %u %%", st.rate_hz, st.byte_rate, st.buffer_pct);
      CHECK(r->logmem.writes == 0, "nothing went to the on-board flash");
      b.rc[2] = 1300; fly(6.0); b.rc[2] = 1000; fly(1.0);
      const uint32_t t_disarm = micros(); b.rc[4] = 1000; r->run(1.5);
      st = bb_status(*r);
      CHECK(!r->tel().armed && st.state == BB_STATE_IDLE && st.sessions == 1 && r->sdcard.open_number == -1 && r->sdcard.files.count(1) == 1, "disarmed: file closed, 1 session");
      const std::vector<uint8_t>& file = r->sdcard.file(1);
      CHECK(file.size() == r->sdcard.files[1].synced && st.used == file.size() && st.used_kb == (file.size() + 1023) / 1024, "every byte is synced; status: %u bytes of logs, %u kB of the card used", st.used, st.used_kb);
      // ---- the app downloads it exactly like a flash log
      const Rig::CmdResult ses = r->command(CMD_BLACKBOX_SESSION, {0, 0}); uint32_t s_start = 1, s_size = 0;
      if (ses.data.size() == 8) { memcpy(&s_start, ses.data.data(), 4); memcpy(&s_size, ses.data.data() + 4, 4); }
      const std::vector<uint8_t> raw = bb_download(*r, s_start, s_size);
      CHECK(ses.err == ERR_OK && s_start == 0 && raw == file, "download through the link = the file on the card (%zu bytes)", raw.size());
      const std::vector<RSession> ss = read_log(file.data(), file.size());
      CHECK(ss.size() == 1 && ss[0].bad_records == 0 && ss[0].seq_gaps == 0, "the file read straight from the card parses as one session");
      if (ss.size() == 1) {
        const RSession& s = ss[0];
        const double span = (s.F.back()[0] - s.F.front()[0]) * 1e-6, flown = (t_disarm - t_arm) * 1e-6;
        int exact = 0, other = 0; double max_gap = 0; for (size_t i = 1; i < s.F.size(); i++) { if (s.F[i][0] - s.F[i - 1][0] == 1000) exact++; else other++; max_gap = fmax(max_gap, s.fv(i, "gap_us")); }
        CHECK(fabs(span - flown) < 0.05 && other == 0 && exact > 15900 && max_gap == 250, "%.2f s of %.2f s armed, %d records exactly 1000 us apart, none late, gap_us always 250", span, flown, exact);
        auto at = [&](double us) -> const Truth& { size_t lo = 0, hi = truth.size() - 1; while (lo < hi) { const size_t mid = (lo + hi) / 2; if (truth[mid].us < us) lo = mid + 1; else hi = mid; } return truth[lo]; };
        auto near = [&](double us) -> const Truth& { const Truth* t0 = &at(us); return t0 > truth.data() ? t0[-1] : *t0; };      // the truth sample just before: a step in the motors may fall between the two
        double e_roll = 0, e_motor = 0, peak_roll = 0; int n = 0;
        for (size_t i = 0; i < s.F.size(); i += 7) {
            const double t = s.F[i][0]; if (t < truth.front().us + 2000 || t > truth.back().us) continue;
            const Truth& tr = at(t); n++;
            e_roll = fmax(e_roll, fabs(s.fv(i, "roll") - tr.roll)); peak_roll = fmax(peak_roll, s.fv(i, "roll"));
            for (int k = 0; k < 4; k++) { const double m = s.fv(i, std::string("motor[") + char('0' + k) + "]"); e_motor = fmax(e_motor, fmin(fabs(m - tr.motor[k]), fabs(m - near(t).motor[k]))); }
        }
        CHECK(n > 1500 && e_roll < 1.5 && e_motor < 0.03 && peak_roll > 8.0, "the values are the flight: roll within %.2f deg, motors within %.3f, the roll input shows (%.1f deg)", e_roll, e_motor, peak_roll);
        CHECK(s.count_events(bb::EV_ARM) == 1 && s.count_events(bb::EV_DISARM) == 1 && s.count_events(bb::EV_DROPPED) == 0 && s.events.back().code == bb::EV_END && s.meta.at("session") == "1", "events: arm, disarm, END; nothing dropped");
      }
      // ---- power cycle with the card selected: found at boot, said in the report; the next flight is the next file
      CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK, "settings saved");
      r->boot(); r->run(2.5);
      CHECK(has_line(*r, "blackbox: SD card 'sim-sd' 64 MB, 62 MB free, 1 log files, buffer 512 kB") && r->fw->blackbox.log().sessions() == 1, "after a power cycle: boot report names the card, 1 log file");
      b.rc[4] = 2000; r->run(1.5); b.rc[4] = 1000; r->run(1.0);
      CHECK(r->sdcard.files.count(2) == 1 && bb_status(*r).sessions == 2, "second flight: wb_00002.wblog");
      // ---- erase from the app
      r->sdcard.other["DCIM"] = 12345; r->run(3.5); const uint32_t t0 = millis();
      CHECK(r->command(CMD_BLACKBOX_ERASE).err == ERR_OK, "ERASE"); r->run(0.3); st = bb_status(*r);
      CHECK(st.state == BB_STATE_IDLE && st.sessions == 0 && r->sdcard.files.empty() && r->sdcard.other.size() == 1 && b.stalled_us == 0, "erased in a moment (files are deleted, nothing is wiped): log files gone, the rest of the card untouched");
      CHECK(runs_are(buzz_runs(b, t0, t0 + 1300), {50, 50, 50}), "two short beeps when it is done"); }

    { // booting without a card, pushing one in later; pulling it in flight
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.blackbox.device = BB_DEVICE_SDCARD; s.blackbox.trigger = BB_TRIGGER_ARMED; st.save(s); }
      r.sdcard.inserted = false; r.boot(); VirtualBoard& b = r.board; arm_switch_setup(r); r.run(2.0);
      CHECK(has_line(r, "blackbox: SD card FAILED - no SD card (it is looked for again every 3 s)") && bb_status(r).state == BB_STATE_NO_STORAGE && r.tel().blockers == 0, "no card at boot: reported, state NO_STORAGE, arming is not blocked");
      r.sdcard.push(); r.run(3.2);
      CHECK(bb_status(r).state == BB_STATE_IDLE, "card pushed in: found within 3 s");
      b.rc[5] = 2000; b.rc[6] = 2000; r.run(0.3); b.rc[4] = 2000; r.run(0.5); b.rc[2] = 1700; r.run(4.0); b.rc[2] = 1500; r.run(1.0);
      CHECK(r.tel().armed && r.fw->blackbox.state() == bb::ST_LOGGING && b.plant.z > 1.0f, "flying and logging");
      const float z0 = b.plant.z; r.sdcard.pull(); r.run(2.0);
      CHECK(r.tel().armed && fabsf(b.plant.z - z0) < 0.3f && fabsf(b.plant.right_down_deg()) < 3.0f && b.stalled_us == 0 && r.fw->blackbox.state() == bb::ST_NO_STORAGE && !r.app().blackbox.active(),
            "card shaken out in flight: the craft holds its height (%.2f -> %.2f m), the log simply ends", z0, b.plant.z);
      const std::vector<RSession> ss = read_log(r.sdcard.file(1).data(), r.sdcard.files[1].synced);
      CHECK(ss.size() == 1 && ss[0].bad_records == 0 && ss[0].F.size() > 1000, "what reached the card before that is a readable log (%zu records)", ss.empty() ? (size_t)0 : ss[0].F.size());
      b.rc[2] = 1300; r.run(5.0); b.rc[2] = 1000; r.run(1.0); b.rc[4] = 1000; r.run(1.0); }

    { // a card that is too slow for the buffer (a board without PSRAM gets 32 kB): records are lost and COUNTED, the flight is not disturbed
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision(); r.card_buffer_bytes = 32 * 1024;
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.blackbox.device = BB_DEVICE_SDCARD; s.blackbox.trigger = BB_TRIGGER_ARMED; s.blackbox.rate_hz = 2000; st.save(s); }
      r.sdcard.stall_every = 10; r.sdcard.stall_us = 900000; r.boot(); VirtualBoard& b = r.board; arm_switch_setup(r); r.run(2.0);
      CHECK(has_line(r, "buffer 32 kB"), "boot report: the small buffer");
      b.rc[4] = 2000; r.run(8.0); const T_blackbox st = bb_status(r); b.rc[4] = 1000; r.run(2.5);
      const std::vector<RSession> ss = read_log(r.sdcard.file(1).data(), r.sdcard.file(1).size());
      uint32_t lost = 0; if (ss.size() == 1) for (const RSession::Ev& e : ss[0].events) if (e.code == bb::EV_DROPPED && e.data.size() == 4) { uint32_t n; memcpy(&n, e.data.data(), 4); lost += n; }
      CHECK(st.dropped > 500 && ss.size() == 1 && lost == r.app().blackbox.ring().dropped() && ss[0].bad_records == 0 && b.stalled_us == 0 && r.tel().loop_hz == 4000,
            "2 kHz on a card with 0.9 s stalls and a 32 kB buffer: %u records lost - all counted in the log, the flight loop untouched", lost); }
}

static void t_pins_and_load() {
    printf("[pins] two functions on one GPIO, module pins; [load] CPU / RAM in the status stream\n");
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.buzzer = VirtualBoard::kMotorPin[2]; st.save(s); }
      r.boot(); arm_switch_setup(r); r.run(2.0);
      CHECK(has_line(r, "pins: FAILED - GPIO6 is set for both motor 3 and buzzer") && r.app().status.pin_conflict && (r.tel().blockers & ARM_BLOCKER_PIN_CONFLICT), "buzzer on motor 3's pin: named in the boot report, arming blocker PIN_CONFLICT");
      r.board.rc[4] = 2000; r.run(1.0); CHECK(!r.tel().armed, "the craft does not arm with a motor output that may be dead"); }
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.sd_clk = VirtualBoard::kMotorPin[0]; s.pins.sd_cs = VirtualBoard::kScl; st.save(s); }
      r.boot(); r.run(1.0);
      CHECK(!r.app().status.pin_conflict && r.tel().blockers == 0, "SD pins that collide do not count while the card is not the log device");
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.blackbox.device = BB_DEVICE_SDCARD; st.save(s); }
      r.boot(); r.run(1.0);
      CHECK(has_line(r, "pins: FAILED - GPIO9 is set for both i2c_scl and sd_cs") && (r.tel().blockers & ARM_BLOCKER_PIN_CONFLICT), "with the card selected they do (the first clash is named)");
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.blackbox.sd_mode = SD_MODE_SDMMC_1BIT; s.pins.sd_clk = 40; st.save(s); }
      r.boot(); r.run(1.0);
      CHECK(!r.app().status.pin_conflict, "SDMMC mode does not use the CS pin: no clash any more"); }
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.gps_tx = 27; st.save(s); }
      r.boot(); r.run(1.0);
      CHECK(has_line(r, "pins: WARNING - GPIO27 (gps_tx) belongs to the module's own flash / PSRAM") && r.tel().blockers == 0, "a pin of the module's flash: warned about in the boot report");
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.gps_tx = 27; s.sensors.gps_enabled = 0; st.save(s); }
      r.boot(); CHECK(!has_line(r, "pins: WARNING"), "...not when the GPS is switched off (the pin is then not used)"); }
    { g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.buzzer = 20; st.save(s); }
      r.boot(); CHECK(!has_line(r, "pins: WARNING"), "GPIO20 is an ordinary pin while the app link is a UART");
      g_rig_usb_link = true; r.boot(); g_rig_usb_link = false;
      CHECK(has_line(r, "pins: WARNING - GPIO20 (buzzer) is the USB socket this firmware talks to the app through"), "with the link on the chip's USB, a function on GPIO19 / 20 is warned about"); }
    { // a unused 5th..8th motor pin does not clash on a quad; on a hexa it does
      g_stub_us = 0; g_stub_delay_advances = true; Rig r; r.provision();
      { wbp::Settings s; wbp::SettingsStore st(r.flash); st.load(s); s.pins.motor[5] = VirtualBoard::kBuzzer; st.save(s); }
      r.boot(); CHECK(!r.app().status.pin_conflict, "quad: the pin typed for motor 6 is not in use"); }
    { auto r = fresh(); ESP.heap_size = 320 * 1024; ESP.heap_free = 200 * 1024; ESP.psram_size = 8 * 1024 * 1024; ESP.psram_free = 7680 * 1024;
      r->stream(STR_STATUS, 10); r->run(2.5); T_status st{}; r->latest(STR_STATUS, st);
      CHECK(st.ram == 38 && st.heap_free == 200 && st.psram_total == 8192 && st.psram_free == 7680, "status stream: RAM 38 %% used (120 of 320 kB), 200 kB free, PSRAM 7680 of 8192 kB free (%u %u %u %u)", st.ram, st.heap_free, st.psram_total, st.psram_free);
      CHECK(has_line(*r, "memory: 240 kB internal RAM free of 320, PSRAM 0 kB"), "boot report: memory");
      ESP.heap_free = 240 * 1024; ESP.psram_size = 0; ESP.psram_free = 0; }
}

// ARM aux0, ANGLE aux1, ALTHOLD aux2 (arm_switch_setup) + STABILIZE on aux3 (ch8) and CALIB on aux4 (ch9)
// Bidirectional DShot: what the ESCs report reaches the gyro filter, motor by motor.
static void t_rpm_filter() {
    printf("[rpm filter] bidirectional DShot: every ESC's motor speed puts a notch on that motor's noise\n");
    auto r = fresh(); r->run(1.5); arm_switch_setup(*r); VirtualBoard& b = r->board;
    { S_motors m; r->get_section(SEC_MOTORS, &m, sizeof(m)); m.bidirectional = 1; r->set_section(SEC_MOTORS, &m, sizeof(m)); CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK, "bidirectional DShot switched on, saved"); r->request(MSG_REBOOT); r->run(2.5); }
    r->stream(STR_TUNING, 50); r->stream(STR_STATUS, 20); r->run(1.0); T_tuning tu{}; T_status st{}; r->latest(STR_TUNING, tu); r->latest(STR_STATUS, st);
    { bool none = true; for (int i = 0; i < 8; i++) none &= tu.rpm_hz[i] == 0; CHECK(r->boots == 2 && none && (st.health & HEALTH_ESC_TELEMETRY), "motors standing: telemetry is there, the notches are parked (a motor below 100 Hz is not filtered)"); }
    b.rc[5] = 2000; r->run(0.3); b.rc[4] = 2000; r->run(0.5); b.rc[6] = 2000; r->run(0.2); b.rc[2] = 1700; r->run(4.0); b.rc[2] = 1500; r->run(4.0); r->latest(STR_TUNING, tu);
    { bool ok = r->tel().armed && b.plant.z > 1.0f; for (int i = 0; i < 4; i++) { const float hz = b.motor_rpm(i) / 60.0f; ok &= hz > 150 && fabsf(tu.rpm_hz[i] - hz) < 0.04f * hz + 2; } for (int i = 4; i < 8; i++) ok &= tu.rpm_hz[i] == 0;
      CHECK(ok, "hovering: the RPM notch of every motor sits on that motor's speed (%u %u %u %u Hz; the motors turn at %.0f Hz), nothing for motors the frame does not have", tu.rpm_hz[0], tu.rpm_hz[1], tu.rpm_hz[2], tu.rpm_hz[3], b.motor_rpm(0) / 60.0f); }
    // every motor has its own: one of them is slowed down by a weak prop's partner working harder
    { b.plant.gain[0] = 0.85f; r->run(3.0); r->latest(STR_TUNING, tu); bool ok = true; float lo = 1e9f, hi = 0; for (int i = 0; i < 4; i++) { const float hz = b.motor_rpm(i) / 60.0f; ok &= fabsf(tu.rpm_hz[i] - hz) < 0.04f * hz + 2; lo = fminf(lo, hz); hi = fmaxf(hi, hz); }
      CHECK(ok && hi > 1.05f * lo, "one motor 15 %% weak: the motors now turn between %.0f and %.0f Hz, and each notch follows its own (%u %u %u %u)", lo, hi, tu.rpm_hz[0], tu.rpm_hz[1], tu.rpm_hz[2], tu.rpm_hz[3]); b.plant.gain[0] = 1.0f; r->run(2.0); }
    // the telemetry wire of one ESC breaks: its notch goes, the other three stay
    { b.esc_silent[2] = true; r->run(1.0); r->latest(STR_TUNING, tu); r->latest(STR_STATUS, st); bool others = true; for (int i : {0, 1, 3}) { const float hz = b.motor_rpm(i) / 60.0f; others &= fabsf(tu.rpm_hz[i] - hz) < 0.04f * hz + 2; }
      CHECK(tu.rpm_hz[2] == 0 && others && !(st.health & HEALTH_ESC_TELEMETRY) && r->tel().armed, "ESC 3 stops answering: its notch is switched off (0 Hz), the other three keep working (%u %u - %u), the app is told the telemetry is not complete", tu.rpm_hz[0], tu.rpm_hz[1], tu.rpm_hz[3]);
      b.esc_replies = false; r->run(1.0); r->latest(STR_TUNING, tu); bool none = true; for (int i = 0; i < 8; i++) none &= tu.rpm_hz[i] == 0; CHECK(none && r->tel().armed, "no ESC answers any more: no RPM notches - the low-passes and the dynamic notch carry on, the craft flies on"); }
}

// Channels from somewhere else than the built-in serial receiver (a radio module): FlightApp::set_rc().
static void t_external_rc() {
    printf("[external receiver] channels handed in with set_rc()\n");
    auto r = fresh(); r->run(1.5); arm_switch_setup(*r); VirtualBoard& b = r->board; r->app().use_external_rc = true;
    uint16_t ch[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    auto fly = [&](double seconds) { for (int i = 0; i < (int)(seconds / 0.01 + 0.5); i++) { r->app().set_rc(ch, true); r->run(0.01); } };
    fly(0.5); ch[5] = 2000; fly(0.3); ch[4] = 2000; fly(0.5); CHECK(r->tel().armed, "armed through the handed-in channels");
    ch[2] = 1650; fly(1.5); const float z = b.plant.z; ch[2] = 1480; ch[0] = 1800; fly(1.0);
    CHECK(z > 0.3f && b.plant.right_down_deg() > 10.0f, "every set_rc() is a new frame for the stick path: the throttle lifts the craft (%.2f m), the roll stick tilts it (%.1f deg)", z, b.plant.right_down_deg());
    for (int i = 0; i < 100; i++) { r->app().set_rc(ch, false); r->run(0.01); }
    CHECK(r->tel().failsafe != FAILSAFE_NONE || !r->tel().armed, "the module reports its link lost (set_rc(..., false)): failsafe");
}

static void more_modes(Rig& r) {
    S_modes m; r.get_section(SEC_MODES, &m, sizeof(m));
    m.mode[3] = MODE_STABILIZE; m.aux[3] = 3; m.lo[3] = 1700; m.hi[3] = 2100;
    m.mode[4] = MODE_CALIB; m.aux[4] = 4; m.lo[4] = 1700; m.hi[4] = 2100;
    r.set_section(SEC_MODES, &m, sizeof(m));
}

static void t_tuning_stream() {
    printf("[tuning] the live state of the filters and the PID helpers, as the app's Filters / PID tabs show it\n");
    auto r = fresh(); r->run(1.5); arm_switch_setup(*r); VirtualBoard& b = r->board;
    CHECK(r->stream(STR_TUNING, 50), "tuning stream on"); r->run(1.0); T_tuning tu{};
    CHECK(r->latest(STR_TUNING, tu) && tu.gyro_lpf1_hz == 250 && tu.dterm_lpf1_hz == 75 && tu.air_state == AIR_STATE_GROUND && tu.tpa == 80, "disarmed: the dynamic low-passes rest at their minimum (gyro %u Hz, D-term %u Hz), state GROUND, TPA %u %% (the low-throttle part)", tu.gyro_lpf1_hz, tu.dterm_lpf1_hz, tu.tpa);
    CHECK(abs((int)tu.rc_rate - 143) <= 4 && abs((int)tu.rc_cutoff - (int)lroundf(tu.rc_rate * 1.5f / 4.0f)) <= 1, "the receiver's frame rate is measured (%u Hz; the simulated CRSF link sends every 7 ms) and sets the RC smoothing cutoff (%u Hz)", tu.rc_rate, tu.rc_cutoff);
    { bool in_range = true, no_rpm = true; for (int i = 0; i < 9; i++) in_range &= tu.dyn_notch_hz[i] >= 100 && tu.dyn_notch_hz[i] <= 600; for (int i = 0; i < 8; i++) no_rpm &= tu.rpm_hz[i] == 0;
      CHECK(in_range && no_rpm, "dynamic notches inside 100 .. 600 Hz; no RPM notches without bidirectional DShot"); }
    { S_filters f; r->get_section(SEC_FILTERS, &f, sizeof(f)); f.dyn_notch_count = 2; r->set_section(SEC_FILTERS, &f, sizeof(f)); r->run(0.3); r->latest(STR_TUNING, tu);
      CHECK(tu.dyn_notch_hz[0] >= 100 && tu.dyn_notch_hz[1] >= 100 && tu.dyn_notch_hz[7] >= 100 && tu.dyn_notch_hz[2] == 0 && tu.dyn_notch_hz[5] == 0 && tu.dyn_notch_hz[8] == 0, "two dynamic notches per axis: the third place of every axis reads 0 (%u %u %u)", tu.dyn_notch_hz[0], tu.dyn_notch_hz[1], tu.dyn_notch_hz[2]);
      f.dyn_notch_count = 3; r->set_section(SEC_FILTERS, &f, sizeof(f)); r->run(0.2); }
    b.rc[5] = 2000; r->run(0.3); b.rc[4] = 2000; r->run(0.5); b.rc[6] = 2000; r->run(0.2); b.rc[2] = 1700; r->run(4.0); b.rc[2] = 1500; r->run(4.0);
    CHECK(r->latest(STR_TUNING, tu) && r->tel().armed && b.plant.z > 1.0f, "flying");
    const float thr = r->tel().throttle, q = lroundf(thr * 100.0f) / 100.0f;
    CHECK(tu.air_state == AIR_STATE_AIR && abs((int)tu.gyro_lpf1_hz - (int)lroundf(fc::dyn_lpf_cutoff(q, 250, 500, 5))) <= 4 && abs((int)tu.dterm_lpf1_hz - (int)lroundf(fc::dyn_lpf_cutoff(q, 75, 150, 5))) <= 2,
          "hovering at %.0f %% throttle: state AIR, gyro low-pass 1 at %u Hz, D-term low-pass 1 at %u Hz (they follow the throttle)", thr * 100, tu.gyro_lpf1_hz, tu.dterm_lpf1_hz);
    CHECK(abs((int)tu.tpa - (int)lroundf(100 * (1 - fmaxf(thr - 0.35f, 0.0f)))) <= 2 && tu.anti_gravity < 300, "TPA %u %% at that throttle (65 %% less from 35 %% throttle on)", tu.tpa);
    CHECK(abs((int)tu.hover - (int)lroundf(100 * (b.plant.hover_cmd() - 0.055f) / 0.945f)) <= 2, "the hover throttle the firmware measured: %u %% (this craft hovers at %.0f %% motor output = %.0f %% throttle)", tu.hover, 100 * b.plant.hover_cmd(), 100 * (b.plant.hover_cmd() - 0.055f) / 0.945f);
    int16_t quiet = 0; for (int a = 0; a < 3; a++) quiet = (int16_t)std::max<int>(quiet, abs(tu.pid_sum[a]));
    b.rc[0] = 1800; int16_t peak = 0; for (int i = 0; i < 30; i++) { r->run(0.01); if (r->latest(STR_TUNING, tu)) peak = (int16_t)std::max<int>(peak, abs(tu.pid_sum[0])); } b.rc[0] = 1500; r->run(1.0);
    CHECK(quiet < 300 && peak > 2 * quiet + 200, "the PID sum of roll: %.1f in the steady hover, up to %.1f while the craft is rolled by the stick", quiet * 0.1, peak * 0.1);
    // a frame resonance on roll, another one on pitch: every axis has its own notches
    b.vib_hz[0] = 450; b.vib_dps[0] = 15; b.vib_hz[1] = 280; b.vib_dps[1] = 15; r->run(1.5); r->latest(STR_TUNING, tu); b.vib_dps[0] = b.vib_dps[1] = 0;
    { auto has = [&](int axis, int hz) { for (int p = 0; p < 3; p++) if (abs((int)tu.dyn_notch_hz[axis * 3 + p] - hz) <= 10) return true; return false; };
      CHECK(has(0, 450) && has(1, 280), "the gyro picks up 450 Hz on roll and 280 Hz on pitch: each axis shows a notch on its own one (roll %u %u %u, pitch %u %u %u)", tu.dyn_notch_hz[0], tu.dyn_notch_hz[1], tu.dyn_notch_hz[2], tu.dyn_notch_hz[3], tu.dyn_notch_hz[4], tu.dyn_notch_hz[5]); }
    r->run(1.0);
    // a motor loses a tenth of its thrust (a chipped prop): the I-term carries the difference, and the PID sum shows it
    { b.plant.gain[0] = 0.9f; r->run(3.0); double mean[3] = {0, 0, 0}; int n = 0; for (int i = 0; i < 50; i++) { r->run(0.02); if (r->latest(STR_TUNING, tu)) { for (int a = 0; a < 3; a++) mean[a] += tu.pid_sum[a] * 0.1; n++; } }
      const fc::PidController& pc = r->app().core.pid(); bool same = n > 0; for (int a = 0; a < 3; a++) { mean[a] /= n > 0 ? n : 1; same &= fabs(mean[a] - pc.i_term[a]) < 0.25 * fabsf(pc.i_term[a]) + 3.0; }
      CHECK(same && fabsf(pc.i_term[0]) > 8 && fabsf(pc.i_term[1]) > 8, "rear-right motor 10 %% weak: PID sum (mean over 1 s) %.1f / %.1f / %.1f = the I-terms holding the craft level (%.1f / %.1f / %.1f)", mean[0], mean[1], mean[2], pc.i_term[0], pc.i_term[1], pc.i_term[2]);
      b.plant.gain[0] = 1.0f; r->run(2.0); }
    // the throttle is chopped by hand: anti-gravity shows
    { b.rc[6] = 1000; r->run(1.0); r->latest(STR_TUNING, tu); const uint16_t calm = tu.anti_gravity; b.rc[2] = 1150; uint16_t ag = 0; for (int i = 0; i < 30; i++) { r->run(0.01); if (r->latest(STR_TUNING, tu)) ag = std::max(ag, tu.anti_gravity); }
      b.rc[2] = 1500; b.rc[6] = 2000; r->run(1.0);
      CHECK(calm < 200 && ag > 2000, "manual throttle, steady: anti-gravity %.1f; chopped from 50 %% to 15 %%: up to %.0f", calm * 0.01, ag * 0.01); }
}

// The board is not mounted level in the frame. The craft hovers, the pilot holds it still against the drift, flips the CALIB
// switch for a few seconds, lands: from then on "level" is where the craft really hovers, with no app involved.
static void t_inflight_trim() {
    printf("[in-flight level trim] hover steadily, CALIB switch on for 2 s, land: the hover attitude is the new level - and it is in the flash\n");
    auto r = fresh(); VirtualBoard& b = r->board; b.mount_roll_deg = 2.4f; b.mount_pitch_deg = -1.6f;
    r->run(1.5); arm_switch_setup(*r); more_modes(*r); r->stream(STR_CALIBRATION, 20);
    CHECK(r->ack_of(r->request(MSG_SETTINGS_SAVE)) == ERR_OK, "modes saved"); const uint32_t writes0 = r->flash.writes;
    // the pilot: holds the craft STILL with the sticks - he stops the drift (leaning against the speed it has), then keeps it level
    float sr = 1500, sp = 1500;
    auto hover = [&](double seconds) { for (int i = 0; i < (int)(seconds / 0.02); i++) { r->run(0.02);
        const float want_r = fc::clampf(-6.0f * b.plant.v_right(), -8, 8), want_p = fc::clampf(-6.0f * b.plant.v_forward(), -8, 8);
        sr = fc::clampf(sr - 1.0f * (b.plant.right_down_deg() - want_r), 1300, 1700); sp = fc::clampf(sp - 1.0f * (b.plant.nose_down_deg() - want_p), 1300, 1700); b.rc[0] = (uint16_t)lroundf(sr); b.rc[1] = (uint16_t)lroundf(sp); } };
    b.rc[5] = 2000; r->run(0.3); b.rc[4] = 2000; r->run(0.5); b.rc[6] = 2000; r->run(0.2); b.rc[2] = 1700; r->run(3.0); b.rc[2] = 1500; r->run(2.0);
    const float off_r = b.plant.right_down_deg(), off_p = b.plant.nose_down_deg();
    CHECK(r->tel().armed && b.plant.z > 0.8f && fabsf(off_r) > 1.5f && fabsf(off_p) > 1.0f && fabsf(r->tel().roll) < 0.5f, "sticks centred, the firmware believes it is level (%.2f) - the craft really leans %.2f / %.2f deg and would drift", r->tel().roll, off_r, off_p);
    const float drift = hypotf(b.plant.vn, b.plant.ve);
    hover(6.0);
    CHECK(drift > 0.8f && hypotf(b.plant.vn, b.plant.ve) < 0.05f && fabsf(b.plant.right_down_deg()) < 0.3f && fabsf(b.plant.nose_down_deg()) < 0.3f && abs((int)b.rc[0] - 1500) > 15,
          "it was drifting at %.1f m/s; the pilot stops it and holds it still with the stick off centre (roll stick at %u, %.2f m/s left)", drift, b.rc[0], hypotf(b.plant.vn, b.plant.ve));
    T_calibration cs{}; r->latest(STR_CALIBRATION, cs); CHECK(cs.inflight_trim == 0, "CALIB off: nothing is measured");
    b.rc[8] = 2000; hover(1.0); r->latest(STR_CALIBRATION, cs); CHECK(cs.inflight_trim > 30 && cs.inflight_trim < 70, "CALIB on: measuring (%u %% after 1 s)", cs.inflight_trim);
    hover(1.6); const uint32_t t_off = millis(); b.rc[8] = 1000; hover(0.3); r->latest(STR_CALIBRATION, cs);
    { const std::vector<int> ok_beep = buzz_runs(b, t_off, t_off + 300); CHECK(runs_are(ok_beep, {50, 50, 50}), "the pilot hears that it is measured: two short beeps (%s)", show(ok_beep).c_str()); }
    S_calibration cal; r->get_section(SEC_CALIBRATION, &cal, sizeof(cal));
    CHECK(cs.inflight_trim == 100 && cal.acc_trim_roll == 0 && cal.acc_trim_pitch == 0 && (uint32_t)r->flash.writes == writes0, "switch off after 2.6 s: measured - but nothing changes in the air (trim still %d / %d, no flash write)", cal.acc_trim_roll, cal.acc_trim_pitch);
    // Measured again right after a dash to the side and a stop. The attitude ESTIMATE is still off then (the accelerometer of a craft
    // that speeds up or brakes does not show its tilt, and the estimate gives way to it for seconds - core/ahrs.h); the accelerometer
    // itself is right the moment the craft stands still. The trim is taken from the accelerometer.
    b.rc[0] = 1750; r->run(1.5); const float dash = hypotf(b.plant.vn, b.plant.ve); hover(2.5);
    const float est_off = fabsf(r->tel().roll - (b.plant.right_down_deg() + 2.4f));
    b.rc[8] = 2000; hover(2.6); b.rc[8] = 1000; hover(0.3); r->latest(STR_CALIBRATION, cs);
    CHECK(dash > 3.0f && est_off > 0.7f && cs.inflight_trim == 100, "after a dash at %.1f m/s and a stop the estimate is %.1f deg off for a while; measured again in exactly that moment", dash, est_off);
    b.rc[2] = 1250; for (int i = 0; i < 400 && b.plant.z > 0.0f; i++) hover(0.05);                                   // come down
    b.rc[0] = b.rc[1] = 1500; b.rc[2] = 1000; r->run(1.0); b.rc[4] = 1000; r->run(0.5);
    r->get_section(SEC_CALIBRATION, &cal, sizeof(cal));
    CHECK(!r->tel().armed && fabsf(cal.acc_trim_roll * 0.1f - 2.4f) < 0.3f && fabsf(cal.acc_trim_pitch * 0.1f + 1.6f) < 0.3f, "landed, disarmed: the trim is set (%.1f / %.1f deg for a board mounted 2.4 / -1.6 deg off)", cal.acc_trim_roll * 0.1, cal.acc_trim_pitch * 0.1);
    b.rest(0, 0); r->run(3.0);
    CHECK(fabsf(r->tel().roll) < 0.35f && fabsf(r->tel().pitch) < 0.35f, "standing level on the ground now reads level: %.2f / %.2f", r->tel().roll, r->tel().pitch);
    { T_status st{}; r->stream(STR_STATUS, 20); r->run(0.3); r->latest(STR_STATUS, st); bool told = false; for (const std::string& l : r->logs()) told |= l.find("calibration") != std::string::npos && l.find("written") != std::string::npos;
      CHECK((uint32_t)r->flash.writes > writes0 && !(st.settings_dirty & SETTINGS_FLAGS_DIRTY) && told, "and it went to the flash by itself: there is no app in the field to press Save (the app, if connected, is told)"); }
    const int16_t tr = cal.acc_trim_roll, tp = cal.acc_trim_pitch;
    r->request(MSG_REBOOT); r->run(2.5); S_calibration cal2; r->get_section(SEC_CALIBRATION, &cal2, sizeof(cal2));
    CHECK(r->boots == 2 && cal2.acc_trim_roll == tr && cal2.acc_trim_pitch == tp, "it survives a power cycle");
    // the next flight: sticks centred = really level
    b.rc[5] = 2000; r->run(0.5); b.rc[4] = 2000; r->run(0.5); b.rc[6] = 2000; r->run(0.2); b.rc[2] = 1700; r->run(3.0); b.rc[2] = 1500; r->run(3.0);
    CHECK(r->tel().armed && b.plant.z > 0.8f && fabsf(b.plant.right_down_deg()) < 0.4f && fabsf(b.plant.nose_down_deg()) < 0.4f, "next flight, sticks centred: the craft hovers level (%.2f / %.2f deg, was %.2f / %.2f)", b.plant.right_down_deg(), b.plant.nose_down_deg(), off_r, off_p);
    // while the craft is being turned the time does not count: 2.5 s of switch in a yaw, and it is refused
    { b.rc[3] = 1750; r->run(0.6); b.rc[8] = 2000; r->run(2.5); r->latest(STR_CALIBRATION, cs); const uint8_t turning = cs.inflight_trim; const uint32_t t0 = millis(); b.rc[8] = 1000; r->run(0.6); b.rc[3] = 1500; r->run(1.0);
      const std::vector<int> bad = buzz_runs(b, t0, t0 + 600); r->latest(STR_CALIBRATION, cs);
      CHECK(turning < 15 && cs.inflight_trim == 0 && bad.size() >= 1 && !runs_are(bad, {50, 50, 50}), "CALIB on for 2.5 s while yawing: %u %% collected, refused - with the other sound (%s)", turning, show(bad).c_str()); }
    // too short: refused, the good trim stays
    b.rc[8] = 2000; r->run(0.6); b.rc[8] = 1000; r->run(0.3); b.rc[2] = 1250; for (int i = 0; i < 400 && b.plant.z > 0.0f; i++) r->run(0.05); b.rc[2] = 1000; r->run(1.0); b.rc[4] = 1000; r->run(0.5);
    r->get_section(SEC_CALIBRATION, &cal2, sizeof(cal2)); CHECK(cal2.acc_trim_roll == tr && cal2.acc_trim_pitch == tp, "a flick of the switch (0.6 s) changes nothing");
    // armed on a sloping table with the switch on: standing on the ground is not a hover, the slope must not become "level"
    b.rest(4, -3); r->run(2.0); b.rc[0] = b.rc[1] = 1500; b.rc[2] = 1000; b.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "armed on the ground, tilted 4 / -3 deg");
    b.rc[8] = 2000; r->run(3.5); b.rc[8] = 1000; r->run(0.3); b.rc[4] = 1000; r->run(0.5); r->get_section(SEC_CALIBRATION, &cal2, sizeof(cal2));
    CHECK(!r->tel().armed && cal2.acc_trim_roll == tr && cal2.acc_trim_pitch == tp, "CALIB on for 3.5 s while standing there: nothing is measured, the trim stays (%d / %d)", cal2.acc_trim_roll, cal2.acc_trim_pitch);
}

// Runaway take-off prevention through the firmware: the settings, the motors, the status stream.
static void t_runaway() {
    printf("[runaway take-off] motor order wrong in the settings: stopped at take-off, shown as a blocker; the setting to switch it off\n");
    for (int on = 1; on >= 0; on--) { auto r = fresh(); r->run(1.5); arm_switch_setup(*r); VirtualBoard& b = r->board;
      S_general g; r->get_section(SEC_GENERAL, &g, sizeof(g)); const bool def = g.runaway_takeoff == 1; g.runaway_takeoff = (uint8_t)on; r->set_section(SEC_GENERAL, &g, sizeof(g));
      r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(3.0);
      CHECK(def && r->app().core.cfg.runaway.enabled == (on != 0), "general.runaway_takeoff: on by default, %s after the reboot", on ? "still on" : "off");
      b.plant.motor_swap(0, 1);                              // two ESC plugs in each other's place
      b.plant.gain[2] = 0.94f; b.plant.ground_holds = true;
      b.rc[4] = 2000; r->run(0.5); const bool armed = r->tel().armed;
      b.rc[2] = 1650; float t_off = -1, tilt_off = 0, tilt_max = 0;
      for (int k = 0; k < 300; k++) { r->run(0.005); const float tilt = acosf(fc::clampf(b.plant.R22(), -1, 1)) * fc::kRad2Deg; tilt_max = fmaxf(tilt_max, tilt); if (t_off < 0 && !r->tel().armed) { t_off = (k + 1) * 0.005f; tilt_off = tilt; } }
      T_status st{}; status_now(*r, st);
      if (on) {
        CHECK(armed && t_off > 0 && t_off < 0.9f && tilt_off < 55.0f && b.motor_frac[0] == 0 && b.motor_frac[3] == 0, "throttle up: the craft starts to go over, the motors stop %.2f s later at %.0f degrees", t_off, tilt_off);
        CHECK((st.blockers & ARM_BLOCKER_RUNAWAY_TAKEOFF) && (st.blockers & ARM_BLOCKER_ARM_SWITCH_ON) && !st.armed, "the status stream shows why (blockers 0x%x)", st.blockers);
        b.plant.set_tilt(0, 0); b.plant.w[0] = b.plant.w[1] = b.plant.w[2] = 0; b.rc[2] = 1000; b.rc[4] = 1000; r->run(1.0); status_now(*r, st);
        CHECK(st.blockers == 0, "arm switch off, craft upright again: nothing blocks (0x%x)", st.blockers);
      } else CHECK(armed && t_off < 0 && tilt_max > 120.0f && !(st.blockers & ARM_BLOCKER_RUNAWAY_TAKEOFF), "switched off: the firmware lets it go over (%.0f degrees), armed to the end", tilt_max);
    }
}


// ------------------------------------------------------------------------------------------------------------------------------
// Navigation on the virtual board: optical flow sensors, the position estimate, Position Hold, GPS Rescue.
// Switches: ARM ch5, ANGLE ch6, ALTHOLD ch7, POSHOLD ch8 (RESCUE ch9 where a test maps it).
template <class F> static std::unique_ptr<Rig> nav_rig(F edit, bool usb = true) {
    g_stub_us = 0; g_stub_delay_advances = true;
    std::unique_ptr<Rig> r(new Rig()); r->provision(); r->usb_link = usb;
    { wbp::Settings s; wbp::SettingsStore st(r->flash); st.load(s);
      const uint8_t mode[4] = {MODE_ARM, MODE_ANGLE, MODE_ALTHOLD, MODE_POSHOLD};
      for (int i = 0; i < 4; i++) { s.modes.mode[i] = mode[i]; s.modes.aux[i] = (uint8_t)i; s.modes.lo[i] = 1700; s.modes.hi[i] = 2100; }
      edit(s, r->board); st.save(s); }
    r->boot();
    return r;
}
static void msp_flow(wbp::Settings& s, VirtualBoard& b) { s.flow.type = FLOW_TYPE_MSP; s.pins.flow_rx = VirtualBoard::kFlowRx; b.mtf.present = true; }
static void pmw_flow(wbp::Settings& s, VirtualBoard& b) {
    s.flow.type = FLOW_TYPE_PMW3901; s.pins.flow_cs = VirtualBoard::kFlowCs; s.pins.flow_sck = VirtualBoard::kFlowSck; s.pins.flow_miso = VirtualBoard::kFlowMiso; s.pins.flow_mosi = VirtualBoard::kFlowMosi; b.has_pmw = true;
}
// Arm in Angle + Altitude Hold, climb for `climb_s` at 0.4 of the stick, hover.
static void nav_takeoff(Rig& r, double climb_s) {
    VirtualBoard& b = r.board; b.rc[5] = 2000; b.rc[6] = 2000; r.run(0.3); b.rc[4] = 2000; r.run(0.5); b.rc[2] = 1700; r.run(climb_s); b.rc[2] = 1500; r.run(3.0);
}
static float nav_far(Rig& r, float n0, float e0) { return hypotf(r.board.plant.pn - n0, r.board.plant.pe - e0); }
static bool nav_now(Rig& r, T_nav& n) { r.clear_inbox(); r.stream(STR_NAV, 50); r.run(0.05); const bool ok = r.latest(STR_NAV, n); r.stream(STR_NAV, 0); return ok; }
// A lidar next to a PMW3901 (the virtual board has no VL53 chip on its bus): the distance along the beam, straight from the plant, 50 times a second.
struct PlantLidar : RangeDriver {
    VirtualBoard* b = nullptr; uint32_t last_us = 0; bool ok = true;
    const char* name() const override { return "test lidar"; }
    bool init() override { return true; }
    bool update(uint32_t now_us, RangeReading& out) override {
        if (!ok || (uint32_t)(now_us - last_us) < 20000u) return false;
        last_us = now_us; out.distance_m = fmaxf(b->plant.z, 0.02f) / fmaxf(b->plant.R22(), 0.3f); out.quality_pct = 90; return true;
    }
    float max_range_m() const override { return 4.0f; }
    float min_range_m() const override { return 0.04f; }
    float nominal_rate_hz() const override { return 50.0f; }
    bool healthy() const override { return ok; }
};
// Somebody carries the craft level, `z` above the floor, at this speed (m/s, north / east).
static void carry(Rig& r, float z, float vn, float ve, float yaw_deg = 0) { Plant& p = r.board.plant; p.held = true; p.set_tilt(0, 0, yaw_deg); p.w[0] = p.w[1] = p.w[2] = 0; p.z = z; p.hold_vn = vn; p.hold_ve = ve; }
static void put_down(Rig& r) { Plant& p = r.board.plant; p.held = false; p.hold_vn = p.hold_ve = 0; p.vn = p.ve = 0; r.board.rest(0, 0); p.pn = p.pe = 0; }

static void t_nav_flow() {
    printf("[optical flow] an MSP sensor on UART0 with its own lidar: detection, the speed it measures, Position Hold on it\n");
    { auto r = nav_rig(msp_flow); VirtualBoard& b = r->board;
      CHECK(has_line(*r, "flow: MSP flow on UART0") && has_line(*r, "range: MSP lidar (flow sensor)") && r->app().status.flow && r->app().status.range, "boot report: the flow sensor on UART0, its lidar as the range sensor");
      CHECK(Serial0.rx_pin == VirtualBoard::kFlowRx && b.mtf.host_baud == 115200 && Serial1.rx_pin == VirtualBoard::kGpsRx && Serial2.rx_pin == VirtualBoard::kRcRx, "UART0 opened on pin flow_rx at 115200 baud; the GPS and the receiver keep UART1 / UART2");
      r->run(2.0); T_nav n{};
      CHECK((r->tel().health & HEALTH_FLOW) && (r->tel().health & HEALTH_RANGE) && r->tel().blockers == 0, "after 2 s: flow and range healthy, nothing blocks arming (health 0x%04x)", r->tel().health);
      CHECK(nav_now(*r, n) && n.flow_rate >= 98 && n.flow_rate <= 101 && n.flow_quality == 78 && n.flow_reject == FLOW_REJECT_HEIGHT && !(n.flags & NAV_FLAGS_POSHOLD),
            "nav stream on the ground: %u samples/s, quality %u, not used: too close to the floor (reject %u)", n.flow_rate, n.flow_quality, n.flow_reject);
      // carried by hand, 1 m up: the speed the app shows for checking the mounting direction
      carry(*r, 1.0f, 0.6f, 0.0f); r->run(2.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_OK && fabsf(n.flow_fwd * 0.01f - 0.6f) < 0.08f && fabsf(n.flow_right * 0.01f) < 0.08f && r->tel().flow_valid && r->tel().vel_valid && !r->tel().pos_valid,
            "carried forwards at 0.6 m/s, 1 m up: the nav stream reads %.2f forwards, %.2f right", n.flow_fwd * 0.01, n.flow_right * 0.01);
      CHECK(fabsf(r->tel().vel_n - 0.6f) < 0.1f && fabsf(r->tel().vel_e) < 0.1f, "and the estimate has it as a velocity (%.2f north, %.2f east; without a compass, north is where the nose pointed at power-up)", r->tel().vel_n, r->tel().vel_e);
      { carry(*r, 1.0f, 0.0f, 0.0f); r->run(0.5); nav_now(*r, n); const float soon = n.flow_fwd * 0.01f; r->run(1.0); nav_now(*r, n);
        CHECK(soon > 0.25f && soon < 0.4f && fabsf(n.flow_fwd * 0.01f) < 0.03f, "stopped (the hand slows the craft down within a few tenths of a second): the speed shown follows, smoothed over a quarter second for the eye (%.2f m/s after 0.5 s, %.2f after 1.5 s)", soon, n.flow_fwd * 0.01); }
      carry(*r, 1.0f, 0.0f, -0.4f); r->run(2.0);
      CHECK(nav_now(*r, n) && fabsf(n.flow_right * 0.01f + 0.4f) < 0.08f && fabsf(n.flow_fwd * 0.01f) < 0.08f, "carried to the LEFT at 0.4 m/s: %.2f right, %.2f forwards", n.flow_right * 0.01, n.flow_fwd * 0.01);
      // tilting on the spot is not travelling: the gyro's part is taken away
      { carry(*r, 1.0f, 0, 0); r->run(1.5); float worst = 0, tilt = 0;
        for (int i = 0; i < 800; i++) { b.plant.w[0] = 1.6f * cosf(i * 0.0025f * 6.2832f) ; r->run(0.0025); worst = fmaxf(worst, hypotf(r->tel().vel_n, r->tel().vel_e)); tilt = fmaxf(tilt, fabsf(b.plant.right_down_deg())); }
        CHECK(tilt > 13.0f && tilt < 16.0f && worst < 0.25f, "rocked +-%.0f deg about the roll axis on the spot, once a second (92 deg/s at most): the estimated speed stays below %.2f m/s", tilt, worst); }
      put_down(*r); r->run(2.0); }
    { // flight
      auto r = nav_rig(msp_flow); VirtualBoard& b = r->board; r->run(2.0);
      b.rc[7] = 2000; nav_takeoff(*r, 3.0); T_nav n{};
      CHECK(r->tel().armed && b.plant.z > 0.6f && b.plant.z < 2.0f, "armed with the Position Hold switch on (a flow sensor is something to hold with), hovering at %.2f m", b.plant.z);
      CHECK(nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && (n.flags & NAV_FLAGS_NAV_LEAN) && !(n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && n.ph_roll == PH_AXIS_LOITER && n.ph_pitch == PH_AXIS_LOITER, "nav stream: Position Hold is flying, both axes LOITER (flags 0x%x)", n.flags);
      CHECK(r->tel().vel_valid && r->tel().flow_valid && r->tel().tilt_aided && !r->tel().pos_valid && (r->tel().modes & (1u << MODE_POSHOLD)), "estimate: a velocity from flow, the tilt corrected from it, no position on the map");
      { const float n0 = b.plant.pn, e0 = b.plant.pe; float far = 0; for (int i = 0; i < 100; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
        CHECK(far < 0.3f, "10 s: it stays within %.3f m", far);
        b.plant.wind_e = 4.0f; float far_w = 0; for (int i = 0; i < 150; i++) { r->run(0.1); far_w = fmaxf(far_w, nav_far(*r, n0, e0)); }
        CHECK(far_w > 0.15f && far_w < 1.5f && nav_far(*r, n0, e0) < 0.4f && b.plant.right_down_deg() < -4.0f && nav_now(*r, n) && n.wind_e * 0.01f < -0.8f,
              "a wind of 4 m/s from the west: blown %.2f m at most, %.2f m off after 15 s, leaning %.1f deg into it (learned %.2f m/s^2)", far_w, nav_far(*r, n0, e0), -b.plant.right_down_deg(), -n.wind_e * 0.01);
        // the camera loses the floor while leaning into the wind: the lean the nav stream shows is the autopilot's only while it flies
        b.mtf.quality = 40; r->run(0.6); const bool gone = nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && !(n.flags & NAV_FLAGS_NAV_LEAN) && n.lean_roll == 0 && n.lean_pitch == 0;
        b.mtf.quality = 200; r->run(3.0);
        CHECK(gone && nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && n.lean_roll < -30, "flow lost in the wind: no autopilot lean in the nav stream (not the last one held); back: %.1f deg against the wind again", n.lean_roll * 0.1);
        b.plant.wind_e = 0; r->run(6.0); }
      // the pilot flies forwards and lets go
      { b.rc[1] = 1750; r->run(2.0); const float v = b.plant.vn; const bool pilot = nav_now(*r, n) && n.ph_pitch == PH_AXIS_PILOT;
        b.rc[1] = 1500; const float n_rel = b.plant.pn; float back = 0; for (int i = 0; i < 80; i++) { r->run(0.1); back = fmaxf(back, -b.plant.nose_down_deg()); }
        CHECK(pilot && v > 1.0f && back > 3.0f && b.plant.pn - n_rel > 0.3f && b.plant.pn - n_rel < 8.0f && hypotf(b.plant.vn, b.plant.ve) < 0.15f && nav_now(*r, n) && n.ph_pitch == PH_AXIS_LOITER,
              "stick forwards: it flies (%.1f m/s); let go: it leans back %.0f deg, stops %.1f m further on and holds there", v, back, b.plant.pn - n_rel);
        CHECK(fabsf(n.target_n * 0.1f - r->tel().pos_n) < 0.5f && fabsf(n.target_e * 0.1f - r->tel().pos_e) < 0.5f && fabsf(n.target_n * 0.1f) > 0.5f, "the nav stream has the point it holds: %.1f / %.1f m (the estimate: %.1f / %.1f)", n.target_n * 0.1, n.target_e * 0.1, r->tel().pos_n, r->tel().pos_e); }
      // the floor gives the camera nothing (water, darkness)
      { b.mtf.quality = 40; r->run(0.6); const bool told = nav_now(*r, n) && n.flow_reject == FLOW_REJECT_QUALITY && n.flow_quality == 15 && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && !(n.flags & NAV_FLAGS_POSHOLD);
        b.rc[0] = 1700; r->run(1.5);
        CHECK(told && !r->tel().vel_valid && b.plant.right_down_deg() > 5.0f && (r->tel().health & HEALTH_FLOW), "surface quality 15 of 100: samples refused, Position Hold says it cannot, the stick flies the craft as in Angle (%.1f deg)", b.plant.right_down_deg());
        b.rc[0] = 1500; b.mtf.quality = 200; r->run(4.0);
        CHECK(nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && hypotf(b.plant.vn, b.plant.ve) < 0.3f, "the picture is back: it holds again (%.2f m/s)", hypotf(b.plant.vn, b.plant.ve)); }
      // the plug comes out in flight
      { b.mtf.present = false; r->run(0.3); const bool not_yet = (r->tel().health & HEALTH_FLOW) != 0; r->run(0.5); const float z0 = b.plant.z; r->run(3.0);
        CHECK(not_yet && !(r->tel().health & HEALTH_FLOW) && !(r->tel().health & HEALTH_RANGE) && nav_now(*r, n) && n.flow_rate == 0 && n.flow_reject == FLOW_REJECT_NO_SAMPLE && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE),
              "the sensor's plug comes out: after half a second neither flow nor range is healthy, 0 samples/s, 'no sample' (not 'used')");
        CHECK(r->tel().armed && fabsf(b.plant.z - z0) < 0.4f && r->tel().alt_valid && fabsf(b.plant.right_down_deg()) < 3.0f, "the craft stays level in Altitude Hold on the barometer (%.2f -> %.2f m)", z0, b.plant.z);
        b.mtf.present = true; r->run(3.0);
        CHECK((r->tel().health & HEALTH_FLOW) && (r->tel().health & HEALTH_RANGE) && nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD), "plugged in again: both back, Position Hold takes over again"); } }
    { // the flight log: the navigation group
      auto r = nav_rig(msp_flow); VirtualBoard& b = r->board; r->run(2.0);
      bb_config(*r, BB_DEVICE_FLASH, BB_TRIGGER_ARMED, 100); r->run(0.2);
      b.rc[7] = 2000; nav_takeoff(*r, 3.0); b.plant.wind_e = 3.0f; r->run(8.0); const float lean = b.plant.right_down_deg();
      b.rc[4] = 1000; r->run(1.0); const T_blackbox st = bb_status(*r);
      const Rig::CmdResult ses = r->command(CMD_BLACKBOX_SESSION, {0, 0}); uint32_t s_start = 1, s_size = 0; if (ses.data.size() == 8) { memcpy(&s_start, ses.data.data(), 4); memcpy(&s_size, ses.data.data() + 4, 4); }
      const std::vector<RSession> ss = read_log(bb_download(*r, s_start, s_size).data(), s_size);
      CHECK(st.sessions == 1 && ss.size() == 1 && ss[0].bad_records == 0 && ss[0].S.size() > 100 && ss[0].col(ss[0].slow, "nav_flags") > 0 && ss[0].col(ss[0].slow, "flow_fwd") > 0 && ss[0].col(ss[0].slow, "pos_n") > 0,
            "a flight in Position Hold logged with the default groups: the slow records carry the navigation fields (%zu records)", ss.empty() ? (size_t)0 : ss[0].S.size());
      if (ss.size() == 1 && ss[0].S.size() > 100) { const RSession& l = ss[0]; const size_t last = l.S.size() - 12, first = 1;        // (12 records = 1.2 s before the end: still flying)
        CHECK(!((int)l.sv(first, "nav_flags") & NAV_FLAGS_POSHOLD) && l.sv(first, "flow_reject") == FLOW_REJECT_HEIGHT && l.sv(first, "flow_quality") == 78 && l.sv(first, "rescue_phase") == RESCUE_PHASE_IDLE, "at the start, on the ground: flow not used (too low), nothing flying the craft");
        CHECK(((int)l.sv(last, "nav_flags") & NAV_FLAGS_POSHOLD) && ((int)l.sv(last, "nav_flags") & NAV_FLAGS_NAV_LEAN) && l.sv(last, "flow_reject") == FLOW_REJECT_OK && (int)l.sv(last, "est_valid") == (EST_VALID_ALT | EST_VALID_VEL | EST_VALID_FLOW | EST_VALID_TILT_AIDED),
              "in the hover: Position Hold flying, flow used, est_valid 0x%x", (int)l.sv(last, "est_valid"));
        CHECK(lean < -3.0f && fabs(l.sv(last, "nav_roll") - lean) < 2.0 && fabs(l.sv(last, "nav_pitch")) < 2.0 && fabs(l.sv(last, "flow_fwd")) < 0.2 && fabs(l.sv(last, "vel_e")) < 0.2,
              "the lean the autopilot asked for against the wind is in the log: %.1f deg (the craft: %.1f)", l.sv(last, "nav_roll"), lean); } }
    { // a sensor that is set up but silent (wrong baud rate): nothing to hold with
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.flow.baud = 57600; }); VirtualBoard& b = r->board; r->run(2.0);
      CHECK(has_line(*r, "flow: MSP flow on UART0") && b.mtf.host_baud == 57600 && !(r->tel().health & HEALTH_FLOW) && !(r->tel().health & HEALTH_RANGE), "sensor at 115200, settings say 57600: the UART is opened, nothing readable arrives, not healthy");
      b.rc[5] = 2000; b.rc[6] = 2000; b.rc[7] = 2000; r->run(0.3);
      CHECK(r->tel().blockers & ARM_BLOCKER_POSHOLD_NO_GPS, "Position Hold switch on without anything to hold with: arming blocker (0x%x)", r->tel().blockers);
      b.rc[4] = 2000; r->run(0.5); CHECK(!r->tel().armed, "does not arm"); }
}

static void t_nav_mounting() {
    printf("[optical flow] the sensor's mounting direction, mirror and scale from the settings; the PMW3901 on its own SPI bus\n");
    { // an MSP sensor turned a quarter to the right, its picture mirrored, and a lens that makes it read 25 % fast
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); b.flow_truth.mount_cw_deg = 90; b.flow_truth.mirrored = true; b.mtf.scale = 1.25f; }); T_nav n{};
      r->run(2.0); carry(*r, 1.0f, 0.6f, 0.0f); r->run(2.0);
      CHECK(nav_now(*r, n) && fabsf(n.flow_fwd * 0.01f) < 0.1f && fabsf(fabsf(n.flow_right * 0.01f) - 0.75f) < 0.1f, "settings untouched, carried forwards at 0.6 m/s: the app would show %.2f forwards, %.2f right - the mounting is wrong", n.flow_fwd * 0.01, n.flow_right * 0.01);
      // set right in the app, with the craft switched on: the nav section is live
      S_nav nv; r->get_section(SEC_NAV, &nv, sizeof(nv)); nv.flow_align = FLOW_ALIGN_CW90;
      CHECK(r->set_section(SEC_NAV, &nv, sizeof(nv)).type == MSG_SECTION && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "flow_align CW90 written: no reboot asked for");
      r->run(1.5); CHECK(nav_now(*r, n) && fabsf(n.flow_fwd * 0.01f + 0.75f) < 0.1f && fabsf(n.flow_right * 0.01f) < 0.1f, "turned right, but forwards reads backwards (%.2f): the picture is mirrored", n.flow_fwd * 0.01);
      nv.flow_flip = 1; r->set_section(SEC_NAV, &nv, sizeof(nv)); r->run(1.5);
      CHECK(nav_now(*r, n) && fabsf(n.flow_fwd * 0.01f - 0.75f) < 0.1f && fabsf(n.flow_right * 0.01f) < 0.1f, "flow_flip set: %.2f forwards - the direction is right, the speed 25 %% too high", n.flow_fwd * 0.01);
      nv.flow_scale = 80; r->set_section(SEC_NAV, &nv, sizeof(nv)); r->run(1.5);
      CHECK(nav_now(*r, n) && fabsf(n.flow_fwd * 0.01f - 0.6f) < 0.08f && fabsf(n.flow_right * 0.01f) < 0.08f, "flow_scale 80 %%: %.2f forwards, %.2f right", n.flow_fwd * 0.01, n.flow_right * 0.01);
      // the sensor type is read at power-up: changing it asks for a reboot
      { S_flow f; r->get_section(SEC_FLOW, &f, sizeof(f)); f.baud = 230400; CHECK(r->set_section(SEC_FLOW, &f, sizeof(f)).type == MSG_SECTION && (r->fw->server.flags() & SF_REBOOT_REQUIRED) && r->board.mtf.host_baud == 115200, "the flow section (type, baud rate) is not live: reboot required, the UART keeps its speed");
        f.baud = 115200; r->set_section(SEC_FLOW, &f, sizeof(f)); }
      r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(3.0); carry(*r, 1.0f, 0.6f, 0.0f); r->run(2.0);
      CHECK(r->boots == 2 && nav_now(*r, n) && fabsf(n.flow_fwd * 0.01f - 0.6f) < 0.08f && fabsf(n.flow_right * 0.01f) < 0.08f, "saved; after a reboot: %.2f forwards, %.2f right", n.flow_fwd * 0.01, n.flow_right * 0.01);
      carry(*r, 1.0f, 0.0f, 0.5f); r->run(2.0);
      CHECK(nav_now(*r, n) && fabsf(n.flow_right * 0.01f - 0.5f) < 0.08f && fabsf(n.flow_fwd * 0.01f) < 0.08f, "carried to the right at 0.5 m/s: %.2f right, %.2f forwards", n.flow_right * 0.01, n.flow_fwd * 0.01);
      // with that mounting it flies
      put_down(*r); r->run(1.5);
      r->board.rc[7] = 2000; nav_takeoff(*r, 3.0); const float n0 = r->board.plant.pn, e0 = r->board.plant.pe; r->board.plant.wind_n = -3.0f; float far = 0; for (int i = 0; i < 120; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
      CHECK(r->tel().armed && far < 1.5f && nav_far(*r, n0, e0) < 0.4f, "Position Hold with the turned, mirrored sensor in a 3 m/s wind: %.2f m at most, %.2f m after 12 s", far, nav_far(*r, n0, e0)); }
    { // PMW3901
      auto r = nav_rig(pmw_flow, false); VirtualBoard& b = r->board; T_nav n{};
      CHECK(has_line(*r, "flow: PMW3901 on its own SPI bus") && has_line(*r, "flow: WARNING - no range sensor") && r->app().status.flow, "boot report: PMW3901 found on the second SPI bus; a warning that there is no lidar");
      CHECK(g_stub_spi2_pins[0] == VirtualBoard::kFlowSck && g_stub_spi2_pins[1] == VirtualBoard::kFlowMiso && g_stub_spi2_pins[2] == VirtualBoard::kFlowMosi && g_spi_begin_pins[0] == VirtualBoard::kSck && g_stub_spi_bus_started == FSPI,
            "its bus is started on the flow pins; the IMU keeps its own bus and pins");
      CHECK(b.pmw.resets == 1 && b.pmw.writes >= 74 && b.pmw.reg[0x7F] == 0x00 && b.pmw.reg[0x5A] == 0x50 && b.pmw.reg[0x4E] == 0xA8, "the chip was reset and got PixArt's start-up tables (%u register writes)", b.pmw.writes);
      r->run(2.0);
      CHECK((r->tel().health & HEALTH_FLOW) && nav_now(*r, n) && n.flow_rate >= 98 && n.flow_rate <= 101 && n.flow_quality == 80 && n.flow_reject == FLOW_REJECT_HEIGHT, "polled %u times a second, surface quality %u; without a height nothing is used", n.flow_rate, n.flow_quality);
      { const uint32_t b0 = b.pmw.bursts, l0 = r->app().loops(); r->run(1.0); CHECK(b.pmw.bursts - b0 >= 99 && b.pmw.bursts - b0 <= 101 && r->app().loops() - l0 == 4000, "one motion burst every 10 ms (%u in a second), the control loop keeps its 4000 cycles", (unsigned)(b.pmw.bursts - b0)); }
      b.rc[5] = 2000; b.rc[6] = 2000; b.rc[7] = 2000; r->run(0.3);
      CHECK(r->tel().blockers & ARM_BLOCKER_POSHOLD_NO_GPS, "a flow sensor without a range sensor is nothing to hold a position with: arming with the switch on is refused");
      b.rc[5] = b.rc[6] = b.rc[7] = 1000;
      // a lidar is fitted
      PlantLidar lidar; lidar.b = &b; r->app().status.range = r->app().sonar.useDriver(&lidar); r->run(1.0);
      CHECK(!(r->tel().blockers & ARM_BLOCKER_POSHOLD_NO_GPS), "with a lidar: no blocker");
      carry(*r, 0.8f, 0.5f, -0.3f); r->run(2.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_OK && fabsf(n.flow_fwd * 0.01f - 0.5f) < 0.06f && fabsf(n.flow_right * 0.01f + 0.3f) < 0.06f, "carried at 0.5 m/s forwards and 0.3 to the left, 0.8 m up: %.2f forwards, %.2f right", n.flow_fwd * 0.01, n.flow_right * 0.01);
      b.pmw.shutter_upper = 0x1F; r->run(1.0);
      CHECK(nav_now(*r, n) && n.flow_quality == 0 && n.flow_reject == FLOW_REJECT_QUALITY && !r->tel().flow_valid && abs(n.flow_fwd) < 3, "too dark for the chip (shutter at its longest): quality 0, nothing used");
      b.pmw.shutter_upper = 0x05; put_down(*r); r->run(1.5);
      b.rc[7] = 2000; nav_takeoff(*r, 3.0); const float n0 = b.plant.pn, e0 = b.plant.pe; b.plant.wind_e = -3.0f; float far = 0; for (int i = 0; i < 120; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
      CHECK(r->tel().armed && b.plant.z > 0.6f && far < 1.5f && nav_far(*r, n0, e0) < 0.4f && nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD), "Position Hold on the PMW3901 in a 3 m/s wind: %.2f m at most, %.2f m after 12 s", far, nav_far(*r, n0, e0));
      // the chip stops answering (a broken wire): every byte reads 0xFF - one count backwards on both axes at best quality, if the driver believed it
      b.plant.wind_e = 0; r->run(8.0); b.pmw.present = false; r->run(0.15); const bool held_on = nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && hypotf(b.plant.vn, b.plant.ve) < 0.08f; r->run(2.3);
      CHECK(held_on && !(r->tel().health & HEALTH_FLOW) && nav_now(*r, n) && n.flow_rate == 0 && !(n.flags & NAV_FLAGS_POSHOLD) && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && fabsf(b.plant.right_down_deg()) < 2.0f && fabsf(b.plant.nose_down_deg()) < 2.0f && hypotf(b.plant.vn, b.plant.ve) < 0.3f,
            "the chip's data wire breaks in flight: no phantom speed, not healthy after half a second, Position Hold gives the craft back level (%.1f / %.1f deg, %.2f m/s)", b.plant.right_down_deg(), b.plant.nose_down_deg(), hypotf(b.plant.vn, b.plant.ve)); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { pmw_flow(s, b); b.pmw.id = 0x30; b.pmw.power_on(); }, false);
      CHECK(has_line(*r, "flow: FAILED - no PMW3901 answered (chip id not 0x49") && !r->app().status.flow && !has_line(*r, "flow: WARNING"), "another chip on those pins: refused by its id");
      r->run(2.0); CHECK(!(r->tel().health & HEALTH_FLOW) && r->tel().blockers == 0, "not healthy; flying without it is not blocked"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { pmw_flow(s, b); s.pins.flow_miso = 255; }, false);
      CHECK(has_line(*r, "flow: FAILED - PMW3901: pins not set"), "a pin left unset: said so"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { pmw_flow(s, b); s.blackbox.device = BB_DEVICE_SDCARD; s.blackbox.sd_mode = SD_MODE_SPI; s.pins.sd_cs = 33; s.pins.sd_clk = 34; s.pins.sd_cmd = 35; s.pins.sd_d0 = 36; }, false);
      CHECK(has_line(*r, "flow: FAILED - PMW3901 and the SD card in SPI mode need the same SPI bus") && r->board.pmw.writes == 0, "PMW3901 + SD card in SPI mode: refused, the bus is left to the card");
      S_blackbox bbs; r->get_section(SEC_BLACKBOX, &bbs, sizeof(bbs)); bbs.sd_mode = SD_MODE_SDMMC_1BIT; r->set_section(SEC_BLACKBOX, &bbs, sizeof(bbs)); r->request(MSG_SETTINGS_SAVE); r->request(MSG_REBOOT); r->run(2.0);
      CHECK(r->boots == 2 && has_line(*r, "flow: PMW3901 on its own SPI bus"), "with the card on SDMMC both work"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { pmw_flow(s, b); s.pins.flow_cs = VirtualBoard::kMotorPin[1]; }, false);
      CHECK(has_line(*r, "pins: FAILED - GPIO5 is set for both motor 2 and flow_cs") && r->app().status.pin_conflict, "flow_cs on a motor's pin: pin conflict"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.pins.flow_rx = VirtualBoard::kMotorPin[2]; });
      CHECK(has_line(*r, "pins: FAILED - GPIO6 is set for both motor 3 and flow_rx") && r->app().status.pin_conflict, "an MSP sensor's flow_rx on a motor's pin: pin conflict"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.pins.flow_cs = VirtualBoard::kMotorPin[1]; s.pins.flow_rx = VirtualBoard::kMotorPin[2]; }, false);
      CHECK(!r->app().status.pin_conflict && !has_line(*r, "flow:"), "no flow sensor selected: its pins are not in use, and the boot report does not mention it"); }
}

static void t_nav_uarts() {
    printf("[optical flow] which UART an MSP sensor gets\n");
    { auto r = nav_rig(msp_flow, false);
      CHECK(has_line(*r, "flow: FAILED - no UART left for the flow sensor") && has_line(*r, "range: FAILED - the lidar of the flow sensor: the flow sensor did not start") && !r->app().status.flow,
            "app on UART0, GPS on UART1, receiver on UART2: no UART left - said so, for the flow sensor and for its lidar");
      CHECK(Serial1.rx_pin == VirtualBoard::kGpsRx && Serial2.rx_pin == VirtualBoard::kRcRx, "(GPS and receiver are untouched)"); r->run(2.0);
      CHECK((r->tel().health & HEALTH_RX) && !(r->tel().health & HEALTH_FLOW) && r->tel().blockers == 0, "the craft works without it"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.sensors.gps_enabled = 0; b.wire_flow_uart(1); }, false); r->run(2.0);
      CHECK(has_line(*r, "flow: MSP flow on UART1") && has_line(*r, "gps: FAILED") && Serial1.rx_pin == VirtualBoard::kFlowRx && (r->tel().health & HEALTH_FLOW) && (r->tel().health & HEALTH_RANGE), "GPS switched off: the flow sensor takes UART1 (on its own pin), and works"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.sensors.gps_enabled = 0; b.wire_flow_uart(1); }, true);
      CHECK(has_line(*r, "flow: MSP flow on UART1"), "a free UART1 is taken before UART0 (UART0 prints the boot messages of the chip at power-up)"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.pins.flow_rx = 255; });
      CHECK(has_line(*r, "flow: FAILED - flow sensor on a UART: pin flow_rx not set") && Serial0.rx_pin != VirtualBoard::kFlowRx, "pin flow_rx not set: said so, no UART opened"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.sensors.range_model = RANGE_MODEL_MSP; }); r->run(2.0);
      CHECK(has_line(*r, "range: MSP lidar (flow sensor)") && (r->tel().health & HEALTH_RANGE), "range model MSP chosen in the settings: the flow sensor's lidar is the range sensor"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.sensors.range_enabled = 0; }); r->run(2.0);
      CHECK(has_line(*r, "range: FAILED - sonar disabled") && has_line(*r, "flow: WARNING - no range sensor") && (r->tel().health & HEALTH_FLOW) && !(r->tel().health & HEALTH_RANGE), "range sensor switched off in the settings: the flow sensor's lidar is not used either, and the report warns"); }
}


static void gps_compass(wbp::Settings& s, VirtualBoard&) { s.sensors.mag_enabled = 1; s.calibration.mag_valid = 1; s.calibration.mag_radius = 0.48f; }
static void rescue_setup(wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); s.nav.fs_procedure = FS_PROCEDURE_RESCUE; s.nav.rescue_initial_climb = 5; s.nav.rescue_min_start_dist = 5; s.nav.rescue_descent_dist = 10; }
// Take off, climb to about 5 m, fly north for `out_s` seconds with the stick at 0.6, stop.
static void fly_out(Rig& r, double out_s = 6.0) {
    VirtualBoard& b = r.board; b.rc[5] = 2000; b.rc[6] = 2000; r.run(0.3); b.rc[4] = 2000; r.run(0.5); b.rc[2] = 1800; r.run(6.0); b.rc[2] = 1500; r.run(2.0);
    b.rc[1] = 1800; r.run(out_s); b.rc[1] = 1500; r.run(3.0);
}
struct RescueSeen { std::string order; float max_z = 0, t = 0, yaw_err_fly = 0, closest = 1e9f; };
// Runs until the craft is disarmed (or `timeout` s): the order of the rescue phases, the highest point, how well the nose pointed home on the way.
static void rescue_watch(Rig& r, double timeout, RescueSeen& s, bool until_disarm = true) {
    static const char* names[] = {"IDLE", "CLIMB", "ROTATE", "FLY_HOME", "DESCENT", "LANDING", "FAILED"}; int last = -1; double fly_t = 0;
    for (double t = 0; t < timeout; t += 0.05) {
        r.run(0.05); s.t += 0.05f; const app::Telemetry& tl = r.tel(); const Plant& p = r.board.plant;
        if (tl.rescue_phase != last) { last = tl.rescue_phase; if (!s.order.empty()) s.order += " "; s.order += names[last < 7 ? last : 0]; fly_t = 0; }
        s.max_z = fmaxf(s.max_z, p.z); s.closest = fminf(s.closest, hypotf(p.pn, p.pe));
        if (last == RESCUE_PHASE_FLY_HOME) { fly_t += 0.05; if (fly_t > 1.5 && hypotf(p.pn, p.pe) > 15.0f) { float e = p.yaw_deg() - atan2f(-p.pe, -p.pn) * fc::kRad2Deg; while (e > 180) e -= 360; while (e < -180) e += 360; s.yaw_err_fly = fmaxf(s.yaw_err_fly, fabsf(e)); } }
        if (until_disarm && !tl.armed) break;
    }
}

static void t_nav_gps() {
    printf("[position hold] GPS + compass on the virtual board; arming rules\n");
    { // the craft stands with its nose to the east-north-east: north of the map is not where the nose points
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); b.plant.set_tilt(0, 0, 60); }, false); VirtualBoard& b = r->board; T_nav n{}; T_estimate e{};
      CHECK(has_line(*r, "mag: QMC5883L") && !has_line(*r, "not calibrated") && !has_line(*r, "flow:"), "boot: a calibrated compass, no flow sensor");
      r->run(6.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_NO_SAMPLE && n.flow_rate == 0 && n.flow_quality == 0 && n.flow_fwd == 0, "nav stream without a flow sensor: 'no sample' - never 'the sample was used'");
      CHECK((r->tel().health & HEALTH_GPS) && (r->tel().health & HEALTH_MAG_READY) && r->tel().yaw_from_mag && fabsf(r->tel().yaw - 60.0f) < 3.0f && r->tel().blockers == 0, "after 6 s: GPS fix, heading %.1f deg from the compass (true 60)", r->tel().yaw);
      b.rc[7] = 2000; nav_takeoff(*r, 4.0);
      CHECK(r->tel().armed && b.plant.z > 1.5f && r->tel().pos_valid && r->tel().vel_valid && r->tel().tilt_aided && !r->tel().flow_valid, "armed with the Position Hold switch on, %.1f m up: a position and a velocity from the GPS, the tilt corrected from it", b.plant.z);
      CHECK(nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && (n.flags & NAV_FLAGS_NAV_LEAN) && n.ph_roll == PH_AXIS_LOITER && n.ph_pitch == PH_AXIS_LOITER && fabsf(n.target_n * 0.1f - b.plant.pn) < 1.0f && fabsf(n.target_e * 0.1f - b.plant.pe) < 1.0f,
            "nav stream: holding at %.1f / %.1f m (the craft is at %.1f / %.1f)", n.target_n * 0.1, n.target_e * 0.1, b.plant.pn, b.plant.pe);
      const float n0 = b.plant.pn, e0 = b.plant.pe; float far = 0; for (int i = 0; i < 100; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
      CHECK(far < 0.6f, "10 s: it stays within %.2f m", far);
      b.plant.wind_n = -5.0f; float far_w = 0; for (int i = 0; i < 200; i++) { r->run(0.1); far_w = fmaxf(far_w, nav_far(*r, n0, e0)); }
      CHECK(far_w > 0.3f && far_w < 3.0f && nav_far(*r, n0, e0) < 0.8f && nav_now(*r, n) && n.wind_n * 0.01f > 1.0f, "a wind of 5 m/s from the north (60 deg off the nose): blown %.2f m at most, %.2f m off after 20 s; learned %.2f m/s^2 northwards against it", far_w, nav_far(*r, n0, e0), n.wind_n * 0.01);
      r->clear_inbox(); r->stream(STR_ESTIMATE, 50); r->run(0.1);
      CHECK(r->latest(STR_ESTIMATE, e) && (e.valid & EST_VALID_POS) && (e.valid & EST_VALID_VEL) && (e.valid & EST_VALID_TILT_AIDED) && (e.valid & EST_VALID_YAW_FROM_MAG) && !(e.valid & EST_VALID_FLOW) && fabsf(e.pos_n * 0.1f - b.plant.pn) < 1.0f && fabsf(e.pos_e * 0.1f - b.plant.pe) < 1.0f,
            "estimate stream: position %.1f / %.1f m (true %.1f / %.1f), valid bits 0x%x", e.pos_n * 0.1, e.pos_e * 0.1, b.plant.pn, b.plant.pe, e.valid);
      r->stream(STR_ESTIMATE, 0); b.plant.wind_n = 0; r->run(8.0);
      // the stick, with the nose at 60 degrees: forwards is forwards
      b.rc[1] = 1750; r->run(2.5); const float vf = b.plant.v_forward(), vr = b.plant.v_right(); b.rc[1] = 1500; r->run(8.0);
      CHECK(vf > 1.5f && fabsf(vr) < 0.5f && hypotf(b.plant.vn, b.plant.ve) < 0.3f && nav_now(*r, n) && n.ph_pitch == PH_AXIS_LOITER, "stick forwards: %.1f m/s along the nose (%.1f sideways); let go: stops and holds", vf, vr);
      // the GPS loses its fix
      b.gps.truth.fix3d = false; b.gps.truth.fix_after_ms = 0; r->run(1.0); const bool still = nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD); r->run(2.5);
      CHECK(still && !(r->tel().health & HEALTH_GPS) && nav_now(*r, n) && !(n.flags & NAV_FLAGS_POSHOLD) && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && !r->tel().pos_valid && r->tel().armed && fabsf(b.plant.right_down_deg()) < 3.0f,
            "the GPS loses its fix: the estimate carries on for 2 s, then Position Hold says it cannot and the pilot has the craft (level, Altitude Hold)"); }
    { auto r = nav_rig([](wbp::Settings&, VirtualBoard&) {}, false); VirtualBoard& b = r->board; r->run(6.0);
      b.rc[5] = 2000; b.rc[6] = 2000; b.rc[7] = 2000; r->run(0.3);
      CHECK((r->tel().health & HEALTH_GPS) && (r->tel().blockers & ARM_BLOCKER_POSHOLD_NO_GPS), "a GPS fix but no compass: nothing to hold a position with (the craft would not know where north is) - arming with the switch on is refused");
      b.rc[7] = 1000; r->run(0.3); CHECK(r->tel().blockers == 0, "switch off: free to arm"); }

    printf("[gps rescue] a lost link far out: the craft comes home and lands; what stops a rescue\n");
    { auto r = nav_rig(rescue_setup, false); VirtualBoard& b = r->board; T_nav n{}; T_status st{};
      r->run(1.5); CHECK(r->tel().blockers & ARM_BLOCKER_RESCUE_NO_FIX, "the failsafe is set to RESCUE and the GPS has no fix yet: arming blocker RESCUE_NO_FIX (0x%x)", r->tel().blockers);
      b.rc[4] = 2000; r->run(0.5); CHECK(!r->tel().armed, "does not arm (there would be no home point)"); b.rc[4] = 1000;
      r->run(4.5); CHECK(r->tel().blockers == 0, "fix: free to arm");
      fly_out(*r); const float d0 = hypotf(b.plant.pn, b.plant.pe), z0 = b.plant.z;
      CHECK(d0 > 30.0f && z0 > 3.0f && nav_now(*r, n) && (n.flags & NAV_FLAGS_RESCUE_AVAILABLE) && !(n.flags & NAV_FLAGS_RESCUE) && fabsf(n.home_dist * 0.1f - d0) < 2.0f, "flown %.0f m north, %.1f m up; nav stream: a rescue is possible, home is %.1f m away", d0, z0, n.home_dist * 0.1);
      b.tx_on = false; r->run(1.5);
      CHECK(r->tel().failsafe == FAILSAFE_DESCENT && status_now(*r, st) && st.fs_cause == FS_CAUSE_LINK && nav_now(*r, n) && (n.flags & NAV_FLAGS_RESCUE) && n.rescue_phase == RESCUE_PHASE_CLIMB && fabsf(n.return_alt * 0.1f - (z0 + 5.0f)) < 1.0f,
            "transmitter off: failsafe (cause LINK) starts the rescue - climbing to %.1f m (highest point of the flight + 5)", n.return_alt * 0.1);
      RescueSeen s; rescue_watch(*r, 90.0, s);
      CHECK(s.order == "CLIMB ROTATE FLY_HOME DESCENT LANDING IDLE", "phases: %s", s.order.c_str());
      CHECK(!r->tel().armed && b.plant.z < 0.05f && hypotf(b.plant.pn, b.plant.pe) < 1.5f && s.t < 45.0f && b.motor_frac[0] == 0, "landed %.2f m from where it took off, %.0f s after the link was lost; disarmed, motors off", hypotf(b.plant.pn, b.plant.pe), s.t + 1.6f);
      CHECK(s.max_z > z0 + 4.0f && s.max_z < z0 + 6.5f && s.yaw_err_fly < 15.0f, "it climbed to %.1f m first and flew home nose first (never more than %.0f deg off)", s.max_z, s.yaw_err_fly);
      b.tx_on = true; r->run(1.5); CHECK(!r->tel().armed && r->tel().blockers, "link back with the arm switch still on: stays disarmed"); }
    { // the link comes back on the way home: the rescue goes on until the pilot moves a stick
      auto r = nav_rig(rescue_setup, false); VirtualBoard& b = r->board; T_nav n{}; r->run(6.0); fly_out(*r);
      b.tx_on = false; for (int i = 0; i < 400 && r->tel().rescue_phase != RESCUE_PHASE_FLY_HOME; i++) r->run(0.05);
      r->run(1.5); const float d1 = hypotf(b.plant.pn, b.plant.pe); b.tx_on = true; r->run(3.0); const float d2 = hypotf(b.plant.pn, b.plant.pe);
      CHECK(r->tel().rc_link && r->tel().failsafe == FAILSAFE_DESCENT && nav_now(*r, n) && (n.flags & NAV_FLAGS_RESCUE) && d2 < d1 - 10.0f, "link back for 3 s, sticks untouched: still flying home (%.0f -> %.0f m)", d1, d2);
      b.rc[0] = 1800; r->run(0.5); b.rc[0] = 1500; r->run(3.0);
      CHECK(r->tel().failsafe == FAILSAFE_NONE && nav_now(*r, n) && !(n.flags & NAV_FLAGS_RESCUE) && n.rescue_phase == RESCUE_PHASE_IDLE && r->tel().armed && b.plant.z > 3.0f, "a stick moved: the pilot has the craft back (Angle + Altitude Hold, %.1f m up)", b.plant.z);
      const float lean0 = b.plant.nose_down_deg(); b.rc[1] = 1800; r->run(1.5);
      CHECK(fabsf(lean0) < 3.0f && b.plant.nose_down_deg() > 10.0f, "it was level; stick forwards: the nose goes down (%.1f deg)", b.plant.nose_down_deg()); }
    { // RESCUE on a switch
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); s.modes.mode[4] = MODE_RESCUE; s.modes.aux[4] = 4; s.modes.lo[4] = 1700; s.modes.hi[4] = 2100; s.nav.rescue_initial_climb = 5; s.nav.rescue_min_start_dist = 5; }, false);
      VirtualBoard& b = r->board; T_nav n{}; r->run(1.5);
      CHECK(r->tel().blockers & ARM_BLOCKER_RESCUE_NO_FIX, "RESCUE on a switch counts as 'a rescue is set up': no arming without a fix");
      r->run(4.5); fly_out(*r); b.rc[8] = 2000; r->run(1.0);
      CHECK(r->tel().failsafe == FAILSAFE_NONE && nav_now(*r, n) && (n.flags & NAV_FLAGS_RESCUE) && n.rescue_phase == RESCUE_PHASE_CLIMB && (r->tel().modes & (1u << MODE_RESCUE)), "switch on: the rescue flies (no failsafe involved)");
      for (int i = 0; i < 400 && r->tel().rescue_phase != RESCUE_PHASE_FLY_HOME; i++) r->run(0.05);
      r->run(2.0); const float v = hypotf(b.plant.vn, b.plant.ve); b.rc[8] = 1000; r->run(3.0);
      CHECK(v > 4.0f && nav_now(*r, n) && !(n.flags & NAV_FLAGS_RESCUE) && r->tel().armed && fabsf(b.plant.nose_down_deg()) < 4.0f, "on the way home at %.1f m/s; switch off: the pilot has it back at once", v);
      // the GPS dies during a rescue the pilot switched on: it gives up, and the pilot (who is there) has the craft
      b.rc[8] = 2000; r->run(3.0); const bool again = nav_now(*r, n) && (n.flags & NAV_FLAGS_RESCUE); b.gps.responds = false; for (int i = 0; i < 400 && r->tel().rescue_phase != RESCUE_PHASE_FAILED; i++) r->run(0.05);
      r->run(3.0);
      CHECK(again && nav_now(*r, n) && n.rescue_phase == RESCUE_PHASE_FAILED && n.rescue_fail != RESCUE_FAIL_NONE && !(n.flags & NAV_FLAGS_RESCUE) && r->tel().armed && r->tel().failsafe == FAILSAFE_NONE && b.plant.z > 3.0f && fabsf(b.plant.nose_down_deg()) < 3.0f,
            "GPS module dead during a rescue on the switch: the rescue gives up (reason %u) and the pilot has the craft, level in Altitude Hold at %.1f m", n.rescue_fail, b.plant.z);
      b.rc[0] = 1750; r->run(1.5); CHECK(b.plant.right_down_deg() > 8.0f, "the sticks work (%.1f deg)", b.plant.right_down_deg()); }
    { // ... and during a rescue the failsafe started: nobody is there, so it lands where it is
      auto r = nav_rig(rescue_setup, false); VirtualBoard& b = r->board; r->run(6.0); fly_out(*r);
      b.tx_on = false; for (int i = 0; i < 400 && r->tel().rescue_phase != RESCUE_PHASE_FLY_HOME; i++) r->run(0.05);
      r->run(1.5); b.gps.responds = false; RescueSeen s; rescue_watch(*r, 90.0, s);
      CHECK(s.order.find("FAILED") != std::string::npos && !r->tel().armed && b.plant.z < 0.05f && s.closest > 10.0f && r->tel().failsafe == FAILSAFE_LANDED,
            "GPS module dead during a failsafe rescue: it gives up and lands where it is, %.0f m from home (phases: %s)", hypotf(b.plant.pn, b.plant.pe), s.order.c_str()); }
    { // critical battery far out: always a landing on the spot
      auto r = nav_rig(rescue_setup, false); VirtualBoard& b = r->board; T_nav n{}; T_status st{}; r->run(6.0); fly_out(*r); const float d0 = hypotf(b.plant.pn, b.plant.pe);
      b.batt_v = 13.0; r->run(4.0);
      CHECK(r->tel().failsafe == FAILSAFE_DESCENT && status_now(*r, st) && (st.fs_cause & FS_CAUSE_BATTERY) && nav_now(*r, n) && !(n.flags & NAV_FLAGS_RESCUE), "battery critical with RESCUE as the failsafe procedure: descent, no rescue (cause 0x%x)", st.fs_cause);
      for (int i = 0; i < 600 && r->tel().armed; i++) r->run(0.1);
      CHECK(!r->tel().armed && b.plant.z < 0.05f && fabsf(hypotf(b.plant.pn, b.plant.pe) - d0) < 8.0f, "it lands where it is (%.0f m out), not after a flight home on an empty pack", hypotf(b.plant.pn, b.plant.pe)); }
    { // no compass
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.nav.fs_procedure = FS_PROCEDURE_RESCUE; }, false); r->run(6.0);
      CHECK((r->tel().health & HEALTH_GPS) && r->tel().blockers == ARM_BLOCKER_RESCUE_NO_COMPASS, "RESCUE set up without a compass: arming blocker RESCUE_NO_COMPASS (0x%x)", r->tel().blockers);
      // ... and the nav section is live: back to LAND, the blocker goes without a reboot
      S_nav nv; r->get_section(SEC_NAV, &nv, sizeof(nv)); nv.fs_procedure = FS_PROCEDURE_LAND;
      CHECK(r->set_section(SEC_NAV, &nv, sizeof(nv)).type == MSG_SECTION && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "failsafe procedure back to LAND: no reboot asked for");
      r->run(0.5); CHECK(r->tel().blockers == 0, "the blocker is gone"); }
    { // no fix allowed
      auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { rescue_setup(s, b); s.nav.rescue_allow_no_fix = 1; b.gps.truth.fix_after_ms = 0; }, false); VirtualBoard& b = r->board; T_nav n{}; r->run(6.0);
      CHECK(!(r->tel().health & HEALTH_GPS) && r->tel().blockers == 0, "rescue_allow_no_fix: free to arm without a fix");
      b.rc[5] = 2000; b.rc[6] = 2000; r->run(0.3); b.rc[4] = 2000; r->run(0.5); b.rc[2] = 1750; r->run(4.0); b.rc[2] = 1500; r->run(2.0); const float z0 = b.plant.z;
      CHECK(r->tel().armed && z0 > 1.5f && nav_now(*r, n) && !(n.flags & NAV_FLAGS_RESCUE_AVAILABLE), "flying; the nav stream says no rescue is possible");
      b.tx_on = false; r->run(3.0);
      CHECK(r->tel().failsafe == FAILSAFE_DESCENT && nav_now(*r, n) && !(n.flags & NAV_FLAGS_RESCUE) && b.plant.z < z0 - 0.3f, "link lost: the failsafe falls back to a landing on the spot (%.1f -> %.1f m)", z0, b.plant.z);
      for (int i = 0; i < 400 && r->tel().armed; i++) r->run(0.1);
      CHECK(!r->tel().armed && b.plant.z < 0.05f, "landed, disarmed"); }
    { // nav settings while the craft is switched on
      auto r = nav_rig(msp_flow); T_nav n{}; r->run(2.0); carry(*r, 1.0f, 0.5f, 0); r->run(1.5);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_OK && n.flow_fwd > 40, "(flow in use, quality 78)");
      S_nav nv; r->get_section(SEC_NAV, &nv, sizeof(nv)); nv.flow_quality_min = 90; r->set_section(SEC_NAV, &nv, sizeof(nv)); r->run(1.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_QUALITY && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "flow_quality_min raised to 90 in the app: samples are refused from now on, without a reboot");
      nv.flow_quality_min = 30; nv.flow_max_height = 80; r->set_section(SEC_NAV, &nv, sizeof(nv)); r->run(1.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_HEIGHT, "flow_max_height 0.8 m with the craft 1 m up: too high");
      nv.flow_max_height = 400; nv.ph_angle_max = 12; nv.tilt_aid = 0; r->set_section(SEC_NAV, &nv, sizeof(nv)); r->run(1.0);
      CHECK(nav_now(*r, n) && n.flow_reject == FLOW_REJECT_OK && fabsf(r->app().core.cfg.poshold.angle_max_deg - 12.0f) < 0.01f && fabsf(r->app().core.poshold().cfg.angle_max_deg - 12.0f) < 0.01f && !r->tel().tilt_aided,
            "ph_angle_max and tilt_aid reach the flight core (and Position Hold itself)");
      S_modes mo; r->get_section(SEC_MODES, &mo, sizeof(mo)); const bool before = r->app().core.cfg.rescue_configured;
      mo.mode[9] = MODE_RESCUE; mo.aux[9] = 5; mo.lo[9] = 1700; mo.hi[9] = 2100; r->set_section(SEC_MODES, &mo, sizeof(mo)); r->run(0.2); const bool with = r->app().core.cfg.rescue_configured;
      mo.mode[9] = 255; r->set_section(SEC_MODES, &mo, sizeof(mo)); r->run(0.2);
      CHECK(!before && with && !r->app().core.cfg.rescue_configured, "a GPS Rescue switch assigned in the Modes tab: the rescue counts as set up at once (arming will want a fix), and not any more when it is taken away"); }
}

// ---------------------------------------------------------------------------------------------------------------------
// servo outputs: each follows one RC channel
// ---------------------------------------------------------------------------------------------------------------------
// CRSF frames the firmware wrote to the receiver's TX pin (parsed here with a CRC of its own)
struct CrsfOut { uint8_t type; std::vector<uint8_t> p; };
static std::vector<CrsfOut> crsf_out(const std::vector<uint8_t>& b, int& bad) {
    std::vector<CrsfOut> v; size_t i = 0; bad = 0;
    while (i + 4 <= b.size()) {
        if (b[i] != 0xC8) { bad++; i++; continue; }
        const size_t len = b[i + 1]; if (i + 2 + len > b.size()) break;
        uint8_t c = 0; for (size_t k = i + 2; k < i + 1 + len; k++) { c ^= b[k]; for (int j = 0; j < 8; j++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
        if (c != b[i + 1 + len]) { bad++; i++; continue; }
        v.push_back({b[i + 2], std::vector<uint8_t>(b.begin() + (long)i + 3, b.begin() + (long)(i + 1 + len))}); i += 2 + len;
    }
    return v;
}
static int be16(const std::vector<uint8_t>& p, int o) { return (int16_t)((p[o] << 8) | p[o + 1]); }
static uint32_t be32(const std::vector<uint8_t>& p, int o) { return ((uint32_t)p[o] << 24) | ((uint32_t)p[o + 1] << 16) | ((uint32_t)p[o + 2] << 8) | p[o + 3]; }

static void t_gps_baud() {
    printf("[gps baud] a module found at the rate it has and switched to the one in the settings (115200 by default)\n");
    { auto r = nav_rig([](wbp::Settings&, VirtualBoard& b) { b.gps.module_baud = 9600; });        // a module out of the box
      Rig& g = *r; g.run(8.0);
      CHECK(has_line(g, "gps: UART1 115200 baud") && g.board.gps.module_baud == 115200 && g.tel().gps_baud == 115200 && g.tel().gps_fix >= 3,
            "module at 9600, default settings: found, switched to 115200, fix (module %u, firmware %u, fix %u)", g.board.gps.module_baud, g.tel().gps_baud, g.tel().gps_fix); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { b.gps.module_baud = 230400; s.gps.baud = 57600; });
      Rig& g = *r; g.run(8.0);
      CHECK(g.board.gps.module_baud == 57600 && g.tel().gps_baud == 57600 && g.tel().gps_fix >= 3, "module at 230400, settings 57600: moved down to 57600 (%u)", g.board.gps.module_baud); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { b.gps.module_baud = 38400; s.gps.baud = 100000; });
      Rig& g = *r; g.run(8.0);
      CHECK(g.board.gps.module_baud == 115200 && g.tel().gps_fix >= 3, "a rate typed by hand (100000): the nearest standard one, 115200 (%u)", g.board.gps.module_baud); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { b.gps.module_baud = 9600; s.gps.auto_baud = 0; });
      Rig& g = *r; g.run(8.0);
      CHECK(g.board.gps.module_baud == 9600 && g.tel().gps_fix == 0 && !(g.tel().health & HEALTH_GPS), "auto baud off: the firmware only listens at 115200, a module at 9600 is not found (and not touched)"); }
}

static void t_ports() {
    printf("[ports] the serial ports table: GPS, receiver and flow sensor on the UARTs the table names; what it refuses\n");
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.ports.assign = PORT_ASSIGN_TABLE; s.ports.function[0] = PORT_FUNCTION_FLOW_MSP; s.ports.function[1] = PORT_FUNCTION_SERIAL_RX; s.ports.function[2] = PORT_FUNCTION_GPS;
                                                              b.swap_gps_rc_uarts(); }, true);
      Rig& g = *r; g.run(5.0);
      CHECK(has_line(g, "gps: UART2") && has_line(g, "rx: CRSF on UART1") && has_line(g, "flow: MSP flow on UART0"), "boot report: GPS on UART2, receiver on UART1, flow sensor on UART0");
      CHECK(g.tel().rc_link && g.tel().gps_fix >= 3 && (g.tel().health & HEALTH_FLOW) && Serial2.rx_pin == VirtualBoard::kGpsRx && Serial1.rx_pin == VirtualBoard::kRcRx, "and all three work there (link %d, fix %u)", (int)g.tel().rc_link, g.tel().gps_fix); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.ports.assign = PORT_ASSIGN_TABLE; s.ports.function[1] = PORT_FUNCTION_NONE; s.ports.function[2] = PORT_FUNCTION_SERIAL_RX; });
      r->run(1.0);
      CHECK(has_line(*r, "gps: FAILED - no UART has the function GPS") && has_line(*r, "rx: CRSF on UART2") && r->tel().rc_link, "the table has no GPS: said so, the receiver still works"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.ports.assign = PORT_ASSIGN_TABLE; s.ports.function[0] = PORT_FUNCTION_GPS; s.ports.function[1] = PORT_FUNCTION_NONE; s.ports.function[2] = PORT_FUNCTION_SERIAL_RX; }, false);
      CHECK(has_line(*r, "ports: WARNING - UART0 carries the app link") && has_line(*r, "gps: FAILED - no UART has the function GPS"), "GPS put on UART0 while the app talks through it: refused, said why"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.ports.function[0] = PORT_FUNCTION_GPS; s.ports.function[1] = PORT_FUNCTION_NONE; s.ports.function[2] = PORT_FUNCTION_NONE; });
      r->run(1.0);
      CHECK(has_line(*r, "gps: UART1") && has_line(*r, "rx: CRSF on UART2") && r->tel().rc_link, "AUTO (the default): the table is not looked at - an old settings record that held anything there changes nothing"); }
}

static void t_frames() {
    printf("[frames] the Betaflight layouts at boot: 6 / 8 motors on Oneshot, refused on DShot (4 RMT transmitters)\n");
    const uint8_t extra[4] = {39, 40, 41, 42};
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.motors.layout = LAYOUT_OCTO_X8; s.motors.protocol = ESC_PROTOCOL_ONESHOT125; for (int i = 0; i < 4; i++) s.pins.motor[4 + i] = extra[i]; });
      CHECK(has_line(*r, "motors: host x8") && r->app().status.motors && r->app().motors.cfg().count == 8, "octo X8 on Oneshot125: 8 outputs"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.motors.layout = LAYOUT_HEX_PLUS; s.motors.protocol = ESC_PROTOCOL_DSHOT600; s.pins.motor[4] = 39; s.pins.motor[5] = 40; });
      CHECK(has_line(*r, "motors: FAILED - DShot / ProShot drive at most 4 motors on the ESP32-S3 (4 RMT transmitters); this frame has 6") && !r->app().status.motors, "hex + on DShot600: refused, and the report says why"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.motors.layout = LAYOUT_HEX_H; s.motors.protocol = ESC_PROTOCOL_MULTISHOT; s.pins.motor[4] = 39; });
      CHECK(has_line(*r, "motors: FAILED - the pin of motor slot 6 is not set (Pins tab)"), "H6 with only 5 motor pins: the missing slot is named"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.motors.layout = LAYOUT_QUAD_X_1234; });
      CHECK(has_line(*r, "motors: host x4") && r->app().status.motors, "quad X 1234 (4 motors, DShot): runs"); }
}

static void t_crsf_telemetry() {
    printf("[crsf telemetry] battery, attitude, flight mode, GPS, vario back to the receiver; ten rounds a second; only with a TX pin, only on CRSF\n");
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.pins.curr_adc = 3; g_stub_adc_mv[3] = 480; (void)b; });     // 12 A
      Rig& g = *r; VirtualBoard& b = g.board;
      CHECK(has_line(g, "rx: CRSF on UART2, telemetry on its TX pin"), "boot report: CRSF with telemetry");
      b.plant.set_tilt(12.0f, 0.0f, 0.0f); g.run(5.0); b.rc_uart_sent().clear(); g.run(2.0);
      int bad = 0; const auto fr = crsf_out(b.rc_uart_sent(), bad);
      int n[256] = {}; for (const auto& f : fr) n[f.type]++;
      CHECK(bad == 0 && fr.size() >= 98 && fr.size() <= 102 && n[0x08] == 20 && n[0x1E] == 20 && n[0x21] == 20 && n[0x02] == 20 && n[0x07] == 20,
            "2 s: %zu good frames, none broken; each of battery / attitude / mode / GPS / vario 20 times (%d %d %d %d %d)", fr.size(), n[0x08], n[0x1E], n[0x21], n[0x02], n[0x07]);
      const app::Telemetry& t = g.tel();
      const CrsfOut* last[256] = {}; for (const auto& f : fr) last[f.type] = &f;
      { const auto& p = last[0x08]->p; const int dv = (p[0] << 8) | p[1], da = (p[2] << 8) | p[3]; const uint32_t mah = ((uint32_t)p[4] << 16) | (p[5] << 8) | p[6];
        CHECK(p.size() == 8 && abs(dv - (int)lroundf(t.batt_v * 10)) <= 1 && abs(dv - 160) <= 1 && abs(da - 120) <= 1 && abs((int)mah - (int)lroundf(t.batt_mah)) <= 1 && p[7] == t.batt_remaining,
              "battery: %d x 0.1 V, %d x 0.1 A, %u mAh, %u %%", dv, da, mah, p[7]); }
      { const auto& p = last[0x1E]->p; const float k = 10000.0f * 3.14159265f / 180.0f;
        CHECK(p.size() == 6 && abs(be16(p, 2) - (int)lroundf(t.roll * k)) <= 30 && abs(be16(p, 2) - 2094) <= 120 && abs(be16(p, 0) - (int)lroundf(t.pitch * k)) <= 30,
              "attitude: roll %d (12 deg = 2094 rad x 10000), pitch %d", be16(p, 2), be16(p, 0)); }
      { const auto& p = last[0x21]->p; const std::string m(p.begin(), p.end() - 1);
        CHECK(p.back() == 0 && m == (t.blockers ? "!ERR" : "ACRO*"), "flight mode while disarmed: '%s' (blockers 0x%x)", m.c_str(), t.blockers); }
      { const auto& p = last[0x02]->p;
        CHECK(p.size() == 15 && (int32_t)be32(p, 0) == t.lat && (int32_t)be32(p, 4) == t.lon && t.gps_fix >= 3 && p[14] == t.gps_sats && abs(((p[12] << 8) | p[13]) - 1000) <= 2,
              "GPS: %d, %d (deg x 1e7), %u satellites, altitude %d (m + 1000)", (int32_t)be32(p, 0), (int32_t)be32(p, 4), p[14], (p[12] << 8) | p[13]); }
      // armed in ANGLE, climbing: mode and vario follow
      b.plant.set_tilt(0.0f, 0.0f, 0.0f); for (int i = 0; i < 4; i++) b.rc[i] = i == 2 ? 1000 : 1500; b.rc[5] = 2000; g.run(3.0);
      b.rc[4] = 2000; g.run(0.5); b.rc[2] = 1800; g.run(1.5); b.rc_uart_sent().clear(); g.run(0.5);
      const auto f2 = crsf_out(b.rc_uart_sent(), bad); std::string mode; int vz = -99999;
      for (const auto& f : f2) { if (f.type == 0x21) mode.assign(f.p.begin(), f.p.end() - 1); if (f.type == 0x07) vz = be16(f.p, 0); }
      CHECK(g.tel().armed && mode == "ANGL", "armed in ANGLE: '%s'", mode.c_str());
      CHECK(vz > 50 && abs(vz - (int)lroundf(g.tel().vz * 100.0f)) <= 60, "climbing: vario %d cm/s (estimate %.2f m/s)", vz, g.tel().vz);
      b.rc[2] = 1000; b.rc[4] = 1000; g.run(1.0);
      b.tx_on = false; g.run(1.0); b.rc_uart_sent().clear(); g.run(1.0);
      CHECK(b.rc_uart_sent().empty(), "link lost: nothing sent (nobody to take it)"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.pins.rx_tx = 255; }); r->run(2.0);
      CHECK(has_line(*r, "rx: CRSF on UART2") && !has_line(*r, "telemetry on its TX pin") && r->board.rc_uart_sent().empty(), "no TX pin: no telemetry (%zu bytes)", r->board.rc_uart_sent().size()); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { b.has_baro = false; s.sensors.gps_enabled = 0; s.sensors.range_enabled = 0; }); r->run(3.0);
      VirtualBoard& b = r->board; b.rc_uart_sent().clear(); r->run(1.0); int bad = 0; const auto fr = crsf_out(b.rc_uart_sent(), bad); int n[256] = {}; for (const auto& f : fr) n[f.type]++;
      CHECK(!r->tel().alt_valid && n[0x07] == 0 && n[0x02] == 0 && n[0x08] == 10 && n[0x1E] == 10 && n[0x21] == 10, "no barometer, no GPS: no vario and no GPS frames, the other three ten times a second (%d %d)", n[0x07], n[0x08]); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.receiver.protocol = RX_PROTOCOL_SBUS; }); r->run(2.0);
      CHECK(has_line(*r, "rx: SBUS on UART2") && !has_line(*r, "telemetry on its TX pin") && r->board.rc_uart_sent().empty(), "SBUS (a TX pin is set): no CRSF frames on it"); }
}

static void t_receivers() {
    printf("[receivers] Spektrum satellite (2048 / 1024), PPM, PWM: channels, link loss, arming and flying on them; wrong settings\n");
    auto same = [](Rig& r, int n, int tol) { for (int i = 0; i < n; i++) if (abs((int)r.tel().rc[i] - (int)r.board.rc[i]) > tol) return false; return true; };
    auto fly = [](Rig& r, int thr_ch) {                      // sticks centred, ANGLE (aux 2), arm (aux 1), throttle up: the craft leaves the ground
        for (int i = 0; i < 4; i++) r.board.rc[i] = i == thr_ch ? 1000 : 1500;
        r.board.rc[5] = 2000; r.board.rc[6] = 1000; r.board.rc[7] = 1000; r.run(0.3);
        r.board.rc[4] = 2000; r.run(0.5); const bool armed = r.tel().armed;
        r.board.rc[thr_ch] = 1750; r.run(2.0); const bool up = r.board.plant.z > 0.5f;
        if (!(armed && up)) printf("    (armed %d, z %.2f, blockers 0x%x, link %d, failsafe %u)\n", (int)armed, r.board.plant.z, r.tel().blockers, (int)r.tel().rc_link, r.tel().failsafe);
        r.board.rc[thr_ch] = 1000; r.board.rc[4] = 1000; r.run(1.0); return armed && up; };
    // ---- Spektrum DSMX (2048), channel order TAER as Spektrum sends it
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_SPEKTRUM; s.receiver.ch_throttle = 0; s.receiver.ch_roll = 1; s.receiver.ch_pitch = 2; s.receiver.ch_yaw = 3;
                                                              b.rc_link = VirtualBoard::RC_SPEKTRUM2048; b.rc[0] = 1000; b.rc[2] = 1500; });
      Rig& g = *r; VirtualBoard& b = g.board;
      CHECK(has_line(g, "rx: SPEKTRUM2048 on UART2") && b.rc_uart_baud() == 115200 && Serial2.format == SERIAL_8N1 && !Serial2.inverted, "boot report: Spektrum 2048 on UART2, 115200 8N1");
      const uint16_t v[12] = {1000, 1250, 1500, 1750, 1000, 1100, 1200, 1300, 1400, 1600, 1700, 1900}; for (int i = 0; i < 12; i++) b.rc[i] = v[i];
      g.run(2.0);
      CHECK(g.tel().rc_link && same(g, 12, 1) && g.tel().rc[12] == 1500, "12 channels over two kinds of frame, each within 1 us (ch 10: %u, ch 13: %u)", g.tel().rc[9], g.tel().rc[12]);
      CHECK(g.tel().rc_rate >= 88 && g.tel().rc_rate <= 92, "a frame every 11 ms: %u Hz", g.tel().rc_rate);
      b.rc[4] = 1000; CHECK(fly(g, 0), "armed on aux 1 and flown with the throttle on channel 1 (TAER)");
      b.tx_on = false; g.run(1.0);
      CHECK(!g.tel().rc_link, "the satellite stops sending: link lost"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_SPEKTRUM1024; b.rc_link = VirtualBoard::RC_SPEKTRUM1024; });
      Rig& g = *r; VirtualBoard& b = g.board; const uint16_t v[7] = {1100, 1900, 1000, 1600, 1000, 1300, 2000}; for (int i = 0; i < 7; i++) b.rc[i] = v[i];
      g.run(2.0);
      CHECK(has_line(g, "rx: SPEKTRUM1024 on UART2") && g.tel().rc_link && same(g, 7, 1) && g.tel().rc_rate >= 44 && g.tel().rc_rate <= 46, "DSM2 1024 (22 ms): 7 channels within 1 us, %u Hz", g.tel().rc_rate); }

    // ---- PPM: 8 channels on one pin
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_PPM; s.pins.ppm = VirtualBoard::kPpmPin; s.receiver.rx_timeout_ms = 200; b.rc_link = VirtualBoard::RC_PPM; });
      Rig& g = *r; VirtualBoard& b = g.board;
      CHECK(has_line(g, "rx: PPM on GPIO40") && g_stub_isr[VirtualBoard::kPpmPin].fn && g_stub_isr[VirtualBoard::kPpmPin].mode == RISING && !Serial2.started, "boot report: PPM on GPIO40, an interrupt on its rising edges, UART2 not opened");
      const uint16_t v[8] = {1234, 1567, 1000, 1890, 1000, 1500, 2000, 1111}; for (int i = 0; i < 8; i++) b.rc[i] = v[i];
      g.run(2.5);
      CHECK(g.tel().rc_link && same(g, 8, 1) && g.tel().rc[8] == 1500 && g.tel().rc[15] == 1500, "8 channels within 1 us, channels 9..16 at 1500 (ch 1: %u, ch 9: %u)", g.tel().rc[0], g.tel().rc[8]);
      CHECK(g.tel().rc_rate >= 43 && g.tel().rc_rate <= 46 && g.app().rc.ppmChannels() == 8, "a frame every 22.5 ms: %u Hz, 8 channels learnt", g.tel().rc_rate);
      b.rc[4] = 1000; CHECK(fly(g, 2), "armed and flown on PPM");
      b.ppm_channels = 6; g.run(0.4);
      CHECK(!g.tel().rc_link, "frames with 6 channels where 8 were learnt: not taken, the link counts as lost (receiver timeout 200 ms)");
      g.run(1.0); CHECK(g.tel().rc_link && g.app().rc.ppmChannels() == 6, "after 25 such frames in a row the new count is learnt (%d)", g.app().rc.ppmChannels());
      b.tx_on = false; g.run(1.0); CHECK(!g.tel().rc_link, "no pulses: link lost"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_PPM; s.pins.ppm = VirtualBoard::kPpmPin; b.rc_link = VirtualBoard::RC_PPM; b.negative_ppm = true; b.ppm_channels = 12; b.ppm_frame_us = 27000; });
      Rig& g = *r; for (int i = 0; i < 12; i++) g.board.rc[i] = (uint16_t)(1050 + 75 * i); g.run(1.5);
      CHECK(g.tel().rc_link && same(g, 12, 1), "negative PPM with 12 channels (27 ms frames): read the same (ch 12: %u)", g.tel().rc[11]); }

    // ---- PWM: 6 channels, one pin each
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_PWM; for (int i = 0; i < 6; i++) s.pins.rx_pwm[i] = VirtualBoard::kPwmPin[i]; b.rc_link = VirtualBoard::RC_PWM; b.pwm_channels = 6; });
      Rig& g = *r; VirtualBoard& b = g.board;
      CHECK(has_line(g, "rx: PWM, 6 channels from GPIO39") && g_stub_isr[VirtualBoard::kPwmPin[5]].mode == CHANGE && !g_stub_isr[VirtualBoard::kPwmPin[6]].fn, "boot report: PWM, 6 channels; an interrupt on both edges of each wired pin");
      const uint16_t v[6] = {1300, 1700, 1000, 1450, 1000, 1999}; for (int i = 0; i < 6; i++) b.rc[i] = v[i];
      g.run(2.0);
      CHECK(g.tel().rc_link && same(g, 6, 0) && g.tel().rc[6] == 1500 && g.tel().rc_rate >= 49 && g.tel().rc_rate <= 51, "6 channels exactly, channel 7 at 1500, %u Hz", g.tel().rc_rate);
      b.rc[4] = 1000; CHECK(fly(g, 2), "armed and flown on PWM");
      b.pwm_dead = 1u << 3; g.run(1.0);
      CHECK(!g.tel().rc_link, "the wire of channel 4 comes loose: no frames any more, link lost (not flying on a stale yaw)");
      b.pwm_dead = 0; g.run(0.5); CHECK(g.tel().rc_link, "plugged back in: link back"); }

    // ---- wrong settings
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.receiver.protocol = RX_PROTOCOL_PPM; });
      CHECK(has_line(*r, "rx: FAILED - PPM pin not set"), "PPM without its pin: said so"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.receiver.protocol = RX_PROTOCOL_PWM; s.pins.rx_pwm[1] = 41; });
      CHECK(has_line(*r, "rx: FAILED - PWM receiver: the pin of channel 1 is not set"), "PWM without the pin of channel 1: said so"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.receiver.protocol = RX_PROTOCOL_PWM; s.pins.rx_pwm[0] = VirtualBoard::kMotorPin[0]; }); r->run(0.5);
      CHECK(has_line(*r, "pins: FAILED - GPIO4 is set for both rx_pwm 1 and motor 1") && (r->tel().blockers & ARM_BLOCKER_PIN_CONFLICT), "a PWM input on a motor pin: pin conflict, no arming"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.receiver.protocol = RX_PROTOCOL_PPM; s.pins.ppm = VirtualBoard::kPpmPin; b.rc_link = VirtualBoard::RC_PPM; s.pins.rx_rx = VirtualBoard::kMotorPin[1]; }); r->run(0.5);
      CHECK(!has_line(*r, "pins: FAILED"), "PPM: the serial receiver pins are not in use, so they cannot clash"); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { msp_flow(s, b); s.receiver.protocol = RX_PROTOCOL_PPM; s.pins.ppm = VirtualBoard::kPpmPin; b.rc_link = VirtualBoard::RC_PPM; b.wire_flow_uart(2); }, false);
      r->run(2.0);
      CHECK(has_line(*r, "flow: MSP flow on UART2") && (r->tel().health & HEALTH_FLOW) && r->tel().rc_link, "PPM leaves UART2 free: the MSP flow sensor gets it (app on UART0, GPS on UART1)"); }
}

static void t_current() {
    printf("[current] ADC current sensor, ESC current (extended DShot telemetry), consumption, what is left; wrong settings\n");
    const uint8_t pc = 3;                                   // the sensor's output on GPIO3 (an ADC pin)
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.pins.curr_adc = pc; s.battery.capacity_mah = 1000; g_stub_adc_mv[pc] = 400; });   // 400 mV at 40 mV/A = 10 A
      CHECK(has_line(*r, "current: ADC sensor on GPIO3, 400 x 0.1 mV/A"), "boot report: the ADC current sensor");
      r->stream(STR_BATTERY, 10); r->run(2.0); T_battery tb{};
      CHECK(r->latest(STR_BATTERY, tb) && abs((int)tb.current - 1000) <= 2 && tb.current_source == CURR_SOURCE_ADC && tb.current_mv == 400, "battery stream: %d x 10 mA, source %u, %u mV at the pin", tb.current, tb.current_source, tb.current_mv);
      const float m0 = r->tel().batt_mah; r->run(3.6); const float m1 = r->tel().batt_mah;
      CHECK(fabsf(m1 - m0 - 10.0f) < 0.3f, "10 A for 3.6 s = 10 mAh (%.2f)", m1 - m0);
      r->latest(STR_BATTERY, tb);
      CHECK(tb.mah == (uint16_t)lroundf(m1) && abs((int)tb.remaining - (int)lroundf(100.0f - m1 / 10.0f)) <= 1 && tb.remaining < 100, "%u mAh of 1000 used: %u %% left", tb.mah, tb.remaining);
      S_battery sb; r->get_section(SEC_BATTERY, &sb, sizeof(sb)); sb.current_scale = 200; CHECK(r->set_section(SEC_BATTERY, &sb, sizeof(sb)).type == MSG_SECTION, "scale changed in the app (calibration)");
      r->run(2.0); r->latest(STR_BATTERY, tb);
      CHECK(abs((int)tb.current - 2000) <= 4 && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "scale 200 (20 mV/A): 20 A at once, no restart (%d)", tb.current);
      sb.current_offset = -1500; r->set_section(SEC_BATTERY, &sb, sizeof(sb)); r->run(2.0); r->latest(STR_BATTERY, tb);
      CHECK(abs((int)tb.current - 1850) <= 4, "offset -1500 mA: 18.5 A (%d)", tb.current); }

    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { s.motors.bidirectional = 1; s.motors.edt = 1; const float a[4] = {2, 3, 4, 5}; for (int i = 0; i < 4; i++) b.esc_amps[i] = a[i]; });
      VirtualBoard& b = r->board; r->stream(STR_BATTERY, 10); r->run(2.0); T_battery tb{};
      CHECK(has_line(*r, "current: from the ESCs (extended DShot telemetry)") && b.esc_edt_on == 0x0F, "no sensor pin, extended telemetry on: the ESCs are asked to send it (slots 0x%x)", b.esc_edt_on);
      CHECK(r->latest(STR_BATTERY, tb) && abs((int)tb.current - 1400) <= 2 && tb.current_source == CURR_SOURCE_ESC, "the sum of what the four ESCs report: %d x 10 mA", tb.current);
      CHECK(abs((int)tb.remaining - 70) <= 2, "no capacity set: what is left from the voltage, 16.0 V on 4 cells = 70 %% (%u)", tb.remaining);
      { float lo = 1e9f, hi = 0; for (int i = 0; i < 20; i++) { r->command(CMD_MOTOR_TEST, {0, 200, 0}); r->run(0.05); if (i >= 10) { const float v = r->tel().rpm[0]; if (v < lo) lo = v; if (v > hi) hi = v; } }
        r->command(CMD_MOTOR_TEST, {255, 0, 0}); r->run(0.5); r->latest(STR_BATTERY, tb);
        CHECK(lo > 1000.0f && hi < lo * 1.05f, "a motor spinning on the bench: its speed stays steady, the current frames in between are not taken for speeds (%.0f..%.0f rpm)", lo, hi);
        CHECK(abs((int)tb.current - 1400) <= 2, "and the current still comes in (%d)", tb.current); }
      const T_battery before = tb;
      b.esc_replies = false; r->run(1.5); r->latest(STR_BATTERY, tb); const uint16_t mah_silent = tb.mah;
      CHECK(tb.current == 0 && tb.mah >= before.mah && tb.mah <= before.mah + 5, "the ESCs went silent (battery unplugged): 0 A after a second (their last reading counts for that second: %u -> %u mAh)", before.mah, tb.mah);
      r->run(1.5); r->latest(STR_BATTERY, tb);
      CHECK(tb.current == 0 && tb.mah == mah_silent, "and nothing is added up while they stay silent: the consumption is kept");
      b.esc_edt_on = 0; b.esc_replies = true; r->run(2.0); r->latest(STR_BATTERY, tb);
      CHECK(b.esc_edt_on == 0x0F && abs((int)tb.current - 1400) <= 2, "they come back having forgotten it (a new battery): asked again, current back (%d)", tb.current);
    }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { s.motors.bidirectional = 1; s.motors.edt = 1; for (int i = 0; i < 4; i++) b.esc_amps[i] = 3; });
      VirtualBoard& b = r->board; r->run(2.0);
      // armed: a DShot command stops the motors for its repeats, so nothing is asked in flight, only after disarming
      b.rc[4] = 2000; r->run(0.5); CHECK(r->tel().armed, "armed (blockers 0x%x)", r->tel().blockers);
      b.esc_replies = false; r->run(0.3); b.esc_edt_on = 0; b.esc_replies = true; const uint32_t t_back = millis(); r->run(1.0);
      bool cmd_armed = false; for (const auto& c : b.esc_cmds) if (c.ms >= t_back && c.cmd == 13) cmd_armed = true;
      CHECK(r->tel().armed && !cmd_armed && b.esc_edt_on == 0, "the ESCs restart while armed (a brown-out): not asked while armed");
      b.rc[4] = 1000; r->run(1.0);
      CHECK(!r->tel().armed && b.esc_edt_on == 0x0F, "disarmed: asked at once (slots 0x%x)", b.esc_edt_on); }

    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.battery.current_source = CURR_SOURCE_ADC; });
      CHECK(has_line(*r, "current: FAILED - current sensor ADC pin not set"), "ADC chosen, no pin: reported"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.battery.current_source = CURR_SOURCE_ESC; });
      CHECK(has_line(*r, "current: FAILED - the ESCs report it only over their telemetry wire, or with bidirectional DShot and extended telemetry on"), "ESC chosen, extended telemetry off: reported"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.pins.curr_adc = 41; });
      CHECK(has_line(*r, "current: FAILED - current sensor pin is not an ADC pin"), "GPIO41 is no ADC pin: reported"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.pins.curr_adc = VirtualBoard::kBattAdc; }); r->run(0.5);
      CHECK(has_line(*r, "pins: FAILED - GPIO2 is set for both batt_adc and curr_adc") && (r->tel().blockers & ARM_BLOCKER_PIN_CONFLICT), "the current sensor on the voltage pin: pin conflict, no arming"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.battery.capacity_mah = 1000; }); r->stream(STR_BATTERY, 10); r->run(2.0); T_battery tb{};
      CHECK(has_line(*r, "current: none (no sensor pin, no ESC telemetry)"), "nothing set up: no current, and no fault either");
      CHECK(r->latest(STR_BATTERY, tb) && abs((int)tb.remaining - 70) <= 2, "a capacity set but no current to count it down with: what is left from the voltage (%u %%), not a full battery for ever", tb.remaining); }
}

static void t_servo() {
    printf("[servo] two servos on RC channels 10 and 11\n");
    const uint8_t pa = 1, pb = 3;
    auto pulse = [](uint8_t pin) { const StubPwm& p = g_stub_pwm[pin]; return p.freq ? (double)p.duty * 1e6 / (double)p.freq / 16384.0 : 0.0; };   // duty -> us
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.pins.servo[0] = pa; s.pins.servo[1] = pb; s.servo.source[0] = SERVO_SOURCE_CH10; s.servo.source[1] = SERVO_SOURCE_CH11;
                                                            s.servo.min_us[1] = 800; s.servo.max_us[1] = 2200; s.servo.reverse[1] = 1; });
      VirtualBoard& b = r->board;
      CHECK(has_line(*r, "servo: 2 outputs at 50 Hz") && g_stub_pwm[pa].attached && g_stub_pwm[pa].freq == 50 && g_stub_pwm[pa].bits == 14 && g_stub_pwm[pb].attached, "boot report: 2 servo outputs, LEDC at 50 Hz, 14 bit");
      r->stream(STR_MOTORS, 20); b.rc[9] = 1500; b.rc[10] = 1250; r->run(1.0); T_motors m{};
      CHECK(fabs(pulse(pa) - 1500) < 2 && fabs(pulse(pb) - (2200 - 0.25 * 1400)) < 2, "channel 10 at 1500 us: servo 1 at %.0f us; channel 11 at 1250 on 800..2200 reversed: %.0f us", pulse(pa), pulse(pb));
      CHECK(r->latest(STR_MOTORS, m) && fabs(m.servo[0] - pulse(pa)) <= 1 && fabs(m.servo[1] - pulse(pb)) <= 1 && m.servo[2] == 0, "the motors stream shows the pulses (%u, %u, %u)", m.servo[0], m.servo[1], m.servo[2]);
      b.rc[9] = 2100; r->run(0.2); const double top = pulse(pa); b.rc[9] = 900; r->run(0.2);
      CHECK(fabs(top - 2000) < 2 && fabs(pulse(pa) - 1000) < 2, "beyond the channel's ends: held at min_us / max_us (%.0f, %.0f)", top, pulse(pa));
      // live: the range changed in the app, no restart
      S_servo sv; r->get_section(SEC_SERVO, &sv, sizeof(sv)); sv.min_us[0] = 1200; sv.max_us[0] = 1800; sv.reverse[0] = 1; CHECK(r->set_section(SEC_SERVO, &sv, sizeof(sv)).type == MSG_SECTION, "servo section set");
      b.rc[9] = 1000; r->run(0.2);
      CHECK(fabs(pulse(pa) - 1800) < 2 && !(r->fw->server.flags() & SF_REBOOT_REQUIRED), "range 1200..1800 reversed, channel at 1000: %.0f us at once, no restart", pulse(pa));
      // link lost: the servos hold their place
      b.rc[9] = 1500; r->run(0.2); const double before = pulse(pa); b.tx_on = false; r->run(1.5);
      CHECK(fabs(pulse(pa) - before) < 1 && fabs(before - 1500) < 2, "receiver link lost: the servo stays where it was (%.0f us)", pulse(pa));
      sv.source[0] = SERVO_SOURCE_OFF; r->set_section(SEC_SERVO, &sv, sizeof(sv)); r->run(0.1);
      CHECK(g_stub_pwm[pa].duty == 0 && r->latest(STR_MOTORS, m) && m.servo[0] == 0, "servo switched OFF: no pulses any more"); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { b.tx_on = false; s.pins.servo[0] = pa; s.servo.source[0] = SERVO_SOURCE_CH10; s.servo.rate_hz = 200; s.servo.min_us[0] = 900; s.servo.max_us[0] = 2100; });
      r->run(0.3);
      CHECK(g_stub_pwm[pa].freq == 200 && fabs(pulse(pa) - 1500) < 2, "before the first RC frame: the middle of 900..2100 (%.0f us), at 200 Hz for a digital servo", pulse(pa)); }
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { s.pins.servo[0] = VirtualBoard::kMotorPin[1]; s.servo.source[0] = SERVO_SOURCE_CH10; }); r->run(0.5);
      CHECK(has_line(*r, "pins: FAILED - GPIO5 is set for both motor 2 and servo 1") && (r->tel().blockers & ARM_BLOCKER_PIN_CONFLICT), "a servo on a motor's pin: pin conflict, no arming"); }
    const uint8_t four[4] = {1, 3, 41, 42};
    { auto r = nav_rig([&](wbp::Settings& s, VirtualBoard&) { for (int i = 0; i < 4; i++) s.pins.servo[i] = four[i]; s.servo.source[3] = SERVO_SOURCE_CH12; });
      CHECK(has_line(*r, "servo: 4 outputs at 50 Hz") && g_stub_pwm[42].attached, "four servos (one of them following channel 12, three switched off but wired)"); }
    { g_stub_us = 0; g_stub_delay_advances = true;          // (the 8 PWM channels are shared: here something else took 7 of them)
      std::unique_ptr<Rig> r(new Rig()); r->provision();
      { wbp::Settings s; wbp::SettingsStore st(r->flash); st.load(s); for (int i = 0; i < 4; i++) s.pins.servo[i] = four[i]; s.servo.source[0] = SERVO_SOURCE_CH10; st.save(s); }
      g_stub_pwm_free = 1; r->boot();
      CHECK(has_line(*r, "servo: FAILED - no free PWM (LEDC) channel for a servo") && g_stub_pwm[1].attached && !g_stub_pwm[3].attached, "only one PWM channel left: the first servo gets it, the report says the others found none"); }
}

// 0.8.5: what the receiver's UART and decoder lose, per second (rc stream); the UART set up with room to spare; the external pull-up setting
static void t_rx_diag() {
    printf("[receiver diagnostics] bad frames, UART overflows and line errors per second; the motor pull-up setting\n");
    { auto r = nav_rig([](wbp::Settings&, VirtualBoard&) {}); Rig& g = *r; VirtualBoard& b = g.board; g.run(3.0);
      CHECK(Serial2.rx_buffer == 2048 && Serial2.rx_fifo_full == 64, "the receiver's UART: 2 kB receive buffer, its FIFO handed over every 64 bytes (%u, %u)", (unsigned)Serial2.rx_buffer, (unsigned)Serial2.rx_fifo_full);
      { const uint32_t s0 = Serial2.single_reads, b0 = Serial2.block_reads, f0 = b.rc_frames, g0 = Serial1.single_reads, gb0 = Serial1.block_reads; g.run(2.0); const uint32_t frames = b.rc_frames - f0;
        CHECK(Serial2.single_reads == s0 && Serial2.block_reads - b0 <= frames + 4 && frames > 250, "0.8.7: the receiver is read in blocks - %u calls for %u frames of 26 bytes, never one byte at a time (on the board that fell behind the receiver)",
              (unsigned)(Serial2.block_reads - b0), (unsigned)frames);
        CHECK(Serial1.single_reads == g0 && Serial1.block_reads > gb0, "... and so is the GPS"); }
      const uint16_t rate0 = g.tel().rc_rate;
      CHECK(rate0 >= 130 && g.tel().rc_bad == 0 && g.tel().rc_ovf == 0 && g.tel().rc_line == 0, "a clean link: %u frames/s, nothing lost", rate0);
      b.rc_corrupt = 30; Serial2.stub_error(UART_FIFO_OVF_ERROR); Serial2.stub_error(UART_BUFFER_FULL_ERROR); Serial2.stub_error(UART_FRAME_ERROR);
      int bad = 0, ovf = 0, line = 0; uint16_t low = 999;
      for (int i = 0; i < 30; i++) { g.run(0.1); if (i % 10 == 9) { bad += g.tel().rc_bad; ovf += g.tel().rc_ovf; line += g.tel().rc_line; low = std::min(low, g.tel().rc_rate); } }
      CHECK(bad == 30 && ovf == 2 && line == 1 && low <= rate0 - 25, "30 damaged frames, two overflows, a framing error: counted %d / %d / %d, the rate dips to %u", bad, ovf, line, low);
      T_rc rc{}; r->clear_inbox(); r->stream(STR_RC, 20); r->run(0.2);
      CHECK(r->latest(STR_RC, rc) && rc.bad_frames == g.tel().rc_bad && rc.uart_ovf == g.tel().rc_ovf, "the rc stream carries them"); }
    for (int ext = 0; ext < 2; ext++) {
      auto r = nav_rig([ext](wbp::Settings& s, VirtualBoard&) { s.motors.protocol = ESC_PROTOCOL_DSHOT600; s.motors.bidirectional = 1; s.motors.ext_pullup = (uint8_t)ext; });
      HostMotorHw* hw = static_cast<HostMotorHw*>(r->app().motors.hw());
      CHECK(hw && hw->internal_pullup == !ext, "motors.ext_pullup = %d: the internal pull-up of the motor pins is %s", ext, ext ? "left off" : "used"); }
}

static uint8_t est_bits(Rig& r) { T_estimate e{}; r.clear_inbox(); r.stream(STR_ESTIMATE, 50); r.run(0.1); const bool ok = r.latest(STR_ESTIMATE, e); r.stream(STR_ESTIMATE, 0); return ok ? e.valid : 0; }
static void t_gps_alt() {
    printf("[gps height] the GPS height and vertical velocity in the height estimate (nav.gps_alt, nav.gps_vel_z)\n");
    { auto r = nav_rig([](wbp::Settings&, VirtualBoard& b) { b.has_baro = false; }, false); VirtualBoard& b = r->board; r->run(6.0);
      b.rc[5] = 2000; b.rc[6] = 2000; r->run(0.3);
      CHECK((r->tel().health & HEALTH_GPS) && !(r->tel().health & HEALTH_BARO) && r->tel().blockers == 0, "no barometer, a GPS fix, the Altitude Hold switch on: arming is allowed - the GPS height stands in (blockers 0x%x)", r->tel().blockers);
      nav_takeoff(*r, 0.0); b.rc[2] = 1700; for (int i = 0; i < 80 && b.plant.z < 3.0f; i++) r->run(0.1); b.rc[2] = 1500; r->run(3.0);
      const uint8_t v = est_bits(*r); const float z0 = b.plant.z; float dev = 0; bool on = true;
      for (int i = 0; i < 100; i++) { r->run(0.1); dev = fmaxf(dev, fabsf(b.plant.z - z0)); on &= r->tel().alt_valid; }
      CHECK(z0 > 2.0f && (v & EST_VALID_ALT) && (v & EST_VALID_GPS_ALT) && (v & EST_VALID_GPS_VZ) && on && dev < 1.5f && r->tel().ah_state == AH_STATE_FLYING,
            "takes off and climbs to %.1f m in Altitude Hold, stick centred: holds within %.2f m for 10 s on the GPS height and vertical velocity (valid bits 0x%x)", z0, dev, v); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { b.has_baro = false; s.nav.gps_alt = GPS_ALT_OFF; }, false); VirtualBoard& b = r->board; r->run(6.0);
      b.rc[5] = 2000; b.rc[6] = 2000; r->run(0.3);
      CHECK(r->tel().blockers & ARM_BLOCKER_ALTHOLD_NO_BARO, "the same with nav.gps_alt = OFF: ALTHOLD_NO_BARO, as before"); }
    for (int mode = GPS_ALT_FALLBACK; mode <= GPS_ALT_BLEND; mode++) {
      auto r = nav_rig([mode](wbp::Settings& s, VirtualBoard&) { s.nav.gps_alt = (uint8_t)mode; }, false); VirtualBoard& b = r->board; r->run(6.0);
      nav_takeoff(*r, 2.0); r->run(5.0); const uint8_t v = est_bits(*r);
      if (mode == GPS_ALT_FALLBACK) CHECK((v & EST_VALID_ALT) && (v & EST_VALID_GPS_VZ) && !(v & EST_VALID_GPS_ALT) && b.plant.z > 0.5f, "with a barometer (FALLBACK, the default): the GPS vertical velocity is used, its height is not (valid 0x%x, %.1f m up)", v, b.plant.z);
      else CHECK((v & EST_VALID_ALT) && (v & EST_VALID_GPS_VZ) && (v & EST_VALID_GPS_ALT), "BLEND: the GPS height corrects the barometer as well (valid 0x%x)", v); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.nav.gps_vel_z = 0; }, false); r->run(6.0); nav_takeoff(*r, 2.0); r->run(3.0);
      CHECK(!(est_bits(*r) & EST_VALID_GPS_VZ), "nav.gps_vel_z off: the GPS vertical velocity is left out"); }
}

// ESC passthrough over the app link: MSP next to WBP, the flight side lets go of the motors (app/link_host.h MspLink)
static void t_esc_passthrough() {
    printf("[esc passthrough] MSP on the app link, SET_PASSTHROUGH stops the motor output\n");
    auto v1 = [](uint8_t cmd) { std::vector<uint8_t> f = {'$', 'M', '<', 0, cmd, cmd}; return f; };
    auto r = nav_rig([](wbp::Settings& s, VirtualBoard&) { s.motors.protocol = ESC_PROTOCOL_DSHOT600; }); r->run(3.0);
    MemTransport mt; app::MspLink ml(r->fw->host, mt);
    for (uint8_t c : v1(msp::FC_VARIANT)) ml.on_byte(c);
    CHECK(mt.out.size() == 10 && memcmp(mt.out.data() + 5, "BTFL", 4) == 0, "an ESC tool asks who we are: it gets an answer it knows (%zu bytes)", mt.out.size());
    mt.out.clear(); for (uint8_t c : v1(msp::MOTOR_CONFIG)) ml.on_byte(c);
    CHECK(mt.out.size() > 11 && mt.out[11] == 4, "four motors");
    {   // BLHeliSuite32 32.10 (0.9.3 on a board): 1, 1, 2, 3, 4, 5, 90, 36, 119 - and gave up on the unanswered 90 / 119
        const uint8_t seq[] = {msp::API_VERSION, msp::FC_VARIANT, msp::FC_VERSION, msp::BOARD_INFO, msp::BUILD_INFO, msp::ADVANCED_CONFIG, msp::FEATURE_CONFIG, msp::BOXIDS};
        int bad = 0; uint8_t proto = 0;
        for (uint8_t cmd : seq) { mt.out.clear(); for (uint8_t c : v1(cmd)) ml.on_byte(c); if (mt.out.size() < 6 || mt.out[2] != '>') bad++; if (cmd == msp::ADVANCED_CONFIG && mt.out.size() > 8) proto = mt.out[8]; }
        CHECK(bad == 0 && proto == 7, "BLHeliSuite32's requests before the passthrough: all answered (%d refused), motor protocol from the settings: Betaflight DShot600 (%u)", bad, proto); }
    {   // the answers come from the firmware itself (nav_rig: switches for ARM, ANGLE, ALTHOLD, POSHOLD; a baro, a GPS)
        mt.out.clear(); for (uint8_t c : v1(msp::BOXIDS)) ml.on_byte(c);
        const bool boxes = mt.out.size() == 10 && mt.out[3] == 4 && mt.out[5] == 0 && mt.out[6] == 1 && mt.out[7] == 3 && mt.out[8] == 11;
        mt.out.clear(); for (uint8_t c : v1(msp::STATUS_EX)) ml.on_byte(c);
        const uint16_t sens = mt.out.size() > 10 ? (uint16_t)(mt.out[9] | mt.out[10] << 8) : 0;
        CHECK(boxes && (sens & 1) && (sens & 32) && (sens & 2), "BOXIDS: ARM, ANGLE, ALTHOLD, POS HOLD - the switches in the settings; STATUS_EX sensors 0x%x (accelerometer, gyro, baro)", sens); }
    HostMotorHw* hw = static_cast<HostMotorHw*>(r->app().motors.hw());
    // armed: refused
    VirtualBoard& b = r->board; b.rc[4] = 2000; r->run(0.5);
    const bool armed = r->tel().armed; uint8_t got = 0; mt.out.clear(); for (uint8_t c : v1(msp::SET_PASSTHROUGH)) got |= ml.on_byte(c);
    CHECK(armed && got == 0 && mt.out.size() == 7 && mt.out[5] == 0 && !r->app().motors.released(), "ARMED: SET_PASSTHROUGH is answered with 0 ESCs, the motors keep running");
    b.rc[4] = 1000; r->run(0.5);
    const uint32_t f0 = hw->frames; mt.out.clear(); got = 0;
    for (uint8_t c : v1(msp::SET_PASSTHROUGH)) got |= ml.on_byte(c);
    const uint32_t f1 = hw->frames; r->run(0.5);
    CHECK(got == 4 && mt.out.size() == 7 && mt.out[5] == 4 && r->app().motors.released() && hw->frames == f1 && f1 >= f0, "disarmed: 4 ESCs, and the motor output stops for good (%u frames before, none after)", (unsigned)(f1 - f0));
    CHECK(r->app().escPin(0) != GPIO_UNUSED && r->app().escPin(3) != GPIO_UNUSED && r->app().escPin(4) == GPIO_UNUSED, "the pins of the four motors are known to the link side (GPIO%u .. GPIO%u)", r->app().escPin(0), r->app().escPin(3));
    b.rc[4] = 2000; r->run(0.5); CHECK(hw->frames == f1, "arming now changes nothing: no output until the restart");
    mt.out.clear(); got = 0; b.rc[4] = 1000; r->run(0.3); for (uint8_t c : v1(msp::SET_PASSTHROUGH)) got |= ml.on_byte(c);
    CHECK(got == 0 && mt.out.size() == 7 && mt.out[5] == 0, "a second request (still in passthrough): 0");
    // WBP keeps working next to MSP bytes
    { r->clear_inbox(); for (uint8_t c : v1(msp::API_VERSION)) r->fw->server.on_byte(c); T_status st{}; r->stream(STR_STATUS, 10); r->run(0.3);
      CHECK(r->latest(STR_STATUS, st), "the app's own protocol is not disturbed by MSP bytes on the same link"); }
}

// The EKF (core/ekf.h) in the real firmware on the virtual board: SHADOW (logged next to the usual estimate) and NAV (Position Hold uses it)
static void t_ekf() {
    printf("[ekf] SHADOW: next to the usual estimators on the virtual board; NAV: Position Hold on it\n");
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); b.plant.set_tilt(0, 0, 60); }, false); VirtualBoard& b = r->board; T_estimate e{};
      r->run(6.0);
      const app::Telemetry& t = r->tel();
      CHECK((t.ekf_flags & EKF_FLAGS_ALIGNED) && (t.ekf_flags & EKF_FLAGS_HEADING_OK) && fabsf(fc::wrap180(t.ekf_yaw - 60.0f)) < 4.0f && !(t.ekf_flags & EKF_FLAGS_USED_FOR_NAV),
            "on the ground: aligned, heading %.1f deg (true 60, sigma %.1f), not used for navigation (SHADOW is the default)", t.ekf_yaw, t.ekf_yaw_sigma);
      b.rc[7] = 2000; nav_takeoff(*r, 4.0);
      float worst_yaw = 0, worst_pos = 0, worst_tilt = 0;
      for (int leg = 0; leg < 4; leg++) {                     // forwards, stop, right, stop: accelerations to see the heading by
          b.rc[leg % 2 ? 0 : 1] = 1750; r->run(2.5); b.rc[0] = b.rc[1] = 1500;
          for (int i = 0; i < 40; i++) { r->run(0.1); const app::Telemetry& u = r->tel();
              worst_yaw = fmaxf(worst_yaw, fabsf(fc::wrap180(u.ekf_yaw - b.plant.yaw_deg()))); worst_pos = fmaxf(worst_pos, hypotf(u.ekf_pn - b.plant.pn, u.ekf_pe - b.plant.pe));
              worst_tilt = fmaxf(worst_tilt, fmaxf(fabsf(u.ekf_roll - b.plant.right_down_deg()), fabsf(u.ekf_pitch - b.plant.nose_down_deg()))); } }
      CHECK(r->tel().armed && (r->tel().ekf_flags & EKF_FLAGS_GPS_AIDING) && worst_yaw < 4.0f && worst_pos < 1.5f && worst_tilt < 3.0f,
            "flying legs with Position Hold: the EKF's heading within %.1f deg of the truth, position within %.2f m, tilt within %.1f deg", worst_yaw, worst_pos, worst_tilt);
      r->clear_inbox(); r->stream(STR_ESTIMATE, 20); r->run(1.2);
      CHECK(r->latest(STR_ESTIMATE, e) && (e.ekf_flags & EKF_FLAGS_GPS_AIDING) && fabsf(e.ekf_pn * 0.1f - b.plant.pn) < 1.5f, "estimate stream: EKF position %.1f / %.1f m, flags 0x%x (the step time, ekf_us, is only real on the board: the PC clock stands still inside a step)", e.ekf_pn * 0.1, e.ekf_pe * 0.1, e.ekf_flags); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); s.nav.ekf = EKF_MODE_NAV; b.plant.set_tilt(0, 0, 60); }, false); VirtualBoard& b = r->board;
      r->run(6.0); b.rc[7] = 2000; nav_takeoff(*r, 4.0);
      const float n0 = b.plant.pn, e0 = b.plant.pe; float far = 0; for (int i = 0; i < 100; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
      CHECK((r->tel().ekf_flags & EKF_FLAGS_USED_FOR_NAV) && far < 0.8f, "nav.ekf = NAV: Position Hold flies on the EKF's position and heading - within %.2f m for 10 s", far);
      b.rc[1] = 1750; r->run(2.5); const float vf = b.plant.v_forward(), vr = b.plant.v_right(); b.rc[1] = 1500; r->run(8.0);
      CHECK(vf > 1.5f && fabsf(vr) < 0.5f && hypotf(b.plant.vn, b.plant.ve) < 0.3f, "stick forwards: %.1f m/s along the nose (%.1f sideways); let go: stops", vf, vr); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { s.sensors.mag_enabled = 0; s.nav.ekf = EKF_MODE_NAV; b.plant.set_tilt(0, 0, 60); }, false); VirtualBoard& b = r->board;
      r->run(6.0); nav_takeoff(*r, 4.0);                      // no compass: the usual estimators never know the heading, so Position Hold is refused
      T_nav n{}; b.rc[7] = 2000; r->run(0.5);
      const bool refused = nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && !(r->tel().ekf_flags & EKF_FLAGS_HEADING_OK);
      b.rc[7] = 1000;
      for (int leg = 0; leg < 4; leg++) { b.rc[leg % 2 ? 0 : 1] = 1750; r->run(2.5); b.rc[0] = b.rc[1] = 1500; r->run(3.0); }   // Altitude Hold, a few accelerations
      const float yaw_err = fabsf(fc::wrap180(r->tel().ekf_yaw - b.plant.yaw_deg()));
      CHECK(refused && (r->tel().ekf_flags & EKF_FLAGS_HEADING_OK) && yaw_err < 6.0f,
            "no compass, nav.ekf = NAV: Position Hold refused before any acceleration; after a few, the EKF has the heading from the GPS velocity (%.1f deg off)", yaw_err);
      b.rc[7] = 2000; r->run(1.0); const float n0 = b.plant.pn, e0 = b.plant.pe; float far = 0;
      for (int i = 0; i < 80; i++) { r->run(0.1); far = fmaxf(far, nav_far(*r, n0, e0)); }
      CHECK(nav_now(*r, n) && (n.flags & NAV_FLAGS_POSHOLD) && !(n.flags & NAV_FLAGS_POSHOLD_UNAVAILABLE) && (r->tel().ekf_flags & EKF_FLAGS_USED_FOR_NAV) && far < 1.5f,
            "... and Position Hold then flies on it: within %.2f m for 8 s (without the EKF it stays refused)", far); }
    { auto r = nav_rig([](wbp::Settings& s, VirtualBoard& b) { gps_compass(s, b); s.nav.ekf = EKF_MODE_OFF; }, false); r->run(6.0);
      CHECK(r->tel().ekf_flags == 0 && r->tel().ekf_us == 0, "nav.ekf = OFF: not run at all"); }
}

// ESC telemetry on its own wire (hal/motor/esc_sensor.h): KISS frames on a UART RX, one ESC asked at a time
static void t_esc_wire() {
    printf("[esc telemetry wire] KISS answers on a UART, asked with the DShot telemetry bit, one ESC at a time\n");
    const uint8_t pin = 44;
    auto wire = [&](wbp::Settings& s, VirtualBoard& b) { s.motors.esc_sensor = 1; s.pins.esc_tlm = pin; Serial0.dev = &b.esc_wire; const float a[4] = {2, 3, 4, 5}; for (int i = 0; i < 4; i++) b.esc_amps[i] = a[i]; };
    {   auto r = nav_rig(wire); VirtualBoard& b = r->board;          // (DShot600, GPS on UART1, receiver on UART2, the app on USB: UART0 is left)
        CHECK(has_line(*r, "esc tlm: KISS telemetry on UART0 RX GPIO44, 115200 baud") && b.esc_wire.baud == 115200, "boot report: the wire on the UART that is left (UART0), 115200 baud");
        CHECK(has_line(*r, "current: from the ESCs (telemetry wire)"), "the current comes from the ESCs");
        T_motors tm{}; T_battery tb{};
        r->stream(STR_MOTORS, 10); r->stream(STR_BATTERY, 10); r->run(3.0);
        CHECK(r->latest(STR_MOTORS, tm) && tm.tlm_frames == 0 && tm.tlm_timeouts == 0, "the first 5 s nobody is asked (the ESCs are still starting)");
        r->run(4.5);
        CHECK(r->latest(STR_MOTORS, tm) && tm.tlm_frames > 400 && tm.tlm_crc == 0 && tm.tlm_timeouts == 0, "one answer per millisecond tick: %u good answers in the last second, %u bad, %u missing", tm.tlm_frames, tm.tlm_crc, tm.tlm_timeouts);
        CHECK(tm.temp[0] == 35 && tm.temp[3] == 38 && tm.amps[0] == 200 && tm.amps[3] == 500, "each ESC's own temperature and current, in motor order (%u / %u degC, %u / %u x 10 mA)", tm.temp[0], tm.temp[3], tm.amps[0], tm.amps[3]);
        CHECK(r->latest(STR_BATTERY, tb) && abs((int)tb.current - 1400) <= 2 && tb.current_source == CURR_SOURCE_ESC, "battery: the sum of the four, %d x 10 mA", tb.current);
        CHECK((r->tel().health & HEALTH_ESC_TELEMETRY) != 0, "health: ESC telemetry");
        // a motor spinning: its speed from the wire (no bidirectional DShot here)
        float rpm = 0, truth = 0; for (int i = 0; i < 20; i++) { r->command(CMD_MOTOR_TEST, {0, 200, 0}); r->run(0.05); rpm = r->tel().rpm[0]; truth = b.motor_rpm(0); }
        r->command(CMD_MOTOR_TEST, {255, 0, 0}); r->run(0.3);
        CHECK(rpm > 1000.0f && fabsf(rpm - truth) < truth * 0.05f + 100.0f, "motor test: its rpm from the wire (%.0f, the motor %.0f)", rpm, truth);
        b.esc_silent[2] = true; r->run(1.5); r->latest(STR_MOTORS, tm);
        CHECK(tm.tlm_timeouts >= 8 && tm.tlm_timeouts <= 11 && tm.tlm_frames > 20, "one ESC does not answer: 100 ms lost on it each round, about 10 a second (%u timeouts, %u answers)", tm.tlm_timeouts, tm.tlm_frames);
        b.esc_silent[2] = false; b.esc_wire_corrupt = 3; r->run(1.1); r->latest(STR_MOTORS, tm);
        CHECK(tm.tlm_crc == 3, "three damaged answers: dropped and counted (%u)", tm.tlm_crc); }
    {   auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { wire(s, b); s.motors.protocol = ESC_PROTOCOL_ONESHOT125; });
        CHECK(has_line(*r, "esc tlm: FAILED - the ESC telemetry wire needs a DShot protocol"), "Oneshot: no DShot frame to carry the request: refused"); }
    {   auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { wire(s, b); s.pins.esc_tlm = 255; });
        CHECK(has_line(*r, "esc tlm: FAILED - no pin for the ESC telemetry wire"), "no pin set: says so"); }
    {   auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { wire(s, b); s.ports.assign = PORT_ASSIGN_TABLE; s.ports.function[0] = PORT_FUNCTION_NONE; s.ports.function[1] = PORT_FUNCTION_GPS; s.ports.function[2] = PORT_FUNCTION_SERIAL_RX; });
        CHECK(has_line(*r, "esc tlm: FAILED - no UART has the function ESC_SENSOR"), "port table without ESC_SENSOR: says so"); }
    {   auto r = nav_rig([&](wbp::Settings& s, VirtualBoard& b) { wire(s, b); s.pins.buzzer = pin; });
        CHECK(has_line(*r, "pins: FAILED - GPIO44 is set for both") && r->app().status.pin_conflict, "its pin is checked against the others (buzzer on the same pin)"); }
}

int main() {
    if (const char* only = getenv("WB_ONLY")) {            // one group alone (while working on it): WB_ONLY=nav ./build/test_fw
        if (!strcmp(only, "nav")) { t_nav_flow(); t_nav_mounting(); t_nav_uarts(); }
        if (!strcmp(only, "gps")) t_nav_gps();
        if (!strcmp(only, "servo")) t_servo();
        if (!strcmp(only, "current")) t_current();
        if (!strcmp(only, "receivers")) t_receivers();
        if (!strcmp(only, "crsf")) t_crsf_telemetry();
        if (!strcmp(only, "frames")) t_frames();
        if (!strcmp(only, "ports")) t_ports();
        if (!strcmp(only, "gpsbaud")) t_gps_baud();
        if (!strcmp(only, "gpsalt")) t_gps_alt();
        if (!strcmp(only, "rxdiag")) t_rx_diag();
        if (!strcmp(only, "esc")) t_esc_passthrough();
        if (!strcmp(only, "ekf")) t_ekf();
        if (!strcmp(only, "escwire")) t_esc_wire();
        printf("%d checks, %d failed (only '%s')\n", g_pass + g_fail, g_fail, only); return 1;
    }
    t_boot();
    t_loop();
    t_gyro_cal();
    t_flight();
    t_ground();
    t_link_streams();
    t_settings();
    t_motor_test();
    t_acc_cal();
    t_level_trim();
    t_inflight_trim();
    t_runaway();
    t_tuning_stream();
    t_rpm_filter();
    t_external_rc();
    t_mag_cal();
    t_misc();
    t_battery();
    t_beeper();
    t_motor_beeper();
    t_blackbox(); t_log_stage_times();
    t_blackbox_card();
    t_pins_and_load();
    t_nav_flow(); t_nav_mounting(); t_nav_uarts(); t_nav_gps();
    t_servo();
    t_current();
    t_receivers();
    t_crsf_telemetry();
    t_frames();
    t_ports();
    t_gps_baud();
    t_esc_wire();
    t_esc_passthrough();
    t_ekf();
    t_gps_alt();
    t_rx_diag();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
