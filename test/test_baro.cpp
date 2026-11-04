// Barometer: drivers against register-level sims (raw ADC values from the datasheet's double-precision model),
// altitude processing, and the real BaroSensor manager + your SPIBus/I2CBus through Arduino stubs.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <random>
#include <memory>
#include "sim_baro.h"
#include "../src/hal/baro/baro_base.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
static std::mt19937 rng(11); static float gauss(float s) { static std::normal_distribution<float> n(0, 1); return n(rng) * s; }

// ---------------- stub wiring (same idea as test_manager.cpp) ----------------
int g_spi_begin_pins[4];
static BaroSim* g_sim = nullptr; static uint8_t g_i2c_addr = 0x76, g_cs_pin = 255, i2c_reg = 0;
static struct { bool active = false; int idx = 0; bool rd = false; uint8_t reg = 0; uint8_t w[8]; } t;
void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin != g_cs_pin) return;
    if (val == LOW) { t.active = true; t.idx = 0; return; }
    if (t.active && g_sim && g_sim->raw()) {                     // command chip: one byte = a command, more = a read after it
        if (t.idx == 1) g_sim->cmd(t.reg); else if (t.idx > 1) g_sim->read_done(t.reg, t.idx - 1);
    } else if (t.active && g_sim) {
        if (t.rd) { if (t.idx > 1) g_sim->read_done(t.reg, t.idx - 1); }
        else for (int i = 0; i < t.idx - 1; i++) g_sim->wr((uint8_t)(t.reg + i), t.w[i]);
    }
    t.active = false;
}
uint8_t stub_spi_transfer(uint8_t v) {
    if (!t.active) return 0xFF;
    uint8_t out = 0xFF;
    if (t.idx == 0 && g_sim && g_sim->raw()) { t.rd = true; t.reg = v; out = 0x00; }
    else if (t.idx == 0) { t.rd = (v & 0x80) != 0; t.reg = g_sim ? g_sim->spi_addr(v & 0x7F) : (v & 0x7F); out = 0x00; }
    else if (t.rd) out = g_sim ? g_sim->byte_at(t.reg, t.idx - 1) : 0xFF;
    else if (t.idx - 1 < 8) t.w[t.idx - 1] = v;
    t.idx++; return out;
}
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t addr, const uint8_t* d, size_t n, bool) {
    if (!g_sim || addr != g_i2c_addr || n == 0) return 2;
    i2c_reg = d[0];
    if (n == 1 && !g_sim->command(d[0])) return 2;                  // (a register chip ignores it: the register address of a read)
    if (n == 2 && !g_sim->write(d[0], d[1])) return 2;
    return 0;
}
int stub_i2c_rx(uint8_t addr, uint8_t* d, size_t n) { if (!g_sim || addr != g_i2c_addr || n > 64) return 0; if (!g_sim->read(i2c_reg, d, n)) return 0; return (int)n; }

static bool near_(double a, double b, double ab) { return fabs(a - b) <= ab; }

// run a driver on a virtual clock until it yields a sample (or gives up)
static bool pump(BaroDriver& d, BaroSample& s, uint32_t& us, int max_steps = 400) {
    for (int i = 0; i < max_steps; i++) { us += 1000; if (d.update(us, s)) return true; }
    return false;
}

static void t_bmp280() {
    printf("[bmp280] datasheet known answer + random environments\n");
    Bmp280Sim sim; sim.adc_t = 519888; sim.adc_p = 415148;           // Bosch datasheet example: 25.08 C, 100653.27 Pa
    Bmp280Baro d(sim); BaroInitConfig cfg;
    CHECK(d.init(cfg), "init");
    BaroSample s; uint32_t us = 0;
    CHECK(pump(d, s, us), "a sample arrives");
    CHECK(near_(s.temp_c, 25.08, 0.02) && near_(s.pressure_pa, 100653.27, 0.6), "known answer: %.3f C  %.2f Pa", s.temp_c, s.pressure_pa);
    CHECK(strcmp(d.name(), "BMP280") == 0 && d.healthy(), "name/health");
    CHECK(sim.ctrl_meas >> 5 == 1 && ((sim.ctrl_meas >> 2) & 7) == 4 && (sim.config & 0x1C) == 0, "x1 temp, x8 pressure, IIR off (ctrl 0x%02x cfg 0x%02x)", sim.ctrl_meas, sim.config);
    int worst = 0; double maxe = 0, maxt = 0;
    for (int k = 0; k < 40; k++) {
        const double p = 55000 + k * 1500.0 + 13, tc = -15 + k * 1.3;
        sim.set_env(p, tc);
        BaroSample q; uint32_t u2 = us; if (!pump(d, q, u2)) { worst++; continue; } us = u2;
        maxe = fmax(maxe, fabs(q.pressure_pa - p)); maxt = fmax(maxt, fabs(q.temp_c - tc));
    }
    CHECK(worst == 0, "all conversions delivered");
    CHECK(maxe < 1.0 && maxt < 0.05, "driver vs datasheet-double model: max |dP| %.3f Pa, max |dT| %.3f C", maxe, maxt);
    CHECK(sim.bad_triggers == 0 && sim.triggers >= 40, "every conversion was a valid forced trigger (%d)", sim.triggers);
    // non-blocking behaviour: before the conversion time elapsed nothing is returned; one sample per trigger
    Bmp280Sim s2; s2.set_env(101325, 20); Bmp280Baro d2(s2); d2.init(cfg); BaroSample q; uint32_t u = 0;
    CHECK(!d2.update(u, q), "trigger"); CHECK(!d2.update(u + 500, q), "too early => no sample");
    int n = 0; for (int i = 0; i < 100; i++) { u += 1000; if (d2.update(u, q)) n++; }
    CHECK(n >= 3 && n <= 6, "about one sample per %.1f ms (%d in 100 ms)", 1000.0 / d2.nominal_rate_hz(), n);
}

static void t_bmp280_faults() {
    printf("[bmp280] faults\n");
    BaroInitConfig cfg;
    { Bmp280Sim s; s.set_env(101325, 20); s.wrong_id = 0x42; Bmp280Baro d(s); CHECK(!d.init(cfg), "wrong chip id rejected"); }
    { Bmp280Sim s; s.set_env(101325, 20); s.bad_cal = true; Bmp280Baro d(s); CHECK(!d.init(cfg), "all-zero calibration rejected"); }
    { Bmp280Sim s; s.set_env(9000, 20); Bmp280Baro d(s); CHECK(!d.init(cfg), "implausible pressure at init rejected (%s)", "9 kPa"); }
    { Bmp280Sim s; s.dead = true; Bmp280Baro d(s); CHECK(!d.init(cfg), "dead bus"); }
    { Bmp280Sim b; b.T1 = 40000; b.P1 = 50000; b.set_env(98000, 10); Bmp280Baro x(b); BaroSample q; uint32_t u = 0;       // T1/P1 are unsigned in the BMP280
      CHECK(x.init(cfg) && pump(x, q, u) && near_(q.pressure_pa, 98000, 1.0) && near_(q.temp_c, 10, 0.05), "unsigned T1/P1 above 32767: %.2f Pa %.2f C", q.pressure_pa, q.temp_c); }
    { Bmp280Sim s; s.id = 0x60; s.set_env(101325, 20); Bmp280Baro d(s); CHECK(d.init(cfg) && strcmp(d.name(), "BME280") == 0, "BME280 accepted"); }
    { Bmp280Sim s; s.set_env(101325, 20); Bmp280Baro d(s); d.init(cfg); BaroSample q; uint32_t us = 0; pump(d, q, us);
      s.dead = true; for (int i = 0; i < 100; i++) { us += 1000; d.update(us, q); }
      CHECK(!d.healthy(), "bus loss => unhealthy");
      s.dead = false; }
    { Bmp280Sim s; s.set_env(101325, 20); Bmp280Baro d(s); d.init(cfg); s.stuck_busy = true; BaroSample q; uint32_t us = 0; bool got = false;
      for (int i = 0; i < 2000; i++) { us += 1000; got |= d.update(us, q); }
      CHECK(!got && !d.healthy(), "conversion that never finishes => no fake samples, unhealthy"); }
}

static void t_ms5611() {
    printf("[ms5611] datasheet known answer, AN520 CRC, second order below 20 C, random environments, faults\n");
    BaroInitConfig cfg;
    Ms5611Sim sim; Ms5611Baro d(sim);
    CHECK(d.init(cfg), "init");
    CHECK(sim.resets == 1 && d.prom().w[1] == 40127 && d.prom().w[6] == 28312, "reset, PROM read big-endian");
    BaroSample s; uint32_t us = 0;
    CHECK(pump(d, s, us), "a sample arrives");
    CHECK(near_(s.temp_c, 20.07, 0.005) && near_(s.pressure_pa, 100009, 0.5), "datasheet example: %.2f C %.1f Pa (20.07, 100009)", s.temp_c, s.pressure_pa);
    CHECK(sim.osr_p == 4 && sim.osr_t == 4, "both conversions at OSR 4096 (0x48 / 0x58)");
    { ms5611::Prom p{}; for (int i = 0; i < 8; i++) p.w[i] = sim.prom_word(i);
      CHECK(ms5611::prom_ok(p) && ms5611::crc4(p) == (sim.prom_word(7) & 0x0F), "CRC-4 the same as AN520's code");
      for (int k = 0; k < 16; k++) { sim.w7_hi = (uint16_t)(k * 0x1110 + 0x0F00); sim.w0 = (uint16_t)(k * 977); ms5611::Prom q{}; for (int i = 0; i < 8; i++) q.w[i] = sim.prom_word(i);
        CHECK(ms5611::prom_ok(q), "AN520 CRC agrees for PROM %d", k); }
      sim.w7_hi = 0; sim.w0 = 0x0050; }
    { int32_t t, p; ms5611::Prom q{}; for (int i = 0; i < 8; i++) q.w[i] = sim.prom_word(i); ms5611::compensate(q, 9085466, 8569150, t, p);
      CHECK(t == 2007 && p == 100009, "integer compensation: %d %d", (int)t, (int)p); }
    double maxe = 0, maxt = 0; int lost = 0;
    for (int k = 0; k < 40; k++) {
        const double p = 55000 + k * 1500.0 + 13, tc = -30 + k * 2.1;           // down to -30 C: both second-order ranges
        sim.set_env(p, tc);
        BaroSample q; int got = 0;
        for (int i = 0; i < 6 && got < 5; i++) if (pump(d, q, us)) got++;      // past the next temperature conversion
        if (got < 5) { lost++; continue; }
        maxe = fmax(maxe, fabs(q.pressure_pa - p)); maxt = fmax(maxt, fabs(q.temp_c - tc));
    }
    CHECK(lost == 0 && maxe < 2.0 && maxt < 0.02, "driver vs datasheet-double model: max |dP| %.2f Pa, max |dT| %.3f C (lost %d)", maxe, maxt, lost);
    { Ms5611Sim s2; s2.set_env(101325, 20); Ms5611Baro d2(s2); d2.init(cfg); BaroSample q; uint32_t u = 0; int n = 0, first = -1;
      const int t0 = s2.conv_t, p0 = s2.conv_p;
      for (int i = 0; i < 1000; i++) { u += 1000; if (d2.update(u, q)) { n++; if (first < 0) first = i; } }
      CHECK(first >= 18 && n >= 70 && n <= 90, "non-blocking: the first sample after two 9.5 ms conversions (%d ms), %d samples a second (nominal %.0f)", first, n, d2.nominal_rate_hz());
      CHECK((s2.conv_t - t0) * 4 >= s2.conv_p - p0 - 4 && (s2.conv_t - t0) * 4 <= s2.conv_p - p0 + 4, "a temperature every 4th pressure (%d / %d)", s2.conv_t - t0, s2.conv_p - p0); }
    { Ms5611Sim b; b.bad_crc = true; Ms5611Baro x(b); CHECK(!x.init(cfg), "bad CRC rejected"); }
    { Ms5611Sim b; b.blank = true; Ms5611Baro x(b); CHECK(!x.init(cfg), "PROM all 0 (nothing on the bus) rejected"); }
    { Ms5611Sim b; b.dead = true; Ms5611Baro x(b); CHECK(!x.init(cfg), "dead bus"); }
    { Ms5611Sim b; b.zero_adc = true; Ms5611Baro x(b); CHECK(!x.init(cfg), "ADC reading 0 at init rejected"); }
    { Ms5611Sim b; b.set_env(9000, 20); Ms5611Baro x(b); CHECK(!x.init(cfg), "implausible pressure rejected"); }
    { Ms5611Sim b; Ms5611Baro x(b); x.init(cfg); BaroSample q; uint32_t u = 0; pump(x, q, u);
      b.zero_adc = true; bool got = false; for (int i = 0; i < 30; i++) { u += 1000; x.update(u, q); }      // (a conversion already started still ends)
      for (int i = 0; i < 400; i++) { u += 1000; got |= x.update(u, q); }
      CHECK(!got && !x.healthy(), "ADC giving 0 (conversion lost): no fake samples, unhealthy");
      b.zero_adc = false; for (int i = 0; i < 100; i++) { u += 1000; got |= x.update(u, q); }
      CHECK(got && x.healthy(), "and back when it answers again"); }
}

static void t_bmp388() {
    printf("[bmp388] normal mode at 50 Hz, Bosch float compensation (0.8.8: the user's driver) vs the float model\n");
    Bmp388Sim sim; sim.set_env(100653.27, 25.08);
    Bmp388Baro d(sim); BaroInitConfig cfg;
    CHECK(d.init(cfg), "init");
    BaroSample s; uint32_t us = 0;
    CHECK(pump(d, s, us), "sample");
    CHECK(near_(s.temp_c, 25.08, 0.05) && near_(s.pressure_pa, 100653.27, 1.0), "%.3f C %.2f Pa", s.temp_c, s.pressure_pa);
    CHECK(strcmp(d.name(), "BMP388") == 0, "name");
    CHECK(sim.osr == 0x03 && ((sim.cfgreg >> 1) & 7) == 2 && sim.odr == 0x02 && sim.normal && ((sim.pwr >> 4) & 3) == 3, "x8 pressure, x1 temperature, IIR coefficient 3, ODR 50 Hz, normal mode (osr 0x%02x, config 0x%02x, odr 0x%02x)", sim.osr, sim.cfgreg, sim.odr);
    double maxe = 0, maxt = 0; int missed = 0;
    for (int k = 0; k < 60; k++) {
        const double p = 40000 + k * 1250.0 + 7.3, tc = -20 + k * 1.05;
        sim.set_env(p, tc);
        BaroSample q; uint32_t u2 = us; if (!pump(d, q, u2)) { missed++; continue; } us = u2;
        maxe = fmax(maxe, fabs(q.pressure_pa - p)); maxt = fmax(maxt, fabs(q.temp_c - tc));
    }
    CHECK(missed == 0, "all conversions delivered");
    CHECK(maxe < 2.0 && maxt < 0.1, "max |dP| %.3f Pa, max |dT| %.3f C over 40..115 kPa, -20..43 C", maxe, maxt);
    CHECK(sim.bad_triggers == 0, "pressure and temperature both enabled");
    { Bmp388Sim b; b.T1 = 40000; b.T2 = 40000; b.P6 = 40000; b.set_env(97000, 30);       // coefficients above 32767 must stay unsigned
      Bmp388Baro x(b); BaroSample q; uint32_t u = 0;
      CHECK(x.init(cfg) && pump(x, q, u) && near_(q.pressure_pa, 97000, 2.0) && near_(q.temp_c, 30, 0.1), "unsigned T1/T2/P6 above 32767: %.2f Pa %.2f C", q.pressure_pa, q.temp_c); }
    { Bmp388Sim b; b.id = 0x60; b.set_env(101000, 15); Bmp388Baro x(b); CHECK(x.init(cfg) && strcmp(x.name(), "BMP390") == 0, "BMP390 (id 0x60)"); }
    { Bmp388Sim b; b.set_env(101000, 15); b.spi = true; b.awake = false; Bmp388Baro x(b); BaroSample q; uint32_t u = 0;
      CHECK(x.init(cfg) && pump(x, q, u) && near_(q.pressure_pa, 101000, 1.5), "SPI: wake-up dummy read + dummy byte handled (%.2f)", q.pressure_pa); }
    printf("[bmp388] faults\n");
    { Bmp388Sim b; b.set_env(101000, 15); b.wrong_id = 0x58; Bmp388Baro x(b); CHECK(!x.init(cfg), "BMP280 id rejected by the BMP388 driver"); }
    { Bmp388Sim b; b.set_env(101000, 15); b.bad_cal = true; Bmp388Baro x(b); CHECK(!x.init(cfg), "zero calibration rejected"); }
    { Bmp388Sim b; b.set_env(101000, 15); b.err_force = 0x04; Bmp388Baro x(b); CHECK(!x.init(cfg), "configuration-error flag fails init"); }
    { Bmp388Sim b; b.set_env(101000, 15); Bmp388Baro x(b); x.init(cfg); b.stuck_busy = true; BaroSample q; uint32_t u = 0; bool got = false;
      for (int i = 0; i < 2000; i++) { u += 1000; got |= x.update(u, q); } CHECK(!got && !x.healthy(), "stuck conversion => unhealthy, no samples"); }
}

static double pressure_at(double h, double p0) { return p0 * pow(1.0 - h / 44330.0, 1.0 / 0.190295); }

static void t_altitude() {
    printf("[altitude] ground reference, drift, outliers, staleness\n");
    BaroAltitude a; a.reset(); uint32_t ms = 0; BaroSample s; s.temp_c = 15;          // (the standard atmosphere's ground temperature)
    auto feed = [&](double p, bool armed, int n, float noise = 0.0f, int step_ms = 20) { bool acc = false; for (int i = 0; i < n; i++) { ms += step_ms; s.pressure_pa = (float)(p + gauss(noise)); acc = a.update(s, ms, armed); } return acc; };
    feed(101325, false, 39);
    CHECK(!a.calibrated() && !a.valid(ms), "not valid before the ground average is done");
    feed(101325, false, 5);
    CHECK(a.calibrated() && a.valid(ms) && fabsf(a.altitude_m()) < 0.05f, "ground set, altitude 0 (%f)", a.altitude_m());
    feed(pressure_at(10, 101325), true, 1);
    CHECK(fabsf(a.altitude_m() - 10.0f) < 0.05f, "10 m up => %.3f", a.altitude_m());
    feed(pressure_at(-3, 101325), true, 1);
    CHECK(fabsf(a.altitude_m() + 3.0f) < 0.05f, "below takeoff is negative: %.3f", a.altitude_m());
    // weather drift while armed is NOT absorbed (ground frozen): 100 Pa = ~8.4 m
    feed(101325 - 100, true, 600);
    CHECK(fabsf(a.altitude_m() - 8.4f) < 0.3f, "armed: ground reference frozen, pressure change reads as altitude (%.2f)", a.altitude_m());
    // outlier
    feed(101325, true, 3); float before = a.altitude_m();
    bool acc = feed(101325 - 900, true, 1);
    CHECK(!acc && fabsf(a.altitude_m() - before) < 0.05f, "single 70 m spike rejected");
    feed(101325 - 900, true, 6);
    CHECK(a.altitude_m() > 50.0f, "...but a sustained change is accepted (%.1f)", a.altitude_m());
    // disarmed: slow weather drift follows, noise is measured
    BaroAltitude b; b.reset(); ms = 0; std::mt19937 r2(3);
    s.pressure_pa = 101325; for (int i = 0; i < 60; i++) { ms += 20; b.update(s, ms, false); }
    double p = 101325;
    for (int i = 0; i < 30000; i++) { ms += 20; p -= 0.0033; s.pressure_pa = (float)(p + gauss(3.0f)); b.update(s, ms, false); }   // -100 Pa over 10 min
    CHECK(fabsf(b.altitude_m()) < 0.8f, "disarmed: 10 min of 100 Pa weather drift absorbed, altitude %.2f", b.altitude_m());
    CHECK(b.sigma_m() > 0.15f && b.sigma_m() < 0.45f, "measured noise sigma %.3f m (true: 3 Pa ~ 0.25 m)", b.sigma_m());
    // staleness and range
    CHECK(b.valid(ms + 400) && !b.valid(ms + 600), "goes invalid 500 ms after the last sample");
    s.pressure_pa = 5000; CHECK(!b.update(s, ms + 30, false), "out-of-range pressure ignored");
    // 0.43: the air's temperature (ArduPilot AP_Baro::get_altitude_difference_simple: 153.8462 x T_ground x (1 - (p/p0)^0.190259)).
    // Warm air is thinner: the pressure falls more slowly with height, so the same pressure step is more metres. The model below is
    // the troposphere with the ground at T0 (lapse rate 6.5 K/km): p = p0 (1 - 0.0065 h / T0)^5.2559.
    auto atm = [](double h, double p0, double t0_c) { const double T0 = t0_c + 273.15; return p0 * pow(1.0 - 0.0065 * h / T0, 5.2559); };
    for (const float tc : {35.0f, 0.0f}) {
        BaroAltitude w; w.reset(); ms = 0; s.temp_c = tc; s.pressure_pa = 100000; for (int i = 0; i < 60; i++) { ms += 20; w.update(s, ms, false); }
        s.pressure_pa = (float)atm(100.0, 100000, tc); for (int i = 0; i < 6; i++) w.update(s, ms += 20, true);      // (a jump that big must hold 5 samples)
        CHECK(fabsf(w.altitude_m() - 100.0f) < 0.3f && fabsf(w.ground_temp_c() - tc) < 0.01f, "ground at %.0f C: 100 m up reads %.2f m (the 15 C formula: %.2f)", tc, w.altitude_m(), BaroAltitude::altitude_of(s.pressure_pa, 100000));
    }
    {   BaroAltitude w; w.reset(); ms = 0; s.temp_c = 60; s.pressure_pa = 100000; for (int i = 0; i < 60; i++) { ms += 20; w.update(s, ms, false); }
        CHECK(fabsf(w.ground_temp_c() - 35.0f) < 0.01f, "a board heated to 60 C: taken as 35 C (ArduPilot: the baro reads high, at most 35 C)");
        s.temp_c = 20; for (int i = 0; i < 6000; i++) { ms += 20; w.update(s, ms, false); }
        const float g = w.ground_temp_c(); s.temp_c = 0; for (int i = 0; i < 500; i++) { ms += 20; w.update(s, ms, true); }
        CHECK(fabsf(g - 20.0f) < 0.5f && w.ground_temp_c() == g, "disarmed it follows the air (%.1f C after 2 min); armed it is frozen with the ground pressure", g); }
}

static void pumpM(BaroSensor& b, int ms_total, bool armed, uint32_t& us, int& got) { for (int i = 0; i < ms_total; i++) { us += 1000; if (b.update(us, us / 1000, armed)) got++; } }

static void t_manager() {
    printf("[manager] detection + full chain through SPIBus / I2CBus\n");
    { Bmp280Sim sim; sim.set_env(100500, 18); g_sim = &sim; g_i2c_addr = 0x76;
      BaroSensor b; b.setI2cPins(8, 9); uint32_t us = 0; int got = 0;
      CHECK(b.BaroInit(false) && strcmp(b.name(), "BMP280") == 0, "BMP280 on I2C 0x76: %s", b.failReason());
      pumpM(b, 2000, false, us, got);
      CHECK(got > 10 && b.groundCalibrated() && b.valid(us / 1000), "samples flow (%d), calibrated, valid", got);
      CHECK(fabsf(b.altitudeM()) < 0.2f && fabsf(b.pressurePa() - 100500) < 1.5f && fabsf(b.tempC() - 18) < 0.1f, "values: %.2f m %.1f Pa %.2f C", b.altitudeM(), b.pressurePa(), b.tempC());
      sim.set_env(pressure_at(5, 100500), 18); pumpM(b, 300, true, us, got);
      CHECK(fabsf(b.altitudeM() - 5.0f) < 0.3f, "a 5 m climb reads %.2f", b.altitudeM());
      sim.dead = true; pumpM(b, 500, true, us, got); CHECK(!b.healthy() || !b.valid(us / 1000), "unplugged sensor is reported invalid (arm check AB_ALTHOLD_NO_BARO)"); }
    { Bmp388Sim sim; sim.set_env(99000, 12); g_sim = &sim; g_i2c_addr = 0x77;
      BaroSensor b; b.setI2cPins(8, 9);
      CHECK(b.BaroInit(false) && strcmp(b.name(), "BMP388") == 0, "BMP388 on I2C 0x77 (scan skipped 0x76): %s", b.failReason()); }
    for (int kind = 0; kind < 2; kind++) {
        std::unique_ptr<BaroSim> sim; if (kind == 0) { auto* s = new Bmp280Sim(); s->set_env(101000, 22); sim.reset(s); } else { auto* s = new Bmp388Sim(); s->set_env(101000, 22); s->awake = false; sim.reset(s); }
        sim->spi = true; g_sim = sim.get(); g_cs_pin = 10;
        BaroSensor b; b.setSpiPins(10, 13, 11, 12); uint32_t us = 0; int got = 0;
        CHECK(b.BaroInit(true) && strcmp(b.name(), kind == 0 ? "BMP280" : "BMP388") == 0, "%s on SPI: %s", kind == 0 ? "BMP280" : "BMP388", b.failReason());
        CHECK(g_spi_begin_pins[0] == 12 && g_spi_begin_pins[1] == 13 && g_spi_begin_pins[2] == 11 && g_spi_begin_pins[3] == 10, "SPI.begin(sck,miso,mosi,cs) order");
        pumpM(b, 2000, false, us, got);
        CHECK(got > 10 && fabsf(b.pressurePa() - 101000) < 2.0f && b.valid(us / 1000), "SPI samples %d, %.1f Pa", got, b.pressurePa());
    }
    { Ms5611Sim sim; sim.set_env(100200, 16); g_sim = &sim; g_i2c_addr = 0x77;
      BaroSensor b; b.setI2cPins(8, 9); uint32_t us = 0; int got = 0;
      CHECK(b.BaroInit(false) && strcmp(b.name(), "MS5611") == 0, "MS5611 on I2C 0x77: %s", b.failReason());
      pumpM(b, 1000, false, us, got);
      CHECK(got > 30 && fabsf(b.pressurePa() - 100200) < 2.0f && fabsf(b.tempC() - 16) < 0.05f, "I2C samples %d, %.1f Pa %.2f C", got, b.pressurePa(), b.tempC()); }
    { Ms5611Sim sim; sim.set_env(100200, 16); sim.spi = true; g_sim = &sim; g_cs_pin = 10;
      BaroSensor b; b.setSpiPins(10, 13, 11, 12); uint32_t us = 0; int got = 0;
      CHECK(b.BaroInit(true) && strcmp(b.name(), "MS5611") == 0, "MS5611 on SPI (commands without the read flag): %s", b.failReason());
      pumpM(b, 1000, false, us, got);
      CHECK(got > 30 && fabsf(b.pressurePa() - 100200) < 2.0f, "SPI samples %d, %.1f Pa", got, b.pressurePa()); }
    { Ms5611Sim sim; sim.blank = true; g_sim = &sim; g_i2c_addr = 0x77; BaroSensor b; b.setI2cPins(8, 9);
      CHECK(!b.BaroInit(false) && strstr(b.failReason(), "no BMP280/BMP388/MS5611"), "a PROM of zeros (its CRC is 0 too) is not taken for an MS5611: '%s'", b.failReason()); }
    { Ms5611Sim sim; sim.bad_crc = true; g_sim = &sim; g_i2c_addr = 0x77; BaroSensor b; b.setI2cPins(8, 9);
      CHECK(!b.BaroInit(false) && strstr(b.failReason(), "MS5611"), "a PROM with a bad CRC is not taken for an MS5611: '%s'", b.failReason()); }
    printf("[manager] a barometer on the IMU's SPI bus (its own chip select), two barometers on I2C\n");
    for (int kind = 0; kind < 2; kind++) {
        std::unique_ptr<BaroSim> sim; if (kind == 0) { auto* s = new Ms5611Sim(); s->set_env(100800, 21); sim.reset(s); } else { auto* s = new Bmp280Sim(); s->set_env(100800, 21); sim.reset(s); }
        sim->spi = true; g_sim = sim.get(); g_cs_pin = 10;
        SPIBus host(SPI_DEVICE_0); SPIBus_config_t c; c.ioCsGpio = 9; c.ioSdoGpio = 13; c.ioSdiGpio = 11; c.ioSckGpio = 12; host.SPIInit(c);
        BaroSensor b; uint32_t us = 0; int got = 0;
        CHECK(b.BaroInitShared(host, 10) && strcmp(b.name(), kind == 0 ? "MS5611" : "BMP280") == 0 && b.onImuBus() && b.i2cAddress() == 0, "%s on the host's bus, CS 10: %s", kind == 0 ? "MS5611" : "BMP280", b.failReason());
        pumpM(b, 1500, false, us, got);
        CHECK(got > 10 && fabsf(b.pressurePa() - 100800) < 2.0f, "samples %d, %.1f Pa", got, b.pressurePa());
        BaroSensor x; CHECK(!x.BaroInitShared(host, 9) && strstr(x.failReason(), "chip-select"), "the IMU's own chip select is refused: '%s'", x.failReason());
    }
    { g_sim = nullptr; SPIBus idle(SPI_DEVICE_0); BaroSensor b; CHECK(!b.BaroInitShared(idle, 10), "a host bus that never started: refused"); }
    { Bmp280Sim sim; sim.set_env(101000, 20); g_sim = &sim; g_i2c_addr = 0x76;
      BaroSensor a, b; a.setI2cPins(8, 9); b.setI2cPins(8, 9);
      CHECK(a.BaroInit(false) && a.i2cAddress() == 0x76 && !a.onImuBus(), "the first one at 0x76");
      b.skipI2cAddress(a.i2cAddress()); CHECK(!b.BaroInit(false) && strstr(b.failReason(), "0x76/0x77"), "the second one skips 0x76: nothing else there (%s)", b.failReason()); }

    printf("[manager] failures\n");
    { g_sim = nullptr; BaroSensor b; b.setI2cPins(8, 9); CHECK(!b.BaroInit(false) && strstr(b.failReason(), "no BMP280/BMP388"), "empty I2C: '%s'", b.failReason()); }
    { g_sim = nullptr; g_cs_pin = 10; BaroSensor b; b.setSpiPins(10, 13, 11, 12); CHECK(!b.BaroInit(true) && strstr(b.failReason(), "SPI"), "empty SPI: '%s'", b.failReason()); }
    { BaroSensor b; CHECK(!b.BaroInit(false) && strstr(b.failReason(), "pins not set"), "no pins: '%s'", b.failReason()); }
    { Bmp280Sim sim; sim.set_env(101325, 20); g_sim = &sim; g_i2c_addr = 0x76; BaroSensor b; b.setI2cPins(8, 9); b.setEnabled(false);
      CHECK(!b.BaroInit(false) && strstr(b.failReason(), "disabled") && !b.available(), "app switch: barometer off"); }
    { Bmp280Sim sim; sim.set_env(101325, 20); sim.bad_cal = true; g_sim = &sim; g_i2c_addr = 0x76; BaroSensor b; b.setI2cPins(8, 9);
      CHECK(!b.BaroInit(false) && strstr(b.failReason(), "init failed"), "bad calibration: '%s'", b.failReason()); }
    { Bmp280Sim sim; sim.set_env(101325, 20); g_sim = &sim; g_i2c_addr = 0x76; I2CBus shared(I2C_DEVICE_0);
      I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = 8; c.ioSclGpio = 9; c.addr = 0x29; shared.I2CInit(c);      // bus already initialised for another device
      BaroSensor b; b.attachI2C(shared); CHECK(b.BaroInit(false) && b.available(), "shared bus: address selected per transaction: %s", b.failReason()); }
}

static void t_select() {
    printf("[select] two barometers: the preferred one, the other while it is unhealthy, no jump, no back and forth\n");
    using barosel::Selector;
    Selector s; s.configure(0, true);
    bool v[2] = {true, true}, f[2] = {true, true}; float a[2] = {1.0f, 3.0f}, sg[2] = {0.3f, 0.5f}, out = -1, so = -1; uint32_t ms = 0;
    CHECK(s.update(v, f, a, sg, false, ms, out, so) && out == 1.0f && so == 0.3f && s.in_use() == 0, "primary 0 flies: %.2f", out);
    f[1] = false; a[0] = 1.1f; CHECK(s.update(v, f, a, sg, true, ms += 20, out, so) && out == 1.1f, "a new sample of the one in use");
    f[0] = false; f[1] = true; a[1] = 3.2f; CHECK(!s.update(v, f, a, sg, true, ms += 20, out, so), "a sample of the other one only: nothing new for the estimator");
    v[0] = false; f[0] = false; f[1] = true; a[1] = 3.3f;
    CHECK(s.update(v, f, a, sg, true, ms += 20, out, so) && s.in_use() == 1 && fabsf(out - 1.1f) < 1e-5f && so == 0.5f && s.switches() == 1, "the first one fails in flight: the second takes over, continuing at %.2f m (not 3.3)", out);
    a[1] = 3.8f; CHECK(s.update(v, f, a, sg, true, ms += 20, out, so) && fabsf(out - 1.6f) < 1e-5f, "and follows its own changes: +0.5 m -> %.2f", out);
    v[0] = true; f[0] = true; a[0] = 1.7f; a[1] = 3.9f;
    CHECK(s.update(v, f, a, sg, true, ms += 20, out, so) && s.in_use() == 1 && fabsf(out - 1.7f) < 1e-5f, "the first one back: not at once (%.2f from the second)", out);
    for (int i = 0; i < 99; i++) s.update(v, f, a, sg, true, ms += 20, out, so);
    CHECK(s.in_use() == 1, "1.98 s valid: still the second");
    a[0] = 1.8f; a[1] = 4.0f; s.update(v, f, a, sg, true, ms += 20, out, so);
    CHECK(s.in_use() == 0 && s.switches() == 2 && fabsf(out - 1.7f) < 1e-4f && fabsf(s.offset(0) + 0.1f) < 1e-4f, "2 s valid: back to the first one, again without a step (%.2f, its offset %.2f)", out, s.offset(0));
    v[0] = false; s.update(v, f, a, sg, true, ms += 20, out, so); v[0] = true; s.update(v, f, a, sg, true, ms += 20, out, so);
    CHECK(s.in_use() == 1 && s.switches() == 3, "a flaky first one: one switch away ...");
    for (int i = 0; i < 50; i++) { v[0] = (i % 10) != 0; s.update(v, f, a, sg, true, ms += 20, out, so); }
    CHECK(s.in_use() == 1 && s.switches() == 3, "... and none back while it keeps dropping out (2 s without a break needed)");
    v[0] = true; for (int i = 0; i < 120; i++) s.update(v, f, a, sg, false, ms += 20, out, so);
    CHECK(s.in_use() == 0 && s.offset(0) == 0.0f && out == a[0], "disarmed on the preferred one: the offset is cleared (each reads its own ground 0)");
    { Selector p; p.configure(1, true); bool vv[2] = {true, true}, ff[2] = {true, true}; float aa[2] = {5, 7}, ss[2] = {1, 1}, o = 0, oo = 0;
      CHECK(p.update(vv, ff, aa, ss, false, 0, o, oo) && p.in_use() == 1 && o == 7.0f, "baro_primary 1: the second one flies"); }
    { Selector p; p.configure(1, false); bool vv[2] = {true, false}, ff[2] = {true, false}; float aa[2] = {5, 0}, ss[2] = {1, 1}, o = 0, oo = 0;
      CHECK(p.update(vv, ff, aa, ss, false, 0, o, oo) && p.in_use() == 0, "baro_primary 1 with one barometer: the one there is"); }
    { Selector p; p.configure(0, true); bool vv[2] = {false, true}, ff[2] = {true, true}; float aa[2] = {5, 7}, ss[2] = {1, 1}, o = 0, oo = 0;
      CHECK(p.update(vv, ff, aa, ss, false, 0, o, oo) && p.in_use() == 1 && o == 7.0f && p.switches() == 0, "the preferred one not valid at the start: the other, no offset (nothing to continue)");
      vv[1] = false; CHECK(!p.update(vv, ff, aa, ss, false, 20, o, oo) && p.in_use() == 1, "both invalid: no altitude, the last one stays chosen"); }
    { Selector p; p.configure(0, true); bool vv[2] = {false, true}, ff[2] = {true, true}; float aa[2] = {0.1f, 0.2f}, ss[2] = {1, 1}, o = 0, oo = 0;
      p.update(vv, ff, aa, ss, false, 0, o, oo); vv[0] = true; p.update(vv, ff, aa, ss, false, 20, o, oo);
      CHECK(p.in_use() == 0 && o == 0.1f, "boot: the faster chip is ready first, the preferred one takes over as soon as it is (disarmed: no 2 s wait)"); }
    { Selector p; p.configure(0, true); bool vv[2] = {false, false}, ff[2] = {false, false}; float aa[2] = {0, 0}, ss[2] = {1, 1}, o = 0, oo = 0;
      CHECK(!p.update(vv, ff, aa, ss, false, 0, o, oo) && p.in_use() == -1, "none valid yet: none in use"); }
}

int main() {
    t_select(); t_bmp280(); t_bmp280_faults(); t_ms5611(); t_bmp388(); t_altitude(); t_manager();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
