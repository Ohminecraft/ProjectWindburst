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
    if (t.active && g_sim) {
        if (t.rd) { if (t.idx > 1) g_sim->read_done(t.reg, t.idx - 1); }
        else for (int i = 0; i < t.idx - 1; i++) g_sim->wr((uint8_t)(t.reg + i), t.w[i]);
    }
    t.active = false;
}
uint8_t stub_spi_transfer(uint8_t v) {
    if (!t.active) return 0xFF;
    uint8_t out = 0xFF;
    if (t.idx == 0) { t.rd = (v & 0x80) != 0; t.reg = g_sim ? g_sim->spi_addr(v & 0x7F) : (v & 0x7F); out = 0x00; }
    else if (t.rd) out = g_sim ? g_sim->byte_at(t.reg, t.idx - 1) : 0xFF;
    else if (t.idx - 1 < 8) t.w[t.idx - 1] = v;
    t.idx++; return out;
}
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t addr, const uint8_t* d, size_t n, bool) { if (!g_sim || addr != g_i2c_addr || n == 0) return 2; i2c_reg = d[0]; if (n == 2 && !g_sim->write(d[0], d[1])) return 2; return 0; }
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
    BaroAltitude a; a.reset(); uint32_t ms = 0; BaroSample s; s.temp_c = 20;
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

int main() {
    t_bmp280(); t_bmp280_faults(); t_bmp388(); t_altitude(); t_manager();
    printf("\n%d passed, %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0;
}
