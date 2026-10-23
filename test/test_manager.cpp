// Runs the REAL IMUAccGyro + your real I2CBus / SPIBus classes (src/hal/bus/*.h) on a PC.
// Arduino.h / SPI.h / Wire.h are stubs (test/arduino_stub) that forward bytes to the simulated chips in sim_imu.h, so this exercises
// the whole chain: SPI byte framing (reg|0x80, CS edges), detection probe order, driver selection, init, sanity check, alignment, bias.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <memory>
#include "sim_imu.h"
#include "../src/hal/acc_gyro/acc_gyro_base.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

// ---------------- stub wiring ----------------
int g_spi_begin_pins[4];
static Sim* g_sim = nullptr;          // chip currently "soldered" on the bus (nullptr = empty pads, MISO floats high)
static uint8_t g_i2c_addr = 0x68;
static uint8_t g_cs_pin = 255;
static struct { bool active = false; int idx = 0; bool rd = false; uint8_t reg = 0; uint8_t cache[16]; uint8_t w[200]; } t;
static uint8_t i2c_reg = 0;

void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin != g_cs_pin) return;
    if (val == LOW) { t.active = true; t.idx = 0; return; }
    if (t.active && !t.rd && t.idx >= 2 && g_sim) {                 // end of a write transaction
        int n = t.idx - 1;
        if (n == 1) g_sim->write(t.reg, t.w[0]); else g_sim->write_buf(t.reg, t.w, (size_t)n);
    }
    t.active = false;
}
uint8_t stub_spi_transfer(uint8_t v) {
    if (!t.active) return 0xFF;
    uint8_t out = 0xFF;
    if (t.idx == 0) { t.rd = (v & 0x80) != 0; t.reg = v & 0x7F; out = 0x00; }
    else if (t.rd) {
        if (t.idx == 1) { if (g_sim) { if (!g_sim->read(t.reg, t.cache, 16)) memset(t.cache, 0xFF, 16); } else memset(t.cache, 0xFF, 16); }
        out = (t.idx - 1 < 16) ? t.cache[t.idx - 1] : 0x00;
    } else { if (t.idx - 1 < (int)sizeof(t.w)) t.w[t.idx - 1] = v; }
    t.idx++;
    return out;
}
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t addr, const uint8_t* d, size_t n, bool) {
    if (!g_sim || addr != g_i2c_addr || n == 0) return 2;            // NACK
    i2c_reg = d[0];
    if (n == 2) g_sim->write(d[0], d[1]);
    return 0;
}
int stub_i2c_rx(uint8_t addr, uint8_t* d, size_t n) {
    if (!g_sim || addr != g_i2c_addr || n > 16) return 0;
    uint8_t tmp[16];
    if (!g_sim->read(i2c_reg, tmp, n)) return 0;
    memcpy(d, tmp, n); return (int)n;
}

static bool near_(float a, float b, float ab = 0.08f, float rel = 0.02f) { return fabsf(a - b) <= ab + rel * fabsf(b); }
static Sim* make(Chip c, bool spi) { static std::unique_ptr<Sim> keep; keep.reset(new Sim(c, spi)); g_sim = keep.get(); return g_sim; }
static void pins(IMUAccGyro& m) { m.setSpiPins(10, 13, 11, 12); g_cs_pin = 10; m.setI2cPins(8, 9); }

int main() {
    printf("[manager] full chain over the real SPIBus for every SPI chip\n");
    struct Row { Chip c; const char* name; IMUAccGyroDevice dev; float rate; };
    const Row rows[] = {
        {C_MPU6000, "MPU6000", MPU6000_SPI, 4000}, {C_MPU6500, "MPU6500", MPU6500_SPI, 4000}, {C_MPU9250, "MPU9250", MPU9520_SPI, 4000},
        {C_ICM20601, "ICM20601", ICM20601_SPI, 4000}, {C_ICM20602, "ICM20602", ICM20602_SPI, 4000},
        {C_ICM42622P, "ICM42622P", ICM42622P_SPI, 4000}, {C_ICM42686P, "ICM42686P", ICM42686P_SPI, 4000}, {C_ICM42688P, "ICM42688P", ICM42688P_SPI, 4000},
        {C_ICM45605, "ICM45605", ICM45605_SPI, 3200}, {C_ICM45686, "ICM45686", ICM45686_SPI, 3200},
        {C_BMI160, "BMI160", BMI160_SPI, 3200}, {C_BMI270, "BMI270", BMI270_SPI, 3200},
        {C_ICM20948, "ICM20948", ICM20948_SPI, 9000}, {C_ICM20649, "ICM20649", ICM20649_SPI, 9000}};
    for (const Row& row : rows) {
        Sim* s = make(row.c, true);
        std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
        bool ok = m->IMUInit(true);
        CHECK(ok, "%s IMUInit failed: %s", row.name, m->failReason());
        if (!ok) continue;
        CHECK(m->detected() == row.dev && strcmp(m->name(), row.name) == 0, "%s detected as %s (%d)", row.name, m->name(), (int)m->detected());
        CHECK(fabsf(m->gyroRateHz() - row.rate) < 1, "%s rate %.0f", row.name, m->gyroRateHz());
        AgSample o{}; bool rd = m->read(o);
        bool same = rd;
        static const float chip[3] = {1.0f, -1.0f, -1.0f};        // default mounting CW0 (Betaflight): chip x = nose, chip y = left, chip z = up -> board frame x, -y, -z
        for (int i = 0; i < 3 && same; i++) same = near_(o.gyro_dps[i], chip[i] * s->tg[i]) && near_(o.acc_g[i], chip[i] * s->ta[i], 0.01f);
        CHECK(same, "%s values through SPIBus: gyro %.2f %.2f %.2f acc %.3f %.3f %.3f", row.name, o.gyro_dps[0], o.gyro_dps[1], o.gyro_dps[2], o.acc_g[0], o.acc_g[1], o.acc_g[2]);
        if (row.c == C_BMI270) CHECK(s->init_ok, "BMI270 microcode arrived intact through SPIBus::SPIWrite");
    }

    { make(C_ICM42688P, true); std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m); m->IMUInit(true);
      CHECK(g_spi_begin_pins[0] == 12 && g_spi_begin_pins[1] == 13 && g_spi_begin_pins[2] == 11 && g_spi_begin_pins[3] == 10,
            "SPI.begin(sck,miso,mosi,cs) got %d %d %d %d, want 12 13 11 10 (setSpiPins order is cs,miso,mosi,sck)", g_spi_begin_pins[0], g_spi_begin_pins[1], g_spi_begin_pins[2], g_spi_begin_pins[3]); }

    printf("[manager] I2C chips and address scan\n");
    { Sim* s = make(C_MPU6050, false); g_i2c_addr = 0x68; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(false) && m->detected() == MPU6050_I2C, "MPU6050 @0x68: %s", m->failReason());
      AgSample o{}; CHECK(m->read(o) && near_(o.gyro_dps[1], -s->tg[1]) && near_(o.gyro_dps[0], s->tg[0]) && near_(o.acc_g[2], -s->ta[2], 0.01f), "i2c values (chip y / z are the board's left / up)"); }
    { make(C_MPU6500, false); g_i2c_addr = 0x69; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(false) && m->detected() == MPU6500_I2C, "MPU6500 @0x69 (AD0 high): %s", m->failReason()); }
    { Sim* s = make(C_ICM20948, false); g_i2c_addr = 0x69; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(false) && m->detected() == ICM20948_I2C && strcmp(m->name(), "ICM20948") == 0, "ICM20948 @0x69: %s", m->failReason());
      AgSample o{}; CHECK(m->read(o) && near_(o.gyro_dps[0], s->tg[0]) && near_(o.acc_g[2], -s->ta[2], 0.01f) && fabsf(m->gyroRateHz() - 1125) < 1, "ICM20948 I2C values, 1125 Hz"); }
    { Sim* s = make(C_ICM20649, false); s->bank = 2; g_i2c_addr = 0x68; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(false) && m->detected() == ICM20649_I2C, "ICM20649 @0x68 left in bank 2: %s", m->failReason()); }
    g_i2c_addr = 0x68;

    printf("[manager] mounting alignment + gyro bias\n");
    { Sim* s = make(C_ICM42688P, true); s->tg[0] = 1.5f; s->tg[1] = -0.7f; s->tg[2] = 0.4f;
      std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m); m->IMUInit(true);
      AgSample a{}; m->read(a);
      m->setAlignment(CW90_DEG); AgSample b{}; m->read(b);
      // the same chip readings, now from a chip that is turned 90 deg clockwise (seen from above): its x points to the RIGHT of the craft
      CHECK(near_(b.acc_g[0], -a.acc_g[1], 0.005f) && near_(b.acc_g[1], a.acc_g[0], 0.005f) && near_(b.acc_g[2], a.acc_g[2], 0.005f), "CW90 acc");
      CHECK(near_(b.gyro_dps[0], -a.gyro_dps[1], 0.05f) && near_(b.gyro_dps[1], a.gyro_dps[0], 0.05f) && near_(b.gyro_dps[2], a.gyro_dps[2], 0.05f), "gyro rotates with accel (same frame)");
      m->setAlignment(CW0_DEG);
      CHECK(!m->gyroCalibrated(), "not calibrated yet");
      CHECK(m->calibrateGyro(200) && m->gyroCalibrated(), "calibrate still board");
      AgSample c{}; m->read(c);
      CHECK(fabsf(c.gyro_dps[0]) < 0.1f && fabsf(c.gyro_dps[1]) < 0.1f && fabsf(c.gyro_dps[2]) < 0.1f, "bias removed: %.3f %.3f %.3f", c.gyro_dps[0], c.gyro_dps[1], c.gyro_dps[2]);
      CHECK(near_(c.acc_g[2], -s->ta[2], 0.01f), "accel untouched by gyro bias");
      s->tg[0] = 150.0f;                                                  // now "moving"
      CHECK(!m->calibrateGyro(100), "moving board must refuse to calibrate"); }

    printf("[manager] failure reporting\n");
    { g_sim = nullptr; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(!m->IMUInit(true) && !m->available() && strstr(m->failReason(), "no known IMU"), "empty SPI pads: '%s'", m->failReason()); }
    { g_sim = nullptr; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(!m->IMUInit(false) && strstr(m->failReason(), "I2C"), "empty I2C: '%s'", m->failReason()); }
    { make(C_ICM42688P, true); std::unique_ptr<IMUAccGyro> m(new IMUAccGyro());    // pins never set => hardware_config.h says GPIO_UNUSED
      CHECK(!m->IMUInit(true) && strstr(m->failReason(), "pins not set"), "no pins: '%s'", m->failReason()); }
    { Sim* s = make(C_ICM42688P, true); s->ta[0] = 0; s->ta[1] = 0; s->ta[2] = 3.0f; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(!m->IMUInit(true) && !m->available() && strstr(m->failReason(), "1 g"), "3 g accel is rejected: '%s'", m->failReason()); }
    { make(C_BMI270, true); g_sim->bad_upload = true; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(!m->IMUInit(true) && strstr(m->failReason(), "init failed"), "bad microcode: '%s'", m->failReason()); }
    { make(C_ICM45686, true); std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(true) && m->IMUInit(true) && m->available(), "re-init twice works"); }
    { Sim* s = make(C_ICM20948, true); s->bank = 2; std::unique_ptr<IMUAccGyro> m(new IMUAccGyro()); pins(*m);
      CHECK(m->IMUInit(true) && m->detected() == ICM20948_SPI, "ICM20948 left in bank 2 by a warm restart: found on the second try: %s", m->failReason()); }

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
