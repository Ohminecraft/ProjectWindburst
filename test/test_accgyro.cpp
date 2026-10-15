// Register-level simulation of every IMU in IMUAccGyroDevice, driven through the real chip drivers.
// The simulated chip encodes its output with the scale implied by WHAT THE DRIVER WROTE to the range registers, applies each
// chip's own endianness / data-register layout, needs its own wake-up steps (sleep bit, SPI wake, sensor power-on, BMI270 microcode),
// and models bank switching / indirect registers.  So a wrong range code, byte order, axis order, bank or sequence fails here.
// Limit: the sim is written from the same datasheet/Betaflight knowledge as the drivers - it cannot catch a misreading shared by both.
// Real silicon is the final judge (see docs/IMU_BRINGUP.md).
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "../src/hal/acc_gyro/acc_gyro_regbus.h"
#include "../src/hal/acc_gyro/acc_gyro_types.h"
#include "../src/hal/acc_gyro/acc_gyro_util.h"
#include "../src/hal/acc_gyro/imu_select.h"
#include "../src/hal/acc_gyro/mpu6050_accgyro.h"
#include "../src/hal/acc_gyro/mpu6500_icm2060x_accgyro.h"
#include "../src/hal/acc_gyro/icm426xx_accgyro.h"
#include "../src/hal/acc_gyro/icm456xx_accgyro.h"
#include "../src/hal/acc_gyro/bmi160_accgyro.h"
#include "../src/hal/acc_gyro/bmi270_accgyro.h"
#include "sim_imu.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static bool near_(float a, float b, float rel = 0.02f, float ab = 0.06f) { return fabsf(a - b) <= ab + rel * fabsf(b); }
static bool sample_ok(const AgSample& s, const Sim& sim) {
    for (int i = 0; i < 3; i++) if (!near_(s.gyro_dps[i], sim.tg[i]) || !near_(s.acc_g[i], sim.ta[i], 0.02f, 0.01f)) return false;
    return true;
}
static void show(const char* what, const AgSample& s) {
    printf("    %s gyro %.2f %.2f %.2f  acc %.3f %.3f %.3f\n", what, s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2], s.acc_g[0], s.acc_g[1], s.acc_g[2]);
}

template <class D> static void run_driver(const char* label, Sim& sim, D& drv, float expect_gyro_hz, const char* expect_name, uint16_t req = 4000) {
    AgInitConfig cfg; cfg.gyro_odr_hz = req;
    bool ok = drv.init(cfg);
    CHECK(ok, "%s init failed", label);
    if (!ok) return;
    AgSample s{}; bool rd = drv.read(s);
    CHECK(rd && sample_ok(s, sim), "%s decoded values wrong", label);
    if (!(rd && sample_ok(s, sim))) show(label, s);
    CHECK(strcmp(drv.name(), expect_name) == 0, "%s name '%s'", label, drv.name());
    CHECK(fabsf(drv.gyro_rate_hz() - expect_gyro_hz) < 1.0f, "%s gyro rate %.0f want %.0f", label, drv.gyro_rate_hz(), expect_gyro_hz);
    CHECK(drv.data_ready(), "%s data_ready", label);
    sim.fail = true; AgSample keep = s; AgSample s2 = keep;
    CHECK(!drv.read(s2) && memcmp(&s2, &keep, sizeof(s)) == 0, "%s read must fail cleanly on bus error and leave the sample untouched", label);
    sim.fail = false;
}


// Two IMUs: imusel::Selector with scripted chips (each answers its own gyro x = 1 / 2 deg/s and acc x = 0.1 / 0.2 g, or not at all)
struct FakeChip { float id; bool ok = true; int reads = 0; bool operator()(AgSample& s) { reads++; if (!ok) return false; for (int k = 0; k < 3; k++) { s.gyro_dps[k] = id; s.acc_g[k] = id * 0.1f; } return true; } };
static void t_imu_select() {
    printf("[two IMUs] which chip is read, which one flies, a chip that fails\n");
    using namespace imusel;
    auto cycles = [](Selector& sel, FakeChip& a, FakeChip& b, int n, AgSample& out) { int got = 0; for (int i = 0; i < n; i++) got += sel.read(a, b, out); return got; };
    { Selector sel; sel.configure(FIRST, true, true); FakeChip a{1}, b{2}; AgSample o;
      const int got = cycles(sel, a, b, 80, o);
      CHECK(got == 80 && a.reads == 80 && b.reads == 10 && o.gyro_dps[0] == 1.0f && o.acc_g[0] == 0.1f && sel.gyro_src() == kImu1 && sel.fallbacks() == 0, "FIRST: IMU 1 flies, IMU 2 is read every 8th cycle (%d of 80)", b.reads);
      a.ok = false; const bool r = sel.read(a, b, o);
      CHECK(r && o.gyro_dps[0] == 2.0f && o.acc_g[0] == 0.2f && sel.gyro_src() == kImu2 && sel.acc_src() == kImu2 && sel.fallbacks() == 1, "IMU 1 misses a sample: that cycle comes from IMU 2 (gyro and accelerometer)");
      cycles(sel, a, b, 25, o); a.reads = 0; b.reads = 0; a.ok = true; cycles(sel, a, b, 50, o);
      CHECK(sel.lost(0) && a.reads == 0 && b.reads == 50 && o.gyro_dps[0] == 2.0f && sel.in_use() == kImu2, "20 in a row: IMU 1 is lost and no longer read even when it would answer again - IMU 2 flies (no back and forth)"); }
    { Selector sel; sel.configure(SECOND, true, true); FakeChip a{1}, b{2}; AgSample o; cycles(sel, a, b, 16, o);
      CHECK(a.reads == 16 && b.reads == 16 && o.gyro_dps[0] == 2.0f && o.acc_g[0] == 0.1f && sel.gyro_src() == kImu2 && sel.acc_src() == kImu1 && sel.in_use() == kImu2, "SECOND: both read, the gyro of IMU 2, the accelerometer of IMU 1 (the calibrated one)");
      b.ok = false; sel.read(a, b, o); CHECK(o.gyro_dps[0] == 1.0f && sel.gyro_src() == kImu1, "IMU 2 misses one: the gyro of IMU 1 in that cycle"); }
    { Selector sel; sel.configure(BOTH, true, true); FakeChip a{1}, b{2}; AgSample o; cycles(sel, a, b, 4, o);
      CHECK(o.gyro_dps[0] == 1.5f && o.acc_g[0] == 0.1f && sel.gyro_src() == (kImu1 | kImu2) && sel.in_use() == (kImu1 | kImu2), "BOTH: the mean of both gyros (%.2f), the accelerometer of IMU 1", o.gyro_dps[0]);
      b.ok = false; for (int i = 0; i < 25; i++) sel.read(a, b, o);
      CHECK(o.gyro_dps[0] == 1.0f && sel.lost(1) && sel.in_use() == kImu1, "IMU 2 dies: IMU 1 alone"); }
    { Selector sel; sel.configure(SECOND, true, false); FakeChip a{1}, b{2}; AgSample o; cycles(sel, a, b, 8, o);
      CHECK(b.reads == 0 && o.gyro_dps[0] == 1.0f && sel.in_use() == kImu1, "SECOND but IMU 2 was not found at boot: IMU 1 flies"); }
    { Selector sel; sel.configure(FIRST, true, false); FakeChip a{1}, b{2}; AgSample o; a.ok = false;
      const int got = cycles(sel, a, b, 30, o);
      CHECK(got == 0 && sel.lost(0) && sel.in_use() == 0, "one IMU that stops answering: no samples, nothing in use");
      a.ok = true; CHECK(sel.read(a, b, o) && !sel.lost(0) && sel.in_use() == kImu1, "it is tried again (nothing else is left) and comes back when it answers"); }
    { Selector sel; sel.configure(BOTH, true, true); FakeChip a{1}, b{2}; AgSample o; a.ok = b.ok = false;
      CHECK(cycles(sel, a, b, 25, o) == 0 && sel.lost(0) && sel.lost(1), "both silent: no sample (the flight loop disarms after 20)");
      b.ok = true; CHECK(sel.read(a, b, o) && o.gyro_dps[0] == 2.0f && !sel.lost(1), "both lost: both tried again, the first to answer flies"); }
}

int main() {
    printf("[drivers] init + decode through simulated silicon\n");
    { Sim s(C_MPU6050, false); Mpu6050AccGyro d(s); run_driver("MPU6050 I2C", s, d, 1000, "MPU6050"); }
    { Sim s(C_MPU6000, true);  Mpu6050AccGyro d(s); run_driver("MPU6000 SPI", s, d, 4000, "MPU6000");
      CHECK(s.r[0][0x1A] == 0 && s.r[0][0x19] == 1, "8kHz base / (1+1) = 4 kHz, DLPF off (CONFIG=%d DIV=%d)", s.r[0][0x1A], s.r[0][0x19]);
      CHECK(s.r[0][0x6A] == 0x10 && s.r[0][0x6B] == 0x03, "I2C disabled, PLL clock"); }
    { Sim s(C_MPU6000, true);  Mpu6050AccGyro d(s); run_driver("MPU6000 SPI 8k", s, d, 8000, "MPU6000", 8000); }
    { Sim s(C_MPU6500, false); Mpu6500Icm2060xAccGyro d(s, Mpu6500Icm2060xAccGyro::Variant::MPU6500); run_driver("MPU6500 I2C", s, d, 1000, "MPU6500"); }
    { Sim s(C_MPU6500, true);  Mpu6500Icm2060xAccGyro d(s, Mpu6500Icm2060xAccGyro::Variant::MPU6500); run_driver("MPU6500 SPI", s, d, 4000, "MPU6500"); }
    { Sim s(C_MPU9250, true);  Mpu6500Icm2060xAccGyro d(s, Mpu6500Icm2060xAccGyro::Variant::MPU9250); run_driver("MPU9250 SPI", s, d, 4000, "MPU9250"); }
    { Sim s(C_ICM20601, true); Mpu6500Icm2060xAccGyro d(s, Mpu6500Icm2060xAccGyro::Variant::ICM20601); run_driver("ICM20601 SPI", s, d, 4000, "ICM20601");
      CHECK(((s.r[0][0x1B] >> 3) & 3) == 2 && ((s.r[0][0x1C] >> 3) & 3) == 2, "ICM20601 uses its own FS codes for 2000 dps / 16 g"); }
    { Sim s(C_ICM20602, true); Mpu6500Icm2060xAccGyro d(s, Mpu6500Icm2060xAccGyro::Variant::ICM20602); run_driver("ICM20602 SPI", s, d, 4000, "ICM20602"); }
    { Sim s(C_ICM42688P, true); Icm426xxAccGyro d(s, Icm426xxAccGyro::Variant::ICM42688P); run_driver("ICM42688P", s, d, 4000, "ICM42688P");
      CHECK(s.bank == 0, "left in bank 0 (data window is bank 0 only)");
      CHECK(s.r[1][0x0C] == 6 && s.r[1][0x0D] == 36 && s.r[1][0x0E] == 0xA0, "gyro AAF 258 Hz in bank 1: %d %d 0x%02x", s.r[1][0x0C], s.r[1][0x0D], s.r[1][0x0E]);
      CHECK(s.r[2][0x03] == 12 && s.r[2][0x04] == 36 && s.r[2][0x05] == 0xA0, "accel AAF in bank 2");
      CHECK((s.r[0][0x4D] & 0xC0) == 0x40, "AFSR disabled"); CHECK(s.r[0][0x4E] == 0x0F, "sensors low-noise");
      CHECK(s.r[0][0x4F] == 0x04 && s.r[0][0x50] == 0x04, "FS_SEL 0, ODR code 4 (4 kHz)"); }
    { Sim s(C_ICM42688P, true); Icm426xxAccGyro d(s, Icm426xxAccGyro::Variant::ICM42688P); run_driver("ICM42688P 8k", s, d, 8000, "ICM42688P", 8000); CHECK(s.r[0][0x4F] == 0x03, "ODR code 3 = 8 kHz"); }
    { Sim s(C_ICM42622P, true); Icm426xxAccGyro d(s, Icm426xxAccGyro::Variant::ICM42622P); run_driver("ICM42622P", s, d, 4000, "ICM42622P"); }
    { Sim s(C_ICM42686P, true); Icm426xxAccGyro d(s, Icm426xxAccGyro::Variant::ICM42686P); run_driver("ICM42686P", s, d, 4000, "ICM42686P");
      CHECK((s.r[0][0x4F] >> 5) == 1, "42686P needs FS_SEL 1 for 2000 dps / 16 g (0 = 4000 dps / 32 g)"); }
    { Sim s(C_ICM45686, true); Icm456xxAccGyro d(s, Icm456xxAccGyro::Variant::ICM45686); run_driver("ICM45686", s, d, 3200, "ICM45686");
      CHECK(s.ireg[0xA57B] == 0x02 && (s.ireg[0xA4A6] & 0x60) == 0x40, "AAF+interpolator enabled via IREG");
      CHECK((s.ireg[0xA583] & 7) == 2 && (s.ireg[0xA4AC] & 7) == 4, "UI LPFs via IREG");
      CHECK(s.r[0][0x1C] == 0x14 && s.r[0][0x1B] == 0x15, "GYRO_CONFIG0 = 2000dps|3.2k, ACCEL_CONFIG0 = 16g|1.6k (0x%02x 0x%02x)", s.r[0][0x1C], s.r[0][0x1B]); }
    { Sim s(C_ICM45686, true); Icm456xxAccGyro d(s, Icm456xxAccGyro::Variant::ICM45686); run_driver("ICM45686 6.4k", s, d, 6400, "ICM45686", 8000); }
    { Sim s(C_ICM45605, true); Icm456xxAccGyro d(s, Icm456xxAccGyro::Variant::ICM45605); run_driver("ICM45605", s, d, 3200, "ICM45605"); }
    { Sim s(C_BMI160, true); Bmi160AccGyro d(s); run_driver("BMI160", s, d, 3200, "BMI160");
      CHECK(s.spi_mode, "SPI woken up"); CHECK(s.r[0][0x41] == 0x08 && s.r[0][0x43] == 0x00, "8 g / 2000 dps"); }
    { Sim s(C_BMI270, true); Bmi270AccGyro d(s); run_driver("BMI270", s, d, 3200, "BMI270");
      CHECK(s.init_ok, "microcode accepted (8192 bytes byte-exact at correct word addresses, power save off)");
      CHECK(s.ucode_hits == 8192 && !s.bad_upload, "upload hits %zu bad=%d", s.ucode_hits, (int)s.bad_upload);
      CHECK(s.r[0][0x7D] == 0x0E && s.r[0][0x7C] == 0x02, "sensors on, performance mode"); }

    printf("[drivers] wrong / dead chip is rejected\n");
    { Sim s(C_MPU6500, true); Bmi270AccGyro d(s); AgInitConfig c; CHECK(!d.init(c), "BMI270 driver must refuse an MPU6500"); }
    { Sim s(C_ICM42688P, true); Icm456xxAccGyro d(s, Icm456xxAccGyro::Variant::ICM45686); AgInitConfig c; CHECK(!d.init(c), "ICM456 driver must refuse an ICM42688P"); }
    { Sim s(C_BMI270, true); s.fail = true; Bmi270AccGyro d(s); AgInitConfig c; CHECK(!d.init(c), "dead bus"); }
    { Sim s(C_ICM42688P, true); Icm426xxAccGyro d(s, Icm426xxAccGyro::Variant::ICM42688P); AgInitConfig c; CHECK(d.init(c), "42688 ok"); }
    { Sim s(C_BMI270, true); s.ucode.assign(8192, 0); Bmi270AccGyro d(s); AgInitConfig c;
      // corrupt blob path: make the sim reject by pre-setting bad_upload
      s.bad_upload = true; CHECK(!d.init(c), "BMI270 must fail when the microcode is not accepted"); }

    printf("[detect] classify from raw ID reads (same probe order as IMUAccGyro::IMUInit)\n");
    struct Row { Chip chip; IMUAccGyroDevice want; };
    const Row rows[] = {{C_MPU6000, MPU6000_SPI}, {C_MPU6500, MPU6500_SPI}, {C_MPU9250, MPU9520_SPI}, {C_ICM20601, ICM20601_SPI},
                        {C_ICM20602, ICM20602_SPI}, {C_ICM42622P, ICM42622P_SPI}, {C_ICM42686P, ICM42686P_SPI}, {C_ICM42688P, ICM42688P_SPI},
                        {C_ICM45605, ICM45605_SPI}, {C_ICM45686, ICM45686_SPI}, {C_BMI160, BMI160_SPI}, {C_BMI270, BMI270_SPI}};
    for (const Row& row : rows) {
        Sim s(row.chip, true);
        uint8_t d2[2] = {0, 0}, r00 = 0, r72 = 0, r75 = 0;
        s.read(0x00, &r00, 1); s.read(0x00, d2, 2); s.read(0x00, &r00, 1); s.read(0x72, &r72, 1); s.read(0x75, &r75, 1);
        IMUAccGyroDevice got = accgyro::classify_spi(d2[1], r00, r72, r75);
        CHECK(got == row.want, "chip %d classified as %d, want %d", (int)row.chip, (int)got, (int)row.want);
    }
    CHECK(accgyro::classify_i2c(0x68) == MPU6050_I2C && accgyro::classify_i2c(0x70) == MPU6500_I2C && accgyro::classify_i2c(0x00) == NONE, "i2c ids");
    CHECK(accgyro::classify_spi(0xFF, 0xFF, 0xFF, 0xFF) == NONE && accgyro::classify_spi(0, 0, 0, 0) == NONE, "floating / dead MISO is NONE");

    printf("[align] Betaflight-compatible mounting\n");
    { float v[3] = {1, 2, 3}; accgyro::align_discrete(v, CW90_DEG); CHECK(v[0] == 2 && v[1] == -1 && v[2] == 3, "CW90 = (y,-x,z)"); }
    { float v[3] = {1, 2, 3}; accgyro::align_discrete(v, CW270_DEG_FLIP); CHECK(v[0] == -2 && v[1] == -1 && v[2] == -3, "CW270 flip"); }
    for (int k = 1; k <= 3; k++) {                                       // fine yaw 90/180/270 must equal the discrete CW rotations
        float m[3][3]; accgyro::build_rotation(m, 0, 0, 90.0f * k);
        float a[3] = {1, 2, 3}, b[3] = {1, 2, 3};
        accgyro::apply_rotation(a, m); accgyro::align_discrete(b, (AccGyroAlign)k);
        CHECK(near_(a[0], b[0], 0, 1e-4f) && near_(a[1], b[1], 0, 1e-4f) && near_(a[2], b[2], 0, 1e-4f), "yaw %d matches CW%d", 90 * k, 90 * k);
    }
    // ---- the eight orientations, from what they mean physically (Betaflight's sensor_align_e), not from the table:
    //      each case names where the chip's own X, Y and Z axes point on the craft (F = nose, R = right wing, D = down).
    { struct Mount { AccGyroAlign a; const char* name; int x[3], y[3], z[3]; };          // chip axis direction in (F, R, D)
      const Mount mounts[] = {
          {CW0_DEG,        "CW0: face up, chip X to the nose",                     { 1, 0, 0}, { 0,-1, 0}, { 0, 0,-1}},     // Y left, Z up
          {CW90_DEG,       "CW90: face up, turned 90 deg clockwise",               { 0, 1, 0}, { 1, 0, 0}, { 0, 0,-1}},     // X right, Y nose
          {CW180_DEG,      "CW180: face up, chip X to the tail",                   {-1, 0, 0}, { 0, 1, 0}, { 0, 0,-1}},
          {CW270_DEG,      "CW270: face up, chip X to the left",                   { 0,-1, 0}, {-1, 0, 0}, { 0, 0,-1}},
          {CW0_DEG_FLIP,   "CW0 FLIP: face down, chip X to the tail",              {-1, 0, 0}, { 0,-1, 0}, { 0, 0, 1}},     // turned over about the left-right axis
          {CW90_DEG_FLIP,  "CW90 FLIP: face down, chip X to the left... (Y nose)", { 0,-1, 0}, { 1, 0, 0}, { 0, 0, 1}},
          {CW180_DEG_FLIP, "CW180 FLIP: face down, chip X to the nose",            { 1, 0, 0}, { 0, 1, 0}, { 0, 0, 1}},
          {CW270_DEG_FLIP, "CW270 FLIP: face down, chip X to the right",           { 0, 1, 0}, {-1, 0, 0}, { 0, 0, 1}},
      };
      for (const Mount& mt : mounts) {
          // a right-handed chip: Z = X x Y (in a right-handed F-R-D frame)
          const int cz[3] = {mt.x[1] * mt.y[2] - mt.x[2] * mt.y[1], mt.x[2] * mt.y[0] - mt.x[0] * mt.y[2], mt.x[0] * mt.y[1] - mt.x[1] * mt.y[0]};
          const bool right_handed = cz[0] == mt.z[0] && cz[1] == mt.z[1] && cz[2] == mt.z[2];
          // a few physical vectors in the craft's frame; what the chip reads of each is its component along the chip's axes
          const float truth[4][3] = {{0, 0, -1}, {0.2f, -0.5f, 0.8f}, {30, 0, 0}, {0, 0, 40}};         // level specific force; something; roll right 30 deg/s; yaw right 40 deg/s
          bool ok = right_handed;
          for (const auto& t : truth) {
              float v[3] = {t[0] * mt.x[0] + t[1] * mt.x[1] + t[2] * mt.x[2], t[0] * mt.y[0] + t[1] * mt.y[1] + t[2] * mt.y[2], t[0] * mt.z[0] + t[1] * mt.z[1] + t[2] * mt.z[2]};
              accgyro::align_chip(v, mt.a);
              ok = ok && near_(v[0], t[0], 0, 1e-6f) && near_(v[1], t[1], 0, 1e-6f) && near_(v[2], t[2], 0, 1e-6f);
          }
          CHECK(ok, "%s", mt.name);
      }
      float rest[3] = {0, 0, 1}; accgyro::align_chip(rest, CW0_DEG);
      CHECK(rest[0] == 0 && rest[1] == 0 && rest[2] == -1, "a chip lying face up reads +1 g on its Z at rest: with CW0 that is 'level' (z down = -1 g), not 'on its back'");
      // the same orientation named in Betaflight's own frame: its table, then y and z negated
      float v[3] = {1, 2, 3}; accgyro::align_chip(v, CW90_DEG); CHECK(v[0] == 2 && v[1] == 1 && v[2] == -3, "align_chip = Betaflight's table, then (x, -y, -z)");
      float w[3] = {1, 2, 3}; accgyro::flu_to_frd(w); CHECK(w[0] == 1 && w[1] == -2 && w[2] == -3, "x forward / y left / z up -> x forward / y right / z down"); }
    { float m[3][3]; accgyro::build_rotation(m, 17, -23, 41); float v[3] = {0.3f, -0.8f, 0.5f};
      float n0 = sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); accgyro::apply_rotation(v, m);
      CHECK(near_(sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]), n0, 0, 1e-5f), "rotation preserves length"); }

    printf("[calibration] gyro bias\n");
    { double s[3] = {5000 * 1.0, 5000 * -2.0, 5000 * 0.5}, s2[3] = {5000 * 1.0, 5000 * 4.0, 5000 * 0.25}; float b[3];
      CHECK(accgyro::bias_from_sums(s, s2, 5000, 500, b) && near_(b[0], 1.0f) && near_(b[1], -2.0f) && near_(b[2], 0.5f), "still board -> bias"); }
    { double s[3] = {0, 0, 0}, s2[3] = {500 * 400.0, 0, 0}; float b[3] = {9, 9, 9};
      CHECK(!accgyro::bias_from_sums(s, s2, 500, 500, b), "moving board rejected"); }

    t_imu_select();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
