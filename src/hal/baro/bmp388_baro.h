/*
 * This file is part of Windburst Project
 */

#pragma once

#include "baro_driver.h"

// ============================================================================
// Bosch BMP388 / BMP390 (BMP3xx), I2C 0x76/0x77 or SPI.
//   chip id @0x00 (0x50 BMP388, 0x60 BMP390) -> soft reset 0xB6 @0x7E -> calibration 21 bytes @0x31 ->
//   OSR @0x1C (osr_t<<3 | osr_p) -> ODR @0x1D -> CONFIG @0x1F (IIR) ->
//   Normal mode: PWR_CTRL @0x1B = press_en | temp_en | mode(11<<4) -> chip free-runs at ODR ->
//   STATUS @0x03 drdy_press(bit5)/drdy_temp(bit6) -> data 6 bytes @0x04: press[23:0] LSB first, temp[23:0] LSB first.
//   SPI reads return one dummy byte first. Floating-point compensation per Bosch BMP3 datasheet (double precision).
// ============================================================================
class Bmp388Baro : public BaroDriver {
public:
    explicit Bmp388Baro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return chip_id_ == 0x60 ? "BMP390" : "BMP388"; }

    bool init(const BaroInitConfig& cfg) override {
        osr_p_ = cfg.osr_p > 5 ? 5 : cfg.osr_p;
        osr_t_ = cfg.osr_t > 5 ? 5 : cfg.osr_t;                                  // register code 0 = x1 ... 5 = x32
        ok_ = false; errors_ = 0; have_sample_ = false; started_ = false;
        if (bus_.is_spi()) { uint8_t dummy; rd(REG_CHIP_ID, &dummy, 1); ag_delay_ms(1); }   // first CS edge switches the chip into SPI mode
        uint8_t id = 0;
        if (!rd(REG_CHIP_ID, &id, 1) || !(id == 0x50 || id == 0x60)) return false;
        chip_id_ = id;
        bus_.write(REG_CMD, 0xB6);                                              // soft reset
        ag_delay_ms(10);
        if (bus_.is_spi()) { uint8_t dummy; rd(REG_CHIP_ID, &dummy, 1); ag_delay_ms(1); }
        uint8_t c[21];
        if (!rd(REG_CALIB, c, 21)) return false;
        bool all0 = true, allF = true; for (int i = 0; i < 21; i++) { if (c[i]) all0 = false; if (c[i] != 0xFF) allF = false; }
        if (all0 || allF) return false;
        uint16_t t1_raw = u16(c, 0);
        uint16_t t2_raw = u16(c, 2);
        int8_t t3_raw = (int8_t)c[4];
        int16_t p1_raw = (int16_t)u16(c, 5);
        int16_t p2_raw = (int16_t)u16(c, 7);
        int8_t p3_raw = (int8_t)c[9];
        int8_t p4_raw = (int8_t)c[10];
        uint16_t p5_raw = u16(c, 11);
        uint16_t p6_raw = u16(c, 13);
        int8_t p7_raw = (int8_t)c[15];
        int8_t p8_raw = (int8_t)c[16];
        int16_t p9_raw = (int16_t)u16(c, 17);
        int8_t p10_raw = (int8_t)c[19];
        int8_t p11_raw = (int8_t)c[20];
        cal_.T1  = (double)t1_raw  / 0.00390625;                 // 2^-8
        cal_.T2  = (double)t2_raw  / 1073741824.0;                // 2^-30
        cal_.T3  = (double)t3_raw  / 281474976710656.0;           // 2^-48

        cal_.P1  = (double)(p1_raw - 16384) / 1048576.0;          // 2^-14, 2^20
        cal_.P2  = (double)(p2_raw - 16384) / 536870912.0;        // 2^-29
        cal_.P3  = (double)p3_raw  / 4294967296.0;                 // 2^-32
        cal_.P4  = (double)p4_raw  / 137438953472.0;               // 2^-37
        cal_.P5  = (double)p5_raw  / 0.125;                        // 2^-3
        cal_.P6  = (double)p6_raw  / 64.0;                         // 2^-6
        cal_.P7  = (double)p7_raw  / 256.0;                        // 2^-8
        cal_.P8  = (double)p8_raw  / 32768.0;                      // 2^-15
        cal_.P9  = (double)p9_raw  / 281474976710656.0;            // 2^-48
        cal_.P10 = (double)p10_raw / 281474976710656.0;            // 2^-48
        cal_.P11 = (double)p11_raw / 36893488147419103232.0;       // 2^-65

        if (!bus_.write(REG_OSR, (uint8_t)((osr_t_ << 3) | osr_p_))) return false;
        if (!bus_.write(REG_ODR, ODR_CODE)) return false;                        // ODR 50Hz
        if (!bus_.write(REG_CONFIG, (uint8_t)(0x02 << 1))) return false;         // IIR coeff 3

        if (!bus_.write(REG_PWR_CTRL, PWR_NORMAL)) return false;                 // enter Normal mode: chip free-runs from here on
        t0_us_ = 0; last_read_us_ = 0;

        // wait for the first sample to confirm the sensor is actually producing data
        BaroSample s;
        uint32_t guard = 0; uint8_t st = 0;
        while (guard++ < 40 && (!rd(REG_STATUS, &st, 1) || (st & 0x60) != 0x60)) ag_delay_ms(2);
        if ((st & 0x60) != 0x60) return false;
        if (!read_and_compensate(s) || !plausible(s)) return false;

        uint8_t err = 0;
        if (rd(REG_ERR, &err, 1) && (err & 0x07)) return false;                  // fatal / command / conf_err - the chip flags an OSR too slow for the ODR
        ok_ = true;
        return true;
    }

    // Normal mode: the chip free-runs at ODR_PERIOD_US. update() just polls STATUS and
    // picks up a new sample whenever drdy is set -- it never re-triggers a measurement.
    bool update(uint32_t now_us, BaroSample& out) override {
        if (!ok_) return false;
        if (!started_) { started_ = true; last_ok_us_ = now_us; }
        if (have_sample_ && (now_us - last_read_us_) < (ODR_PERIOD_US / 2)) return false;  // cheap throttle, avoid hammering the bus
        uint8_t st;
        if (!rd(REG_STATUS, &st, 1)) { fail(); return false; }
        if ((st & 0x60) != 0x60) {                                              // no new press+temp sample yet
            // a chip that stopped converting (brown-out reset into sleep mode, a stuck conversion): one error per sample period
            // after 5 silent periods, so healthy() goes false instead of the last value being held for ever
            if ((uint32_t)(now_us - last_ok_us_) > 5 * ODR_PERIOD_US) { fail(); last_ok_us_ = now_us - 4 * ODR_PERIOD_US; }
            return false;
        }
        BaroSample s;
        if (!read_and_compensate(s) || !plausible(s)) { fail(); return false; }
        errors_ = 0; have_sample_ = true; last_read_us_ = now_us; last_ok_us_ = now_us;
        out = s;
        return true;
    }
    float nominal_rate_hz() const override { return 1e6f / (float)ODR_PERIOD_US; }
    bool healthy() const override { return ok_ && errors_ < 10; }

private:
    struct Cal { double T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11; };
    static uint16_t u16(const uint8_t* b, int i) { return (uint16_t)(b[i] | (b[i + 1] << 8)); }
    bool rd(uint8_t reg, uint8_t* buf, size_t len) {
        if (!bus_.is_spi()) return bus_.read(reg, buf, len);
        uint8_t tmp[32];                                                         // SPI: dummy byte first
        if (len + 1 > sizeof(tmp) || !bus_.read(reg, tmp, len + 1)) return false;
        for (size_t i = 0; i < len; i++) buf[i] = tmp[i + 1];
        return true;
    }
    void fail() { if (errors_ < 255) errors_++; }
    static bool plausible(const BaroSample& s) { return s.pressure_pa > 30000.0f && s.pressure_pa < 125000.0f && s.temp_c > -40.0f && s.temp_c < 85.0f; }

    bool read_and_compensate(BaroSample& out) {
        uint8_t d[6];
        if (!rd(REG_DATA, d, 6)) return false;
        const uint32_t up = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
        const uint32_t ut = (uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16);

        // --- temperature ---
        const double pd1 = (double)ut - cal_.T1;
        const double pd2 = cal_.T2 * pd1;
        const double t_lin = pd2 + (pd1 * pd1) * cal_.T3;                        // degC

        // --- pressure ---
        double p1 = t_lin * cal_.P6;
        double p2 = t_lin * t_lin * cal_.P7;
        double p3 = t_lin * t_lin * t_lin * cal_.P8;
        const double o1 = cal_.P5 + p1 + p2 + p3;

        p1 = t_lin * cal_.P2;
        p2 = t_lin * t_lin * cal_.P3;
        p3 = t_lin * t_lin * t_lin * cal_.P4;
        const double o2 = (double)up * (cal_.P1 + p1 + p2 + p3);

        p1 = (double)up * (double)up;
        p2 = cal_.P9 + t_lin * cal_.P10;
        p3 = p1 * p2;
        const double o3 = p3 + ((double)up * (double)up * (double)up) * cal_.P11;

        out.pressure_pa = (float)(o1 + o2 + o3);
        out.temp_c = (float)t_lin;
        return true;
    }

    static constexpr uint8_t REG_CHIP_ID = 0x00, REG_ERR = 0x02, REG_STATUS = 0x03, REG_DATA = 0x04,
                              REG_PWR_CTRL = 0x1B, REG_OSR = 0x1C, REG_ODR = 0x1D, REG_CONFIG = 0x1F;
    static constexpr uint8_t REG_CALIB = 0x31, REG_CMD = 0x7E;
    static constexpr uint8_t ODR_CODE = 0x02;                                    // 50 Hz (table 3.4 in datasheet)
    static constexpr uint32_t ODR_PERIOD_US = 20000;                             // must match ODR_CODE above
    static constexpr uint8_t PWR_NORMAL = (uint8_t)((1U << 0) | (1U << 1) | (3U << 4)); // press_en | temp_en | mode=normal

    AgRegBus& bus_;
    Cal cal_{};
    uint8_t chip_id_ = 0x50, osr_p_ = 3, osr_t_ = 0, errors_ = 0;
    uint32_t t0_us_ = 0, last_read_us_ = 0, last_ok_us_ = 0;
    bool have_sample_ = false, started_ = false;
    bool ok_ = false;
};