/*
 * This file is part of Windburst Project
 */

#pragma once

#include "baro_driver.h"

// ============================================================================
// Bosch BMP388 / BMP390 (BMP3xx), I2C 0x76/0x77 or SPI. Sequence and integer compensation follow Betaflight barometer_bmp388.c
// (= Bosch BMP3-Sensor-API). Pressure is returned in Pa * 100 by that integer algorithm, so /100 here.
//   chip id @0x00 (0x50 BMP388, 0x60 BMP390) -> soft reset 0xB6 @0x7E -> calibration 21 bytes @0x31 -> OSR @0x1C (osr_t<<3 | osr_p) ->
//   forced mode: PWR_CTRL @0x1B = 0x13 (mode 01<<4 | temp_en | press_en) -> STATUS @0x03 drdy_press(bit5)/drdy_temp(bit6) ->
//   data 6 bytes @0x04: press[23:0] LSB first, temp[23:0] LSB first.   SPI reads return one dummy byte first.
// ============================================================================
class Bmp388Baro : public BaroDriver {
public:
    explicit Bmp388Baro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return chip_id_ == 0x60 ? "BMP390" : "BMP388"; }

    bool init(const BaroInitConfig& cfg) override {
        osr_p_ = cfg.osr_p > 5 ? 5 : cfg.osr_p;
        osr_t_ = cfg.osr_t > 5 ? 5 : cfg.osr_t;                                  // register code 0 = x1 ... 5 = x32
        ok_ = false; state_ = State::IDLE; errors_ = 0;
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
        cal_.T1 = u16(c, 0); cal_.T2 = u16(c, 2); cal_.T3 = (int8_t)c[4];
        cal_.P1 = (int16_t)u16(c, 5); cal_.P2 = (int16_t)u16(c, 7); cal_.P3 = (int8_t)c[9]; cal_.P4 = (int8_t)c[10];
        cal_.P5 = u16(c, 11); cal_.P6 = u16(c, 13); cal_.P7 = (int8_t)c[15]; cal_.P8 = (int8_t)c[16];
        cal_.P9 = (int16_t)u16(c, 17); cal_.P10 = (int8_t)c[19]; cal_.P11 = (int8_t)c[20];
        if (!bus_.write(REG_OSR, (uint8_t)((osr_t_ << 3) | osr_p_))) return false;
        bus_.write(REG_CONFIG, 0x00);                                           // IIR off (Kalman does the smoothing)
        // datasheet 3.9.2 conversion time: 234 + (392 + 2^osr_p * 2000) + (313 + 2^osr_t * 2000) us   (Betaflight doubles the osr terms; we poll the status bits anyway)
        conv_us_ = 234 + 392 + (1u << osr_p_) * 2000 + 313 + (1u << osr_t_) * 2000;
        BaroSample s;
        if (!trigger()) return false;
        ag_delay_us(conv_us_ + 2000);
        uint32_t guard = 0; uint8_t st = 0;
        while (guard++ < 20 && (!rd(REG_STATUS, &st, 1) || (st & 0x60) != 0x60)) ag_delay_ms(2);
        if (!read_and_compensate(s) || !plausible(s)) return false;
        uint8_t err = 0;
        if (rd(REG_ERR, &err, 1) && (err & 0x07)) return false;                 // fatal / command / configuration error flags
        ok_ = true;
        return true;
    }

    bool update(uint32_t now_us, BaroSample& out) override {
        if (!ok_) return false;
        switch (state_) {
        case State::IDLE:
            if (!trigger()) { fail(); return false; }
            t0_ = now_us; state_ = State::WAIT; return false;
        case State::WAIT: {
            const uint32_t dt = now_us - t0_;
            if (dt < conv_us_) return false;
            uint8_t st;
            if (!rd(REG_STATUS, &st, 1)) { fail(); state_ = State::IDLE; return false; }
            if ((st & 0x60) != 0x60) { if (dt > conv_us_ * 4 + 20000) { fail(); state_ = State::IDLE; } return false; }
            BaroSample s;
            state_ = State::IDLE;
            if (!read_and_compensate(s) || !plausible(s)) { fail(); return false; }
            errors_ = 0; out = s;
            return true;
        }
        }
        return false;
    }
    float nominal_rate_hz() const override { return 1e6f / (float)(conv_us_ + 1000); }
    bool healthy() const override { return ok_ && errors_ < 10; }

private:
    enum class State : uint8_t { IDLE, WAIT };
    struct Cal { uint16_t T1, T2; int8_t T3; int16_t P1, P2; int8_t P3, P4; uint16_t P5, P6; int8_t P7, P8; int16_t P9; int8_t P10, P11; };
    static uint16_t u16(const uint8_t* b, int i) { return (uint16_t)(b[i] | (b[i + 1] << 8)); }
    bool rd(uint8_t reg, uint8_t* buf, size_t len) {
        if (!bus_.is_spi()) return bus_.read(reg, buf, len);
        uint8_t tmp[32];                                                         // SPI: dummy byte first
        if (len + 1 > sizeof(tmp) || !bus_.read(reg, tmp, len + 1)) return false;
        for (size_t i = 0; i < len; i++) buf[i] = tmp[i + 1];
        return true;
    }
    bool trigger() { return bus_.write(REG_PWR_CTRL, (uint8_t)((0x01 << 4) | 0x03)); }
    void fail() { if (errors_ < 255) errors_++; }
    static bool plausible(const BaroSample& s) { return s.pressure_pa > 30000.0f && s.pressure_pa < 125000.0f && s.temp_c > -40.0f && s.temp_c < 85.0f; }

    bool read_and_compensate(BaroSample& out) {
        uint8_t d[6];
        if (!rd(REG_DATA, d, 6)) return false;
        const uint32_t up = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16);
        const uint32_t ut = (uint32_t)d[3] | ((uint32_t)d[4] << 8) | ((uint32_t)d[5] << 16);
        // --- temperature (Bosch BMP3 integer algorithm) ---
        uint64_t pd1 = ut - (256ull * cal_.T1);
        uint64_t pd2 = cal_.T2 * pd1;
        uint64_t pd3 = pd1 * pd1;
        int64_t pd4 = (int64_t)pd3 * cal_.T3;
        int64_t pd5 = (int64_t)(pd2 * 262144) + pd4;
        const int64_t t_lin = pd5 / 4294967296ll;
        const int64_t T = (t_lin * 25) / 16384;                                  // 0.01 degC
        // --- pressure ---
        int64_t p1 = t_lin * t_lin;
        int64_t p2 = p1 / 64;
        int64_t p3 = (p2 * t_lin) / 256;
        int64_t p4 = (cal_.P8 * p3) / 32;
        int64_t p5 = (cal_.P7 * p1) * 16;
        int64_t p6 = (cal_.P6 * t_lin) * 4194304;
        const int64_t offset = (cal_.P5 * 140737488355328ll) + p4 + p5 + p6;
        p2 = (cal_.P4 * p3) / 32;
        p4 = (cal_.P3 * p1) * 4;
        p5 = (cal_.P2 - 16384) * t_lin * 2097152;
        const int64_t sensitivity = ((cal_.P1 - 16384) * 70368744177664ll) + p2 + p4 + p5;
        p1 = (sensitivity / 16777216) * up;
        p2 = cal_.P10 * t_lin;
        p3 = p2 + (65536 * (int64_t)cal_.P9);
        p4 = (p3 * up) / 8192;
        p5 = (p4 * up) / 512;
        p6 = (int64_t)((uint64_t)up * (uint64_t)up);
        p2 = (cal_.P11 * p6) / 65536;
        p3 = (p2 * up) / 128;
        p4 = (offset / 4) + p1 + p5 + p3;
        const uint64_t comp = (((uint64_t)p4 * 25) / (uint64_t)1099511627776ull);   // Pa * 100
        out.pressure_pa = (float)((double)comp / 100.0);
        out.temp_c = (float)T / 100.0f;
        return true;
    }

    static constexpr uint8_t REG_CHIP_ID = 0x00, REG_ERR = 0x02, REG_STATUS = 0x03, REG_DATA = 0x04, REG_PWR_CTRL = 0x1B, REG_OSR = 0x1C, REG_CONFIG = 0x1F;
    static constexpr uint8_t REG_CALIB = 0x31, REG_CMD = 0x7E;
    AgRegBus& bus_;
    Cal cal_{};
    uint8_t chip_id_ = 0x50, osr_p_ = 3, osr_t_ = 0, errors_ = 0;
    uint32_t conv_us_ = 25000, t0_ = 0;
    State state_ = State::IDLE;
    bool ok_ = false;
};
