/*
 * This file is part of Windburst Project
 */

#pragma once

#include "baro_driver.h"

// ============================================================================
// Bosch BMP280 (and BME280's pressure/temperature side), I2C 0x76/0x77 or SPI. Sequence and compensation follow Betaflight
// drivers/barometer/barometer_bmp280.c (= Bosch datasheet section 3.11.3, 32/64-bit integer version).
//   chip id 0xD0 (0x58 BMP280, 0x60 BME280, 0x56/0x57 early samples) -> soft reset 0xB6 @0xE0 -> wait status.im_update=0 ->
//   calibration 24 bytes @0x88 -> CONFIG=0 (IIR off; the Kalman filter does the smoothing, chip IIR only adds lag) ->
//   forced mode via CTRL_MEAS (osrs_t<<5 | osrs_p<<2 | 01), data 6 bytes @0xF7 (press[19:0], temp[19:0], MSB first).
// ============================================================================
class Bmp280Baro : public BaroDriver {
public:
    explicit Bmp280Baro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return chip_id_ == 0x60 ? "BME280" : "BMP280"; }

    bool init(const BaroInitConfig& cfg) override {
        osr_p_ = (uint8_t)((cfg.osr_p > 4 ? 4 : cfg.osr_p) + 1);      // register code: 1 = x1 ... 5 = x16 (0 would skip the measurement)
        osr_t_ = (uint8_t)((cfg.osr_t > 4 ? 4 : cfg.osr_t) + 1);
        ok_ = false; state_ = State::IDLE; errors_ = 0;
        uint8_t id = 0;
        if (!bus_.read_u8(REG_ID, id) || !(id == 0x58 || id == 0x60 || id == 0x56 || id == 0x57)) return false;
        chip_id_ = id;
        bus_.write(REG_RESET, 0xB6);
        bool ready = false;
        for (int i = 0; i < 20 && !ready; i++) { ag_delay_ms(2); uint8_t st; ready = bus_.read_u8(REG_STATUS, st) && !(st & 0x01); }
        if (!ready) return false;
        uint8_t c[24];
        if (!bus_.read(REG_CALIB, c, 24)) return false;
        bool all0 = true, allF = true; for (int i = 0; i < 24; i++) { if (c[i]) all0 = false; if (c[i] != 0xFF) allF = false; }
        if (all0 || allF) return false;                                       // dead bus / unprogrammed
        cal_.T1 = u16(c, 0); cal_.T2 = s16(c, 2); cal_.T3 = s16(c, 4);
        cal_.P1 = u16(c, 6); cal_.P2 = s16(c, 8); cal_.P3 = s16(c, 10); cal_.P4 = s16(c, 12); cal_.P5 = s16(c, 14);
        cal_.P6 = s16(c, 16); cal_.P7 = s16(c, 18); cal_.P8 = s16(c, 20); cal_.P9 = s16(c, 22);
        if (chip_id_ == 0x60) bus_.write(REG_CTRL_HUM, 0x00);                 // BME280: humidity skipped (must be written before CTRL_MEAS)
        bus_.write(REG_CONFIG, 0x00);                                         // standby n/a in forced mode, IIR off, no 3-wire SPI
        // conversion time (datasheet max): 1.25 + 2.3*Tosr + (2.3*Posr + 0.575) + 0.575 ms  (Betaflight uses a slightly larger formula)
        const float tosr = osr_t_ ? (float)(1 << (osr_t_ - 1)) : 0, posr = osr_p_ ? (float)(1 << (osr_p_ - 1)) : 0;
        conv_us_ = (uint32_t)((1.25f + 2.3f * tosr + (osr_p_ ? 2.3f * posr + 0.575f : 0.0f) + (osr_t_ ? 0.575f : 0.0f)) * 1000.0f) + 500;
        // self-check: one blocking conversion must give a plausible result
        BaroSample s;
        if (!trigger()) return false;
        ag_delay_us(conv_us_ + 2000);
        uint32_t guard = 0; uint8_t st = 0xFF;
        while (guard++ < 20 && (!bus_.read_u8(REG_STATUS, st) || (st & 0x08))) ag_delay_ms(2);
        if (!read_and_compensate(s) || !plausible(s)) return false;
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
            if (!bus_.read_u8(REG_STATUS, st)) { fail(); state_ = State::IDLE; return false; }
            if (st & 0x08) { if (dt > conv_us_ * 4 + 20000) { fail(); state_ = State::IDLE; } return false; }     // still measuring
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
    struct Cal { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; };
    static uint16_t u16(const uint8_t* b, int i) { return (uint16_t)(b[i] | (b[i + 1] << 8)); }
    static int16_t s16(const uint8_t* b, int i) { return (int16_t)u16(b, i); }
    bool trigger() { return bus_.write(REG_CTRL_MEAS, (uint8_t)((osr_t_ << 5) | (osr_p_ << 2) | 0x01)); }
    void fail() { if (errors_ < 255) errors_++; }
    static bool plausible(const BaroSample& s) { return s.pressure_pa > 30000.0f && s.pressure_pa < 125000.0f && s.temp_c > -40.0f && s.temp_c < 85.0f; }

    bool read_and_compensate(BaroSample& out) {
        uint8_t d[6];
        if (!bus_.read(REG_DATA, d, 6)) return false;
        const int32_t up = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
        const int32_t ut = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
        if (up == 0x80000 || ut == 0x80000) return false;                     // measurement skipped / not yet written
        // temperature (datasheet 3.11.3, 32-bit)
        int32_t var1 = (((ut >> 3) - ((int32_t)cal_.T1 << 1)) * (int32_t)cal_.T2) >> 11;
        int32_t var2 = (((((ut >> 4) - (int32_t)cal_.T1) * ((ut >> 4) - (int32_t)cal_.T1)) >> 12) * (int32_t)cal_.T3) >> 14;
        const int32_t t_fine = var1 + var2;
        const int32_t T = (t_fine * 5 + 128) >> 8;
        // pressure (64-bit), result Q24.8 Pa
        int64_t v1 = (int64_t)t_fine - 128000;
        int64_t v2 = v1 * v1 * (int64_t)cal_.P6;
        v2 = v2 + ((v1 * (int64_t)cal_.P5) << 17);
        v2 = v2 + (((int64_t)cal_.P4) << 35);
        v1 = ((v1 * v1 * (int64_t)cal_.P3) >> 8) + ((v1 * (int64_t)cal_.P2) << 12);
        v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)cal_.P1) >> 33;
        if (v1 == 0) return false;
        int64_t p = 1048576 - up;
        p = (((p << 31) - v2) * 3125) / v1;
        v1 = (((int64_t)cal_.P9) * (p >> 13) * (p >> 13)) >> 25;
        v2 = (((int64_t)cal_.P8) * p) >> 19;
        p = ((p + v1 + v2) >> 8) + (((int64_t)cal_.P7) << 4);
        out.pressure_pa = (float)p / 256.0f;
        out.temp_c = (float)T / 100.0f;
        return true;
    }

    static constexpr uint8_t REG_ID = 0xD0, REG_RESET = 0xE0, REG_CTRL_HUM = 0xF2, REG_STATUS = 0xF3, REG_CTRL_MEAS = 0xF4, REG_CONFIG = 0xF5, REG_DATA = 0xF7, REG_CALIB = 0x88;
    AgRegBus& bus_;
    Cal cal_{};
    uint8_t chip_id_ = 0x58, osr_p_ = 4, osr_t_ = 1, errors_ = 0;
    uint32_t conv_us_ = 25000, t0_ = 0;
    State state_ = State::IDLE;
    bool ok_ = false;
};
