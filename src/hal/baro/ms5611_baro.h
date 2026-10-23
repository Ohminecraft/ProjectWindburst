/*
 * This file is part of Windburst Project
 */

#pragma once

#include "baro_driver.h"

// ============================================================================
// TE MS5611-01BA03, I2C 0x77 (CSB low) / 0x76 (CSB high) or SPI (mode 0/3, up to 20 MHz). A command chip, no registers: every
// transaction is one command byte, alone or followed by a read (AgRegBus::command / command_read). Sequence and maths from the
// datasheet and TE's AN520, as ArduPilot's AP_Baro_MS5611 and Betaflight's barometer_ms5611.c do them:
//   reset 0x1E (2.8 ms) -> PROM 0xA0 + 2 i, eight 16-bit words big-endian: word 0 factory data, 1..6 the coefficients C1..C6,
//   word 7 the CRC-4 in its low nibble (AN520's algorithm) -> no chip id, so the CRC and "not all 0 / all 1" are the detection.
//   conversion D1 (pressure) 0x48, D2 (temperature) 0x58, both OSR 4096 (9.04 ms max, 0.012 mbar rms - ArduPilot's choice too),
//   then ADC read 0x00: 24 bits big-endian. A read during or without a conversion gives 0 (rejected).
//   Temperature is converted before the first pressure and then every 4th sample (it moves slowly - ArduPilot does every 5th):
//   about 80 pressure samples a second.
//   Compensation: first and second order (datasheet figure "second order temperature compensation", below 20 C and -15 C).
// ============================================================================
namespace ms5611 {

struct Prom { uint16_t w[8]; };

// AN520's CRC-4 over the eight PROM words with the CRC nibble (word 7, low 4 bits) taken as 0.
inline uint8_t crc4(const Prom& p) {
    uint16_t rem = 0;
    for (int cnt = 0; cnt < 16; cnt++) {
        uint16_t w = p.w[cnt >> 1];
        if (cnt == 15) w &= 0xFF00;
        rem ^= (cnt & 1) ? (uint16_t)(w & 0x00FF) : (uint16_t)(w >> 8);
        for (int b = 8; b > 0; b--) rem = (rem & 0x8000) ? (uint16_t)((rem << 1) ^ 0x3000) : (uint16_t)(rem << 1);
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}
inline bool prom_ok(const Prom& p) {
    bool all0 = true, all1 = true;
    for (int i = 0; i < 8; i++) { if (p.w[i] != 0) all0 = false; if (p.w[i] != 0xFFFF) all1 = false; }
    return !all0 && !all1 && crc4(p) == (p.w[7] & 0x0F);
}
// Datasheet integer compensation; temp in 0.01 C, pressure in Pa.
inline void compensate(const Prom& p, uint32_t d1, uint32_t d2, int32_t& temp, int32_t& pres) {
    const int64_t C1 = p.w[1], C2 = p.w[2], C3 = p.w[3], C4 = p.w[4], C5 = p.w[5], C6 = p.w[6];
    const int64_t dT = (int64_t)d2 - (C5 << 8);
    int64_t t = 2000 + ((dT * C6) >> 23);
    int64_t off = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t sens = (C1 << 15) + ((C3 * dT) >> 8);
    if (t < 2000) {
        const int64_t t2 = (dT * dT) >> 31, d = t - 2000;
        int64_t off2 = 5 * d * d / 2, sens2 = 5 * d * d / 4;
        if (t < -1500) { const int64_t e = t + 1500; off2 += 7 * e * e; sens2 += 11 * e * e / 2; }
        t -= t2; off -= off2; sens -= sens2;
    }
    temp = (int32_t)t;
    pres = (int32_t)(((((int64_t)d1 * sens) >> 21) - off) >> 15);
}

} // namespace ms5611

class Ms5611Baro : public BaroDriver {
public:
    explicit Ms5611Baro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return "MS5611"; }

    // Reset and PROM: the detection (BaroSensor) and the start of init.
    bool probe() {
        if (!bus_.command(CMD_RESET)) return false;
        ag_delay_ms(3);
        for (int i = 0; i < 8; i++) {
            uint8_t b[2];
            if (!bus_.command_read((uint8_t)(CMD_PROM + 2 * i), b, 2)) return false;
            prom_.w[i] = (uint16_t)((b[0] << 8) | b[1]);
        }
        return ms5611::prom_ok(prom_);
    }
    bool init(const BaroInitConfig&) override {
        ok_ = false; state_ = State::IDLE; errors_ = 0; n_ = 0;
        if (!probe()) return false;
        // self-check: one blocking temperature + pressure pair must give a plausible result
        uint32_t d2 = 0, d1 = 0;
        if (!bus_.command(CMD_D2)) return false;
        ag_delay_us(kConvUs);
        if (!adc(d2) || !bus_.command(CMD_D1)) return false;
        ag_delay_us(kConvUs);
        if (!adc(d1)) return false;
        d2_ = d2;
        BaroSample s;
        if (!make(d1, s)) return false;
        ok_ = true;
        return true;
    }

    bool update(uint32_t now_us, BaroSample& out) override {
        if (!ok_) return false;
        switch (state_) {
        case State::IDLE:
            if (!bus_.command(CMD_D2)) { fail(); return false; }
            t0_ = now_us; state_ = State::WAIT_T; return false;
        case State::WAIT_T: {
            if (now_us - t0_ < kConvUs) return false;
            uint32_t d2;
            if (!adc(d2)) { fail(); state_ = State::IDLE; return false; }
            d2_ = d2;
            if (!bus_.command(CMD_D1)) { fail(); state_ = State::IDLE; return false; }
            t0_ = now_us; state_ = State::WAIT_P; return false;
        }
        case State::WAIT_P: {
            if (now_us - t0_ < kConvUs) return false;
            uint32_t d1; BaroSample s;
            const bool good = adc(d1) && make(d1, s);
            if (++n_ % kTempEvery == 0 || !good) state_ = State::IDLE;      // a new temperature (and after an error: start over)
            else if (bus_.command(CMD_D1)) t0_ = now_us;
            else { fail(); state_ = State::IDLE; return false; }
            if (!good) { fail(); return false; }
            errors_ = 0; out = s;
            return true;
        }
        }
        return false;
    }
    // kTempEvery pressure samples take kTempEvery + 1 conversions, each followed by up to 1 ms of scheduling
    float nominal_rate_hz() const override { return 1e6f * kTempEvery / ((float)(kConvUs + 1000) * (kTempEvery + 1)); }
    bool healthy() const override { return ok_ && errors_ < 10; }
    const ms5611::Prom& prom() const { return prom_; }

private:
    enum class State : uint8_t { IDLE, WAIT_T, WAIT_P };
    bool adc(uint32_t& v) {
        uint8_t b[3];
        if (!bus_.command_read(CMD_ADC, b, 3)) return false;
        v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
        return v != 0;                                                      // 0 = read too early / no conversion started
    }
    bool make(uint32_t d1, BaroSample& s) const {
        int32_t t, p;
        ms5611::compensate(prom_, d1, d2_, t, p);
        s.pressure_pa = (float)p; s.temp_c = (float)t / 100.0f;
        return s.pressure_pa > 30000.0f && s.pressure_pa < 125000.0f && s.temp_c > -40.0f && s.temp_c < 85.0f;
    }
    void fail() { if (errors_ < 255) errors_++; }

    static constexpr uint8_t CMD_RESET = 0x1E, CMD_PROM = 0xA0, CMD_D1 = 0x48, CMD_D2 = 0x58, CMD_ADC = 0x00;
    static constexpr uint32_t kConvUs = 9500;                               // OSR 4096: 9.04 ms max
    static constexpr uint32_t kTempEvery = 4;
    AgRegBus& bus_;
    ms5611::Prom prom_{};
    uint32_t d2_ = 0, t0_ = 0, n_ = 0;
    uint8_t errors_ = 0;
    State state_ = State::IDLE;
    bool ok_ = false;
};
