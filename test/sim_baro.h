// Register-level simulated BMP280 / BMP388. Raw ADC values are produced by inverting the datasheet's DOUBLE-precision compensation
// (independent of the drivers' integer maths), so a driver that decodes bits, scales or compensates wrongly cannot pass.
#pragma once
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "../src/hal/bus/reg_bus.h"

struct BaroSim : AgRegBus {
    bool spi = false, dead = false, stuck_busy = false, bad_cal = false;
    int busy = 0, busy_reads = 2;            // status polls that still report "measuring" after a trigger
    int triggers = 0, bad_triggers = 0, resets = 0;
    uint8_t wrong_id = 0;                    // nonzero: report this id instead
    virtual ~BaroSim() {}
    bool is_spi() const override { return spi; }
    virtual uint8_t spi_addr(uint8_t a7) const { return a7; }   // SPI carries 7 address bits; BMP280 registers (0x80..0xFF) are addressed without bit 7
    virtual uint8_t byte_at(uint8_t reg, int i) = 0;       // byte i of a read transaction that started at `reg`
    virtual void read_done(uint8_t reg, int n) = 0;        // side effects of a finished read (status/data-ready clearing ...)
    virtual void wr(uint8_t reg, uint8_t v) = 0;
    bool read(uint8_t reg, uint8_t* buf, size_t len) override {
        if (dead) return false;
        for (size_t i = 0; i < len; i++) buf[i] = byte_at(reg, (int)i);
        read_done(reg, (int)len); return true;
    }
    bool write(uint8_t reg, uint8_t v) override { if (dead) return false; wr(reg, v); return true; }
    bool write_buf(uint8_t reg, const uint8_t* d, size_t n) override { if (dead) return false; for (size_t i = 0; i < n; i++) wr((uint8_t)(reg + i), d[i]); return true; }
};

// ------------------------------------------------------------------------------------------------------------ BMP280
struct Bmp280Sim : BaroSim {
    // Bosch datasheet example calibration
    uint16_t T1 = 27504, P1 = 36477; int16_t T2 = 26435, T3 = -1000, P2 = -10685, P3 = 3024, P4 = 2855, P5 = 140, P6 = -7, P7 = 15500, P8 = -14600, P9 = 6000;
    uint8_t id = 0x58, ctrl_hum = 0, ctrl_meas = 0, config = 0;
    int32_t adc_p = 0x80000, adc_t = 0x80000;          // what the next conversion produces
    int32_t out_p = 0x80000, out_t = 0x80000;          // what the data registers hold
    bool fresh = false;
    uint8_t spi_addr(uint8_t a7) const override { return (uint8_t)(a7 | 0x80); }

    // datasheet double precision
    double comp_t(int32_t a, double* t_fine) const {
        double v1 = (a / 16384.0 - T1 / 1024.0) * T2;
        double v2 = (a / 131072.0 - T1 / 8192.0) * (a / 131072.0 - T1 / 8192.0) * T3;
        *t_fine = (double)(int32_t)(v1 + v2);
        return (v1 + v2) / 5120.0;
    }
    double comp_p(int32_t a, double t_fine) const {
        double v1 = t_fine / 2.0 - 64000.0;
        double v2 = v1 * v1 * P6 / 32768.0;
        v2 = v2 + v1 * P5 * 2.0;
        v2 = v2 / 4.0 + P4 * 65536.0;
        v1 = (P3 * v1 * v1 / 524288.0 + P2 * v1) / 524288.0;
        v1 = (1.0 + v1 / 32768.0) * P1;
        if (v1 == 0.0) return 0;
        double p = 1048576.0 - a;
        p = (p - v2 / 4096.0) * 6250.0 / v1;
        v1 = P9 * p * p / 2147483648.0;
        v2 = p * P8 / 32768.0;
        return p + (v1 + v2 + P7) / 16.0;
    }
    void set_env(double pa, double tc) {
        int32_t lo = 0, hi = (1 << 20) - 1;
        while (lo < hi) { int32_t m = (lo + hi) / 2; double tf; if (comp_t(m, &tf) < tc) lo = m + 1; else hi = m; }
        adc_t = lo; double tf; comp_t(adc_t, &tf);
        lo = 0; hi = (1 << 20) - 1;
        while (lo < hi) { int32_t m = (lo + hi) / 2; if (comp_p(m, tf) > pa) lo = m + 1; else hi = m; }   // pressure falls as adc rises
        adc_p = lo;
    }
    uint8_t cal_byte(int i) const {
        const int16_t v[12] = {(int16_t)T1, T2, T3, (int16_t)P1, P2, P3, P4, P5, P6, P7, P8, P9};
        if (bad_cal) return 0x00;
        const uint16_t w = (uint16_t)v[i / 2]; return (i & 1) ? (uint8_t)(w >> 8) : (uint8_t)w;
    }
    uint8_t reg_value(uint8_t r) const {
        if (r == 0xD0) return wrong_id ? wrong_id : id;
        if (r == 0xF3) return (busy > 0 || stuck_busy) ? 0x08 : 0x00;
        if (r == 0xF4) return ctrl_meas;
        if (r == 0xF5) return config;
        if (r == 0xF2) return ctrl_hum;
        if (r >= 0x88 && r < 0x88 + 24) return cal_byte(r - 0x88);
        if (r == 0xF7) return (uint8_t)(out_p >> 12);
        if (r == 0xF8) return (uint8_t)(out_p >> 4);
        if (r == 0xF9) return (uint8_t)((out_p & 0xF) << 4);
        if (r == 0xFA) return (uint8_t)(out_t >> 12);
        if (r == 0xFB) return (uint8_t)(out_t >> 4);
        if (r == 0xFC) return (uint8_t)((out_t & 0xF) << 4);
        return 0;
    }
    uint8_t byte_at(uint8_t reg, int i) override { return reg_value((uint8_t)(reg + i)); }
    void read_done(uint8_t reg, int n) override {
        for (int i = 0; i < n; i++) if ((uint8_t)(reg + i) == 0xF3 && busy > 0) busy--;
    }
    void wr(uint8_t r, uint8_t v) override {
        if (r == 0xE0 && v == 0xB6) { ctrl_meas = 0; config = 0; ctrl_hum = 0; out_p = out_t = 0x80000; busy = 0; resets++; return; }
        if (r == 0xF2) { ctrl_hum = v; return; }
        if (r == 0xF5) { config = v; return; }
        if (r == 0xF4) {
            ctrl_meas = v;
            const int mode = v & 3, osp = (v >> 2) & 7, ost = (v >> 5) & 7;
            if (mode == 1 || mode == 2) {
                if (osp || ost) { out_p = adc_p; out_t = adc_t; busy = busy_reads; triggers++; }
                else bad_triggers++;
                ctrl_meas = (uint8_t)(v & ~3);                  // forced mode returns to sleep
            }
        }
    }
};

// ------------------------------------------------------------------------------------------------------------ BMP388
struct Bmp388Sim : BaroSim {
    // raw NVM coefficients in the ranges of a real part
    uint16_t T1 = 27000, T2 = 19000; int8_t T3 = -10;
    int16_t P1 = 32270, P2 = 25500; int8_t P3 = 4, P4 = -1; uint16_t P5 = 2400, P6 = 1500; int8_t P7 = 3, P8 = -5;
    int16_t P9 = 500; int8_t P10 = 3, P11 = -3;
    uint8_t id = 0x50, osr = 0, pwr = 0, cfgreg = 0, err = 0, err_force = 0;
    uint32_t up_next = 0, ut_next = 0, up_out = 0, ut_out = 0;
    bool drdy = false, awake = true, fresh_frames = false;
    Bmp388Sim() {}
    // Bosch BMP3 floating point algorithm
    double par_t1() const { return T1 * 256.0; } double par_t2() const { return T2 / 1073741824.0; } double par_t3() const { return T3 / 281474976710656.0; }
    double par_p1() const { return (P1 - 16384) / 1048576.0; } double par_p2() const { return (P2 - 16384) / 536870912.0; }
    double par_p3() const { return P3 / 4294967296.0; } double par_p4() const { return P4 / 137438953472.0; }
    double par_p5() const { return P5 * 8.0; } double par_p6() const { return P6 / 64.0; } double par_p7() const { return P7 / 256.0; }
    double par_p8() const { return P8 / 32768.0; } double par_p9() const { return P9 / 281474976710656.0; }
    double par_p10() const { return P10 / 281474976710656.0; } double par_p11() const { return P11 / 36893488147419103232.0; }
    double comp_t(uint32_t ut) const { double p1 = (double)ut - par_t1(); return p1 * par_t2() + p1 * p1 * par_t3(); }
    double comp_p(uint32_t up, double tl) const {
        double d1 = par_p6() * tl, d2 = par_p7() * tl * tl, d3 = par_p8() * tl * tl * tl;
        double out1 = par_p5() + d1 + d2 + d3;
        d1 = par_p2() * tl; d2 = par_p3() * tl * tl; d3 = par_p4() * tl * tl * tl;
        double out2 = (double)up * (par_p1() + d1 + d2 + d3);
        d1 = (double)up * (double)up; d2 = par_p9() + par_p10() * tl; d3 = d1 * d2;
        double d4 = d3 + (double)up * (double)up * (double)up * par_p11();
        return out1 + out2 + d4;
    }
    void set_env(double pa, double tc) {
        uint32_t lo = 0, hi = (1u << 24) - 1;
        while (lo < hi) { uint32_t m = (lo + hi) / 2; if (comp_t(m) < tc) lo = m + 1; else hi = m; }
        ut_next = lo; double tl = comp_t(ut_next);
        lo = 0; hi = (1u << 24) - 1;
        while (lo < hi) { uint32_t m = (lo + hi) / 2; if (comp_p(m, tl) < pa) lo = m + 1; else hi = m; }
        up_next = lo;
    }
    uint8_t cal_byte(int i) const {
        uint8_t b[21];
        auto w16 = [&](int at, uint16_t v) { b[at] = (uint8_t)v; b[at + 1] = (uint8_t)(v >> 8); };
        w16(0, T1); w16(2, T2); b[4] = (uint8_t)T3; w16(5, (uint16_t)P1); w16(7, (uint16_t)P2); b[9] = (uint8_t)P3; b[10] = (uint8_t)P4;
        w16(11, P5); w16(13, P6); b[15] = (uint8_t)P7; b[16] = (uint8_t)P8; w16(17, (uint16_t)P9); b[19] = (uint8_t)P10; b[20] = (uint8_t)P11;
        return bad_cal ? 0 : b[i];
    }
    uint8_t reg_value(uint8_t r) const {
        if (r == 0x00) return wrong_id ? wrong_id : id;
        if (r == 0x02) return (uint8_t)(err | err_force);
        if (r == 0x03) return (uint8_t)(0x10 | ((drdy && busy == 0 && !stuck_busy) ? 0x60 : 0x00));
        if (r >= 0x04 && r <= 0x06) return (uint8_t)(up_out >> (8 * (r - 4)));
        if (r >= 0x07 && r <= 0x09) return (uint8_t)(ut_out >> (8 * (r - 7)));
        if (r == 0x1B) return pwr;
        if (r == 0x1C) return osr;
        if (r == 0x1F) return cfgreg;
        if (r >= 0x31 && r < 0x31 + 21) return cal_byte(r - 0x31);
        return 0;
    }
    uint8_t byte_at(uint8_t reg, int i) override {
        if (spi) { if (!awake) return 0xFF; if (i == 0) return 0x00; return reg_value((uint8_t)(reg + i - 1)); }
        return reg_value((uint8_t)(reg + i));
    }
    void read_done(uint8_t reg, int n) override {
        if (spi && !awake) { awake = true; return; }
        const int first = spi ? n - 1 : n;
        for (int i = 0; i < first; i++) {
            const uint8_t r = (uint8_t)(reg + i);
            if (r == 0x03 && busy > 0) busy--;
            if (r >= 0x04 && r <= 0x09) drdy = false;           // reading data clears data-ready
        }
    }
    void wr(uint8_t r, uint8_t v) override {
        if (spi && !awake) return;
        if (r == 0x7E && v == 0xB6) { pwr = 0; osr = 0; cfgreg = 0; drdy = false; busy = 0; err = 0; resets++; return; }
        if (r == 0x1C) { osr = v; return; }
        if (r == 0x1F) { cfgreg = v; return; }
        if (r == 0x1B) {
            pwr = v;
            const int mode = (v >> 4) & 3;
            if (mode == 1 || mode == 3) {
                if ((v & 3) == 3) { up_out = up_next; ut_out = ut_next; drdy = true; busy = busy_reads; triggers++; }
                else bad_triggers++;
                pwr = (uint8_t)(v & ~0x30);
            }
        }
    }
};
