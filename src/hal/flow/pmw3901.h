/*
 * This file is part of Windburst Project
 */

#pragma once

#include "flow_driver.h"

// ============================================================================
// PixArt PMW3901 optical flow sensor on SPI. Register sequences and timing: ArduPilot AP_OpticalFlow_Pixart (the "3901" branch).
//
//   * the chip does not do ordinary SPI register access: address and data are separate transfers with fixed pauses between
//     them (write: 300 us before the value, 120 us after; read: 35 us before the value, 200 us after; burst: 150 us)
//   * identity: register 0x00 reads 0x49, register 0x5F its inverse 0xB6
//   * start-up: reset (0x3A = 0x5A), then two tables of "magic" register writes from PixArt (each write is read back and
//     repeated up to five times; some of these registers never read back what was written - that is not an error)
//   * motion: burst read of 12 bytes from 0x16: motion, observation, delta_x (int16), delta_y (int16), squal, raw sum, raw max,
//     raw min, shutter upper, shutter lower. The deltas are counts since the previous read; 1 count = 1.26e-3 rad.
//
// The sensor is polled every 10 ms (the deltas add up inside the chip, nothing is lost between two polls) and five polls make
// one reading (flow_driver.h, Accumulator); one poll keeps the bus for about 0.3 ms, which is why this chip gets an SPI bus
// of its own and is read in the sensor thread, never in the control cycle.
// Quality: the chip's SQUAL (number of features it tracks), taken as 0..100; 0 when the picture is too dark (shutter at its
// longest: Bitcraze's rule for this chip).
// ============================================================================
namespace flow {

// The four things the driver needs from the bus (ESP32: SPIClass + a GPIO; tests: a simulated chip).
class PixartPort {
public:
    virtual ~PixartPort() = default;
    virtual void select(bool on) = 0;                          // chip select (true = low)
    virtual uint8_t transfer(uint8_t v) = 0;
    virtual void delay_us(uint32_t us) = 0;
};

class Pmw3901 : public Driver {
public:
    static constexpr float kRadPerCount = 1.26e-3f;            // ArduPilot flow_pixel_scaling
    static constexpr uint32_t kPollUs = 10000;
    static constexpr uint8_t kId = 0x49, kIdInverse = 0xB6;

    explicit Pmw3901(PixartPort& port) : port_(port) {}
    const char* name() const override { return "PMW3901"; }
    float nominal_rate_hz() const override { return 1e6f / (kPollUs * 5); }
    uint32_t samples() const override { return polls_; }

    bool init() override {
        ok_ = false; have_prev_ = false; sum_.reset();
        write_reg(0x3A, 0x5A);                                 // power-up reset
        port_.delay_us(50000);
        id_ = read_reg(0x00); id_inv_ = read_reg(0x5F);
        if (id_ != kId || id_inv_ != kIdInverse) return false;
        load(kInit1, sizeof(kInit1) / sizeof(kInit1[0]));
        port_.delay_us(100000);
        load(kInit2, sizeof(kInit2) / sizeof(kInit2[0]));
        port_.delay_us(50000);
        uint8_t b[12]; burst(b);                               // empties the motion registers: the first real poll starts from zero
        ok_ = true;
        return true;
    }

    bool update(uint32_t now_us, Reading& out) override {
        if (!ok_) return false;
        if (have_prev_ && (uint32_t)(now_us - prev_us_) < kPollUs) return false;
        uint8_t b[12]; burst(b);
        const uint32_t dt_us = now_us - prev_us_;
        const bool first = !have_prev_;
        prev_us_ = now_us; have_prev_ = true;
        // a data line that hangs (broken wire, chip without power) reads all ones: that would be "one count backwards on both axes,
        // best quality" on every poll - a steady phantom speed. No real burst looks like that (the shutter's upper byte has 5 bits).
        bool dead = true; for (int i = 0; i < 12; i++) if (b[i] != 0xFF) { dead = false; break; }
        if (dead) { dead_bursts_++; sum_.reset(); return false; }
        if (first || dt_us == 0 || dt_us > 200000) { sum_.reset(); return false; }      // (after a pause the counts cover an unknown stretch of time)
        const int16_t dx = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8)), dy = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
        squal_ = b[6]; shutter_upper_ = b[10]; polls_++;
        sum_.add((float)dx * kRadPerCount, (float)dy * kRadPerCount, dt_us, shutter_upper_ == 0x1F ? (uint8_t)0 : (squal_ > 100 ? (uint8_t)100 : squal_));
        if (!sum_.ready()) return false;
        sum_.take(out); out.age_s = 0.0f;
        return true;
    }
    uint8_t chip_id() const { return id_; }
    uint8_t squal() const { return squal_; }
    uint32_t dead_bursts() const { return dead_bursts_; }      // polls that read nothing but ones

private:
    struct Reg { uint8_t reg, val; };
    void write_reg(uint8_t reg, uint8_t val) {
        port_.select(true); port_.transfer((uint8_t)(reg | 0x80)); port_.delay_us(300); port_.transfer(val); port_.select(false); port_.delay_us(120);
    }
    uint8_t read_reg(uint8_t reg) {
        port_.select(true); port_.transfer((uint8_t)(reg & 0x7F)); port_.delay_us(35); const uint8_t v = port_.transfer(0); port_.select(false); port_.delay_us(200);
        return v;
    }
    void load(const Reg* t, size_t n) {
        for (size_t i = 0; i < n; i++) {
            for (int tries = 0; tries < 5; tries++) { write_reg(t[i].reg, t[i].val); if (read_reg(t[i].reg) == t[i].val) break; }
        }
    }
    void burst(uint8_t b[12]) {
        port_.select(true); port_.transfer(0x16); port_.delay_us(150);
        for (int i = 0; i < 12; i++) b[i] = port_.transfer(0);
        port_.select(false);
    }
    static constexpr Reg kInit1[] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08},
        {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF}, {0x5F, 0x40}, {0x48, 0x80},
        {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80},
        {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08},
        {0x63, 0x78}, {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
        {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00}};
    static constexpr Reg kInit2[] = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0},
        {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80}};

    PixartPort& port_;
    bool ok_ = false, have_prev_ = false; uint32_t prev_us_ = 0;
    uint8_t id_ = 0, id_inv_ = 0, squal_ = 0, shutter_upper_ = 0; uint32_t dead_bursts_ = 0, polls_ = 0; Accumulator sum_;
};

} // namespace flow
