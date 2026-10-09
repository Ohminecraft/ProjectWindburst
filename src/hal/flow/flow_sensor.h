/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include "flow_driver.h"
#include "pmw3901.h"
#include "flow_msp.h"
#include "../bus/bus_spi.h"
#include "../range/range_driver.h"

// ============================================================================
// FlowSensor (the "Position" tab): the optical flow sensor, whichever kind is fitted.
//
//   PMW3901   SPI, on the SECOND SPI bus of the chip (pins flow_cs / flow_sck / flow_miso / flow_mosi). A poll keeps the bus
//             for 0.3 ms; on the IMU's bus that would hold up a control cycle. The SD card in SPI mode needs the same bus:
//             both together are refused (put the card on SDMMC). Needs a separate lidar for the height.
//   MSP       UART (pin flow_rx): MicoAir MTF-01 / MTF-02, Matek 3901-L0X. Carries its own lidar: rangeDriver() hands the
//             distances to the RangeFinder.
//
//   flow.setType(...); flow.setSpiPins(...) / flow.setUart(Serial0, rx, tx, baud);
//   if (!flow.begin()) report(flow.failReason());
//   sensor thread:  if (flow.update(micros(), millis())) core.on_flow(...flow.reading()...);
//                   health.flow_ok = flow.healthy(millis());
// ============================================================================
class ArduinoPixartPort : public flow::PixartPort {
public:
    bool begin(uint8_t cs, uint8_t sck, uint8_t miso, uint8_t mosi) {
        cs_ = cs; pinMode(cs_, OUTPUT); digitalWrite(cs_, HIGH);
        return spi_.begin((int8_t)sck, (int8_t)miso, (int8_t)mosi, (int8_t)-1);
    }
    void select(bool on) override {
        if (on) { spi_.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3)); digitalWrite(cs_, LOW); }
        else { digitalWrite(cs_, HIGH); spi_.endTransaction(); }
    }
    uint8_t transfer(uint8_t v) override { return spi_.transfer(v); }
    void delay_us(uint32_t us) override { if (us >= 1000) delay(us / 1000); else delayMicroseconds(us); }      // (long pauses let other tasks run)
private:
    SPIClass spi_{(int)SPI_DEVICE_1}; uint8_t cs_ = 255;
};

class ArduinoBytePort : public flow::BytePort {
public:
    void attach(HardwareSerial& s) { s_ = &s; }
    int read() override { return s_ && s_->available() ? s_->read() : -1; }
private:
    HardwareSerial* s_ = nullptr;
};

// The lidar inside an MSP flow sensor, as a range driver.
class FlowRangeDriver : public RangeDriver {
public:
    static constexpr uint32_t kTimeoutUs = 500000;             // no distance for this long: not healthy (ArduPilot AP_RangeFinder_MSP)
    const char* name() const override { return "MSP lidar (flow sensor)"; }
    bool init() override { return true; }
    void push(const flow::RangeSample& r, uint32_t now_us) { r_ = r; last_us_ = now_us; seen_ = true; fresh_ = true; }
    bool update(uint32_t now_us, RangeReading& out) override {
        healthy_ = seen_ && (uint32_t)(now_us - last_us_) < kTimeoutUs;
        if (!fresh_) return false;
        fresh_ = false; out.distance_m = r_.distance_m; out.quality_pct = r_.quality_pct ? r_.quality_pct : (uint8_t)50;      // (some of these sensors send no quality for the range)
        return true;
    }
    float max_range_m() const override { return 8.0f; }       // MTF-01: 8 m (MTF-02: 2.5 m - it reports "out of range" itself beyond that)
    float min_range_m() const override { return 0.02f; }
    float nominal_rate_hz() const override { return 50.0f; }
    bool healthy() const override { return healthy_; }
private:
    flow::RangeSample r_; uint32_t last_us_ = 0; bool seen_ = false, fresh_ = false, healthy_ = false;
};

class FlowSensor {
public:
    enum Type : uint8_t { NONE = 0, PMW3901 = 1, MSP = 2 };
    static constexpr uint32_t kHealthyMs = 500;                // no reading for this long: not healthy

    void setType(uint8_t t) { type_ = t <= MSP ? (Type)t : NONE; }
    void setSpiPins(uint8_t cs, uint8_t sck, uint8_t miso, uint8_t mosi) { spi_pins_[0] = cs; spi_pins_[1] = sck; spi_pins_[2] = miso; spi_pins_[3] = mosi; }
    // uart: nullptr = none could be given to this sensor (uart_name says why in failReason)
    void setUart(HardwareSerial* uart, const char* uart_name, uint8_t rx, uint8_t tx, uint32_t baud) { uart_ = uart; uart_name_ = uart_name; rx_ = rx; tx_ = tx; baud_ = baud; }
    void setSpiBusTaken(bool taken) { spi_taken_ = taken; }    // the second SPI bus already belongs to the SD card
    // Test hook / other sensor types: any driver (init() is called here).
    bool useDriver(flow::Driver* d) { drv_ = d && d->init() ? d : nullptr; fail_ = drv_ ? "" : "the flow driver did not start"; return drv_ != nullptr; }

    bool begin() {
        drv_ = nullptr; fail_ = ""; msp_active_ = false; samples_ = 0; rate_ = 0;
        if (type_ == NONE) { fail_ = "no optical flow sensor selected"; return false; }
        if (type_ == PMW3901) {
            for (uint8_t p : spi_pins_) if (p == 255) { fail_ = "PMW3901: pins not set (flow_cs, flow_sck, flow_miso, flow_mosi)"; return false; }
            if (spi_taken_) { fail_ = "PMW3901 and the SD card in SPI mode need the same SPI bus: put the card on SDMMC, or use a UART flow sensor"; return false; }
            if (!pix_port_.begin(spi_pins_[0], spi_pins_[1], spi_pins_[2], spi_pins_[3])) { fail_ = "PMW3901: the second SPI bus did not start"; return false; }
            if (!pmw_.init()) { fail_ = "no PMW3901 answered (chip id not 0x49: wiring, 3.3 V supply, CS pin)"; return false; }
            drv_ = &pmw_;
            return true;
        }
        if (rx_ == 255) { fail_ = "flow sensor on a UART: pin flow_rx not set"; return false; }
        if (!uart_) { fail_ = "no UART left for the flow sensor: the GPS, the receiver and the app's link use all of them"; return false; }
        uart_->setRxBufferSize(512);
        uart_->begin(baud_, SERIAL_8N1, rx_, tx_ == 255 ? -1 : (int)tx_);
        byte_port_.attach(*uart_);
        msp_.init(); drv_ = &msp_; msp_active_ = true;
        return true;
    }

    // Returns true when a NEW reading is in reading().
    bool update(uint32_t now_us, uint32_t now_ms) {
        if ((uint32_t)(now_ms - rate_ms_) >= 1000u) {
            const uint32_t n = drv_ ? drv_->samples() : 0u, d = n - samples_;
            rate_ = d > 255u ? (uint8_t)255 : (uint8_t)d; samples_ = n; rate_ms_ = now_ms;
        }
        if (!drv_) return false;
        const bool got = drv_->update(now_us, reading_);
        if (msp_active_) { flow::RangeSample r; if (msp_.take_range(r)) range_.push(r, now_us); }
        if (!got) return false;
        last_ms_ = now_ms; seen_ = true;
        return true;
    }
    const flow::Reading& reading() const { return reading_; }
    bool available() const { return drv_ != nullptr; }
    bool healthy(uint32_t now_ms) const { return drv_ && seen_ && (uint32_t)(now_ms - last_ms_) <= kHealthyMs; }
    uint8_t rateHz() const { return rate_; }                   // what the sensor delivered in the last full second (frames / polls: about five make one reading)
    const char* name() const { return drv_ ? drv_->name() : "none"; }
    const char* uartName() const { return uart_name_; }
    const char* failReason() const { return fail_; }
    Type type() const { return type_; }
    // The sensor's own lidar (MSP only), for RangeFinder::useDriver(); nullptr when this sensor has none.
    RangeDriver* rangeDriver() { return msp_active_ ? &range_ : nullptr; }

private:
    ArduinoPixartPort pix_port_; flow::Pmw3901 pmw_{pix_port_};
    ArduinoBytePort byte_port_; flow::MspFlow msp_{byte_port_};
    FlowRangeDriver range_;
    flow::Driver* drv_ = nullptr; flow::Reading reading_;
    HardwareSerial* uart_ = nullptr; const char* uart_name_ = ""; const char* fail_ = "";
    Type type_ = NONE; uint8_t spi_pins_[4] = {255, 255, 255, 255}, rx_ = 255, tx_ = 255, rate_ = 0; uint32_t baud_ = 115200;
    bool spi_taken_ = false, msp_active_ = false, seen_ = false; uint32_t last_ms_ = 0, rate_ms_ = 0, samples_ = 0;
};
