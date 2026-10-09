/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include <stdint.h>
#include <stddef.h>
#include "bus_i2c.h"
#include "bus_spi.h"
#include "reg_bus.h"

// ---------------------------------------------------------------------------------------------------------------------
// Adapters: your I2CBus / SPIBus -> AgRegBus, so the chip drivers never see Arduino.
// ---------------------------------------------------------------------------------------------------------------------
class AgI2CRegBus : public AgRegBus {
public:
    // addr >= 0: several devices share this I2CBus - the address is selected before every transaction (single task only, no mutex here).
    explicit AgI2CRegBus(I2CBus& b, int addr = -1) : b_(&b), addr_(addr) {}
    void bind(I2CBus& b, int addr = -1) { b_ = &b; addr_ = addr; }
    bool read(uint8_t reg, uint8_t* buf, size_t len) override { sel(); return len <= 255 && b_->I2CReadRegister(reg, buf, (uint8_t)len); }
    bool write(uint8_t reg, uint8_t v) override { sel(); return b_->I2CWriteRegister(reg, v); }
    bool write_buf(uint8_t, const uint8_t*, size_t) override { return false; }   // no I2C chip in the list needs a burst write
    bool is_spi() const override { return false; }
private:
    void sel() { if (addr_ >= 0) b_->I2CSetAddress((uint8_t)addr_); }
    I2CBus* b_;
    int addr_;
};

class AgSpiRegBus : public AgRegBus {
public:
    explicit AgSpiRegBus(SPIBus& b) : b_(b) {}
    WB_FAST bool read(uint8_t reg, uint8_t* buf, size_t len) override { return len == 1 ? b_.SPIReadRegisterMask(reg, buf[0]) : b_.SPIReadRegisterMask(reg, buf, len); }
    bool write(uint8_t reg, uint8_t v) override { return b_.SPIWriteRegisterMask(reg, v); }
    bool write_buf(uint8_t reg, const uint8_t* data, size_t len) override {
        uint8_t tmp[1 + 160];
        if (len > 160) return false;
        tmp[0] = reg & 0x7F;
        for (size_t i = 0; i < len; i++) tmp[1 + i] = data[i];
        return b_.SPIWrite(tmp, len + 1);
    }
    bool is_spi() const override { return true; }
private:
    SPIBus& b_;
};

// Drivers hold a reference to this proxy for their whole life; IMUInit() points it at the I2C or the SPI adapter.
class AgBusProxy : public AgRegBus {
public:
    void set(AgRegBus* t) { t_ = t; }
    bool read(uint8_t reg, uint8_t* buf, size_t len) override { return t_ && t_->read(reg, buf, len); }
    bool write(uint8_t reg, uint8_t v) override { return t_ && t_->write(reg, v); }
    bool write_buf(uint8_t reg, const uint8_t* d, size_t len) override { return t_ && t_->write_buf(reg, d, len); }
    bool is_spi() const override { return t_ && t_->is_spi(); }
private:
    AgRegBus* t_ = nullptr;
};
