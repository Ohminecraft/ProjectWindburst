/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"
#include "bmi270_config.h"

// ============================================================================
// Bosch BMI270 (SPI). Sequence from Betaflight accgyro_spi_bmi270.c + Bosch's config-upload procedure.
//   CS pulse (I2C->SPI) -> soft reset (0xB6, 100 ms) -> CS pulse -> PWR_CONF=0 (advanced power save OFF) -> INIT_CTRL=0 ->
//   upload 8 KB microcode to INIT_DATA (chunked, INIT_ADDR_0/1 = word index) -> INIT_CTRL=1 -> INTERNAL_STATUS.message == 1 ->
//   ACC_CONF/RANGE, GYR_CONF/RANGE -> INT mapping -> PWR_CONF=0x02 -> PWR_CTRL=0x0E (acc+gyro+temp on).
// SPI register reads return one dummy byte first (the value is in byte 2) - handled in rd().
// Data little-endian: accel @0x0C, gyro @0x12 (contiguous). +-16 g => 2048 LSB/g; +-2000 dps => 16.384 LSB/dps.
// Gyro ODR max 3200 Hz (normal mode).
// ============================================================================
class Bmi270AccGyro : public AccGyroDriver {
public:
    Bmi270AccGyro(AgRegBus& bus) : bus_(bus) {}
    const char* name() const override { return "BMI270"; }

    bool init(const AgInitConfig& cfg) override {
        wake_spi();
        ag_delay_ms(10);
        uint8_t id = 0;
        if (!rd(CHIP_ID, &id, 1) || id != BMI270_WHO_AM_I) return false;

        bus_.write(CMD, 0xB6);                                   // soft reset
        ag_delay_ms(100);
        wake_spi();
        ag_delay_ms(10);
        if (!rd(CHIP_ID, &id, 1) || id != BMI270_WHO_AM_I) return false;

        if (!upload_config()) return false;

        // odr codes: gyro 0x0D=3200 0x0C=1600 0x0B=800 (BF writes 0x0D for 3200 Hz); acc 0x0B=800
        static const uint8_t osr[3] = {0x00 /*OSR4*/, 0x01 /*OSR2*/, 0x02 /*normal*/};
        uint8_t godr = 0x0D; gyro_hz_ = 3200.0f;
        if (cfg.gyro_odr_hz < 2400) { godr = 0x0C; gyro_hz_ = 1600.0f; }
        if (cfg.gyro_odr_hz < 1200) { godr = 0x0B; gyro_hz_ = 800.0f; }
        bus_.write(ACC_CONF, (uint8_t)((1 << 7) | (1 << 4) | 0x0B)); ag_delay_ms(1);   // high-perf | OSR2 | 800 Hz
        bus_.write(ACC_RANGE, 0x03); ag_delay_ms(1);                                    // +-16 g
        bus_.write(GYR_CONF, (uint8_t)((1 << 7) | (1 << 6) | (osr[cfg.hw_lpf > 2 ? 0 : cfg.hw_lpf] << 4) | godr)); ag_delay_ms(1);
        bus_.write(GYR_RANGE, 0x08); ag_delay_ms(1);                                    // 2000 dps (+ois_range bit, as BF)
        bus_.write(INT_MAP_DATA, 0x04); ag_delay_ms(1);                                 // DRDY -> INT1
        bus_.write(INT1_IO_CTRL, 0x0A); ag_delay_ms(1);                                 // push-pull, active high, output
        bus_.write(PWR_CONF, 0x02); ag_delay_ms(1);                                     // performance mode, FIFO self-wake
        bus_.write(PWR_CTRL, 0x0E); ag_delay_ms(1);                                     // gyro + acc + temp on
        ag_delay_ms(50);                                                                // gyro start-up

        AgSample s;
        return read(s);
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[12];
        if (!rd(ACC_DATA_X_LSB, b, 12)) return false;
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = le16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = le16(b + 6 + 2 * i) * (1.0f / 16.384f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return rd(STATUS, &v, 1) && (v & 0x40); }   // drdy_gyr
    float gyro_rate_hz() const override { return gyro_hz_; }
    float acc_rate_hz() const override { return 800.0f; }

private:
    bool upload_config() {
        bus_.write(PWR_CONF, 0x00); ag_delay_ms(1);              // advanced power save off (needs >= 450 us before init)
        bus_.write(INIT_CTRL, 0x00); ag_delay_ms(1);
        constexpr size_t kChunk = 128;                           // even, so word index = byte index / 2
        for (size_t i = 0; i < BMI270_CONFIG_SIZE; i += kChunk) {
            const uint16_t word = (uint16_t)(i / 2);
            bus_.write(INIT_ADDR_0, (uint8_t)(word & 0x0F));
            bus_.write(INIT_ADDR_1, (uint8_t)(word >> 4));
            if (!bus_.write_buf(INIT_DATA, bmi270_config_file + i, kChunk)) return false;
        }
        ag_delay_ms(10);
        bus_.write(INIT_CTRL, 0x01);                             // start the microcode
        for (int t = 0; t < 30; t++) {                           // Bosch: up to ~150 ms
            ag_delay_ms(10);
            uint8_t st = 0;
            if (rd(INTERNAL_STATUS, &st, 1) && (st & 0x0F) == 0x01) return true;   // message: init_ok
        }
        return false;
    }
    void wake_spi() { if (bus_.is_spi()) { uint8_t d; bus_.read_u8(CHIP_ID, d); } }   // CS edge switches the chip into SPI mode
    bool rd(uint8_t reg, uint8_t* buf, size_t len) {
        if (!bus_.is_spi()) return bus_.read(reg, buf, len);
        uint8_t tmp[16];                                         // len <= 12 here; 1 dummy byte first
        if (len + 1 > sizeof(tmp) || !bus_.read(reg, tmp, len + 1)) return false;
        for (size_t i = 0; i < len; i++) buf[i] = tmp[i + 1];
        return true;
    }

    static constexpr uint8_t CHIP_ID = 0x00, STATUS = 0x03, ACC_DATA_X_LSB = 0x0C, INTERNAL_STATUS = 0x21;
    static constexpr uint8_t ACC_CONF = 0x40, ACC_RANGE = 0x41, GYR_CONF = 0x42, GYR_RANGE = 0x43;
    static constexpr uint8_t INT1_IO_CTRL = 0x53, INT_MAP_DATA = 0x58, INIT_CTRL = 0x59, INIT_ADDR_0 = 0x5B, INIT_ADDR_1 = 0x5C;
    static constexpr uint8_t INIT_DATA = 0x5E, PWR_CONF = 0x7C, PWR_CTRL = 0x7D, CMD = 0x7E;
    AgRegBus& bus_;
    float gyro_hz_ = 3200.0f;
};
