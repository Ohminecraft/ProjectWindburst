/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../../config/fast_code.h"

#include "acc_gyro_regbus.h"
#include "acc_gyro_types.h"

// ============================================================================
// TDK ICM-42688-P / ICM-42686-P / ICM-42622-P / ICM-42605 (SPI). Sequence and values from Betaflight accgyro_spi_icm426xx.c.
//   soft reset -> WHO_AM_I -> sensors off -> gyro AAF (bank 1) + accel AAF (bank 2) -> UI filter low-latency -> INT1 (DRDY) ->
//   AFSR disable -> sensors low-noise -> GYRO_CONFIG0 / ACCEL_CONFIG0 = (FS_SEL<<5)|ODR.
// FS_SEL 0 = +-2000 dps / +-16 g on 42688/42622/42605; ICM-42686-P needs FS_SEL 1 for that range (0 = +-4000 dps / +-32 g).
// Scale: 16.384 LSB/dps, 2048 LSB/g. Data big-endian: accel @0x1F, gyro @0x25 (contiguous, one 12-byte burst).
// ============================================================================
class Icm426xxAccGyro : public AccGyroDriver {
public:
    enum class Variant : uint8_t { ICM42605, ICM42622P, ICM42686P, ICM42688P };
    Icm426xxAccGyro(AgRegBus& bus, Variant v) : bus_(bus), v_(v) {}

    const char* name() const override {
        switch (v_) { case Variant::ICM42605: return "ICM42605"; case Variant::ICM42622P: return "ICM42622P";
                      case Variant::ICM42686P: return "ICM42686P"; default: return "ICM42688P"; }
    }

    bool init(const AgInitConfig& cfg) override {
        ag_delay_ms(1);                                          // power-on time
        set_bank(0);
        bus_.write(DEVICE_CONFIG, 0x01);                         // soft reset
        ag_delay_ms(10);                                         // datasheet >= 1 ms, BF 1 ms; generous
        bus_.write(PWR_MGMT0, 0x00);                             // gyro + accel off while configuring

        uint8_t who = 0; bool ok = false;
        for (int i = 0; i < 20 && !ok; i++) {
            ag_delay_ms(1);
            ok = bus_.read_u8(WHO_AM_I, who) && who == expected_who();
        }
        if (!ok) return false;

        // gyro anti-alias filter (bank 1) - hw_lpf 0/1/2 = 258/536/997 Hz
        const Aaf g = aaf(cfg.hw_lpf);
        set_bank(1);
        bus_.write(GYRO_CONFIG_STATIC3, g.delt);
        bus_.write(GYRO_CONFIG_STATIC4, (uint8_t)(g.delt_sqr & 0xFF));
        bus_.write(GYRO_CONFIG_STATIC5, (uint8_t)((g.delt_sqr >> 8) | (g.bitshift << 4)));
        // accel AAF fixed at 258 Hz (BF: sized for the ~1 kHz accel task)
        const Aaf a = aaf(0);
        set_bank(2);
        bus_.write(ACCEL_CONFIG_STATIC2, (uint8_t)(a.delt << 1));
        bus_.write(ACCEL_CONFIG_STATIC3, (uint8_t)(a.delt_sqr & 0xFF));
        bus_.write(ACCEL_CONFIG_STATIC4, (uint8_t)((a.delt_sqr >> 8) | (a.bitshift << 4)));

        set_bank(0);
        bus_.write(GYRO_ACCEL_CONFIG0, 0xFF);                    // UI filter BW: low latency for both
        bus_.write(INT_CONFIG, 0x03);                            // INT1 pulsed, push-pull, active high
        bus_.write(INT_CONFIG0, 0x00);                           // DRDY clears on status read
        bus_.write(INT_SOURCE0, 0x08);                           // UI DRDY -> INT1
        uint8_t v;
        if (bus_.read_u8(INT_CONFIG1, v)) {
            v &= (uint8_t)~(1u << 4);                            // datasheet: INT_ASYNC_RESET must be 0 for proper INT operation
            v |= (uint8_t)((1u << 6) | (1u << 5));               // TPULSE 8 us, TDEASSERT disabled
            bus_.write(INT_CONFIG1, v);
        }
        if (bus_.read_u8(INTF_CONFIG1, v)) bus_.write(INTF_CONFIG1, (uint8_t)((v & ~0xC0) | 0x40));  // AFSR off: avoids gyro output stalls

        bus_.write(PWR_MGMT0, 0x0F);                             // accel + gyro low-noise
        ag_delay_ms(1);

        uint8_t odr = 4; gyro_hz_ = 4000.0f;
        if (cfg.gyro_odr_hz >= 6000)      { odr = 3; gyro_hz_ = 8000.0f; }
        else if (cfg.gyro_odr_hz >= 3000) { odr = 4; gyro_hz_ = 4000.0f; }
        else if (cfg.gyro_odr_hz >= 1500) { odr = 5; gyro_hz_ = 2000.0f; }
        else                              { odr = 6; gyro_hz_ = 1000.0f; }
        const uint8_t fs = (v_ == Variant::ICM42686P) ? 1 : 0;
        bus_.write(GYRO_CONFIG0, (uint8_t)((fs << 5) | odr));
        ag_delay_ms(15);
        bus_.write(ACCEL_CONFIG0, (uint8_t)((fs << 5) | odr));
        ag_delay_ms(15);

        AgSample s;
        return read(s);
    }

    WB_FAST bool read(AgSample& out) override {
        uint8_t b[12];
        if (!bus_.read(ACCEL_DATA_X1, b, 12)) return false;      // accel X1..Z0 then gyro X1..Z0
        // -32768 on any axis = no valid data yet (Invensense v3, as Betaflight / ArduPilot treat it): drop the whole sample
        for (int i = 0; i < 6; i++) if (be16(b + 2 * i) == INT16_MIN) return false;
        for (int i = 0; i < 3; i++) {
            out.acc_g[i] = be16(b + 2 * i) * (1.0f / 2048.0f);
            out.gyro_dps[i] = be16(b + 6 + 2 * i) * (1.0f / 16.384f);
        }
        return true;
    }

    bool data_ready() override { uint8_t v; return bus_.read_u8(INT_STATUS, v) && (v & 0x08); }   // DATA_RDY_INT (clears on read)
    float gyro_rate_hz() const override { return gyro_hz_; }
    uint32_t max_spi_read_hz() const override { return 20000000; }         // datasheet: 24 MHz
    float acc_rate_hz() const override { return gyro_hz_; }

private:
    struct Aaf { uint8_t delt; uint16_t delt_sqr; uint8_t bitshift; };
    // Betaflight aafLUT42688 / aafLUT42605 (datasheet section 5.3)
    Aaf aaf(uint8_t lpf) const {
        static const Aaf t88[3] = {{6, 36, 10}, {12, 144, 8}, {21, 440, 6}};
        static const Aaf t05[3] = {{21, 440, 6}, {39, 1536, 4}, {63, 3968, 3}};
        return (v_ == Variant::ICM42605 ? t05 : t88)[lpf > 2 ? 0 : lpf];
    }
    uint8_t expected_who() const {
        switch (v_) { case Variant::ICM42605: return ICM42605_WHO_AM_I; case Variant::ICM42622P: return ICM42622P_WHO_AM_I;
                      case Variant::ICM42686P: return ICM42686P_WHO_AM_I; default: return ICM42688P_WHO_AM_I; }
    }
    void set_bank(uint8_t b) { bus_.write(REG_BANK_SEL, (uint8_t)(b & 7)); }

    static constexpr uint8_t DEVICE_CONFIG = 0x11, INT_CONFIG = 0x14, ACCEL_DATA_X1 = 0x1F, INT_STATUS = 0x2D;
    static constexpr uint8_t INTF_CONFIG1 = 0x4D, PWR_MGMT0 = 0x4E, GYRO_CONFIG0 = 0x4F, ACCEL_CONFIG0 = 0x50, GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr uint8_t INT_CONFIG0 = 0x63, INT_CONFIG1 = 0x64, INT_SOURCE0 = 0x65, REG_BANK_SEL = 0x76, WHO_AM_I = 0x75;
    static constexpr uint8_t GYRO_CONFIG_STATIC3 = 0x0C, GYRO_CONFIG_STATIC4 = 0x0D, GYRO_CONFIG_STATIC5 = 0x0E;      // bank 1
    static constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03, ACCEL_CONFIG_STATIC3 = 0x04, ACCEL_CONFIG_STATIC4 = 0x05;   // bank 2
    AgRegBus& bus_;
    Variant v_;
    float gyro_hz_ = 4000.0f;
};
