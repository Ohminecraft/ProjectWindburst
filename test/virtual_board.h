// Virtual flight-controller board for firmware-in-the-loop tests and the PC simulator.
//
// The REAL firmware body (FlightApp: drivers, receiver decoder, flight core, motor manager, app request handling) runs unmodified
// against register-level chip simulators and a rigid-body plant:
//
//      plant (physics) -> ICM-42688-P on SPI, BMP280 + QMC5883L on I2C, u-blox on UART1, CRSF receiver on UART2
//      FlightApp       -> DShot frames (captured by HostMotorHw) -> plant
//
// Only the ESP32 peripherals themselves (SPI/I2C/UART/RMT drivers, NVS, FreeRTOS) are replaced by test/arduino_stub.
// Include this header from exactly ONE translation unit per program (it defines the stub hooks).
#pragma once
#include <algorithm>
#include <deque>
#include <random>
#include <vector>
#include <math.h>
#include "sim_plant.h"
#include "sim_imu.h"
#include "sim_baro.h"
#include "sim_mag.h"
#include "sim_gps.h"
#include "sim_flow.h"
#include "../src/app/flight_app.h"

struct VirtualBoard;
static VirtualBoard* g_board = nullptr;
int g_spi_begin_pins[4];

struct VirtualBoard {
    // ---- the wiring of this imaginary board
    static constexpr uint8_t kCs = 10, kMiso = 13, kMosi = 11, kSck = 12, kSda = 8, kScl = 9, kGpsRx = 17, kGpsTx = 18, kRcRx = 16, kRcTx = 15;
    static constexpr uint8_t kMotorPin[4] = {4, 5, 6, 7};
    static constexpr uint8_t kBattAdc = 2, kBuzzer = 21;
    // optical flow: a PMW3901 on the second SPI bus, or an MSP sensor on a UART (44 / 43 are UART0's own pins on an ESP32-S3)
    static constexpr uint8_t kFlowCs = 38, kFlowSck = 47, kFlowMiso = 48, kFlowMosi = 14, kFlowRx = 44, kFlowTx = 43;
    static constexpr float kBattDivider = 11.0f;                    // 10k : 1k
    static constexpr double kLat0 = 47.3977419, kLon0 = 8.5455938;  // where this board stands when the plant is at its origin

    Plant plant;
    Sim imu{C_ICM42688P, true};
    Bmp280Sim baro;
    QmcSim mag;
    SimGps gps;
    FlowTruth flow_truth; Pmw3901Sim pmw; MspFlowSim mtf;          // what a downward camera sees; the two kinds of sensor that report it (fitted or not: has_pmw / mtf.present)
    bool has_pmw = false;
    bool has_imu = true, has_baro = true, has_mag = true, has_rc = true;

    // ---- battery: resting pack voltage, internal sag under load (V at full throttle on all motors); 0 V = running from USB only
    double batt_v = 16.0, batt_sag_v = 0.8, batt_noise_v = 0.01;
    // ---- buzzer: every change of the output with its time (ms), for checking the beep patterns
    struct BuzzEdge { uint32_t ms; bool on; };
    std::vector<BuzzEdge> buzz;
    bool buzzer_on() const { return g_stub_pwm[kBuzzer].attached ? (g_stub_pwm[kBuzzer].duty != 0 && g_stub_pwm[kBuzzer].duty != 1023) : g_stub_gpio[kBuzzer] != 0; }
    uint32_t buzz_ms(uint32_t from_ms, uint32_t to_ms) const {      // how long the buzzer sounded inside [from, to)
        uint32_t total = 0, last = from_ms; bool on = false;
        for (const BuzzEdge& e : buzz) { if (e.ms >= to_ms) break; if (e.ms <= from_ms) { on = e.on; continue; } if (on) total += e.ms - last; last = e.ms; on = e.on; }
        if (on) total += to_ms - last;
        return total;
    }

    // The IMU and the compass sit on this board the way Betaflight calls CW0: chip X to the nose, chip Y to the LEFT, chip Z UP.
    // The plant works in the firmware's frame (x forward, y right, z down), so what a chip reads is that with y and z negated.
    static constexpr float kChip[3] = {1.0f, -1.0f, -1.0f};

    // ---- sensor imperfections (what the calibrations have to find; offsets, scales and the gyro bias are the CHIP's, per chip axis)
    float acc_offset[3] = {0, 0, 0}, acc_scale[3] = {1, 1, 1};     // chip reading = true / scale + offset   (so calibrated = (raw - offset) * scale)
    float gyro_bias[3] = {0.8f, -0.5f, 0.3f};                       // deg/s
    float mount_roll_deg = 0, mount_pitch_deg = 0;                  // the board sits crooked in the frame
    float gyro_noise = 0.05f, acc_noise = 0.002f, baro_noise_pa = 1.5f, mag_noise = 0.002f;
    double mag_hard[3] = {0, 0, 0};                                 // hard-iron offset, gauss
    bool shaking = false;                                           // somebody is moving the craft by hand
    float vib_dps[3] = {0, 0, 0}, vib_hz[3] = {0, 0, 0};            // a vibration the gyro picks up on roll / pitch / yaw (a frame resonance, an unbalanced prop)
    double ground_pa = 101325.0, temp_c = 25.0;

    // ---- the pilot
    uint16_t rc[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    bool tx_on = true; uint8_t lq = 100;
    uint32_t rc_frames = 0;
    // what the receiver on this board speaks: CRSF on UART2 (default), a Spektrum satellite on UART2, or pulses on GPIO pins
    enum RcLink { RC_CRSF, RC_SPEKTRUM2048, RC_SPEKTRUM1024, RC_PPM, RC_PWM } rc_link = RC_CRSF;
    static constexpr uint8_t kPpmPin = 40;
    static constexpr uint8_t kPwmPin[8] = {39, 40, 41, 42, 1, 3, 45, 46};
    int ppm_channels = 8, pwm_channels = 8;                          // how many channels the PPM / PWM receiver puts out
    uint8_t pwm_dead = 0;                                            // PWM channels whose wire is broken (no pulses)
    uint32_t ppm_frame_us = 22500;
    bool negative_ppm = false;                                       // idle high, the pulses go low

    std::mt19937 rng{12345};
    uint64_t steps = 0;

    VirtualBoard() {
        g_board = this;
        gps.version = gps::UBX_M10; gps.module_baud = 115200; gps.truth.fix_after_ms = 3000;
        gps_uart.b = this; rc_uart.b = this; flow_uart.b = this; mtf.present = false;
        g_stub_spi2 = [](uint8_t v) -> uint8_t { return g_board && g_board->has_pmw ? g_board->pmw.xfer(v) : (uint8_t)0xFF; };
        for (auto& p : g_stub_pwm) p = StubPwm(); g_stub_pwm_free = 8; for (auto& g : g_stub_gpio) g = 0; for (auto& a : g_stub_adc_mv) a = 0; g_stub_adc_reads = 0; for (auto& i : g_stub_isr) i = StubIsr(); for (auto& l : g_stub_pin_in) l = 0;
        Serial0 = HardwareSerial(); Serial1 = HardwareSerial(); Serial2 = HardwareSerial();
        Serial1.dev = &gps_uart; Serial2.dev = &rc_uart; Serial0.dev = &flow_uart;
        update_sensors();
    }
    ~VirtualBoard() { if (g_board == this) { g_board = nullptr; g_stub_spi2 = nullptr; } Serial0.dev = nullptr; Serial1.dev = nullptr; Serial2.dev = nullptr; }
    // The MSP flow sensor's wire goes to this UART's RX pin (0: the default; 1 / 2: in place of the GPS / the receiver).
    // The GPS module and the receiver plugged into each other's UART (UART2 / UART1): what the ports table is for
    void swap_gps_rc_uarts() { Serial1.dev = &rc_uart; Serial2.dev = &gps_uart; }
    void wire_flow_uart(int n) { Serial0.dev = nullptr; if (n == 1) Serial1.dev = &flow_uart; else if (n == 2) Serial2.dev = &flow_uart; else Serial0.dev = &flow_uart; }

    // Pins of this board into a settings object (what the user would enter in the Pins tab).
    static void wire(wbp::Settings& s) {
        s.pins.imu_cs = kCs; s.pins.imu_miso = kMiso; s.pins.imu_mosi = kMosi; s.pins.imu_sck = kSck; s.pins.i2c_sda = kSda; s.pins.i2c_scl = kScl;
        s.pins.gps_rx = kGpsRx; s.pins.gps_tx = kGpsTx; s.pins.rx_rx = kRcRx; s.pins.rx_tx = kRcTx;
        for (int i = 0; i < 4; i++) s.pins.motor[i] = kMotorPin[i];
        s.pins.batt_adc = kBattAdc; s.pins.buzzer = kBuzzer;
        s.battery.divider = (uint16_t)(kBattDivider * 100.0f + 0.5f);
    }

    // One simulation step: time advances by dt_us, the firmware runs, the plant reacts to the motor outputs.
    void step(FlightApp& app, uint32_t dt_us = 250) {
        g_stub_us += dt_us; steps++;
        const uint32_t now_ms = millis();
        if (now_ms != last_ms_) { last_ms_ = now_ms; every_ms(now_ms); }
        pulse_frames(); deliver_edges();
        update_sensors();
        { HostMotorHw* esc = static_cast<HostMotorHw*>(app.motors.hw());                // bidirectional DShot: every ESC answers each frame with its motor speed
          if (esc && esc->bidir && esc_replies) for (int i = 0; i < plant.n && i < app.motors.cfg().count; i++) {
              if (esc_silent[i]) continue; const int slot = app.motors.cfg().order[i];
              // extended telemetry (once switched on by command 13): every 8th reply carries the ESC's current instead of its speed
              const bool edt_frame = (esc_edt_on & (1u << slot)) && steps % 8 == (uint32_t)i;
              esc->reply[slot] = edt_frame ? motor::gcr_encode((uint16_t)(0x0600 | (uint8_t)lroundf(esc_amps[i] < 0 ? 0 : esc_amps[i] > 255 ? 255 : esc_amps[i]))) : esc_gcr(motor_rpm(i), app.motors.cfg().pole_count);
              esc->has_reply[slot] = true; } }
        app.run();
        // motor outputs -> plant (DShot value 48..2047 = 0..100 %)
        float cmd[8] = {};
        HostMotorHw* hw = static_cast<HostMotorHw*>(app.motors.hw());
        if (hw) {                                                       // DShot commands (values 1..47) as the ESCs hear them
            uint8_t c = 0, slots = 0;
            if (hw->last.send) for (int k = 0; k < 8; k++) { const uint16_t v = hw->last.value[k]; if (v >= 1 && v <= 47) { c = (uint8_t)v; slots |= (uint8_t)(1u << k); } }
            if (c && !(last_cmd_ == c && last_slots_ == slots)) esc_cmds.push_back({now_ms, c, slots});
            // an ESC acts on command 13 / 14 (extended telemetry on / off) after hearing it 6 times with nothing else in between
            // but motor-stop frames (value 0: the gaps the firmware leaves between the repeats)
            if (hw->last.send) for (int k = 0; k < 8; k++) {
                const uint16_t v = hw->last.value[k];
                if (v == 0) continue;
                if (v == 13 || v == 14) { edt_run_[k] = edt_last_[k] == v ? (uint8_t)(edt_run_[k] < 255 ? edt_run_[k] + 1 : 255) : 1; edt_last_[k] = (uint8_t)v; }
                else { edt_run_[k] = 0; edt_last_[k] = 0; }
                if (edt_run_[k] >= 6) { if (v == 13) esc_edt_on |= (uint8_t)(1u << k); else esc_edt_on &= (uint8_t)~(1u << k); }
            }
            last_cmd_ = c; last_slots_ = slots;
        }
        if (hw) for (int i = 0; i < plant.n && i < app.motors.cfg().count; i++) {
            const uint16_t v = hw->last.value[app.motors.cfg().order[i]];
            cmd[i] = v < 48 ? 0.0f : (v - 48) / 1999.0f;
            motor_frac[i] = cmd[i];
        }
        plant.step(cmd, dt_us * 1e-6f);
    }
    // ---- two threads (test_threads): the plant stands still, and each thread refreshes only the devices its side of the
    //      firmware reads - the flight thread the IMU, the sensor thread everything else.
    void refresh_imu(std::mt19937& r) {
        std::normal_distribution<float> n(0.0f, 1.0f);
        const Vec3 a = plant.acc(), g = plant.gyro_dps();
        const float ab[3] = {a.x, a.y, a.z}, gb[3] = {g.x, g.y, g.z}; float ac[3], gc[3];
        mount(ab, ac); mount(gb, gc);
        for (int k = 0; k < 3; k++) { imu.ta[k] = kChip[k] * ac[k] / acc_scale[k] + acc_offset[k] + acc_noise * n(r); imu.tg[k] = kChip[k] * gc[k] + gyro_bias[k] + gyro_noise * n(r); }
    }
    void refresh_slow(uint32_t now_ms) {
        baro.set_env(ground_pa * pow(1.0 - plant.z / 44330.0, 5.255) + noise(baro_noise_pa), temp_c);
        const double B[3] = {0.30, 0.0, 0.38}; double bb[3]; plant.body_from_world(B, bb);
        for (int k = 0; k < 3; k++) mag.field[k] = kChip[k] * bb[k] + mag_hard[k] + noise(mag_noise);
        if (now_ms != last_ms_) { last_ms_ = now_ms; every_ms(now_ms); }
    }

    // The flash is busy: on the ESP32 neither core runs meanwhile. Time passes, the ESCs keep their last command, the firmware does nothing.
    void stall(uint32_t us) {
        stalled_us += us; if (us > longest_stall_us) longest_stall_us = us;
        while (us) { const uint32_t d = us > 250 ? 250 : us; g_stub_us += d; plant.step(motor_frac, d * 1e-6f); us -= d; }
    }
    uint64_t stalled_us = 0; uint32_t longest_stall_us = 0;
    // ESCs with bidirectional DShot. Rotor speed ~ sqrt(thrust); `rpm_at_full` at full thrust (a 5-inch prop on a 1750 kv motor: about 24000).
    float esc_amps[8] = {}; uint8_t esc_edt_on = 0;                 // current each ESC reports (A, extended telemetry); hardware slots that switched it on
    bool esc_replies = true, esc_silent[8] = {}; float rpm_at_full = 24000.0f;      // esc_silent: that one ESC's telemetry wire is broken
    float motor_rpm(int i) const { return rpm_at_full * sqrtf(fmaxf(plant.T[i] / (plant.Tmax * plant.gain[i]), 0.0f)); }
    static uint32_t esc_gcr(float rpm, int poles) {                 // the reply a real ESC sends: the eRPM period in microseconds, 9 bits of mantissa + 3 of exponent
        const float erpm = rpm * (float)(poles / 2); if (erpm < 100.0f) return motor::gcr_encode(0x0FFF);      // standing still
        uint32_t m = (uint32_t)lroundf(60e6f / erpm); int e = 0; while (m > 0x1FF) { m >>= 1; e++; }
        return motor::gcr_encode((uint16_t)((e << 9) | m));
    }
    struct EscCmd { uint32_t ms; uint8_t cmd, slots; };             // slots: bit per hardware slot that got the command in that frame
    std::vector<EscCmd> esc_cmds;
    // beacon tones (commands 1..5) sent to ALL of the first n motors at once inside [from, to): their times
    std::vector<uint32_t> tones(uint32_t from_ms, uint32_t to_ms, uint8_t tone, int n = 4) const {
        std::vector<uint32_t> t; for (const EscCmd& e : esc_cmds) if (e.ms >= from_ms && e.ms < to_ms && e.cmd == tone && e.slots == (uint8_t)((1u << n) - 1u)) t.push_back(e.ms); return t;
    }
    void run(FlightApp& app, double seconds, uint32_t dt_us = 250) { const uint64_t n = (uint64_t)(seconds * 1e6 / dt_us); for (uint64_t i = 0; i < n; i++) step(app, dt_us); }
    float motor_frac[8] = {};

    // craft resting on the ground in a given attitude (for the 6-face accelerometer calibration)
    void rest(float roll_right_deg, float nose_down_deg) { plant.set_tilt(roll_right_deg, nose_down_deg, 0); plant.w[0] = plant.w[1] = plant.w[2] = 0; plant.z = 0; plant.vz = 0; plant.ground = true; }

    // ---------------------------------------------------------------- bus hooks (called from the Arduino stubs)
    uint32_t cost_imu_us = 0;                                       // how long one IMU transaction takes on the SPI bus (0 = no time, the default)
    uint32_t cost_i2c_us = 0;                                       // ... and one read on the sensor I2C bus
    void cs(uint8_t pin, uint8_t val) {
        if (pin == kFlowCs) { pmw.cs(val == LOW); return; }
        if (pin != kCs) return;
        if (val == LOW) { t.active = true; t.idx = 0; g_stub_us += cost_imu_us; return; }
        if (t.active && !t.rd && t.idx >= 2 && has_imu) { const int n = t.idx - 1; if (n == 1) imu.write(t.reg, t.w[0]); else imu.write_buf(t.reg, t.w, (size_t)n); }
        t.active = false;
    }
    uint8_t spi(uint8_t v) {
        if (!t.active) return 0xFF;
        uint8_t out = 0xFF;
        if (t.idx == 0) { t.rd = (v & 0x80) != 0; t.reg = v & 0x7F; out = 0x00; }
        else if (t.rd) {
            if (t.idx == 1) { if (!has_imu || !imu.read(t.reg, t.cache, 16)) memset(t.cache, 0xFF, 16); }
            out = (t.idx - 1 < 16) ? t.cache[t.idx - 1] : 0x00;
        } else if (t.idx - 1 < (int)sizeof(t.w)) t.w[t.idx - 1] = v;
        t.idx++;
        return out;
    }
    AgRegBus* i2c_dev(uint8_t addr) { if (addr == 0x76 && has_baro) return &baro; if (addr == 0x0D && has_mag) return &mag; return nullptr; }
    int i2c_tx(uint8_t addr, const uint8_t* d, size_t n) {
        AgRegBus* dev = i2c_dev(addr); if (!dev || n == 0) return 2;         // NACK
        i2c_reg_ = d[0]; if (n == 2 && !dev->write(d[0], d[1])) return 2;
        return 0;
    }
    int i2c_rx(uint8_t addr, uint8_t* d, size_t n) { g_stub_us += cost_i2c_us; AgRegBus* dev = i2c_dev(addr); if (!dev || n > 64) return 0; return dev->read(i2c_reg_, d, n) ? (int)n : 0; }

private:
    struct GpsUart : StubUartDevice {
        VirtualBoard* b = nullptr;
        int dev_read() override { return b->gps.read(); }
        void dev_write(const uint8_t* d, size_t n) override { b->gps.write(d, n); }
        void dev_config(uint32_t baud, uint32_t, bool) override { b->gps.set_baud(baud); }
    } gps_uart;
    struct FlowUart : StubUartDevice {
        VirtualBoard* b = nullptr;
        int dev_read() override { return b->mtf.read(); }
        void dev_write(const uint8_t*, size_t) override {}
        void dev_config(uint32_t baud, uint32_t, bool) override { b->mtf.host_baud = baud; }
    } flow_uart;
    struct RcUart : StubUartDevice {
        VirtualBoard* b = nullptr; std::deque<uint8_t> q; uint32_t baud = 0, format = 0; bool inverted = false; std::vector<uint8_t> sent;     // sent: what the firmware wrote (telemetry)
        int dev_read() override { if (q.empty()) return -1; const uint8_t c = q.front(); q.pop_front(); return c; }
        int dev_pending() override { return (int)q.size(); }
        void dev_write(const uint8_t* d, size_t n) override { sent.insert(sent.end(), d, d + n); }
        void dev_config(uint32_t bd, uint32_t f, bool inv) override { baud = bd; format = f; inverted = inv; }
    } rc_uart;
public:
    int rc_corrupt = 0;                                   // this many of the next CRSF frames arrive with one byte damaged
    uint32_t rc_uart_baud() const { return rc_uart.baud; }
    std::vector<uint8_t>& rc_uart_sent() { return rc_uart.sent; }

private:
    float noise(float sigma) { std::normal_distribution<float> n(0.0f, sigma); return sigma > 0 ? n(rng) : 0.0f; }

    // chip = Rx(roll) * Ry(pitch) * body : an arbitrary but fixed small rotation (the tests only require that LEVEL_TRIM removes it)
    void mount(const float in[3], float out[3]) const {
        const float r = mount_roll_deg * 0.017453292f, p = mount_pitch_deg * 0.017453292f;
        const float x1 = cosf(p) * in[0] + sinf(p) * in[2], y1 = in[1], z1 = -sinf(p) * in[0] + cosf(p) * in[2];
        out[0] = x1; out[1] = cosf(r) * y1 - sinf(r) * z1; out[2] = sinf(r) * y1 + cosf(r) * z1;
    }
    void update_sensors() {
        const Vec3 a = plant.acc(), g = plant.gyro_dps();
        float ab[3] = {a.x, a.y, a.z}, gb[3] = {g.x, g.y, g.z}, ac[3], gc[3];
        if (shaking) { for (int k = 0; k < 3; k++) { ab[k] += noise(0.15f); gb[k] += noise(25.0f); } }
        for (int k = 0; k < 3; k++) if (vib_dps[k] > 0) gb[k] += vib_dps[k] * sinf(6.2831853f * vib_hz[k] * (float)(g_stub_us % 1000000u) * 1e-6f);
        mount(ab, ac); mount(gb, gc);
        for (int k = 0; k < 3; k++) { imu.ta[k] = kChip[k] * ac[k] / acc_scale[k] + acc_offset[k] + noise(acc_noise); imu.tg[k] = kChip[k] * gc[k] + gyro_bias[k] + noise(gyro_noise); }
        baro.set_env(ground_pa * pow(1.0 - plant.z / 44330.0, 5.255) + noise(baro_noise_pa), temp_c);
        const double B[3] = {0.30, 0.0, 0.38}; double bb[3]; plant.body_from_world(B, bb);              // earth field: north + down (northern hemisphere)
        for (int k = 0; k < 3; k++) mag.field[k] = kChip[k] * bb[k] + mag_hard[k] + noise(mag_noise);   // a real compass never repeats itself exactly (the driver treats a frozen output as a fault)
    }
    void every_ms(uint32_t now_ms) {
        if (now_ms % 5 == 0) mag.drdy = true;                                                           // 200 Hz compass
        { float load = 0; for (int i = 0; i < plant.n; i++) load += motor_frac[i]; load /= (float)(plant.n > 0 ? plant.n : 1);
          const double v = batt_v > 0.1 ? batt_v - batt_sag_v * load + noise((float)batt_noise_v) : 0.0;
          g_stub_adc_mv[kBattAdc] = v > 0 ? (uint32_t)lround(v / kBattDivider * 1000.0) : 0; }
        { const bool on = buzzer_on(); if (buzz.empty() ? on : buzz.back().on != on) buzz.push_back({now_ms, on}); }
        gps.truth.alt_m = 488.2 + plant.z; gps.truth.vd = -plant.vz;
        // the module sits on the craft: it goes where the plant goes (Betaflight's metres per degree, as the firmware's own maths uses)
        gps.truth.lat = kLat0 + plant.pn / 111319.5; gps.truth.lon = kLon0 + plant.pe / (111319.5 * cos(kLat0 * M_PI / 180.0));
        gps.truth.vn = plant.vn; gps.truth.ve = plant.ve;
        // the camera under the craft (rates in rad/s about the board's X / Y; the lidar next to it sees the same ground)
        flow_truth.push(plant.w[0], plant.w[1], plant.v_forward(), plant.v_right(), plant.z, plant.R22());
        { float fx, fy; flow_truth.mean(0, 1, fx, fy); pmw.integrate(fx, fy, 0.001f); }
        mtf.tick(now_ms, flow_truth);
        gps.tick(now_ms);
        if (has_rc && tx_on && rc_link == RC_CRSF && now_ms % 7 == 0) {                                 // ~143 Hz CRSF
            send_crsf_channels(); rc_frames++;
            if (rc_frames % 10 == 0) send_crsf_link();
        }
        if (has_rc && tx_on && ((rc_link == RC_SPEKTRUM2048 && now_ms % 11 == 0) || (rc_link == RC_SPEKTRUM1024 && now_ms % 22 == 0))) { send_spektrum(); rc_frames++; }
    }
    // ---- CRSF encoder (written independently of the decoder: plain bit loop + bitwise CRC)
    static uint8_t crc8(const uint8_t* p, size_t n) {
        uint8_t c = 0;
        for (size_t i = 0; i < n; i++) { c ^= p[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
        return c;
    }
    void crsf_frame(uint8_t type, const uint8_t* payload, size_t n) {
        std::vector<uint8_t> f; f.push_back(0xC8); f.push_back((uint8_t)(n + 2)); f.push_back(type);
        for (size_t i = 0; i < n; i++) f.push_back(payload[i]);
        f.push_back(crc8(f.data() + 2, n + 1));
        if (rc_corrupt > 0 && f.size() > 4) { f[4] ^= 0x55; rc_corrupt--; }      // a byte hit by noise: the checksum no longer fits
        for (uint8_t c : f) rc_uart.q.push_back(c);
    }
    void send_crsf_channels() {
        uint8_t p[22] = {};
        for (int ch = 0; ch < 16; ch++) {
            long raw = lround((rc[ch] - 881.0) / 0.62477120195241); if (raw < 0) raw = 0; if (raw > 2047) raw = 2047;
            for (int b = 0; b < 11; b++) if ((raw >> b) & 1) { const int bit = ch * 11 + b; p[bit / 8] |= (uint8_t)(1u << (bit % 8)); }
        }
        crsf_frame(0x16, p, 22);
    }
    // ---- Spektrum satellite (written from the frame layout, not from the decoder): [fades][system][7 x (channel << bits | value), big-endian]
    //      2048: DSMX 11 ms, frames take turns: channels 0..6, then 7..11 (+ two empty slots, bit 15 set as the "second frame" mark)
    void send_spektrum() {
        const bool hi = rc_link == RC_SPEKTRUM2048; uint16_t w[7];
        const bool second = hi && (spek_n_++ & 1);
        for (int k = 0; k < 7; k++) {
            const int ch = second ? 7 + k : k;
            if (ch >= 12) { w[k] = 0xFFFF; continue; }
            double v = hi ? (rc[ch] - 988.0) * 2.0 : (rc[ch] - 988.0); const double top = hi ? 2047 : 1023; if (v < 0) v = 0; if (v > top) v = top;
            w[k] = (uint16_t)((ch << (hi ? 11 : 10)) | (uint16_t)lround(v)); if (second) w[k] |= 0x8000;
        }
        rc_uart.q.push_back(spek_fades); rc_uart.q.push_back(hi ? 0xB2 : 0x01);
        for (int k = 0; k < 7; k++) { rc_uart.q.push_back((uint8_t)(w[k] >> 8)); rc_uart.q.push_back((uint8_t)w[k]); }
    }
    uint32_t spek_n_ = 0;
public:
    uint8_t spek_fades = 0;
    // ---- PPM / PWM: the edges of the next frame, delivered to the pin interrupt at their own time (see step())
    void pulse_frames() {
        if (!has_rc || !tx_on || (rc_link != RC_PPM && rc_link != RC_PWM)) { pulse_next_us_ = 0; return; }
        if (g_stub_us < pulse_next_us_) return;
        const uint64_t t0 = pulse_next_us_ ? pulse_next_us_ : g_stub_us + 100;
        if (rc_link == RC_PPM) {                                     // a short pulse starts every channel; the gap after the last one is the sync
            const uint8_t on = negative_ppm ? 0 : 1; uint64_t t = t0;
            for (int k = 0; k <= ppm_channels; k++) { pin_edges.push_back({t, kPpmPin, on}); pin_edges.push_back({t + 300, kPpmPin, (uint8_t)!on}); if (k < ppm_channels) t += rc[k]; }
            pulse_next_us_ = t0 + ppm_frame_us;
        } else {                                                     // one channel after the other, each on its own pin, every 20 ms
            uint64_t t = t0;
            for (int k = 0; k < pwm_channels; k++) { if (!(pwm_dead & (1u << k))) { pin_edges.push_back({t, kPwmPin[k], 1}); pin_edges.push_back({t + rc[k], kPwmPin[k], 0}); } t += rc[k] + 20; }
            pulse_next_us_ = t0 + 20000;
        }
        rc_frames++;
        std::sort(pin_edges.begin(), pin_edges.end(), [](const PinEdge& a, const PinEdge& b) { return a.us < b.us; });
    }
    void deliver_edges() {
        size_t k = 0;
        while (k < pin_edges.size() && pin_edges[k].us <= g_stub_us) { stub_pin_edge(pin_edges[k].pin, pin_edges[k].level, pin_edges[k].us); k++; }
        if (k) pin_edges.erase(pin_edges.begin(), pin_edges.begin() + (long)k);
    }
    struct PinEdge { uint64_t us; uint8_t pin, level; };
    std::vector<PinEdge> pin_edges;
private:
    uint64_t pulse_next_us_ = 0;
public:
    void send_crsf_link() { const uint8_t p[10] = {60, 62, lq, 8, 0, 4, 2, 70, 100, 6}; crsf_frame(0x14, p, 10); }
private:

    struct { bool active = false; int idx = 0; bool rd = false; uint8_t reg = 0; uint8_t cache[16]; uint8_t w[200]; } t;
    uint8_t i2c_reg_ = 0; uint32_t last_ms_ = 0xFFFFFFFFu; uint8_t last_cmd_ = 0, last_slots_ = 0, edt_run_[8] = {}, edt_last_[8] = {};
};

// ---- the stub hooks
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) g_stub_gpio[pin] = val; if (g_board) g_board->cs(pin, val); }
uint8_t stub_spi_transfer(uint8_t v) { return g_board ? g_board->spi(v) : 0xFF; }
bool stub_i2c_probe(uint8_t) { return true; }
int stub_i2c_tx(uint8_t addr, const uint8_t* d, size_t n, bool) { stub_io(); return g_board ? g_board->i2c_tx(addr, d, n) : 2; }
int stub_i2c_rx(uint8_t addr, uint8_t* d, size_t n) { stub_io(); return g_board ? g_board->i2c_rx(addr, d, n) : 0; }
