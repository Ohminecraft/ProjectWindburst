/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "../hal/acc_gyro/acc_gyro_base.h"
#include "../hal/acc_gyro/imu_select.h"
#include "../hal/led/ws2812.h"
#include "../hal/can/can_port.h"
#include "../hal/can/dronecan_gps.h"
#include "../hal/motor/motor.h"
#include "../hal/motor/esc_sensor.h"
#include "../hal/baro/baro_base.h"
#include "../hal/range/range_base.h"
#include "../hal/gps/gps_base.h"
#include "../hal/flow/flow_sensor.h"
#include "../hal/servo/servo_out.h"
#include "../hal/mag/mag_base.h"
#include "../hal/rx/rx_base.h"
#include "../hal/battery/battery_base.h"
#include "../hal/beeper/beeper_base.h"
#include "../core/flight_core.h"
#include "../core/inflight_trim.h"
#include "../hal/mag/mag_autocal.h"
#include "stick_commands.h"
#include "../core/ekf.h"
#include "../proto/settings_apply.h"
#include "telemetry.h"
#include "../telemetry/mavlink_telemetry.h"
#include "../osd/osd.h"
#include "../proto/wbp_fields.h"
#include "exchange.h"
#include "sync.h"
#include "link_host.h"
#include "../blackbox/bb_recorder.h"
#include "../config/pin_check.h"
#include "../config/version.h"
#include "../port/platform.h"
#include "../port/pin_names.h"
#include "sysload.h"

// ============================================================================
// FlightApp: the body of the RX firmware = drivers + flight core + the flight-loop end of the app link.
// Plain forwarding, no flight logic: settings -> drivers/core, driver outputs -> FlightCore, core output -> motors,
// everything the app shows -> app::Telemetry, app commands -> the driver / calibration that implements them.
//
//   wbp::Settings settings;  store.load(settings);
//   FlightApp app;
//   setup():  app.configure(settings);     // pins, sensor switches, alignment, calibration, PID, rates, modes, motors, GPS
//             app.begin();                 // per-sensor result in app.status and in the boot report (app.report())
//   board:    flight task -> app.flight_cycle(micros()) on every timer tick;  loop() -> app.slow_update() every millisecond
//   PC:       app.run();                   // both from one thread: a cycle when it is due + the slower jobs
//   another task: wbp::Server + app::LinkHost on app.link (see link_arduino.h)
//
// Threading rule (sync.h): the control cycle runs in the flight task, everything else in the sensor thread; whatever the two
// share (core, IMU, motors, telemetry) is touched only with the lock held, and nothing waits for a device while holding it.
// The link task only talks to `link` (app::Exchange). The Settings object belongs to the link task; this class reads / writes
// it only inside execute(), i.e. while the link task is blocked waiting for that very request.
// ============================================================================
// A pin number of the settings as the boot report names it: GPIO12 on an ESP32, PB3 on an STM32 (port/pin_names.h).
inline const char* pin_str(uint8_t p, char (&b)[8]) { port::pin_name(p, WB_PLATFORM_STM32, b, sizeof(b)); return b; }

class FlightApp {
public:
    struct Status { bool imu = false, motors = false, baro = false, range = false, gps = false, mag = false, rx = false, battery = false, buzzer = false, motor_beep = false, pin_conflict = false, flow = false, servo = false, imu2 = false, can = false, led = false; } status;

    fc::FlightCore core;
    IMUAccGyro imu;
    IMUAccGyro imu2;                                      // a second IMU on imu's SPI bus (pins.imu2_cs, sensors.gyro_use): imu_sel_ picks
    MotorSystem motors;
    BaroSensor baro;
    RangeFinder sonar;
    GpsSensor gnss;
    FlowSensor flow;
    ServoOutputs servos;
    port::SerialPort* uart0 = nullptr;                    // UART0, when the app's link does not use it (native USB): a third UART for the flow sensor
#if WB_PLATFORM_STM32
    port::SerialPort* uart1 = nullptr; port::SerialPort* uart2 = nullptr;      // (main_stm32.cpp: the UART instance follows the pins)
#else
    port::SerialPort* uart1 = &Serial1; port::SerialPort* uart2 = &Serial2;
#endif
    MagSensor mag;
    RcReceiver rc;
    BatterySensor battery;
    BuzzerOutput buzzer;
    beep::Beeper beeper;                                  // the buzzer's patterns
    beep::Beeper motor_beeper;                            // the same conditions played by the motors (DShot beacon), for a craft without a buzzer
    bb::Recorder blackbox;                                // the flight-loop end of the flight log (the rest is bb::Service in the link task)
    I2CBus sensorBus{I2C_DEVICE_0};
    app::Exchange link;                                   // the only object shared with the link task
    mav::Ring mav_net;                                    // MAVLink frames for the WiFi side (the link task sends them as UDP): sensor thread -> link task
    fc::InflightTrim inflight_trim;
    bool use_external_rc = false;                         // true: channels come from set_rc() (tests / a radio module) instead of `rc`
    bool usb_link = false;                                // the app link is the chip's own USB (GPIO19 / 20): set before begin(), for the pin check
    CanPort* can_port = nullptr;                          // the CAN controller (twai_bus.h on the ESP32): set before begin(); nullptr = none on this chip
    dcan::Node can_node;                                  // the flight controller as a DroneCAN node (section can)
    uint8_t chip_uid[12] = {};                            // the chip's unique id (DroneCAN's hardware unique id): set before begin()

    // ---------------------------------------------------------------- settings -> configuration (before begin())
    void configure(wbp::Settings& s) {
        s_ = &s;
        wbp::apply_core(s, core.cfg, core.rc);
        ekf_mode_ = s.nav.ekf; ekf_.cfg.gps_delay_s = core.cfg.horizontal.gps_delay_s;
        wbp::apply_motors(s, motors.cfg()); motors_run_ = s.motors;
        wbp::apply_gps(s, gnss.config());
        apply_battery(); apply_beeper(); apply_servo();
        baro.setEnabled(s.sensors.baro_enabled != 0); baro.setModel((BaroSensor::Model)s.sensors.baro_model); sonar.setEnabled(s.sensors.range_enabled != 0);
        mag.setEnabled(s.sensors.mag_enabled != 0); gnss.setEnabled(s.sensors.gps_enabled != 0);
        mag.setModel((MagSensor::Model)s.sensors.mag_model);
        sonar.setModel((RangeFinder::Model)s.sensors.range_model); sonar.setL1xMode(s.sensors.range_l1x_mode);
        flow.setType(s.flow.type);
        mag.setAlignment((AccGyroAlign)s.sensors.mag_align, s.sensors.mag_trim_roll * 0.1f, s.sensors.mag_trim_pitch * 0.1f, s.sensors.mag_trim_yaw * 0.1f);
        rc.setTimeoutMs(s.receiver.rx_timeout_ms);
        apply_calibration();
    }

#if WB_PLATFORM_STM32
    static const wbp::Settings*& self_pins() { static const wbp::Settings* p = nullptr; return p; }     // (for port::g_pin_taken)
#endif
    // ---------------------------------------------------------------- hardware bring-up
    bool begin() {
        wbp::Settings& s = *s_; HardwareConfig hw;
        n_report_ = 0;
#if WB_PLATFORM_STM32
        port::g_pin_taken = [](uint8_t p) {                    // any pin of the "pins" section (all of it is pin bytes)
            const uint8_t* b = reinterpret_cast<const uint8_t*>(&self_pins()->pins); for (size_t i = 0; i < sizeof(wbp::S_pins); i++) if (b[i] == p) return true; return false; };
        self_pins() = s_;
#endif

        // ---- pins: the app's "pins" section (kept in NVS by the settings store) goes into HardwareConfig, where every driver asks for its pins.
        //      A pin the app left at 255 keeps the board default typed in hardware_config.h.
        {
            HardwarePins hp;
            hp.imu_cs = s.pins.imu_cs; hp.imu_miso = s.pins.imu_miso; hp.imu_mosi = s.pins.imu_mosi; hp.imu_sck = s.pins.imu_sck;
            hp.i2c_sda = s.pins.i2c_sda; hp.i2c_scl = s.pins.i2c_scl;
            hp.gps_rx = s.pins.gps_rx; hp.gps_tx = s.pins.gps_tx; hp.rx_rx = s.pins.rx_rx; hp.rx_tx = s.pins.rx_tx;
            for (int i = 0; i < 8; i++) hp.motor[i] = s.pins.motor[i];
            hp.batt_adc = s.pins.batt_adc; hp.buzzer = s.pins.buzzer;
            hp.sd_cs = s.pins.sd_cs; hp.sd_clk = s.pins.sd_clk; hp.sd_cmd = s.pins.sd_cmd; hp.sd_d0 = s.pins.sd_d0;
            HardwareConfig::setAppPins(hp);
        }
        check_pins();

        // ---- IMU: SPI when a chip-select pin exists, otherwise I2C on the shared sensor bus
        uint8_t spi[4]; hw.getSPIAccGyroSensorPins(spi);
        uint8_t bus[2]; hw.getI2CBaroSensorPins(bus);
        const bool imu_spi = spi[0] != GPIO_UNUSED;
        AgInitConfig ic; ic.gyro_odr_hz = s.general.loop_hz;
        status.imu = imu.IMUInit(imu_spi, ic);

        // ---- loop rate: never faster than the gyro delivers new samples; an I2C IMU cannot be read faster than ~1 kHz
        // and a divisor of it (app::loop_rate_for: 6000 Hz on a chip at 8000 becomes 4000)
        float hz = s.general.loop_hz;
        if (status.imu && imu.gyroRateHz() > 0) hz = app::loop_rate_for(hz, imu.gyroRateHz());
        // ---- a second IMU: on the first one's SPI bus, its own chip select
        const uint8_t cs2 = s.pins.imu2_cs; imu2_cfg_ = cs2 != GPIO_UNUSED;
        if (imu2_cfg_) {
            const char* why = !status.imu ? "the first IMU is missing (the second one shares its SPI bus)" : !imu_spi ? "the first IMU is on I2C (a second IMU shares the first one's SPI bus)" : nullptr;
            status.imu2 = !why && imu2.IMUInitShared(imu, cs2, ic);
            if (!why) why = imu2.failReason();
            if (status.imu2 && s.sensors.gyro_use != imusel::FIRST && imu2.gyroRateHz() > 0 && hz > imu2.gyroRateHz()) hz = imu2.gyroRateHz();     // (flown on: its rate counts too)
            static const char* const kUse[3] = {"stand-by (gyro_use FIRST)", "its gyro flies (gyro_use SECOND)", "both gyros averaged (gyro_use BOTH)"};
            char pb[8]; say(status.imu2, "imu2", imu2.name(), why, "SPI %s on CS %s, %s", imu2.name(), pin_str(cs2, pb), kUse[s.sensors.gyro_use < 3 ? s.sensors.gyro_use : 0]);
        }
        imu_sel_.configure((imusel::Use)s.sensors.gyro_use, status.imu, status.imu2);
        if (!imu_spi && hz > 1000.0f) hz = 1000.0f;
        core.cfg.loop_hz = hz;
        // the control rate: loop_hz / pid_denom, never below 500 Hz (the denominator is lowered until it fits)
        while (core.cfg.pid_denom > 1 && hz / (float)core.cfg.pid_denom < 500.0f) core.cfg.pid_denom--;
        const float pid_hz = core.cfg.pid_hz();
        if (core.cfg.est_hz > pid_hz) core.cfg.est_hz = pid_hz;
        period_us_ = (uint32_t)(1e6f / hz + 0.5f);
        pub_div_ = (uint16_t)(hz / 1000.0f + 0.5f); if (pub_div_ < 1) pub_div_ = 1;
        cal_samples_ = hz > 8000.0f ? 8000 : (uint16_t)hz;                         // ~1 s of samples
        if (status.imu) { apply_alignment(); start_gyro_cal(); cal_kind_ = wbp::CAL_KIND_GYRO; }
        {   char clk_txt[32] = "", rate_txt[64] = "";       // (SPI: the read clock, and "direct" when it is read past the Arduino library - bus_spi.h)
            if (imu_spi && status.imu) snprintf(clk_txt, sizeof(clk_txt), " (%.0f MHz%s)", (double)(imu.spiClockHz() / 1e6f), imu.spiDirect() ? " direct" : "");
            if (status.imu && (uint32_t)(hz + 0.5f) != (uint32_t)s.general.loop_hz)
                snprintf(rate_txt, sizeof(rate_txt), " (setting %u Hz: the chip delivers %.0f Hz)", (unsigned)s.general.loop_hz, (double)imu.gyroRateHz());
            say(status.imu, "imu", imu.name(), imu.failReason(), "%s %s%s, gyro %.0f Hz%s, control %.0f Hz (1/%d)", imu_spi ? "SPI" : "I2C", imu.name(), clk_txt, (double)hz, rate_txt, (double)pid_hz, core.cfg.pid_denom); }

        // ---- shared sensor I2C bus: barometer, lidar, compass
        if (bus[0] != GPIO_UNUSED && bus[1] != GPIO_UNUSED) {
            I2CBus_config_t c; c.bus_clock_speed = 400000; c.ioSdaGpio = bus[0]; c.ioSclGpio = bus[1]; c.addr = 0x76; sensorBus.I2CInit(c);
            baro.attachI2C(sensorBus); sonar.attachI2C(sensorBus); mag.attachI2C(sensorBus);
        }
        const bool can_baro = s.sensors.baro_model == wbp::BARO_MODEL_DRONECAN;
        if (!can_baro) { status.baro = baro.BaroInit(false); say(status.baro, "baro", baro.name(), baro.failReason(), "%s", baro.name()); }
        status.range = sonar.RangeInit();
        // the lidar may be the one inside an MSP flow sensor (asked for, or nothing else answered): that is known after the UARTs
        const bool range_from_flow = app::lidar_of_flow(s.sensors.range_enabled, s.flow.type == FlowSensor::MSP, s.sensors.range_model == wbp::RANGE_MODEL_MSP,
                                                        s.sensors.range_model == wbp::RANGE_MODEL_AUTO, status.range);
        if (!range_from_flow) say(status.range, "range", sonar.name(), sonar.failReason(), "%s", sonar.name());
        // ---- CAN bus: DroneCAN modules (a GPS, a compass) - before the compass and the GPS, which may take their data from it
        status.can = false; can_gps_ = can_mag_ = can_baro_ = can_batt_ = false;
        if (s.can.mode == wbp::CAN_MODE_DRONECAN) {
            static const uint32_t kBps[4] = {125000, 250000, 500000, 1000000};
            const uint32_t bps = kBps[s.can.bitrate < 4 ? s.can.bitrate : 3];
            const char* why = !can_port ? "this chip has no CAN controller in this build" : (s.pins.can_tx == GPIO_UNUSED || s.pins.can_rx == GPIO_UNUSED) ? "CAN pins not set (Pins tab: CAN bus)" : nullptr;
            if (!why && !can_port->start(s.pins.can_tx, s.pins.can_rx, bps)) why = can_port->why();
            status.can = !why;
            if (status.can) {
                dcan::Node::Config nc; nc.node_id = s.can.node_id; nc.dna_server = s.can.dna_server != 0; nc.sw_major = WB_FW_MAJOR; nc.sw_minor = WB_FW_MINOR;
                for (int i = 0; i < 16; i++) nc.uid[i] = chip_uid[i % 12] ^ (uint8_t)(i / 12);
                can_node.begin(can_port, nc, millis());
                can_gps_ = s.gps.provider == wbp::GPS_PROVIDER_DRONECAN; can_mag_ = s.sensors.mag_model == wbp::MAG_MODEL_DRONECAN; can_baro_ = can_baro;
                can_batt_ = s.battery.source == batt::SRC_DRONECAN || s.battery.current_source == batt::CUR_DRONECAN;
            }
            char tb[8], rb[8];
            say(status.can, "can", "", why, "DroneCAN node %u, %lu kbit/s, TX %s RX %s%s", (unsigned)s.can.node_id, (unsigned long)(bps / 1000u), pin_str(s.pins.can_tx, tb), pin_str(s.pins.can_rx, rb),
                s.can.dna_server ? ", hands out node ids" : "");
        }
        if (can_baro) {                                      // a DroneCAN barometer: only once the bus is known to run
            status.baro = status.can && baro.BaroInit(false);
            say(status.baro, "baro", "DroneCAN", status.can ? baro.failReason() : "sensors.baro_model is DRONECAN but the CAN bus is not running (section can)", "DroneCAN (the first node on the bus that sends StaticPressure)");
        }
        mag.attachImuBus(imu.regBus(), imu.detected());
        status.mag = mag.MagInit();
        say(status.mag, "mag", mag.name(), mag.failReason(), "%s%s", mag.name(), mag.calibrated() ? "" : " (not calibrated)");
        core.cfg.use_mag = core.cfg.use_mag && status.mag;                         // the compass is used only when it exists AND is switched on

        // ---- UARTs: which one serves the GPS, the serial receiver and an MSP flow sensor (section ports: AUTO = GPS on UART1,
        //      receiver on UART2, the flow sensor on what is left; TABLE = the function table)
        port::SerialPort* const uarts[3] = {uart0, uart1, uart2};
        static const char* const uname[3] = {"UART0", "UART1", "UART2"};
        const rx::Protocol rxp_ = (rx::Protocol)s.receiver.protocol;
        const bool serial_rx = !use_external_rc && rxp_ != rx::PROTO_PPM && rxp_ != rx::PROTO_PWM;
        const bool port_table = s.ports.assign == wbp::PORT_ASSIGN_TABLE;
        uint8_t pfn[3]; for (int i = 0; i < 3; i++) pfn[i] = s.ports.function[i];
        const bool gps_uart = s.sensors.gps_enabled && gps::uses_uart((gps::Provider)s.gps.provider);        // (a DroneCAN GPS needs no UART)
        const app::UartPlan plan = app::plan_uarts(port_table, pfn, gps_uart, serial_rx, s.flow.type == FlowSensor::MSP, uart0 != nullptr, false, false);
        if (*plan.why) { char line[kReportLen]; snprintf(line, sizeof(line), "ports: WARNING - %s", plan.why); add_report(line); }
        if (s.sensors.gps_enabled && s.gps.provider == wbp::GPS_PROVIDER_DRONECAN) {
            status.gps = status.can && gnss.beginExternal();
            say(status.gps, "gps", "DroneCAN", status.can ? gnss.failReason() : "gps.provider is DRONECAN but the CAN bus is not running (section can)", "DroneCAN (the first node on the bus that sends a fix)");
        } else if (plan.gps >= 0 || !s.sensors.gps_enabled) {
            status.gps = gnss.begin(*uarts[plan.gps >= 0 ? plan.gps : 1]);        // (disabled: begin() only says so)
            say(status.gps, "gps", "started", gnss.failReason(), "%s %lu baud", uname[plan.gps >= 0 ? plan.gps : 1], (unsigned long)gnss.config().baud);
        } else { status.gps = false; say(false, "gps", "", "no UART has the function GPS in the serial ports table (Pins and ports tab)", ""); }
        if (!use_external_rc) {
            const rx::Protocol rp = rxp_;
            if (rp == rx::PROTO_PPM || rp == rx::PROTO_PWM) {             // pulses on GPIO pins: UART2 stays free
                uint8_t pp[RcReceiver::kMaxPulsePins]; int wired = 0;    // (copied: the settings struct is packed)
                for (int i = 0; i < RcReceiver::kMaxPulsePins; i++) { pp[i] = rp == rx::PROTO_PPM ? (i == 0 ? s.pins.ppm : 255) : s.pins.rx_pwm[i]; if (pp[i] != 255) wired++; }
                status.rx = rc.beginPulse(rp, pp, RcReceiver::kMaxPulsePins);
                if (rp == rx::PROTO_PPM) { char pb[8]; say(status.rx, "rx", rc.name(), rc.failReason(), "PPM on %s", pin_str(pp[0], pb)); }
                else { char pb[8]; say(status.rx, "rx", rc.name(), rc.failReason(), "PWM, %d channel%s from %s", wired, wired == 1 ? "" : "s", pin_str(pp[0], pb)); }
            } else if (plan.rx >= 0) {
                uint8_t rxp[3]; hw.getSerialRxPins(rxp);
                status.rx = rc.begin(rp, *uarts[plan.rx], rxp[0], rxp[1]);
                rx_tlm_on_ = s.receiver.telemetry != 0;
                say(status.rx, "rx", rc.name(), rc.failReason(), "%s on %s%s", rc.name(), uname[plan.rx], !rc.telemetryPossible() ? "" : rx_tlm_on_ ? ", telemetry on its TX pin" : ", telemetry off (receiver.telemetry)");
                crsf_tlm_.reset();
            } else { status.rx = false; say(false, "rx", "", "no UART has the function SERIAL_RX in the serial ports table (Pins and ports tab)", ""); }
        }
        // ---- optical flow: PMW3901 on the second SPI bus, or an MSP sensor on whichever UART is left (UART1 without a GPS,
        //      UART2 without a serial receiver, UART0 when the app is on USB)
        int flow_u = -1;
        if (s.flow.type != FlowSensor::NONE) {
            const int u = port_table ? plan.flow : app::flow_uart(status.gps, status.rx && !use_external_rc && rc.onUart(), uart0 != nullptr);
            if (s.flow.type == FlowSensor::MSP) flow_u = u;
            flow.setUart(u >= 0 ? uarts[u] : nullptr, u >= 0 ? uname[u] : "", s.pins.flow_rx, s.pins.flow_tx, s.flow.baud);
            flow.setSpiPins(s.pins.flow_cs, s.pins.flow_sck, s.pins.flow_miso, s.pins.flow_mosi);
            flow.setSpiBusTaken(s.blackbox.device == bb::DEV_SDCARD && s.blackbox.sd_mode == wbp::SD_MODE_SPI);
            status.flow = flow.begin();
            say(status.flow, "flow", flow.name(), flow.failReason(), "%s%s%s", flow.name(), flow.type() == FlowSensor::MSP ? " on " : " on its own SPI bus", flow.type() == FlowSensor::MSP ? flow.uartName() : "");
        }
        if (range_from_flow) {
            if (status.flow && flow.rangeDriver()) status.range = sonar.useDriver(flow.rangeDriver());
            say(status.range, "range", sonar.name(), status.flow ? sonar.failReason() : "the lidar of the flow sensor: the flow sensor did not start", "%s", sonar.name());
        }
        if (status.flow && !status.range) add_report("flow: WARNING - no range sensor: optical flow needs the distance to the ground (a lidar) to turn the picture's motion into a speed");

        // ---- ESC telemetry on its own wire (hal/motor/esc_sensor.h): a UART RX at 115200, on the UART that is left (AUTO) or the
        //      one with the function ESC_SENSOR (TABLE)
        esc_uart_ = nullptr; int esc_u = -1;
        if (s.motors.esc_sensor) {
            const int u = port_table ? plan.esc : app::esc_uart(status.gps, status.rx && !use_external_rc && rc.onUart(), uart0 != nullptr, flow_u);
            char pb[8];
            if (!motor::is_dshot((motor::Protocol)s.motors.protocol)) say(false, "esc tlm", "", "the ESC telemetry wire needs a DShot protocol (the request is a bit of the DShot frame)", "");
            else if (s.pins.esc_tlm == 255) say(false, "esc tlm", "", "no pin for the ESC telemetry wire (Pins tab: ESC telemetry)", "");
            else if (u < 0) say(false, "esc tlm", "", port_table ? "no UART has the function ESC_SENSOR in the serial ports table" : "no UART left for the ESC telemetry wire (GPS, receiver and flow sensor have them)", "");
            else if (!port::uart_begin(*uarts[u], escs::kBaud, SERIAL_8N1, s.pins.esc_tlm, 255, false, 256)) say(false, "esc tlm", "", "the UART did not start on that pin", "");
            else { esc_uart_ = uarts[u]; esc_u = u; say(true, "esc tlm", "", "", "KISS telemetry on %s RX %s, 115200 baud (the first request after 5 s)", uname[u], pin_str(s.pins.esc_tlm, pb)); }
        }

        // ---- OSD (section osd, osd/osd.h): MSP DisplayPort to the video transmitter of a digital FPV system, on the UART that is
        //      left (AUTO) or the one with the function DISPLAYPORT (TABLE); it talks back on the same UART (armed state, canvas)
        osd_uart_ = nullptr; int osd_u = -1; osd_fly_ms_ = 0; osd_last_ms_ = 0;
        if (s.osd.type == wbp::OSD_TYPE_MSP_DISPLAYPORT) {
            const int u = port_table ? plan.osd : app::telemetry_uart(status.gps, status.rx && !use_external_rc && rc.onUart(), uart0 != nullptr, flow_u, esc_u);
            const char* why = s.osd.tx_pin == 255 ? "no TX pin (osd.tx_pin)"
                            : u < 0 ? (port_table ? "no UART has the function DISPLAYPORT in the serial ports table" : "no UART left (GPS, receiver, flow sensor and ESC wire have them)") : nullptr;
            if (!why) {
#if !WB_PLATFORM_STM32
                uarts[u]->setTxBufferSize(2048);                 // (a screen is ~0.5 kB; poll() never hands over more than the room)
#endif
                if (!port::uart_begin(*uarts[u], s.osd.baud, SERIAL_8N1, s.osd.rx_pin, s.osd.tx_pin, false, 256)) why = WB_PLATFORM_STM32 ? "the UART did not start on those pins (an STM32 needs osd.rx_pin as well)" : "the UART did not start on those pins";
            }
            if (why) say(false, "osd", "", why, "");
            else {
                osd_uart_ = uarts[u]; osd_u = u;
                osd_.configure(s.osd.canvas, s.osd.rate);
                memcpy(osd_layout_.pos, s.osd.pos, sizeof(osd_layout_.pos));
                size_t k = 0; for (; k < sizeof(s.osd.name) && s.osd.name[k]; k++) { const char c = (char)s.osd.name[k]; osd_layout_.name[k] = c >= 'a' && c <= 'z' ? (char)(c - 32) : (c >= 0x20 && c < 0x7F ? c : ' '); }   // (Betaflight shows the name in capitals)
                osd_layout_.name[k] = 0;
                char tb[8], rb[8];
                say(true, "osd", "", "", "MSP DisplayPort on %s TX %s RX %s, %lu baud, %u x %u%s, %u screens/s", uname[u], pin_str(s.osd.tx_pin, tb), pin_str(s.osd.rx_pin, rb), (unsigned long)s.osd.baud,
                    (unsigned)osd_.cols(), (unsigned)osd_.rows(), s.osd.canvas == osd::DisplayPort::CANVAS_AUTO ? " (or what the video transmitter asks for)" : "", (unsigned)s.osd.rate);
            }
        }

        // ---- MAVLink telemetry (section telemetry, telemetry/mavlink_telemetry.h): a UART TX (AUTO: the UART left; TABLE: the one with
        //      the function TELEMETRY) and / or WiFi (the link task sends the frames as UDP broadcasts to port 14550)
        mav_uart_ = nullptr; mav_on_ = mav_wifi_ = false; mav_report_i_ = 0; mav_blk_sent_ = kMavNeverSent; mav_blk_ms_ = 0; mav_armed_ = false;
        if (s.telemetry.protocol == wbp::TELEMETRY_PROTOCOL_MAVLINK) {
            const wbp::S_telemetry& tc = s.telemetry;
            mav::Rates mr; mr.attitude = tc.rate_attitude; mr.position = tc.rate_position; mr.hud = tc.rate_hud; mr.status = tc.rate_status; mr.rc = tc.rate_rc;
            mav_.configure(mr, tc.sysid);
            const int nm = fc::mixer_motor_count((fc::Layout)s.motors.layout);
            mav_type_ = nm == 6 ? mav::MAV_TYPE_HEXAROTOR : nm == 8 ? mav::MAV_TYPE_OCTOROTOR : nm == 4 ? mav::MAV_TYPE_QUADROTOR : mav::MAV_TYPE_GENERIC;
            mav_wifi_ = !WB_PLATFORM_STM32 && tc.wifi && s.wifi.mode != wbp::WIFI_MODE_OFF;
            const char* why = nullptr; int u = -1; char wire[64] = "";
            if (tc.tx_pin != 255) {
                u = port_table ? plan.tlm : app::telemetry_uart(status.gps, status.rx && !use_external_rc && rc.onUart(), uart0 != nullptr, flow_u, esc_u, osd_u);
                if (u < 0) why = port_table ? "no UART has the function TELEMETRY in the serial ports table" : "no UART left (GPS, receiver, flow sensor, ESC wire and OSD have them)";
                else {
#if !WB_PLATFORM_STM32
                    uarts[u]->setTxBufferSize(1024);                 // (frames wait there, the sensor thread never does: poll() only fills the room it has)
#endif
                    if (!port::uart_begin(*uarts[u], tc.baud, SERIAL_8N1, tc.rx_pin, tc.tx_pin, false, 256)) why = WB_PLATFORM_STM32 ? "the UART did not start on those pins (an STM32 needs telemetry.rx_pin as well)" : "the UART did not start on that pin";
                    else { mav_uart_ = uarts[u]; char pb[8]; snprintf(wire, sizeof(wire), "%s TX %s, %lu baud", uname[u], pin_str(tc.tx_pin, pb), (unsigned long)tc.baud); }
                }
            } else if (!mav_wifi_) why = "no TX pin (telemetry.tx_pin) and no WiFi (telemetry.wifi with wifi.mode on)";
            mav_on_ = mav_uart_ || mav_wifi_;
            if (!mav_on_) say(false, "telemetry", "", why, "");
            else {
                say(true, "telemetry", "", "", "MAVLink 2, system id %u, %s%s%s", (unsigned)tc.sysid, wire, *wire && mav_wifi_ ? " + " : "", mav_wifi_ ? "WiFi UDP 14550" : "");
                if (why) { char line[kReportLen]; snprintf(line, sizeof(line), "telemetry: WARNING - no UART: %s", why); add_report(line); }
            }
        }

        // ---- core + motors
        core.configure();
        motors.setCanNode(status.can ? &can_node : nullptr);               // (protocol DRONECAN: the ESCs' commands go out on the bus)
        status.motors = motors.begin(1.0f / core.cfg.pid_hz());          // a motor command per control cycle
        esc_poll_.begin(esc_uart_ ? motors.cfg().count : 0, millis());
        {   // why the outputs did not start, as precisely as it can be said
            static char why[150]; why[0] = 0;
            const motor::MotorConfig& mc = motors.cfg();
            uint8_t mp[8]; hw.getMotorPins(mp);
            if (motor::is_can(mc.protocol) && !status.can) snprintf(why, sizeof(why), "motors.protocol is DRONECAN but the CAN bus is not running (section can)");
            for (int i = 0; i < mc.count && !why[0] && !motor::is_can(mc.protocol); i++) if (mp[i] == GPIO_UNUSED) snprintf(why, sizeof(why), "the pin of motor slot %d is not set (Pins tab)", i + 1);
            if (!why[0] && motor::is_dshot(mc.protocol) && mc.count > 4)
                snprintf(why, sizeof(why), WB_PLATFORM_STM32 ? "DShot drives at most 4 motors here; this frame has %d: use PWM, Oneshot or Multishot"
                                                             : "DShot / ProShot drive at most 4 motors on the ESP32-S3 (4 RMT transmitters); this frame has %d: use PWM, Oneshot or Multishot", (int)mc.count);
            if (!why[0] && motors.failure()) snprintf(why, sizeof(why), "%s: %s", motors.backend(), motors.failure());
            if (!why[0]) snprintf(why, sizeof(why), "output driver failed: %s", motors.backend());
            if (motor::is_can(mc.protocol)) say(status.motors, "motors", motors.backend(), why, "DroneCAN x%d (esc.RawCommand, ESC index = motor slot)", (int)mc.count);
            else say(status.motors, "motors", motors.backend(), why, "%s x%d", motors.backend(), (int)mc.count);
        }
        edt_wanted_ = status.motors && s.motors.edt && s.motors.bidirectional && motor::is_dshot_wire(motors.cfg().protocol); edt_asked_ = false;

        // ---- battery voltage and buzzer
        status.battery = battery.begin() && (battery.source() != batt::SRC_DRONECAN || can_batt_);
        if (battery.source() == batt::SRC_DRONECAN && !can_batt_) say(false, "battery", "", "battery.source is DRONECAN but the CAN bus is not running (section can)", "");
        else
        say(status.battery, "battery", battery.name(), battery.failReason(), "%s%s", battery.name(), battery.source() == batt::SRC_ADC || battery.source() == batt::SRC_DRONECAN ? "" : " (no ADC pin)");
        // current: the ADC sensor, or the ESCs (only with extended DShot telemetry); AUTO with neither is no fault, just no current
        if (*battery.currentFailReason()) say(false, "current", "", battery.currentFailReason(), "");
        else if (battery.currentSource() == batt::CUR_DRONECAN) say(can_batt_, "current", "", "battery.current_source is DRONECAN but the CAN bus is not running (section can)", "from a DroneCAN power module (BatteryInfo)");
        else if (battery.currentSource() == batt::CUR_ADC) { char pb[8]; say(true, "current", "", "", "ADC sensor on %s, %d x 0.1 mV/A", pin_str(s.pins.curr_adc, pb), (int)s.battery.current_scale); }
        else if (battery.currentSource() == batt::CUR_ESC) {
            if (esc_uart_) say(true, "current", "", "", "from the ESCs (telemetry wire)");
            else if (edt_wanted_) say(true, "current", "", "", "from the ESCs (extended DShot telemetry)");
            else if (s.battery.current_source == batt::CUR_ESC) say(false, "current", "", "the ESCs report it only over their telemetry wire, or with bidirectional DShot and extended telemetry on (Motor tab)", "");
            else say(true, "current", "", "", "none (no sensor pin, no ESC telemetry)");
        }
        status.buzzer = buzzer.begin();
        status.motor_beep = status.motors && motor::is_dshot(motors.cfg().protocol);       // the ESCs can beep the motors
        say(status.buzzer || status.motor_beep, "beeper", "", status.motors ? "no buzzer pin set, and the motors can only beep with a DShot protocol" : buzzer.failReason(), "%s%s%s",
            status.buzzer ? buzzer.name() : "", status.buzzer && status.motor_beep ? " + " : "", status.motor_beep ? "motors (DShot beacon, disarmed only)" : "");
        beeper.silence(); motor_beeper.silence(); beeper.request(beep::SYSTEM_INIT);
        // ---- servos (after the motors and the buzzer: all three take LEDC channels, the flying parts first)
        { bool any = false; for (int i = 0; i < servo::kServos; i++) any |= s.pins.servo[i] != 255;
          if (any) { status.servo = servos.begin(s.pins.servo, s.servo.rate_hz);
                     say(status.servo, "servo", "", servos.failReason(), "%d output%s at %u Hz", servos.count(), servos.count() == 1 ? "" : "s", (unsigned)servos.rateHz()); } }
        // ---- LED strip (WS2812 on pins.led, through the second SPI bus)
        status.led = false;
        if (s.led.count) {
            const bool spi2_taken = (s.blackbox.device == bb::DEV_SDCARD && s.blackbox.sd_mode == wbp::SD_MODE_SPI) || s.flow.type == FlowSensor::PMW3901;
            const char* why = s.pins.led == GPIO_UNUSED ? "no pin for the LED strip (Pins tab: LED)" : WB_PLATFORM_STM32 ? "not on the STM32 yet"
                            : spi2_taken ? "the second SPI bus is taken (SD card on SPI, or a PMW3901)" : nullptr;
#if WB_HAS_LED_STRIP
            if (!why && !led_strip_.begin(s.pins.led)) why = "the SPI bus did not start";
#endif
            status.led = !why;
            static_assert((int)app::LedLogic::MANUAL == (int)wbp::LED_LAYOUT_MANUAL && (int)app::LedLogic::F_THROTTLE == (int)wbp::LED_FUNCTION_THROTTLE && app::LedLogic::kMax == 32, "led_logic.h follows the schema");
            led_logic_.configure(s.led.count, s.led.brightness, (app::LedLogic::Layout)s.led.layout);
            { uint32_t col[app::LedLogic::kMax]; memcpy(col, s.led.color, sizeof(col)); led_logic_.manual(s.led.fn, col, s.led.warnings != 0, s.led.speed); }   // (copied: the packed record's u32s may be unaligned)
            const char* look = s.led.layout == wbp::LED_LAYOUT_NAV ? ", navigation lights" : s.led.layout == wbp::LED_LAYOUT_MANUAL ? ", manual" : "";
            char pb[8]; say(status.led, "led", "", why, "%u LEDs on %s, %u %% brightness%s", (unsigned)s.led.count, pin_str(s.pins.led, pb), (unsigned)s.led.brightness, look);
        }
        if (s.general.stick_arm) {                           // arming with the sticks (safety.h StickArm)
            bool arm_switch = false; for (int i = 0; i < 16; i++) if (s.modes.mode[i] == wbp::MODE_ARM && s.modes.hi[i] > s.modes.lo[i]) arm_switch = true;
            say(true, "arming", "", nullptr, "sticks: throttle + pitch down, yaw and roll to opposite sides, 1 s (disarm: the same, on the ground)%s", arm_switch ? "; the ARM switch must be on" : "");
        }
        { char line[kReportLen]; sys::Memory m = sys::memory();
          snprintf(line, sizeof(line), "memory: %lu kB internal RAM free of %lu, PSRAM %lu kB", (unsigned long)(m.heap_free / 1024u), (unsigned long)(m.heap_total / 1024u), (unsigned long)(m.psram_total / 1024u));
          add_report(line); }

        next_us_ = micros() + period_us_; last_us_ = micros(); start_us_ = last_us_; slow_us_ = last_us_; stat_ms_ = millis();
        health_ = health(millis()); busy_other_ = busy_other(millis()); batt_state_ = (uint8_t)battery.monitor.state(); tel_.loop_target_hz = (uint16_t)(core.cfg.loop_hz + 0.5f); tel_.pid_target_hz = (uint16_t)(core.cfg.pid_hz() + 0.5f); tel_.motor_count = status.motors ? motors.cfg().count : 0;
        fill_slow(millis()); link.publish(tel_);
        return status.imu && status.motors;
    }

    // Boot report lines (static after begin()): what was detected, or why not. The link task sends them on CMD_REPORT.
    const char* const* report() const { return report_ptr_; }
    int report_lines() const { return n_report_; }
    // One more line for the report (e.g. the result of loading the settings). Only before the link task is started.
    void add_report(const char* text) {
        if (n_report_ >= kReportLines) return;
        snprintf(report_[n_report_], kReportLen, "%s", text); report_ptr_[n_report_] = report_[n_report_]; n_report_++;
    }

    // RC from somewhere else than the built-in serial receiver (use_external_rc = true)
    // Call once per received frame: the stick path (feedforward, RC smoothing) works per frame.
    void set_rc(const uint16_t ch[16], bool link_ok) { app::Guard g(*sync_); for (int i = 0; i < 16; i++) rc_[i] = ch[i]; rc_link_ = link_ok; rc_seq_++; }

    // ---------------------------------------------------------------- the loop
    // Two threads on the board (see sync.h): the flight task calls flight_cycle() when its timer fires, the sensor thread calls
    // slow_update() about every millisecond. On a PC one thread does both through run().
    void set_sync(app::Sync* s) { sync_ = s ? s : &no_sync_; }
    uint32_t period_us() const { return period_us_; }

    void run() {
        fast_update();
        const uint32_t now = micros();
        if ((uint32_t)(now - slow_us_) >= 1000u) { slow_us_ = now; slow_update(); }
    }

    // One control cycle if it is due (the single-thread way of keeping the rate). false: not yet, or the IMU read failed.
    bool fast_update() {
        const uint32_t now_us = micros();
        if ((int32_t)(now_us - next_us_) < 0) return false;
        next_us_ += period_us_;
        if ((int32_t)(now_us - next_us_) > 0) next_us_ = now_us + period_us_;      // late: the missed cycles are skipped, never run in a burst
        return flight_cycle(now_us);
    }

    // One control cycle, now (the flight task: once per timer tick). Returns false when the IMU read failed.
    // (lock / unlock written out, not a Guard: nothing in between throws, and the cycle keeps clear of the unwinding code)
    WB_FAST bool flight_cycle(uint32_t now_us) { sync_->lock(); const bool ok = fast_cycle(now_us, micros()); sync_->unlock(); return ok; }

    // ~1 kHz, the sensor thread.
    WB_FAST void slow_update() {
        const uint32_t now_us = micros(), now_ms = millis();
        // how regularly this thread gets the CPU: it runs the app's requests, so a long pause here is a firmware that "does not answer"
        if (slow_seen_) { const uint32_t gap = now_ms - slow_last_ms_; if (gap > slow_gap_cur_) slow_gap_cur_ = gap; }
        slow_seen_ = true; slow_last_ms_ = now_ms;
        // ---- A. device I/O, no lock: nothing here is shared with the flight task except the three values copied first
        bool armed, esc_a_ok; float up_cos, esc_v, esc_a, gyro_rate;
        { app::Guard g(*sync_); armed = last_out_.armed; up_cos = core.ahrs().up_cos(); esc_v = motors.manager().esc_voltage(now_ms); esc_a_ok = motors.manager().esc_current(now_ms, esc_a);
          const float* gw = last_out_.gyro_f_dps; gyro_rate = sqrtf(gw[0] * gw[0] + gw[1] * gw[1] + gw[2] * gw[2]); }

        bool rc_new = false;
        if (!use_external_rc) rc_new = rc.update(now_us, now_ms);
        if (status.can) {                                    // DroneCAN: what came in on the bus; a fix / a field for the GPS / the compass
            can_port->service(now_ms); can_node.update(now_ms);
            dcan::Fix2 fx; dcan::GnssAux aux; bool aux_ok = false;
            if (can_node.take_gnss(fx, &aux, aux_ok) && can_gps_) gnss.inject(dcan::to_solution(fx, aux_ok ? &aux : nullptr), now_ms);
            float ga[3]; if (can_node.take_mag(ga) && can_mag_) mag.pushExternal(ga, now_us);
            float pa, tc; if (can_node.take_baro(pa, tc) && can_baro_) baro.pushExternal(pa, tc, now_us);
            if (can_batt_) { dcan::Battery bi; const bool ok = can_node.battery(bi, now_ms); battery.setExternal(bi.voltage, bi.current, ok); }
            if (motor::is_can(motors.cfg().protocol)) for (int i = 0; i < dcan::Node::kMaxEsc; i++) { dcan::EscStatus e; if (can_node.take_esc(i, e)) { can_esc_[i] = e; can_esc_new_ |= (uint8_t)(1u << i); } }
        }
        gnss.update(now_ms, armed);
        const bool gps_new = gnss.newSolution() && gnss.usable() && gnss.home().has_home();
        const fc::GpsMeas gm = gps_new ? gps_meas() : fc::GpsMeas();
        battery.update(now_ms, esc_v, esc_a, esc_a_ok);
        const bool flow_new = status.flow && flow.update(now_us, now_ms);        // (PMW3901: one poll in ten passes, 0.3 ms on its own bus; MSP: the bytes that came in)
        int esc_ask = -1; bool esc_new = false; int esc_m = 0; escs::Reading esc_r;
        if (esc_uart_) {                                     // ESC telemetry wire: the bytes that came in, then whom to ask next
            uint8_t b[32]; size_t n;
            while ((n = port::uart_read(*esc_uart_, b, sizeof(b))) > 0) esc_poll_.feed(b, n, now_ms);
            esc_new = esc_poll_.take(esc_m, esc_r);
            esc_ask = esc_poll_.ask_now(now_ms);
        }
        bool baro_new = false, range_new = false, mag_new = false;
        switch (slot_++ % 3) {                               // one I2C device per pass
        case 0: baro_new = baro.update(now_us, now_ms, armed); break;
        case 1: range_new = sonar.update(now_us, now_ms, up_cos, armed); break;
        default: mag_new = mag.update(now_us, now_ms); break;
        }
        if (mag_auto_.active()) { if (mag_new) mag_auto_.feed(mag.rawGauss(), gyro_rate, now_ms); else mag_auto_.tick(now_ms); }     // (this thread's alone)
        const uint32_t io_us = micros() - now_us;
        // (the EKF's measurements wait here for its next step: this thread only)
        if (ekf_mode_) {
            if (gps_new) { ekf_gm_ = gm; ekf_gps_pend_ = true; }
            if (baro_new) { ekf_baro_ = baro.altitudeM(); ekf_baro_pend_ = true; }
            if (mag_new && mag.readyForFusion(now_ms)) { const float* b = mag.board(); ekf_mag_ = {b[0], b[1], b[2]}; ekf_mag_pend_ = true; }
        }

        // ---- B. with the lock: results into the core, one app request, slow telemetry. Nothing below waits for a device.
        bool restart = false;
        {
            app::Guard g(*sync_);
            const bool now_armed = last_out_.armed;
            if (now_armed && !was_armed_) { core.reset_position(); ekf_rehome_ = true; }   // (the GPS home point is set by gnss.update() when it sees the same edge)
            was_armed_ = now_armed;
            if (ekf_mode_) {
                if (ekf_acc_.dt >= 0.004f) { ekf_in_ = ekf_acc_; ekf_acc_ = EkfDelta(); ekf_due_ = true; ekf_ahrs_yaw_ = core.ahrs().euler_aero().yaw; ekf_armed_ = now_armed; ekf_gyro_ok_ = gyro_calibrated(); }
                core.on_ekf(ekf_nav_, now_ms);                                  // (the result of the last step)
            }

            if (!use_external_rc) { if (rc_new) rc_seq_++; rc_link_ = rc.linkOk(now_ms); const uint16_t* c = rc.channels(); for (int i = 0; i < 16; i++) rc_[i] = c[i]; }
            for (int i = 0; i < 16; i++) servo_rc_[i] = rc_[i];                      // (the servos keep the last channels when the link drops)
            servo_have_rc_ = rc_seq_ != 0;
            if (gps_new) core.on_gps(gm, now_ms);
            if (baro_new) core.on_baro(baro.altitudeM(), baro.sigmaM(), now_ms);
            if (range_new) core.on_range(sonar.heightM(), sonar.sigmaM(), now_ms);
            if (flow_new) { const flow::Reading& r = flow.reading(); fc::FlowSample f; f.x = r.x; f.y = r.y; f.dt = r.dt; f.age_s = r.age_s; f.quality = r.quality; core.on_flow(f, now_ms); flow_quality_ = r.quality; }
            if (mag_new) { const float* b = mag.board(); core.on_mag({b[0], b[1], b[2]}, mag.readyForFusion(now_ms), now_ms); }
            if (esc_new) motors.manager().feed_serial(esc_m, esc_r.temp_c, esc_r.volt, esc_r.amps, esc_r.erpm, now_ms);
            if (can_esc_new_) {                                                  // esc.Status of the ESCs on the bus (index = hardware slot)
                for (int i = 0; i < dcan::Node::kMaxEsc; i++) if (can_esc_new_ & (1u << i)) { const dcan::EscStatus& e = can_esc_[i]; motors.manager().feed_can(i, e.temperature_k > 0 ? e.temperature_k - 273.15f : 0.0f, e.voltage, e.current, e.rpm, now_ms); }
                can_esc_new_ = 0;
            }
            if (esc_ask >= 0) motors.manager().request_telemetry(esc_ask);       // (the bit goes out with the next control cycle's frame)
            if (blackbox.take_erase_done()) beep_event(beep::BLACKBOX_ERASE);
            update_inflight_trim(now_ms);
            update_stick_commands(now_ms, now_armed);
            update_mag_autocal();
            update_beeper(now_ms, now_armed);
            // extended DShot telemetry: an ESC sends voltage / current / temperature only after it was asked (command 13). Asked when the
            // ESCs start answering (power-up, or the battery plugged in after USB), asked again after they all went silent. Never armed.
            if (edt_wanted_) {
                bool any = false; for (int i = 0; i < motors.cfg().count; i++) any |= motors.escAnswers(motors.cfg().order[i]);
                if (!any) edt_asked_ = false;                                     // (noticed armed too: asked again after disarming)
                else if (!edt_asked_ && !now_armed) edt_asked_ = motors.manager().enable_extended_telemetry(true);
            }

            // a gyro calibration that failed at boot (the craft was moved) is retried until it succeeds, like Betaflight
            if (!imu.gyroCalibrated() && imu.gyroCalState() == CalState::FAILED && !now_armed && status.imu) imu.startGyroCalibration(cal_samples_);
            if (!imu2.gyroCalibrated() && imu2.gyroCalState() == CalState::FAILED && !now_armed && status.imu2) imu2.startGyroCalibration(cal2_samples());

            if (status.led && (uint32_t)(now_ms - led_ms_) >= 20u) { led_ms_ = now_ms; led_in_ = led_input(); led_due_ = true; }      // (50 pictures a second, sent below)
            link.service([this](const app::Request& r) { return execute(r); });
            if (reboot_pending_ && (int32_t)(now_ms - reboot_at_ms_) >= 0) { float z[8] = {}; motors.update(micros(), z, false); restart = true; }
        }
        // ---- C. the slow telemetry, in short pieces with the lock let go in between: a control cycle that comes due meanwhile waits
        // for one piece, not for all of them (0.8.7 logs: the flight task waited ~240 us for the whole of B and C in one go, the
        // larger part of a 4 kHz period). Each piece is consistent on its own; the telemetry never needed more.
        {
            app::Guard g(*sync_);
            const uint32_t slow_us = micros() - now_us;
            t_slow_.add(slow_us > io_us ? slow_us : io_us);
            if ((uint32_t)(now_ms - stat_ms_) >= 1000u) { every_second(now_us, now_ms); tel_.slow_gap_ms = app::sat_u16(slow_gap_cur_); slow_gap_cur_ = 0; tel_.ekf_us = app::sat_u16(ekf_us_cur_); ekf_us_cur_ = 0; }
            health_ = health(now_ms); busy_other_ = busy_other(now_ms); if (busy_other_) busy_seen_ = true; batt_state_ = (uint8_t)battery.monitor.state();
        }
        { app::Guard g(*sync_); fill_slow(now_ms); }
        {
            app::Guard g(*sync_);
            if (rx_tlm_on_ && rc.telemetryPossible() && rc_link_) tlm_n_ = crsf_tlm_.next(now_us, crsf_in(), tlm_buf_);     // (only while the receiver is there to take it)
            if (mav_on_) { mav_st_ = mav_in(now_ms); mav_blk_ = tel_.blockers; }
            if (osd_uart_) { osd_st_ = osd_in(now_ms); app::msp_info_of(tel_, s_, osd_info_); }
            if ((uint32_t)(now_ms - bb_slow_ms_) >= 1000u / bb::kSlowHz) { bb_slow_ms_ = now_ms; blackbox.log_slow(now_us, tel_); }
        }
        if (ekf_due_) { ekf_due_ = false; ekf_step(now_ms); }  // (outside the lock: the EKF is this thread's alone, ~30 - 60 us)
        if (led_due_) {                                      // the LED strip (outside the lock: ~30 us per LED on the SPI bus)
            led_due_ = false; led_n_ = led_logic_.render(led_in_, now_ms, led_rgb_);
#if WB_HAS_LED_STRIP
            led_strip_.show(led_rgb_, led_n_);
#endif
        }
        servos.update(servo_rc_, servo_have_rc_);              // (outside the lock: it writes the PWM peripheral; the channels were copied under it)
        if (tlm_n_) { rc.sendTelemetry(tlm_buf_, tlm_n_); tlm_n_ = 0; }                  // (outside the lock: a UART write)
        if (mav_on_) mav_send(now_ms);                                                     // (outside the lock: the state was copied under it)
        if (osd_uart_) osd_send(now_ms);
        if (restart) {
            delay(20);
#if WB_PLATFORM_STM32
            NVIC_SystemReset();
#else
            ESP.restart();
#endif
        }
    }

    const fc::CoreOutput& output() const { return last_out_; }
    const app::Telemetry& telemetry() const { return tel_; }
    uint32_t imuErrors() const { return imu_errors_; }
    uint32_t loops() const { return loops_; }                 // control cycles executed since boot
    uint32_t beaconsSent() const { return beacons_; }         // DShot beacon rounds requested by the beeper
    uint8_t escPin(int m) const { return motors.pinOf(m); }   // ESC passthrough: the pin of logical motor m (fixed after begin())
    bool escPullup() { return !motors.cfg().ext_pullup; }   // ... internal pull-up wanted (no 2k resistor on the board)

    // ---------------------------------------------------------------- app requests (flight loop context, called by link.service)
    app::Reply execute(const app::Request& r) {
        const bool armed = last_out_.armed;
        const wbp::Err gate = app::check_request(r, armed);
        if (gate != wbp::ERR_OK) return gate;
        wbp::Settings& s = *s_;
        const uint32_t now_us = micros(), now_ms = millis();
        motor::MotorOutputManager& mm = motors.manager();
        switch (r.cmd) {
        case app::REQ_APPLY_SECTION: return apply_live(r.args[0]) ? wbp::ERR_OK : wbp::ERR_NOT_SUPPORTED;
        case app::REQ_REBOOT: mm.test_stop(); reboot_pending_ = true; reboot_at_ms_ = now_ms + 300; return wbp::ERR_OK;   // 300 ms: time for the ACK to leave
        case app::REQ_ESC_PASSTHROUGH: {                     // (disarmed: check_request) the link task talks to the ESC bootloaders on the motor pins
            if (!status.motors || motors.released()) return wbp::ERR_NOT_SUPPORTED;
            mm.test_stop(); { float z[8] = {}; motors.update(now_us, z, false); }           // the disarmed level, once more
            motors.release(); motor_beeper.silence();
            int n = 0; while (n < motors.cfg().count && motors.pinOf(n) != GPIO_UNUSED) n++;
            return app::Reply(wbp::ERR_OK, (uint8_t)n); }
        case wbp::CMD_APPLY_SETTINGS: for (const wbp::SectionInfo& si : wbp::kSections) if (si.live) apply_live(si.id); return wbp::ERR_OK;

        case wbp::CMD_GYRO_CAL:
            if (!status.imu) return wbp::ERR_NO_SENSOR;
            start_gyro_cal(); cal_kind_ = wbp::CAL_KIND_GYRO; return wbp::ERR_OK;

        case wbp::CMD_ACC_CAL_POINT:
            if (!status.imu) return wbp::ERR_NO_SENSOR;
            if (acc_cal_.running()) return wbp::ERR_BUSY;
            if (!acc_session_) { acc_cal_.reset(); acc_session_ = true; }
            acc_cal_.begin_point(cal_samples_); cal_kind_ = wbp::CAL_KIND_ACCEL; return wbp::ERR_OK;
        case wbp::CMD_ACC_CAL_FINISH: {
            if (!acc_session_ || acc_cal_.running() || !acc_cal_.complete()) return wbp::ERR_BAD_STATE;       // not all six faces yet
            AccCal c;
            if (!acc_cal_.fit(c)) { beep_event(beep::ACC_CALIBRATION_FAIL); return wbp::ERR_FAILED; }
            imu.setAccCalibration(c); wbp::from_acccal(c, s.calibration); acc_session_ = false; acc_done_ = true;
            beep_event(beep::ACC_CALIBRATION);
            return wbp::ERR_OK; }
        case wbp::CMD_ACC_CAL_CANCEL: acc_cal_.reset(); acc_session_ = false; if (cal_kind_ == wbp::CAL_KIND_ACCEL) cal_kind_ = wbp::CAL_KIND_NONE; return wbp::ERR_OK;

        case wbp::CMD_LEVEL_TRIM: {
            if (!status.imu) return wbp::ERR_NO_SENSOR;
            for (int a = 0; a < 3; a++) if (fabsf(last_out_.gyro_f_dps[a]) > 3.0f) return wbp::ERR_FAILED;   // must be still
            const fc::EulerDeg e = core.ahrs().euler_aero();                                                 // the (already filtered) attitude estimate at rest
            return set_level_from_attitude(e.roll, e.pitch) ? wbp::ERR_OK : wbp::ERR_FAILED;                 // more than 20 degrees: that is a board alignment, not a trim
        }

        case wbp::CMD_MAG_AUTOCAL:
            return start_mag_autocal(now_ms) ? wbp::ERR_OK : (status.mag ? wbp::ERR_BUSY : wbp::ERR_NO_SENSOR);
        case wbp::CMD_MAG_CAL_START:
            if (!status.mag) return wbp::ERR_NO_SENSOR;
            mag_auto_.cancel();
            mag.startCalibration(); cal_kind_ = wbp::CAL_KIND_MAG; mag_cal_state_ = CalState::RUNNING; return wbp::ERR_OK;
        case wbp::CMD_MAG_CAL_FINISH: {
            if (!mag.calibrating()) return wbp::ERR_BAD_STATE;
            const MagCalResult res = mag.finishCalibration();
            mag_result_ = (uint8_t)res;
            if (res != MagCalResult::OK) { beep_event(beep::ACC_CALIBRATION_FAIL); return app::Reply(wbp::ERR_FAILED, (uint8_t)res); }   // keeps collecting: rotate more, then finish again
            wbp::from_magcal(mag.calibration(), s.calibration); mag_cal_state_ = CalState::DONE;
            beep_event(beep::GYRO_CALIBRATED);                                                                // Betaflight reuses the 3 beeps for "compass calibrated"
            return app::Reply(wbp::ERR_OK, (uint8_t)res); }
        case wbp::CMD_MAG_CAL_CANCEL: mag.cancelCalibration(); mag_auto_.cancel(); mag_cal_state_ = CalState::IDLE; if (cal_kind_ == wbp::CAL_KIND_MAG || cal_kind_ == wbp::CAL_KIND_MAG_AUTO) cal_kind_ = wbp::CAL_KIND_NONE; return wbp::ERR_OK;

        case wbp::CMD_MOTOR_TEST: {
            if (!status.motors) return wbp::ERR_NO_SENSOR;
            const uint8_t m = r.args[0]; const uint16_t v = (uint16_t)(r.args[1] | (r.args[2] << 8));
            if (v > 1000 || (m != 255 && m >= mm.cfg.count)) return wbp::ERR_BAD_ARG;
            if (m == 255) for (int i = 0; i < mm.cfg.count; i++) mm.test_motor(i, v * 0.001f, now_us); else mm.test_motor(m, v * 0.001f, now_us);
            test_until_ms_ = now_ms + 600; return wbp::ERR_OK; }
        case wbp::CMD_MOTOR_STOP: mm.test_stop(); test_until_ms_ = now_ms; return wbp::ERR_OK;
        case wbp::CMD_MOTOR_DIRECTION:
            if (!status.motors) return wbp::ERR_NO_SENSOR;
            if (r.args[0] >= mm.cfg.count || r.args[1] > 1) return wbp::ERR_BAD_ARG;
            return mm.request_direction(r.args[0], r.args[1] != 0) ? wbp::ERR_OK : wbp::ERR_NOT_SUPPORTED;    // DShot only
        case wbp::CMD_MOTOR_BEACON:
            if (!status.motors) return wbp::ERR_NO_SENSOR;
            if (r.args[0] >= mm.cfg.count) return wbp::ERR_BAD_ARG;
            return mm.beacon(r.args[0]) ? wbp::ERR_OK : wbp::ERR_NOT_SUPPORTED;
        case wbp::CMD_ESC_TELEMETRY:
            if (!status.motors) return wbp::ERR_NO_SENSOR;
            return mm.enable_extended_telemetry(r.args[0] != 0) ? wbp::ERR_OK : wbp::ERR_NOT_SUPPORTED;

        case wbp::CMD_SET_HOME:
            if (!status.gps || !gnss.usable()) return wbp::ERR_NO_SENSOR;
            gnss.home().reset(gnss.solution()); core.reset_position();
            return gnss.home().has_home() ? wbp::ERR_OK : wbp::ERR_FAILED;

        case wbp::CMD_BEEP:                                    // the app's "try it" button; 255 = silence
            if (r.args[0] == 255) { beeper.silence(); motor_beeper.silence(); return wbp::ERR_OK; }
            if (!beep::has_sound(r.args[0])) return wbp::ERR_BAD_ARG;
            if (!status.buzzer && !status.motor_beep) return wbp::ERR_NO_SENSOR;
            beeper.force(r.args[0]);
            if (beep_cond_.motors_ready()) motor_beeper.force(r.args[0]);   // the test button also works on the bench (forced sounds are not muted)
            return wbp::ERR_OK;
        default: return wbp::ERR_NOT_SUPPORTED;
        }
    }

private:
    // One control cycle. The caller holds the lock: it asked for it at now_us and got it at locked_us.
    WB_FAST bool fast_cycle(uint32_t now_us, uint32_t locked_us) {
        AgSample s;
        if (!imu_sel_.read([this](AgSample& x) { return imu.read(x); }, [this](AgSample& x) { return imu2.read(x); }, s)) {
            // one missed sample: skip this cycle (the ESCs keep their last command). 20 in a row (5 ms at 4 kHz): the IMU is gone -
            // stop the motors explicitly, stay disarmed until the arm switch is cycled, and tell the app why.
            imu_errors_++;
            if (++imu_fail_streak_ >= 20) {
                core.emergency_disarm();
                last_out_ = fc::CoreOutput(); last_out_.arm_blockers = fc::AB_NO_IMU;
                motors.update(now_us, last_out_.motor, false);
                tel_.loop_us = 0; if (++pub_n_ >= pub_div_) { pub_n_ = 0; fill_fast(); link.publish(tel_); }
            }
            return false;
        }
        imu_fail_streak_ = 0;
        const uint32_t t_imu = micros();
        { const uint32_t gap = now_us - start_us_; start_us_ = now_us; cycle_gap_ = gap; if (gap > gap_max_) gap_max_ = gap; }
        const uint32_t now_ms = millis();                 // NOT now_us / 1000: micros() wraps after 71 minutes, millis() after 49 days
        // dt is the time that really passed since the previous cycle: the gyro is integrated over exactly that. It is bounded
        // only against nonsense (a clock step); a cycle that came late is NOT shortened.
        float dt = (now_us - last_us_) * 1e-6f; last_us_ = now_us;
        const float nominal = period_us_ * 1e-6f;
        if (dt < 0.25f * nominal) dt = 0.25f * nominal; else if (dt > 8.0f * nominal) dt = 8.0f * nominal;
        if (acc_cal_.running()) acc_cal_.feed(imu.rawAcc());
        // the EKF's input: every sample integrated (delta angle, delta velocity, FRD) - the sensor thread takes the sums at 250 Hz
        if (ekf_mode_) {
            if (ekf_acc_.dt > 0.1f) ekf_acc_ = EkfDelta();             // (nobody took them: the EKF starts over from the next ones)
            for (int k = 0; k < 3; k++) { ekf_acc_.dang[k] += s.gyro_dps[k] * fc::kDeg2Rad * dt; ekf_acc_.dvel[k] += s.acc_g[k] * fc::kG0 * dt; }
            ekf_acc_.dt += dt;
        }

        fc::FastInput in;
        in.gyro_dps = {s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2]}; in.acc_g = {s.acc_g[0], s.acc_g[1], s.acc_g[2]};
        in.dt = dt; in.now_ms = now_ms; in.rc_link = rc_link_; in.rc_seq = rc_seq_; for (int i = 0; i < 16; i++) in.rc_us[i] = rc_[i];
        in.health = health_;
        // (the sensor thread fills the health once per millisecond; what the IMU itself knows is taken now)
        in.health.imu_ok = imu_sel_.in_use() != 0; in.health.gyro_calibrated = gyro_calibrated(); in.health.calibrating = busy_other_ || gyro_cal_state() == CalState::RUNNING;
        in.motors.valid_mask = motors.manager().rpm_array(in.motors.rpm, now_ms); in.motors.rpm_valid = motors.manager().rpm_any_valid(now_ms);     // (a motor without telemetry reads 0: its notches fade out, the others stay)
        in.limits.idle = motors.idleFraction(now_ms, last_out_.throttle > 0.1f); in.limits.limit = motors.cfg().output_limit;
        last_out_ = core.update(in);
        const uint32_t t_core = micros();
        const bool ctl = last_out_.pid_stepped;           // a control cycle (every pid_denom-th sample): a new motor command, a log record
        if (ctl) {
            motors.update(now_us, last_out_.motor, last_out_.armed);
            motors.pollTelemetry(now_ms);                 // the ESC replies of the previous frame: fresh rpm for the next cycle's filters
            pid_loops_++;
        }
        const uint32_t t_motor = micros();
        ctl_gap_ += cycle_gap_;
        if (blackbox.active()) {                           // where the time goes: the longest of each stage, for the log (bb_format.h)
            peak_.lock = peak16(peak_.lock, locked_us - now_us); peak_.imu = peak16(peak_.imu, t_imu - locked_us); if (ctl) peak_.motor = peak16(peak_.motor, t_motor - t_core);
            if (last_out_.est_stepped) peak_.est = peak16(peak_.est, t_core - t_imu); else peak_.core = peak16(peak_.core, t_core - t_imu);
        } else peak_ = StagePeaks();

        // ---- telemetry: fast fields every cycle (a few copies), published at <= 1 kHz
        tel_.gyro_raw[0] = s.gyro_dps[0]; tel_.gyro_raw[1] = -s.gyro_dps[1]; tel_.gyro_raw[2] = s.gyro_dps[2];   // pilot convention, like the core
        tel_.gyro_src = imu_sel_.gyro_src();
        for (int i = 0; i < 3; i++) { tel_.acc[i] = s.acc_g[i]; tel_.gyro_f[i] = last_out_.gyro_f_dps[i]; tel_.sp[i] = last_out_.sp_dps[i]; }
        loops_++;
        // ---- blackbox: edge events every cycle, one fast record every few cycles while a session runs
        blackbox.note_state(now_us, last_out_.armed, (uint8_t)last_out_.failsafe, last_out_.modes, batt_state_, (uint8_t)last_out_.ah_state);
        const uint32_t ctl_gap = ctl_gap_; if (ctl) ctl_gap_ = 0;
        if (ctl && blackbox.fast_due(ctl_gap)) {
            float mo[8]; for (int i = 0; i < 8; i++) mo[i] = motors.manager().output(i);
            bb::FastSample fs;
            fs.t_us = now_us; fs.gyro = last_out_.gyro_f_dps; fs.gyro_raw = tel_.gyro_raw; fs.setpoint = last_out_.sp_dps;
            fs.pid_p = last_out_.pid_p; fs.pid_i = last_out_.pid_i; fs.pid_d = last_out_.pid_d; fs.pid_f = last_out_.pid_f;
            fs.stick = last_out_.stick; fs.throttle = last_out_.throttle; fs.motor = mo; fs.rpm = in.motors.rpm; fs.acc = tel_.acc;
            fs.roll = last_out_.est.att.roll; fs.pitch = last_out_.est.att.pitch; fs.yaw = app::heading360(last_out_.est.att.yaw);
            fs.loop_us = micros() - now_us;
            fs.t_lock_us = peak_.lock; fs.t_imu_us = peak_.imu; fs.t_core_us = peak_.core; fs.t_est_us = peak_.est; fs.t_motor_us = peak_.motor; peak_ = StagePeaks();
            blackbox.log_fast(fs);
        }
        if (++pub_n_ >= pub_div_) { pub_n_ = 0; fill_fast(); link.publish(tel_); }
        const uint32_t t_end = micros(), took = t_end - now_us;
        // ("rest" = waiting for the lock + log + telemetry: the IMU number is the bus alone)
        t_imu_.add(t_imu - locked_us); t_core_.add(t_core - t_imu); if (ctl) t_motor_.add(t_motor - t_core); t_rest_.add((t_end - t_motor) + (locked_us - now_us));
        load_.add(took);
        tel_.loop_us = took > 65535u ? 65535 : (uint16_t)took; if (tel_.loop_us > loop_max_) loop_max_ = tel_.loop_us;
        return true;
    }

    // Once a second (sensor thread, lock held): the numbers of the last second.
    void every_second(uint32_t now_us, uint32_t now_ms) {
        const sys::Memory m = sys::memory();
        tel_.cpu_pct = load_.take_percent(now_us); tel_.ram_pct = m.used_percent(); tel_.heap_free_kb = sys::kb16(m.heap_free); tel_.psram_free_kb = sys::kb16(m.psram_free); tel_.psram_total_kb = sys::kb16(m.psram_total);
        const uint32_t span_ms = now_ms - stat_ms_; stat_ms_ = now_ms;
        tel_.loop_max_us = loop_max_; loop_max_ = 0; tel_.gap_max_us = gap_max_ > 65535u ? 65535 : (uint16_t)gap_max_; gap_max_ = 0;
        const uint32_t cycles = loops_ - loops_mark_; loops_mark_ = loops_;
        const uint32_t hz = span_ms ? (cycles * 1000u + span_ms / 2) / span_ms : 0;
        tel_.loop_hz = hz > 65535u ? 65535 : (uint16_t)hz;
        { const uint32_t pc = pid_loops_ - pid_mark_; pid_mark_ = pid_loops_; const uint32_t ph = span_ms ? (pc * 1000u + span_ms / 2) / span_ms : 0; tel_.pid_hz = ph > 65535u ? 65535 : (uint16_t)ph; }
        tel_.t_imu_us = t_imu_.take_avg(); tel_.t_core_us = t_core_.take_avg(); tel_.t_motor_us = t_motor_.take_avg(); tel_.t_rest_us = t_rest_.take_avg(); tel_.t_slow_max_us = t_slow_.take_max();
        tel_.overruns = app::cycles_lost(cycles, span_ms, core.cfg.loop_hz);
        // "The loop cannot hold its rate" is a lasting condition: two seconds in a row below 80 %. One slow second is a pause
        // somebody asked for (saving the settings stops both cores for a moment), and so is every second during which the app
        // was doing something with the craft at any time (erasing the log memory takes seconds).
        const bool slow = status.imu && !busy_seen_ && !app::loop_rate_held(hz, core.cfg.loop_hz);
        busy_seen_ = busy_other_;
        slow_seconds_ = slow ? (uint8_t)(slow_seconds_ < 0xFF ? slow_seconds_ + 1 : 0xFF) : (uint8_t)0;
        loop_slow_ = slow_seconds_ >= 2;
        { auto d = [](uint32_t now, uint32_t& was) { const uint32_t x = now - was; was = now; return (uint16_t)(x > 65535u ? 65535u : x); };
          tel_.esc_tlm_frames = d(esc_poll_.frames(), esc_mark_[0]); tel_.esc_tlm_crc = d(esc_poll_.crc_errors(), esc_mark_[1]); tel_.esc_tlm_timeouts = d(esc_poll_.timeouts(), esc_mark_[2]); }
        tel_.stall_motor = last_out_.stall_motor;
        if (status.can) {                                    // the CAN bus and its nodes, for the app
            app::Telemetry::Can& c = tel_.can; auto d = [](uint32_t now, uint32_t& was) { const uint32_t x = now - was; was = now; return (uint16_t)(x > 65535u ? 65535u : x); };
            c.state = can_port->state(); c.node_id = can_node.id(); c.tec = can_port->tec(); c.rec = can_port->rec();
            c.rx_fps = d(can_node.rx_frames(), can_mark_[0]); c.tx_fps = d(can_node.tx_frames(), can_mark_[1]); c.tx_dropped = d(can_node.tx_dropped(), can_mark_[2]);
            c.crc_errors = (uint16_t)(can_node.crc_errors() > 65535u ? 65535u : can_node.crc_errors()); c.bus_off = (uint16_t)(can_port->bus_offs() > 65535u ? 65535u : can_port->bus_offs());
            c.allocations = (uint8_t)(can_node.allocations() > 255u ? 255u : can_node.allocations()); c.gnss_node = can_node.gnss_node(); c.mag_node = can_node.mag_node();
            c.baro_node = can_node.baro_node(); c.batt_node = can_node.battery_node(); c.esc_mask = can_node.esc_mask(now_ms); c.esc_sent = can_node.esc_sent();
            c.esc_dropped = (uint16_t)(can_node.esc_dropped() > 65535u ? 65535u : can_node.esc_dropped());
            c.nodes = (uint8_t)can_node.nodes(); memset(c.names, 0, sizeof(c.names));
            for (int i = 0; i < 8; i++) {
                if (i >= can_node.nodes()) { c.node[i] = 0; c.health[i] = 0; c.mode[i] = 0; c.uptime[i] = 0; continue; }
                const dcan::NodeEntry& e = can_node.node(i); const bool on = can_node.online(e, now_ms);
                c.node[i] = e.id; c.health[i] = on ? e.health : 255; c.mode[i] = on ? e.mode : 7; c.uptime[i] = e.uptime;
                for (int k = 0; k < 16 && e.name[k]; k++) c.names[i * 16 + k] = e.name[k];
            }
        } else tel_.can.state = s_->can.mode == wbp::CAN_MODE_OFF ? 0 : 3;      // (asked for but not running: FAILED)
        const motor::HwStats hs = motors.hwStats();
        tel_.hw_skipped = (uint16_t)(hs.skipped - hw_mark_.skipped > 65535u ? 65535u : hs.skipped - hw_mark_.skipped);
        tel_.hw_late = (uint16_t)(hs.late_switch - hw_mark_.late_switch > 65535u ? 65535u : hs.late_switch - hw_mark_.late_switch);
        tel_.hw_replies = (uint16_t)(hs.replies - hw_mark_.replies > 65535u ? 65535u : hs.replies - hw_mark_.replies);
        tel_.hw_garbled = (uint16_t)(hs.garbled - hw_mark_.garbled > 65535u ? 65535u : hs.garbled - hw_mark_.garbled);
        // receiver: frames thrown away, UART overflows / line errors in the last second
        { const uint32_t bad = rc.frameErrors(), ovf = rc.uartOverflows(), line = rc.uartLineErrors();
          auto d16 = [](uint32_t now, uint32_t was) { const uint32_t d = now - was; return (uint16_t)(d > 65535u ? 65535u : d); };
          tel_.rc_bad = d16(bad, rc_mark_[0]); tel_.rc_ovf = d16(ovf, rc_mark_[1]); tel_.rc_line = d16(line, rc_mark_[2]);
          rc_mark_[0] = bad; rc_mark_[1] = ovf; rc_mark_[2] = line; }
        tel_.hw_no_start = (uint16_t)(hs.no_start - hw_mark_.no_start > 65535u ? 65535u : hs.no_start - hw_mark_.no_start);
        tel_.hw_bad_bits = (uint16_t)(hs.bad_bits - hw_mark_.bad_bits > 65535u ? 65535u : hs.bad_bits - hw_mark_.bad_bits);
        { const uint32_t crc = motors.manager().reply_crc_errors(); tel_.hw_bad_crc = (uint16_t)(crc - crc_mark_ > 65535u ? 65535u : crc - crc_mark_); crc_mark_ = crc; }
        hw_mark_ = hs;
    }

    // The level trim that makes the attitude (roll, pitch: Ahrs::euler_aero(), degrees) read as level. Stores it in the calibration
    // section (RAM) and applies it. false: more than 20 degrees - that is a board alignment, not a trim.
    bool set_level_from_attitude(float roll_deg, float pitch_deg) {
        wbp::Settings& s = *s_; const float d2r = 0.017453292519943295f;
        // the resting specific force in the board frame for that attitude
        const float acc[3] = {sinf(pitch_deg * d2r), -sinf(roll_deg * d2r) * cosf(pitch_deg * d2r), -cosf(roll_deg * d2r) * cosf(pitch_deg * d2r)};
        float cr, cp, cy, nr, np; wbp::board_fine_alignment(s, cr, cp, cy);
        if (!level_trim(acc, cr, cp, cy, nr, np)) return false;
        s.calibration.acc_trim_roll = (int16_t)lroundf(nr * 10.0f) - s.general.board_trim_roll;
        s.calibration.acc_trim_pitch = (int16_t)lroundf(np * 10.0f) - s.general.board_trim_pitch;
        wbp::validate(s.calibration); apply_alignment();
        return true;
    }
    // In-flight level trim (mode CALIB, core/inflight_trim.h). The result is applied after the disarm and written to flash by the
    // link task (Telemetry::fw_saved_*): the pilot is in the field, there is no app to press Save.
    // Stick commands while disarmed (app/stick_commands.h): the gyro calibration, the automatic compass calibration.
    void update_stick_commands(uint32_t now_ms, bool armed) {
        const wbp::S_receiver& rx = s_->receiver; auto ch = [this](uint8_t i) { return i < 16 ? rc_[i] : (uint16_t)1500; };
        switch (sticks_.update(ch(rx.ch_roll), ch(rx.ch_pitch), ch(rx.ch_yaw), ch(rx.ch_throttle), armed, rc_link_ && rc_seq_ != 0, now_ms)) {
        case app::StickCommands::GYRO_CAL: if (status.imu) { start_gyro_cal(); cal_kind_ = wbp::CAL_KIND_GYRO; } break;
        case app::StickCommands::MAG_CAL: if (!start_mag_autocal(now_ms)) beep_event(beep::ACC_CALIBRATION_FAIL); break;
        default: break;
        }
    }
    // The automatic compass calibration (hal/mag/mag_autocal.h): started by the app or the sticks, disarmed, ends by itself.
    bool start_mag_autocal(uint32_t now_ms) {
        if (!status.mag || last_out_.armed || mag.calibrating()) return false;
        mag_auto_.start(now_ms, mag.calibration()); cal_kind_ = wbp::CAL_KIND_MAG_AUTO; mag_result_ = 0;
        beep_event(beep::ACC_CALIBRATION);
        return true;
    }
    // Its end: the new offset applied at once and written to flash by the link task (the pilot is in the field: no Save button).
    void update_mag_autocal() {
        switch (mag_auto_.take_finished()) {
        case MagAutoCal::DONE:
            mag.setCalibration(mag_auto_.calibration()); wbp::from_magcal(mag_auto_.calibration(), s_->calibration); wbp::validate(s_->calibration);
            mag_result_ = (uint8_t)MagCalResult::OK; tel_.fw_saved_section = wbp::SEC_CALIBRATION; tel_.fw_saved_seq++; beep_event(beep::GYRO_CALIBRATED);
            break;
        case MagAutoCal::FAILED: mag_result_ = (uint8_t)mag_auto_.result(); beep_event(beep::ACC_CALIBRATION_FAIL); break;
        default: break;
        }
    }
    void update_inflight_trim(uint32_t now_ms) {
        const float dt = (uint32_t)(now_ms - trim_ms_) * 0.001f; trim_ms_ = now_ms;
        if (!status.imu || dt <= 0.0f || dt > 0.1f) return;
        fc::InflightTrim::Input in;
        // flying = in the air by the core's own judgement - not "the stick is above 15 %": a strong craft hovers below that, and a
        // craft idling on a slope before its take-off is not flying.
        in.armed = last_out_.armed; in.flying = last_out_.armed && last_out_.airborne && !last_out_.ground_idle; in.switch_on = ((last_out_.modes >> fc::M_CALIB) & 1u) != 0;
        const fc::EulerDeg e = core.ahrs().accel_euler_aero(); in.roll_deg = e.roll; in.pitch_deg = e.pitch;        // the accelerometer itself: see inflight_trim.h
        in.gyro_dps = fmaxf(fabsf(last_out_.gyro_f_dps[0]), fmaxf(fabsf(last_out_.gyro_f_dps[1]), fabsf(last_out_.gyro_f_dps[2])));
        switch (inflight_trim.update(in, dt)) {
        case fc::InflightTrim::MEASURED: beep_event(beep::ACC_CALIBRATION); break;
        case fc::InflightTrim::FAILED: beep_event(beep::ACC_CALIBRATION_FAIL); break;
        case fc::InflightTrim::APPLY:
            if (set_level_from_attitude(inflight_trim.roll_deg(), inflight_trim.pitch_deg())) { tel_.fw_saved_section = wbp::SEC_CALIBRATION; tel_.fw_saved_seq++; beep_event(beep::ACC_CALIBRATION); }
            else beep_event(beep::ACC_CALIBRATION_FAIL);
            break;
        default: break;
        }
        tel_.trim_progress = inflight_trim.collecting() ? (uint8_t)(inflight_trim.progress() * 100.0f + 0.5f) : (inflight_trim.pending() ? 100 : 0);
    }
    void apply_alignment() {
        float r, p, y; wbp::board_fine_alignment(*s_, r, p, y);
        imu.setAlignment((AccGyroAlign)s_->general.board_align, r, p, y);
        imu2.setAlignment((AccGyroAlign)s_->sensors.imu2_align, r, p, y);      // its own chip orientation, the same board trims
    }
    // ---- the two IMUs as one: what counts is the chip(s) the loop flies on (imusel::Selector::in_use)
    uint16_t cal2_samples() const { return imu_sel_.use() == imusel::FIRST ? (uint16_t)(cal_samples_ / imusel::Selector::kStandbyEvery) : cal_samples_; }     // (the stand-by is read every 8th cycle)
    void start_gyro_cal() { imu.startGyroCalibration(cal_samples_); if (status.imu2) imu2.startGyroCalibration(cal2_samples()); }
    WB_FAST bool gyro_calibrated() const {
        const uint8_t u = imu_sel_.in_use(); if (!u) return false;
        return (!(u & imusel::kImu1) || imu.gyroCalibrated()) && (!(u & imusel::kImu2) || imu2.gyroCalibrated());
    }
    WB_FAST CalState gyro_cal_state() const {                    // RUNNING while one of them runs, FAILED when one failed, else the first one's
        const uint8_t u = imu_sel_.in_use(); const CalState a = imu.gyroCalState(), b = imu2.gyroCalState();
        const bool ua = (u & imusel::kImu1) != 0 || !u, ub = (u & imusel::kImu2) != 0;
        if ((ua && a == CalState::RUNNING) || (ub && b == CalState::RUNNING)) return CalState::RUNNING;
        if ((ua && a == CalState::FAILED) || (ub && b == CalState::FAILED)) return CalState::FAILED;
        return ua ? a : b;
    }
    uint8_t gyro_cal_progress() const {
        const uint8_t u = imu_sel_.in_use(); uint8_t p = 100;
        if ((u & imusel::kImu1) || !u) p = imu.gyroCalProgress();
        if ((u & imusel::kImu2) && imu2.gyroCalProgress() < p) p = imu2.gyroCalProgress();
        return p;
    }
    void apply_calibration() {
        MagCal m; wbp::to_magcal(s_->calibration, m); if (m.valid) mag.setCalibration(m);
        AccCal a; wbp::to_acccal(s_->calibration, a); imu.setAccCalibration(a);
        apply_alignment();
    }
    void apply_battery() {
        wbp::Settings& s = *s_;
        wbp::apply_battery(s, battery.monitor.cfg); battery.setSource((batt::Source)s.battery.source); battery.setDivider(wbp::battery_divider(s));
        battery.setCurrent((batt::CurrentSource)s.battery.current_source, s.pins.curr_adc, s.battery.current_scale, s.battery.current_offset);
        land_on_critical_ = s.battery.land_on_critical != 0; capacity_mah_ = s.battery.capacity_mah;
    }
    void apply_servo() {                                 // (copied out: the settings struct is packed, a pointer into it may be unaligned - an exception on the ESP32)
        const wbp::S_servo& v = s_->servo; uint8_t src[servo::kServos], rev[servo::kServos]; uint16_t lo[servo::kServos], hi[servo::kServos];
        for (int i = 0; i < servo::kServos; i++) { src[i] = v.source[i]; lo[i] = v.min_us[i]; hi[i] = v.max_us[i]; rev[i] = v.reverse[i]; }
        servos.configure(src, lo, hi, rev);
    }
    void apply_beeper() {
        wbp::Settings& s = *s_;
        wbp::apply_beeper(s, beeper, motor_beeper);
        buzzer.configure((BuzzerOutput::Type)s.beeper.type, s.beeper.inverted != 0, s.beeper.frequency, s.beeper.volume);
    }
    // Sections that take effect without a reboot (schema: "live"). Disarmed only (checked by the gate).
    bool apply_live(uint8_t section) {
        wbp::Settings& s = *s_;
        switch (section) {
        case wbp::SEC_MODES: wbp::apply_modes(s, core.rc); wbp::apply_nav(s, core.cfg); return true;      // (a RESCUE switch makes the rescue "set up": arming then wants a fix)
        case wbp::SEC_NAV: wbp::apply_nav(s, core.cfg); core.apply_nav(); return true;
        case wbp::SEC_RATES: wbp::apply_rates_and_rc(s, core.cfg, core.rc); core.apply_tuning(); return true;      // (the tilt limit of Angle / Stabilize is in this section)
        case wbp::SEC_PID: wbp::apply_pid(s, core.cfg); core.apply_tuning(); return true;
        case wbp::SEC_FILTERS: wbp::apply_filters(s, core.cfg); core.apply_filters(); return true;
        case wbp::SEC_CALIBRATION: apply_calibration(); return true;
        case wbp::SEC_BEEPER: apply_beeper(); return true;
        case wbp::SEC_BATTERY: apply_battery(); return true;
        case wbp::SEC_SERVO: apply_servo(); return true;
        case wbp::SEC_ALTHOLD: wbp::apply_althold(s, core.cfg.alt); core.apply_althold(); return true;
        case wbp::SEC_BLACKBOX: return true;                                      // used by bb::Service (link task), nothing to do in the flight loop
        case wbp::SEC_MOTORS: return apply_motor_order();
        default: return false;
        }
    }
    // The motor wizard (app, Motor tab) changes motors.order / reversed_mask and wants the result at once: the next step spins
    // the motors in the new order. Only those two fields take effect live (disarmed: the gate); any other change of the section
    // since the outputs started (protocol, layout, pulses ...) still needs the restart - then nothing is applied.
    bool apply_motor_order() {
        wbp::S_motors want = s_->motors, run = motors_run_;
        memcpy(want.order, run.order, sizeof(want.order)); want.reversed_mask = run.reversed_mask;
        if (!status.motors || memcmp(&want, &run, sizeof(run)) != 0) return false;
        motor::MotorOutputManager& mm = motors.manager();
        if (!mm.set_order(s_->motors.order)) return false;                      // not a permutation of the motors: the old order stays
        for (int i = 0; i < motor::kMaxMotors; i++) mm.cfg.reversed[i] = ((s_->motors.reversed_mask >> i) & 1) != 0;
        return true;
    }

    // anything the app is doing with the craft right now: arming is refused meanwhile
    bool busy_other(uint32_t now_ms) const { return acc_session_ || mag.calibrating() || mag_auto_.active() || (int32_t)(test_until_ms_ - now_ms) > 0 || reboot_pending_ || blackbox.busy(); }
    fc::Health health(uint32_t now_ms) {
        fc::Health h;
        h.imu_ok = imu_sel_.in_use() != 0; h.gyro_calibrated = gyro_calibrated();      // (`calibrating` is filled in by the flight loop, see fill_fast)
        h.baro_ok = baro.valid(now_ms); h.range_ok = sonar.valid(now_ms); h.gps_ok = gnss.usable(); h.flow_ok = status.flow && flow.healthy(now_ms) && sonar.available();      // (flow without a range sensor gives no speed: not something to hold a position with)
        h.mag_calibrated = mag.calibrated(); h.mag_ok = mag.readyForFusion(now_ms);
        h.batt_low = battery.monitor.low(); h.batt_critical = land_on_critical_ && battery.monitor.critical();
        h.pin_conflict = status.pin_conflict; h.loop_slow = loop_slow_;
        return h;
    }
    // ---- beeper: the state of the craft -> sounds; the buzzer pin follows the running pattern
    void update_beeper(uint32_t now_ms, bool armed) {
        // one calibration point finished: tell the person holding the craft without looking at the screen
        const bool acc_running = acc_cal_.running();
        if (acc_was_running_ && !acc_running) beep_event(acc_cal_.state() == CalState::DONE ? beep::ACC_CALIBRATION : beep::ACC_CALIBRATION_FAIL);
        acc_was_running_ = acc_running;

        beep::Input bi;
        bi.now_ms = now_ms; bi.armed = armed; bi.rx_ok = rc_link_; bi.failsafe_landing = last_out_.failsafe == fc::FsStage::DESCENT;
        bi.gyro_cal_running = gyro_cal_state() == CalState::RUNNING; bi.gyro_calibrated = gyro_calibrated();
        bi.gps_enabled = status.gps; bi.gps_ready = gnss.usable();
        bi.batt_state = (uint8_t)battery.monitor.state();
        bi.beeper_switch = ((last_out_.modes >> fc::M_BEEPER) & 1u) != 0; bi.throttle_low = last_out_.throttle_low; bi.arm_refused = last_out_.arm_refused;
        bi.app_active = link.link_active(now_ms);
        // the motors can beep: DShot, outputs running, and the app is not turning a motor with the test slider
        motor::MotorOutputManager& mm = motors.manager();
        const bool motors_ok = status.motor_beep && !mm.testing(micros());
        beep_cond_.update(bi, beeper, &motor_beeper, motors_ok);
        buzzer.set(beeper.update(now_ms));
        motor_beeper.update(now_ms);
        if (motor_beeper.take_start() && mm.beacon_all(s_->beeper.dshot_beacon_tone, micros())) beacons_++;
    }
    // a one-shot sound (calibration finished ...). On the bench Conditions::update() silences it within a millisecond, before the pin moves.
    void beep_event(uint8_t mode) { beep_cond_.event(mode, beeper); }

    // Every pin that is really in use, checked once at boot: two functions on one GPIO, or a pin the module needs for itself.
    void check_pins() {
        wbp::Settings& s = *s_; HardwareConfig hw;
        pins::Use use[48]; int n = 0;
        auto add = [&](uint8_t pin, const char* what) { if (n < 48) use[n++] = {pin, what}; };
        uint8_t p[8];
        add(s.pins.imu2_cs, "imu2_cs");
        hw.getSPIAccGyroSensorPins(p); add(p[0], "imu_cs"); add(p[1], "imu_miso"); add(p[2], "imu_mosi"); add(p[3], "imu_sck");
        hw.getI2CBaroSensorPins(p); add(p[0], "i2c_sda"); add(p[1], "i2c_scl");
        if (s.sensors.gps_enabled && gps::uses_uart((gps::Provider)s.gps.provider)) { hw.getGpsPins(p); add(p[0], "gps_rx"); add(p[1], "gps_tx"); }
        if (s.can.mode != wbp::CAN_MODE_OFF) { add(s.pins.can_tx, "can_tx"); add(s.pins.can_rx, "can_rx"); }
        if (!use_external_rc) {
            static const char* const pwname[8] = {"rx_pwm 1", "rx_pwm 2", "rx_pwm 3", "rx_pwm 4", "rx_pwm 5", "rx_pwm 6", "rx_pwm 7", "rx_pwm 8"};
            if (s.receiver.protocol == rx::PROTO_PPM) add(s.pins.ppm, "ppm");
            else if (s.receiver.protocol == rx::PROTO_PWM) { for (int i = 0; i < 8; i++) add(s.pins.rx_pwm[i], pwname[i]); }
            else { hw.getSerialRxPins(p); add(p[0], "rx_rx"); add(p[1], "rx_tx"); }
        }
        static const char* const mname[8] = {"motor 1", "motor 2", "motor 3", "motor 4", "motor 5", "motor 6", "motor 7", "motor 8"};
        if (!motor::is_can((motor::Protocol)s.motors.protocol)) hw.getMotorPins(p); else memset(p, GPIO_UNUSED, sizeof(p));        // (ESCs on the CAN bus: no motor pins)
        for (int i = 0; i < fc::mixer_motor_count((fc::Layout)s.motors.layout); i++) add(p[i], mname[i]);
        hw.getBatteryAdcPin(p); if (s.battery.source != batt::SRC_ESC && s.battery.source != batt::SRC_NONE && s.battery.source != batt::SRC_DRONECAN) add(p[0], "batt_adc");
        if (s.battery.current_source != batt::CUR_ESC && s.battery.current_source != batt::CUR_NONE && s.battery.current_source != batt::CUR_DRONECAN) add(s.pins.curr_adc, "curr_adc");
        hw.getBuzzerPin(p); add(p[0], "buzzer");
        static const char* const sname[servo::kServos] = {"servo 1", "servo 2", "servo 3", "servo 4"};
        for (int i = 0; i < servo::kServos; i++) add(s.pins.servo[i], sname[i]);
        if (s.flow.type == FlowSensor::PMW3901) { add(s.pins.flow_cs, "flow_cs"); add(s.pins.flow_sck, "flow_sck"); add(s.pins.flow_miso, "flow_miso"); add(s.pins.flow_mosi, "flow_mosi"); }
        if (s.flow.type == FlowSensor::MSP) { add(s.pins.flow_rx, "flow_rx"); add(s.pins.flow_tx, "flow_tx"); }
        if (s.motors.esc_sensor) add(s.pins.esc_tlm, "esc_tlm");
        if (s.telemetry.protocol == wbp::TELEMETRY_PROTOCOL_MAVLINK) { add(s.telemetry.tx_pin, "telemetry_tx"); add(s.telemetry.rx_pin, "telemetry_rx"); }
        if (s.osd.type != wbp::OSD_TYPE_NONE) { add(s.osd.tx_pin, "osd_tx"); add(s.osd.rx_pin, "osd_rx"); }
        if (s.led.count) add(s.pins.led, "led");
        if (s.blackbox.device == bb::DEV_SDCARD) { hw.getSdCardPins(p); if (s.blackbox.sd_mode == wbp::SD_MODE_SPI) add(p[0], "sd_cs"); add(p[1], "sd_clk"); add(p[2], "sd_cmd"); add(p[3], "sd_d0"); }
        const pins::Conflict c = pins::find_conflict(use, n);
        status.pin_conflict = c.found;
        char line[kReportLen];
        char pb[8];
        if (c.found) { snprintf(line, sizeof(line), "pins: FAILED - %s is set for both %s and %s (arming is blocked until the Pins tab is fixed)", pin_str(c.pin, pb), c.a, c.b); add_report(line); }
        const pins::Use* r = pins::find_reserved(use, n, sys::memory().psram_octal, WB_PLATFORM_STM32);
        if (r) { snprintf(line, sizeof(line), WB_PLATFORM_STM32 ? "pins: WARNING - %s (%s) is the debug port (SWD): a debug probe cannot connect while the firmware runs" : "pins: WARNING - %s (%s) belongs to the module's own flash / PSRAM on an ESP32-S3: pick another pin", pin_str(r->pin, pb), r->what); add_report(line); }
        const pins::Use* u = usb_link ? pins::find_on_usb(use, n, WB_PLATFORM_STM32) : nullptr;
        if (u) { snprintf(line, sizeof(line), "pins: WARNING - %s (%s) is the USB socket this firmware talks to the app through: pick another pin", pin_str(u->pin, pb), u->what); add_report(line); }
    }

    // what the CRSF telemetry frames carry, from the telemetry the app gets as well (called with the lock held)
    crsf::TelemetryIn crsf_in() const {
        const app::Telemetry& t = tel_; crsf::TelemetryIn c;
        c.volts = t.batt_v; c.amps = t.batt_amps; c.mah = t.batt_mah; c.remaining = t.batt_remaining;
        c.roll_deg = t.roll; c.pitch_deg = t.pitch; c.yaw_deg = t.yaw;
        c.have_gps = status.gps; c.gps_fix = t.gps_fix >= 3; c.lat = t.lat; c.lon = t.lon; c.gs_cms = t.gs_cms; c.course_ddeg = t.course_ddeg; c.sats = t.gps_sats;
        c.alt_m = t.alt_valid ? t.z : t.alt_cm * 0.01f;
        c.have_vario = t.alt_valid; c.vz_ms = t.vz;
        c.armed = t.armed; c.arming_blocked = !t.armed && t.blockers != 0; c.failsafe = t.failsafe != 0;
        c.rescue = (t.modes >> fc::M_RESCUE) & 1u; c.poshold = (t.modes >> fc::M_POSHOLD) & 1u; c.althold = (t.modes >> fc::M_ALTHOLD) & 1u;
        c.angle = (t.modes >> fc::M_ANGLE) & 1u; c.stabilize = (t.modes >> fc::M_STABILIZE) & 1u;
        return c;
    }

    // ---- MAVLink telemetry (telemetry/mavlink_telemetry.h): the state in MAVLink's units and conventions (called with the lock held)
    mav::State mav_in(uint32_t now_ms) const {
        const app::Telemetry& t = tel_; mav::State m; const float d2r = 0.017453292519943295f;
        m.time_ms = now_ms; m.vehicle_type = mav_type_;
        m.armed = t.armed; m.failsafe = t.failsafe != 0; m.calibrating = busy_other_ || gyro_cal_state() == CalState::RUNNING;
        m.angle = (t.modes >> fc::M_ANGLE) & 1u; m.stabilize = (t.modes >> fc::M_STABILIZE) & 1u; m.althold = (t.modes >> fc::M_ALTHOLD) & 1u;
        m.poshold = (t.modes >> fc::M_POSHOLD) & 1u; m.rescue = (t.modes >> fc::M_RESCUE) & 1u;
        // pilot convention (pitch nose up negative) -> MAVLink (FRD: pitch nose up positive); yaw -180 .. 180
        m.roll = t.roll * d2r; m.pitch = -t.pitch * d2r; m.yaw = t.yaw * d2r;
        m.p = t.gyro_f[0] * d2r; m.q = -t.gyro_f[1] * d2r; m.r = t.gyro_f[2] * d2r;
        m.have_gps = status.gps; m.gps_fix = t.gps_fix; m.sats = t.gps_sats; m.lat = t.lat; m.lon = t.lon; m.alt_msl_mm = t.alt_cm * 10;
        m.hdop_x100 = t.hdop; m.gs_cms = t.gs_cms; m.cog_cdeg = (uint16_t)(t.course_ddeg * 10u); m.hacc_mm = t.hacc_mm;
        m.alt_valid = t.alt_valid; m.rel_alt_m = t.z; m.climb_ms = t.vz; m.vel_valid = t.vel_valid; m.vn = t.vel_n; m.ve = t.vel_e;
        m.heading_deg = app::heading360(t.yaw); m.throttle = t.throttle;
        uint32_t pr = 0, hl = 0;                          // MAV_SYS_STATUS_SENSOR: fitted (= enabled), and healthy when its health bits are all set
        const struct { bool on; uint16_t need; uint32_t bits; } sm[] = {
            {status.imu, wbp::HEALTH_IMU, mav::SENSOR_GYRO | mav::SENSOR_ACCEL}, {status.imu, wbp::HEALTH_IMU | wbp::HEALTH_GYRO_CAL, mav::SENSOR_AHRS},
            {status.mag, wbp::HEALTH_MAG_READY, mav::SENSOR_MAG}, {status.baro, wbp::HEALTH_BARO, mav::SENSOR_BARO}, {status.gps, wbp::HEALTH_GPS, mav::SENSOR_GPS},
            {status.flow, wbp::HEALTH_FLOW, mav::SENSOR_FLOW}, {status.range, wbp::HEALTH_RANGE, mav::SENSOR_LASER}, {status.rx || use_external_rc, wbp::HEALTH_RX, mav::SENSOR_RC},
            {status.motors, wbp::HEALTH_MOTORS, mav::SENSOR_MOTORS}, {status.battery, wbp::HEALTH_BATTERY, mav::SENSOR_BATTERY}};
        for (const auto& x : sm) if (x.on) { pr |= x.bits; if ((t.health & x.need) == x.need) hl |= x.bits; }
        m.sensors_present = m.sensors_enabled = pr; m.sensors_health = hl; m.load_pct = t.cpu_pct;
        m.batt_present = status.battery && t.batt_v > 0.5f; m.volts = t.batt_v; m.cells = t.batt_cells; m.remaining = m.batt_present ? (int8_t)(t.batt_remaining > 100 ? 100 : t.batt_remaining) : -1;
        m.have_current = t.curr_source == batt::CUR_ADC || t.curr_source == batt::CUR_ESC || t.curr_source == batt::CUR_DRONECAN; m.amps = t.batt_amps; m.mah = t.batt_mah;
        m.rc_errors = t.rc_bad; m.rc_link = t.rc_link; for (int i = 0; i < 16; i++) m.rc[i] = t.rc[i]; m.rssi_pct = t.rssi;
        return m;
    }
    // Texts for the ground station: the boot report (one line whenever the queue is empty), why the craft cannot arm (when it changes,
    // at most every 2 s, and again every 10 s), armed / disarmed. Then the frames that are due and fit the UART's room.
    // ---- OSD (osd/osd.h): what the goggles show, in Betaflight's conventions (called with the lock held)
    osd::State osd_in(uint32_t now_ms) {
        const app::Telemetry& t = tel_; osd::State o;
        if (t.armed && osd_last_ms_) osd_fly_ms_ += now_ms - osd_last_ms_;
        osd_last_ms_ = now_ms;
        o.armed = t.armed; o.failsafe = t.failsafe != 0; o.fly_s = osd_fly_ms_ / 1000u; o.on_s = now_ms / 1000u;
        const uint32_t m = t.modes;
        o.mode = (m >> fc::M_RESCUE) & 1u ? osd::MODE_RESCUE : (m >> fc::M_POSHOLD) & 1u ? osd::MODE_POSHOLD : (m >> fc::M_ALTHOLD) & 1u ? osd::MODE_ALTHOLD
               : (m >> fc::M_STABILIZE) & 1u ? osd::MODE_STAB : (m >> fc::M_ANGLE) & 1u ? osd::MODE_ANGLE : osd::MODE_ACRO;
        o.rc_link = t.rc_link; o.rssi_pct = t.rssi;
        o.batt = status.battery && t.batt_v > 0.5f; o.volts = t.batt_v; o.cells = t.batt_cells;
        o.batt_level = t.batt_state == wbp::BATT_STATE_CRITICAL ? 2 : t.batt_state == wbp::BATT_STATE_WARNING ? 1 : 0;
        o.cell_min = battery.monitor.cfg.cell_min_v; o.cell_max = battery.monitor.cfg.cell_max_v;
        o.have_current = t.curr_source == batt::CUR_ADC || t.curr_source == batt::CUR_ESC || t.curr_source == batt::CUR_DRONECAN; o.amps = t.batt_amps; o.mah = t.batt_mah;
        o.gps = status.gps; o.sats = t.gps_sats; o.fix = t.gps_fix; o.speed_ms = t.gs_cms * 0.01f;
        o.home = gnss.home().has_home(); o.home_dist_m = gnss.home().distance_to_home_cm() * 0.01f; o.home_dir_deg = t.home_dir_ddeg * 0.1f;
        o.alt_valid = t.alt_valid; o.alt_m = t.z; o.vz_ms = t.vz; o.throttle = t.throttle;
        o.roll_deg = t.roll; o.pitch_deg = t.pitch; o.heading_deg = app::heading360(t.yaw);
        for (int i = 0; i < 8; i++) if (t.esc_temp[i] > o.esc_temp_c) o.esc_temp_c = t.esc_temp[i];
        // the warning: the most important one only (Betaflight's order, roughly)
        const char* w = o.failsafe ? "FAILSAFE" : o.batt_level == 2 ? "LAND NOW" : o.batt_level == 1 ? "LOW BATTERY" : !t.rc_link ? "RXLOSS" : nullptr;
        if (w) snprintf(o.warning, sizeof(o.warning), "%s", w);
        else if (!t.armed && t.blockers) {
            for (int b = 0; b < 32; b++) if ((t.blockers >> b) & 1u) {
                const char* name = b < (int)(sizeof(wbp::kB_arm_blocker) / sizeof(wbp::kB_arm_blocker[0])) ? wbp::kB_arm_blocker[b] : "?";
                size_t k = 0; for (; name[k] && k < sizeof(o.warning) - 1; k++) o.warning[k] = name[k] == '_' ? ' ' : name[k];
                o.warning[k] = 0; break;
            }
        }
        return o;
    }
    // The video transmitter's requests in, the screen out - never more than the UART has room for (sensor thread, outside the lock)
    void osd_send(uint32_t now_ms) {
        for (int k = 0; k < 64 && osd_uart_->available() > 0; k++) osd_.feed((uint8_t)osd_uart_->read(), osd_info_, osd_layout_.name);
        size_t room = port::uart_room(*osd_uart_); if (room > sizeof(osd_buf_)) room = sizeof(osd_buf_);
        const size_t n = osd_.poll(now_ms, osd_st_, osd_layout_, osd_buf_, room);
        if (n) osd_uart_->write(osd_buf_, n);
    }

    void mav_send(uint32_t now_ms) {
        if (mav_.queued_texts() == 0 && mav_report_i_ < n_report_) {
            const char* l = report_ptr_[mav_report_i_++];
            mav_.text(strstr(l, "FAILED") ? mav::MAV_SEVERITY_ERROR : strstr(l, "WARNING") ? mav::MAV_SEVERITY_WARNING : mav::MAV_SEVERITY_INFO, l);
        }
        if (mav_st_.armed != mav_armed_) { mav_armed_ = mav_st_.armed; mav_.text(mav::MAV_SEVERITY_NOTICE, mav_armed_ ? "Armed" : "Disarmed"); mav_blk_sent_ = kMavNeverSent; mav_blk_ms_ = now_ms; }
        const uint32_t blk = mav_st_.armed ? 0u : mav_blk_;
        const uint32_t since = now_ms - mav_blk_ms_;
        if ((blk != mav_blk_sent_ && since >= 2000u) || (blk && since >= 10000u)) {
            char line[mav::Scheduler::kTextLen + 1]; size_t n = 0;
            if (!blk) n = (size_t)snprintf(line, sizeof(line), "Ready to arm");
            else {
                n = (size_t)snprintf(line, sizeof(line), "Arming blocked:");
                for (int b = 0; b < 32 && n < sizeof(line) - 1; b++) if ((blk >> b) & 1u) n += (size_t)snprintf(line + n, sizeof(line) - n, " %s", b < (int)(sizeof(wbp::kB_arm_blocker) / sizeof(wbp::kB_arm_blocker[0])) ? wbp::kB_arm_blocker[b] : "?");
            }
            (void)n; mav_.text(blk ? mav::MAV_SEVERITY_WARNING : mav::MAV_SEVERITY_INFO, line); mav_blk_sent_ = blk; mav_blk_ms_ = now_ms;
        }
        size_t room = sizeof(mav_buf_);
        if (mav_uart_) { const size_t r = port::uart_room(*mav_uart_); if (r < room) room = r; }
        const size_t n = mav_.poll(now_ms, mav_st_, mav_buf_, room);
        if (!n) return;
        if (mav_uart_) mav_uart_->write(mav_buf_, n);
        if (mav_wifi_) for (size_t i = 0; i + mav::kHeaderLen + 2 <= n;) { const size_t l = mav::kHeaderLen + mav_buf_[i + 1] + 2u; mav_net.push(mav_buf_ + i, l); i += l; }
    }

    fc::GpsMeas gps_meas() {
        const gps::Solution& s = gnss.solution(); fc::GpsMeas g; float e = 0, n = 0;
        if (!gnss.home().offset_from_home_cm(e, n)) return g;
        g.valid = true; g.north_m = n * 0.01f; g.east_m = e * 0.01f; g.alt_m = (s.alt_cm - gnss.home().home().alt_cm) * 0.01f;
        g.vn = s.vel_n_cms * 0.01f; g.ve = s.vel_e_cms * 0.01f; g.vd = s.vel_d_cms * 0.01f;
        g.sigma_pos = s.h_acc_mm ? s.h_acc_mm * 0.001f : 2.0f; g.sigma_vel = s.s_acc_mm ? s.s_acc_mm * 0.001f : 0.3f; g.sigma_alt = s.v_acc_mm ? s.v_acc_mm * 0.001f : 3.0f;
        g.vd_valid = s.has_vel_d;
        g.sats = s.num_sat; g.rate_hz = gnss.rateHz();
        return g;
    }

    // ---- the EKF (core/ekf.h), sensor thread. Aligned on the ground (accelerometer tilt, compass heading), then predicted from the
    // IMU sums and corrected with whatever arrived since the last step.
    void ekf_step(uint32_t now_ms) {
        const uint32_t t0 = micros();
        const EkfDelta d = ekf_in_; if (!(d.dt > 0.0f)) return;
        const float fx = d.dvel[0] / d.dt, fy = d.dvel[1] / d.dt, fz = d.dvel[2] / d.dt, an = sqrtf(fx * fx + fy * fy + fz * fz) / fc::kG0;
        const float decl = core.cfg.declination_deg; const bool use_mag = core.cfg.use_mag && status.mag;
        if (!ekf_.aligned()) {
            ekf_gps_pend_ = ekf_baro_pend_ = false;
            if (ekf_armed_ || !ekf_gyro_ok_ || an < 0.9f || an > 1.1f) { ekf_mag_pend_ = false; return; }
            if (use_mag && !ekf_mag_pend_ && (uint32_t)(now_ms - ekf_wait_ms_) < 3000u) return;    // a compass reading first (3 s at most)
            const fc::Vec3 a{fx / fc::kG0, fy / fc::kG0, fz / fc::kG0};
            float yaw0 = ekf_ahrs_yaw_ * fc::kDeg2Rad; bool known = false;
            if (ekf_mag_pend_) {                         // the field, levelled with the tilt and a heading of 0: its angle is (declination - heading)
                const float roll = atan2f(-a.y, -a.z), pitch = atan2f(a.x, sqrtf(a.y * a.y + a.z * a.z)), cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
                const fc::Vec3 m = ekf_mag_; const float mn = cp * m.x + sr * sp * m.y + cr * sp * m.z, me = cr * m.y - sr * m.z;
                if (mn * mn + me * me > 1e-8f) { yaw0 = decl * fc::kDeg2Rad - atan2f(me, mn); known = true; }
            }
            ekf_.init(a, yaw0, known); ekf_mag_pend_ = false; ekf_rehome_ = true;
            return;
        }
        ekf_wait_ms_ = now_ms;
        ekf_.predict({d.dang[0], d.dang[1], d.dang[2]}, {d.dvel[0], d.dvel[1], d.dvel[2]}, d.dt);
        const float turn = sqrtf(d.dang[0] * d.dang[0] + d.dang[1] * d.dang[1] + d.dang[2] * d.dang[2]) / d.dt;
        if (!ekf_armed_ && turn < 0.1f && an > 0.95f && an < 1.05f) ekf_.fuse_still();
        if (ekf_gps_pend_) {
            ekf_gps_pend_ = false; const fc::GpsMeas& g = ekf_gm_;
            if (ekf_rehome_) { ekf_.set_position(g.north_m, g.east_m); ekf_rehome_ = false; }   // (a new home point: the GPS position jumped)
            ekf_.fuse_gps(g.north_m, g.east_m, g.vn, g.ve, g.vd, g.vd_valid, g.sigma_pos, g.sigma_vel);
        }
        if (ekf_baro_pend_) { ekf_baro_pend_ = false; ekf_.fuse_baro(ekf_baro_); }
        if (ekf_mag_pend_) { ekf_mag_pend_ = false; if (use_mag) ekf_.fuse_mag(ekf_mag_, decl); }
        bool finite = true; for (int i = 0; i < 3; i++) finite &= ekf_.vel()[i] == ekf_.vel()[i] && ekf_.pos()[i] == ekf_.pos()[i];
        if (!finite || !(ekf_.q()[0] == ekf_.q()[0])) { const fc::EkfConfig c = ekf_.cfg; ekf_ = fc::Ekf(); ekf_.cfg = c; ekf_nav_ = fc::EkfNav(); return; }   // (never seen: start over)
        const fc::EkfStatus& st = ekf_.status(); float r, p, y; ekf_.euler_rad(r, p, y);
        ekf_nav_.valid = true; ekf_nav_.gps_aiding = st.gps_aiding; ekf_nav_.heading_ok = st.heading_ok;
        ekf_nav_.pn = ekf_.pos()[0]; ekf_nav_.pe = ekf_.pos()[1]; ekf_nav_.vn = ekf_.vel()[0]; ekf_nav_.ve = ekf_.vel()[1]; ekf_nav_.yaw_deg = y * fc::kRad2Deg;
        ekf_out_.roll = r * fc::kRad2Deg; ekf_out_.pitch = -p * fc::kRad2Deg; ekf_out_.yaw = y * fc::kRad2Deg; ekf_out_.z = -ekf_.pos()[2]; ekf_out_.vd = ekf_.vel()[2];
        ekf_out_.yaw_sigma = ekf_.yaw_sigma() * fc::kRad2Deg; ekf_out_.test_vel = st.test_vel; ekf_out_.test_mag = st.test_mag;
        const uint32_t us = micros() - t0; if (us > ekf_us_cur_) ekf_us_cur_ = us;
    }

    // ---- telemetry
    WB_FAST void fill_fast() {
        const fc::Estimate& e = last_out_.est;
        tel_.roll = e.att.roll; tel_.pitch = e.att.pitch; tel_.yaw = e.att.yaw;
        tel_.armed = last_out_.armed; tel_.blockers = last_out_.arm_blockers; tel_.failsafe = (uint8_t)last_out_.failsafe; tel_.fs_cause = last_out_.fs_cause; tel_.modes = last_out_.modes;
        tel_.ah_state = (uint8_t)last_out_.ah_state; tel_.throttle = last_out_.throttle;
        for (int i = 0; i < 8; i++) tel_.motor_cmd[i] = motors.manager().output(i);
        for (int i = 0; i < servo::kServos; i++) tel_.servo_us[i] = servos.pulse(i);        // what really goes to the ESC (also during a motor test)
        tel_.z = e.z; tel_.vz = e.vz; tel_.z_sigma = e.z_sigma; tel_.alt_valid = e.alt_valid;
        tel_.pos_n = e.pos_n; tel_.pos_e = e.pos_e; tel_.vel_n = e.vel_n; tel_.vel_e = e.vel_e; tel_.pos_valid = e.pos_valid; tel_.yaw_from_mag = e.yaw_from_mag;
        tel_.vel_valid = e.vel_valid; tel_.flow_valid = e.flow_valid; tel_.tilt_aided = e.tilt_aided;
        tel_.gps_alt_used = e.gps_alt_used; tel_.gps_vz_used = e.gps_vz_used;
    }
    void fill_slow(uint32_t now_ms) {
        tel_.z_resets = app::sat_u16(core.vertical().baro_resets());
        {   const EkfOut& e = ekf_out_; uint8_t f = 0;
            if (ekf_.aligned()) f |= wbp::EKF_FLAGS_ALIGNED;
            if (ekf_nav_.gps_aiding) f |= wbp::EKF_FLAGS_GPS_AIDING;
            if (ekf_nav_.heading_ok) f |= wbp::EKF_FLAGS_HEADING_OK;
            if (core.ekf_used()) f |= wbp::EKF_FLAGS_USED_FOR_NAV;
            tel_.ekf_flags = ekf_mode_ ? f : 0; tel_.ekf_roll = e.roll; tel_.ekf_pitch = e.pitch; tel_.ekf_yaw = e.yaw; tel_.ekf_z = e.z; tel_.ekf_vd = e.vd;
            tel_.ekf_pn = ekf_nav_.pn; tel_.ekf_pe = ekf_nav_.pe; tel_.ekf_vn = ekf_nav_.vn; tel_.ekf_ve = ekf_nav_.ve;
            tel_.ekf_yaw_sigma = e.yaw_sigma; tel_.ekf_test_vel = e.test_vel; tel_.ekf_test_mag = e.test_mag; }
        // navigation (changes at the estimators' rate at most)
        { const fc::CoreOutput& o = last_out_; const fc::Estimate& e = o.est; uint8_t f = 0;
          if (o.poshold_active) f |= wbp::NAV_FLAGS_POSHOLD;
          if (o.poshold_unavailable) f |= wbp::NAV_FLAGS_POSHOLD_UNAVAILABLE;
          if (o.rescue_active) f |= wbp::NAV_FLAGS_RESCUE;
          if (o.rescue_available) f |= wbp::NAV_FLAGS_RESCUE_AVAILABLE;
          if (o.nav_lean) f |= wbp::NAV_FLAGS_NAV_LEAN;
          if (o.poshold_slow_gps) f |= wbp::NAV_FLAGS_POSHOLD_SLOW_GPS;
          tel_.nav_flags = f; tel_.flow_quality = flow_quality_; tel_.flow_reject = e.flow_reject; tel_.flow_rate = flow.rateHz();
          // the speed the flow sensor measures, for the eye: single samples of these sensors are coarse (an MSP sensor counts in steps of 0.5 rad/s), so a quarter second of them is averaged
          { const uint32_t d = now_ms - flow_tel_ms_; flow_tel_ms_ = now_ms; const float k = d >= 250u ? 1.0f : (float)d / 250.0f;
            tel_.flow_fwd += k * ((e.flow_valid ? e.flow_forward : 0.0f) - tel_.flow_fwd); tel_.flow_right += k * ((e.flow_valid ? e.flow_right : 0.0f) - tel_.flow_right); }
          tel_.ph_state[0] = o.ph_state[0]; tel_.ph_state[1] = o.ph_state[1]; tel_.nav_roll = o.nav_lean ? o.nav_roll : 0.0f; tel_.nav_pitch = o.nav_lean ? o.nav_pitch : 0.0f;
          tel_.nav_target_n = o.nav_target_n; tel_.nav_target_e = o.nav_target_e; tel_.nav_wind_n = o.nav_wind_n; tel_.nav_wind_e = o.nav_wind_e; tel_.nav_home_dist = o.nav_home_dist;
          tel_.rescue_phase = o.rescue_phase; tel_.rescue_fail = o.rescue_fail; tel_.rescue_return_alt = core.rescue().return_height(); }
        for (int i = 0; i < 16; i++) tel_.rc[i] = rc_[i];
        tel_.rc_link = rc_link_; tel_.rssi = rc.linkQuality(); tel_.rc_rate = rc.rateHz();
        // ---- live state of the filters and the PID helpers (the app's tuning stream)
        { const fc::GyroFilter& gf = core.gyro_filter(); const fc::PidController& pc = core.pid();
          tel_.gyro_lpf1_hz = gf.lpf1_cutoff_hz(); tel_.dterm_lpf1_hz = pc.dterm_lpf1_cutoff_hz();
          for (int a = 0; a < 3; a++) for (int p = 0; p < 3; p++) tel_.dyn_notch_hz[a * 3 + p] = p < gf.dyn_notch().count() ? gf.dyn_notch().center_hz(a, p) : 0.0f;
          for (int m = 0; m < 8; m++) tel_.rpm_notch_hz[m] = gf.rpm_filter().active() && gf.rpm_filter().notch_weight(m, 0) > 0.0f ? gf.rpm_filter().notch_hz(m, 0) : 0.0f;
          tel_.tpa = pc.tpa_factor(); tel_.anti_gravity = pc.anti_gravity(); tel_.rc_smoothed_hz = core.setpoints().rx_rate().smoothed_hz(); tel_.rc_cutoff_hz = core.setpoints().setpoint_cutoff_hz();
          tel_.air_state = !last_out_.armed || last_out_.ground_idle ? wbp::AIR_STATE_GROUND : last_out_.airmode_active ? wbp::AIR_STATE_AIRMODE : wbp::AIR_STATE_AIR; tel_.hover_throttle = core.althold().hover_throttle();
          for (int a = 0; a < 3; a++) tel_.pid_sum[a] = last_out_.pid_p[a] + last_out_.pid_i[a] + last_out_.pid_d[a] + last_out_.pid_f[a]; }
        uint16_t h = 0;
        if (imu_sel_.in_use()) h |= wbp::HEALTH_IMU;
        if (gyro_calibrated()) h |= wbp::HEALTH_GYRO_CAL;
        if (status.imu2 && !imu_sel_.lost(1)) h |= wbp::HEALTH_IMU2;
        if (status.can && can_port->state() == CanPort::RUNNING) h |= wbp::HEALTH_CAN;
        tel_.imu2 = !imu2_cfg_ ? imusel::NONE : !status.imu2 ? imusel::FAILED : imu_sel_.lost(1) ? imusel::LOST : imusel::OK;
        { const AgSample& b = imu_sel_.last2(); tel_.gyro2[0] = b.gyro_dps[0]; tel_.gyro2[1] = -b.gyro_dps[1]; tel_.gyro2[2] = b.gyro_dps[2]; }
        if (baro.valid(now_ms)) h |= wbp::HEALTH_BARO;
        if (sonar.valid(now_ms)) h |= wbp::HEALTH_RANGE;
        if (gnss.usable()) h |= wbp::HEALTH_GPS;
        if (status.flow && flow.healthy(now_ms)) h |= wbp::HEALTH_FLOW;
        if (mag.readyForFusion(now_ms)) h |= wbp::HEALTH_MAG_READY;
        if (mag.calibrated()) h |= wbp::HEALTH_MAG_CAL;
        if (rc_link_) h |= wbp::HEALTH_RX;
        if (status.motors) h |= wbp::HEALTH_MOTORS;
        motor::MotorOutputManager& mm = motors.manager();
        if (mm.rpm_all_valid(now_ms) || (esc_uart_ && esc_poll_.alive(now_ms))) h |= wbp::HEALTH_ESC_TELEMETRY;
        if (imu.accCalibration().valid) h |= wbp::HEALTH_ACC_CAL;
        if (battery.monitor.present()) h |= wbp::HEALTH_BATTERY;
        tel_.health = h;
        tel_.esc_volt = mm.esc_voltage(now_ms); tel_.dyn_idle = mm.dyn_idle_increase();
        tel_.rpm_ok = mm.rpm_valid_mask(now_ms);
        tel_.bidir_state = !(mm.cfg.bidirectional && motor::is_dshot_wire(mm.cfg.protocol)) ? wbp::BIDIR_STATE_OFF : motors.bidirectionalActive() ? wbp::BIDIR_STATE_ACTIVE : wbp::BIDIR_STATE_UNAVAILABLE;
        { uint8_t ans = 0; for (int i = 0; i < mm.cfg.count; i++) if (motors.escAnswers(mm.cfg.order[i])) ans |= (uint8_t)(1u << i); tel_.esc_answers = ans; }
        tel_.batt_v = battery.monitor.voltage(); tel_.cell_v = battery.monitor.cell_voltage(); tel_.batt_cells = battery.monitor.cells();
        tel_.batt_state = (uint8_t)battery.monitor.state(); tel_.batt_source = (uint8_t)battery.source(); tel_.batt_pin_mv = battery.pinMv();
        tel_.batt_amps = battery.current.amps(); tel_.batt_mah = battery.current.mah_drawn(); tel_.curr_source = (uint8_t)battery.currentSource(); tel_.curr_pin_mv = battery.currentPinMv();
        { const batt::Config& bc = battery.monitor.cfg; tel_.batt_remaining = batt::remaining_percent(battery.current.mah_drawn(), battery.current.valid() ? capacity_mah_ : 0, battery.monitor.voltage(), battery.monitor.cells(), bc.cell_min_v, bc.cell_max_v); }
        tel_.beeping = beeper.current() == beep::NO_MODE ? 0 : (uint8_t)(beeper.current() + 1);
        tel_.motor_beeping = motor_beeper.current() == beep::NO_MODE ? 0 : (uint8_t)(motor_beeper.current() + 1);
        for (int i = 0; i < 8; i++) {
            const motor::MotorTelemetry t = mm.telemetry(i, now_ms);
            tel_.rpm[i] = t.rpm; tel_.esc_err_pct[i] = t.error_pct; tel_.esc_temp[i] = t.temp_c; tel_.esc_amps[i] = t.amps;
        }
        const float* b = mag.board(); for (int i = 0; i < 3; i++) tel_.mag[i] = b[i];
        tel_.baro_alt = baro.altitudeM(); tel_.baro_pa = baro.pressurePa(); tel_.temp_c = baro.tempC(); tel_.range_m = sonar.valid(now_ms) ? sonar.heightM() : 0.0f;
        const gps::Solution& g = gnss.solution();
        tel_.gps_fix = g.fix_type; tel_.gps_sats = g.num_sat; tel_.lat = g.lat; tel_.lon = g.lon; tel_.alt_cm = g.alt_cm; tel_.gs_cms = g.ground_speed_cms;
        tel_.course_ddeg = g.course_ddeg; tel_.hdop = g.hdop; tel_.hacc_mm = g.h_acc_mm; tel_.home_dist_m = gnss.home().distance_to_home_m();
        tel_.home_dir_ddeg = gnss.home().direction_to_home_ddeg(); tel_.gps_state = (uint8_t)gnss.state(); tel_.gps_baud = gnss.baud(); tel_.gps_rate_hz = gnss.rateHz();
        // calibration progress
        tel_.cal_kind = cal_kind_; tel_.cal_mask = acc_cal_.mask(); tel_.cal_result = mag_result_;
        const float* raw = imu.rawAcc();
        switch (cal_kind_) {
        case wbp::CAL_KIND_GYRO: tel_.cal_state = (uint8_t)gyro_cal_state(); tel_.cal_progress = gyro_cal_progress(); raw = tel_.gyro_raw; break;
        case wbp::CAL_KIND_ACCEL: tel_.cal_state = (uint8_t)(acc_done_ && !acc_session_ ? CalState::DONE : acc_cal_.state()); tel_.cal_progress = acc_cal_.progress(); break;
        case wbp::CAL_KIND_MAG: tel_.cal_state = (uint8_t)mag_cal_state_; tel_.cal_progress = (uint8_t)(mag.calibrationProgress() * 100.0f + 0.5f); raw = mag.rawGauss(); break;
        case wbp::CAL_KIND_MAG_AUTO: {
            const MagAutoCal::State a = mag_auto_.state();
            tel_.cal_state = (uint8_t)(a == MagAutoCal::DONE ? CalState::DONE : a == MagAutoCal::FAILED ? CalState::FAILED : a == MagAutoCal::IDLE ? CalState::IDLE : CalState::RUNNING);
            tel_.cal_progress = mag_auto_.progress(); raw = mag.rawGauss(); break; }
        default: tel_.cal_state = 0; tel_.cal_progress = 0; break;
        }
        for (int i = 0; i < 3; i++) tel_.cal_raw[i] = raw[i];
    }

    // ---- boot report
    void say(bool ok, const char* what, const char* name, const char* why, const char* fmt, ...) {
        if (n_report_ >= kReportLines) return;
        char* line = report_[n_report_]; report_ptr_[n_report_] = line; n_report_++;
        int n = snprintf(line, kReportLen, "%s: ", what);
        if (!ok) { snprintf(line + n, kReportLen - n, "FAILED - %s", (why && *why) ? why : "not detected"); return; }
        va_list ap; va_start(ap, fmt); vsnprintf(line + n, kReportLen - n, fmt, ap); va_end(ap);
        (void)name;
    }

    static constexpr int kReportLines = 40, kReportLen = 176;      // (the longest boot: 16 devices, 3 pin lines, SD card, WiFi, loop, link, settings)
    char report_[kReportLines][kReportLen] = {}; const char* report_ptr_[kReportLines] = {}; int n_report_ = 0;

    wbp::Settings* s_ = nullptr;
    app::Telemetry tel_;
    AccCalibrator acc_cal_; bool acc_session_ = false, acc_done_ = false; CalState mag_cal_state_ = CalState::IDLE; uint8_t cal_kind_ = 0, mag_result_ = 0;
    MagAutoCal mag_auto_; app::StickCommands sticks_; uint16_t cal_samples_ = 1000;
    uint16_t rc_[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint32_t rc_seq_ = 0;
    bool rc_link_ = false, was_armed_ = false, reboot_pending_ = false, land_on_critical_ = true, acc_was_running_ = false; uint16_t capacity_mah_ = 0; bool edt_wanted_ = false, edt_asked_ = false;
    mav::Scheduler mav_; mav::State mav_st_; port::SerialPort* mav_uart_ = nullptr; bool mav_on_ = false, mav_wifi_ = false, mav_armed_ = false; uint8_t mav_type_ = mav::MAV_TYPE_QUADROTOR;
    static constexpr uint32_t kMavNeverSent = 0xFFFFFFFFu;     // (mav_blk_sent_ before the first text: 'Ready to arm' is news as well)
    uint8_t mav_buf_[512] = {}; int mav_report_i_ = 0; uint32_t mav_blk_ = 0, mav_blk_sent_ = kMavNeverSent, mav_blk_ms_ = 0;
    crsf::Scheduler crsf_tlm_; uint8_t tlm_buf_[crsf::kMaxFrame] = {}; size_t tlm_n_ = 0;
public:
    osd::DisplayPort osd_;                                // (public for the tests: what the video transmitter asked)
private:
    osd::Layout osd_layout_; osd::State osd_st_; msp::Info osd_info_; port::SerialPort* osd_uart_ = nullptr; uint8_t osd_buf_[512] = {}; uint32_t osd_fly_ms_ = 0, osd_last_ms_ = 0;
    beep::Conditions beep_cond_; uint32_t beacons_ = 0, start_us_ = 0, gap_max_ = 0, cycle_gap_ = 0, bb_slow_ms_ = 0;
    uint32_t last_us_ = 0, next_us_ = 0, slow_us_ = 0, period_us_ = 250, stat_ms_ = 0, test_until_ms_ = 0, reboot_at_ms_ = 0, imu_errors_ = 0, imu_fail_streak_ = 0, slot_ = 0, loops_ = 0, loops_mark_ = 0;
    uint16_t pub_div_ = 4, pub_n_ = 0, loop_max_ = 0;
    uint32_t pid_loops_ = 0, pid_mark_ = 0, ctl_gap_ = 0;   // control cycles (the gyro cycles are loops_); the time since the last control cycle
    sys::Load load_;
    app::Sync no_sync_; app::Sync* sync_ = &no_sync_;
    app::StageTime t_imu_, t_core_, t_motor_, t_rest_, t_slow_;
    bool slow_seen_ = false; uint32_t slow_last_ms_ = 0, slow_gap_cur_ = 0;
    // the EKF: sums of the IMU (flight task adds, sensor thread takes - under the lock), the filter and its inputs (sensor thread only)
    struct EkfDelta { float dang[3] = {0, 0, 0}, dvel[3] = {0, 0, 0}, dt = 0; };
    struct EkfOut { float roll = 0, pitch = 0, yaw = 0, z = 0, vd = 0, yaw_sigma = 0, test_vel = 0, test_mag = 0; };
    EkfDelta ekf_acc_, ekf_in_; uint8_t ekf_mode_ = 1; bool ekf_due_ = false, ekf_armed_ = false, ekf_gyro_ok_ = false; float ekf_ahrs_yaw_ = 0;
    fc::Ekf ekf_; fc::EkfNav ekf_nav_; EkfOut ekf_out_; fc::GpsMeas ekf_gm_; float ekf_baro_ = 0; fc::Vec3 ekf_mag_;
    bool ekf_gps_pend_ = false, ekf_baro_pend_ = false, ekf_mag_pend_ = false, ekf_rehome_ = true; uint32_t ekf_wait_ms_ = 0, ekf_us_cur_ = 0;       // sensor thread: longest pause between two passes, this second
    imusel::Selector imu_sel_; bool imu2_cfg_ = false;
    // the LED strip (section led, pins.led): what it shows (app/led_logic.h), the colors of the last picture
public:
    const app::Rgb* ledColors() const { return led_rgb_; }
    int ledCount() const { return led_n_; }
private:
    app::LedLogic led_logic_; app::LedInput led_in_; app::Rgb led_rgb_[app::LedLogic::kMax]; int led_n_ = 0; uint32_t led_ms_ = 0; bool led_due_ = false;
#if WB_HAS_LED_STRIP
    led::SpiStrip led_strip_;
#endif
    app::LedInput led_input() const {                     // (lock held)
        app::LedInput in; const fc::CoreOutput& o = last_out_;
        in.armed = o.armed; in.ready = o.arm_blockers == 0; in.failsafe = o.failsafe != fc::FsStage::NONE;
        in.batt_warning = batt_state_ == wbp::BATT_STATE_WARNING; in.batt_critical = batt_state_ == wbp::BATT_STATE_CRITICAL;
        in.calibrating = gyro_cal_state() == CalState::RUNNING || acc_cal_.running() || mag.calibrating() || mag_auto_.active();
        in.stall = (o.arm_blockers & (fc::AB_MOTOR_STALL | fc::AB_RUNAWAY_TAKEOFF)) != 0;
        in.batt_pct = tel_.batt_cells ? tel_.batt_remaining : 255;
        const float thr = o.stick[3] * 100.0f; in.throttle = (uint8_t)(thr <= 0.0f ? 0 : thr >= 100.0f ? 100 : thr + 0.5f);
        in.mode = o.rescue_active ? app::LedInput::RESCUE : o.poshold_active ? app::LedInput::POSHOLD : o.althold_active ? app::LedInput::ALTHOLD
                : ((o.modes >> fc::M_ANGLE) & 1u) ? app::LedInput::ANGLE : ((o.modes >> fc::M_STABILIZE) & 1u) ? app::LedInput::STABILIZE : app::LedInput::ACRO;
        return in;
    }
    bool can_gps_ = false, can_mag_ = false, can_baro_ = false, can_batt_ = false; uint32_t can_mark_[3] = {};
    dcan::EscStatus can_esc_[dcan::Node::kMaxEsc]; uint8_t can_esc_new_ = 0;   // (esc.Status taken off the bus in part A, fed under the lock in part B)      // (DroneCAN GPS / compass in use; frame counters of the last second)
    struct StagePeaks { uint16_t lock = 0, imu = 0, core = 0, est = 0, motor = 0; } peak_;
    WB_FAST static uint16_t peak16(uint16_t was, uint32_t us) { return us > 65535u ? (uint16_t)65535 : us > was ? (uint16_t)us : was; }
    bool loop_slow_ = false, busy_other_ = false, busy_seen_ = false; uint8_t slow_seconds_ = 0, batt_state_ = 0, flow_quality_ = 0; uint32_t flow_tel_ms_ = 0;
    uint16_t servo_rc_[16] = {}; bool servo_have_rc_ = false;
    fc::Health health_; motor::HwStats hw_mark_; uint32_t crc_mark_ = 0; uint32_t rc_mark_[3] = {};
    bool rx_tlm_on_ = true;                               // receiver.telemetry: CRSF telemetry back through the receiver
    wbp::S_motors motors_run_ = {};                       // the motors section the outputs run with (apply_motor_order)
    escs::Poller esc_poll_; port::SerialPort* esc_uart_ = nullptr; uint32_t esc_mark_[3] = {};      // ESC telemetry wire
    fc::CoreOutput last_out_;
    uint32_t trim_ms_ = 0;
};
