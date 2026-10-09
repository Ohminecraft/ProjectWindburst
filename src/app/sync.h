/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <stdint.h>

// ============================================================================
// The lock between the two threads of FlightApp.
//
//   flight task   (high priority, started by a hardware timer at the loop rate)
//                 lock - IMU -> filters / estimator / PID / mixer -> motors -> log + telemetry - unlock
//   sensor thread (low priority, about 1 kHz)
//                 A. device I/O with NO lock: receiver UART, GPS UART, barometer / compass / lidar on I2C, battery ADC.
//                    Only this thread touches those drivers; an I2C transaction may take its 300 us in peace.
//                 B. lock - hand the results to the core, run one app request, fill the slow telemetry - unlock
//                    Short and never waits for a device.
//
// So a control cycle can be delayed by phase B (tens of microseconds), never by a bus. On a PC (tests, simulator) both run in
// one thread and the lock is this empty one; test_app runs them in two real threads under ThreadSanitizer with a real mutex.
// ============================================================================
namespace app {

struct Sync {
    virtual ~Sync() = default;
    virtual void lock() {}
    virtual void unlock() {}
};

class Guard {
public:
    explicit Guard(Sync& s) : s_(s) { s_.lock(); }
    ~Guard() { s_.unlock(); }
    Guard(const Guard&) = delete; Guard& operator=(const Guard&) = delete;
private:
    Sync& s_;
};

// Average and maximum of a duration over one reporting period (the per-stage loop timing shown in the app).
class StageTime {
public:
    WB_FAST void add(uint32_t us) { sum_ += us; n_++; if (us > max_) max_ = us; }
    // average since the last call, saturated to 16 bits; clears
    uint16_t take_avg() { const uint32_t a = n_ ? (sum_ + n_ / 2) / n_ : 0; sum_ = 0; n_ = 0; max_ = 0; return a > 65535u ? (uint16_t)65535u : (uint16_t)a; }
    uint16_t take_max() { const uint32_t m = max_; sum_ = 0; n_ = 0; max_ = 0; return m > 65535u ? (uint16_t)65535u : (uint16_t)m; }
private:
    uint32_t sum_ = 0, n_ = 0, max_ = 0;
};

// Control cycles that did not happen: `cycles` ran in `span_ms` at a configured rate of `target_hz`.
inline uint16_t cycles_lost(uint32_t cycles, uint32_t span_ms, float target_hz) {
    const int32_t want = (int32_t)(target_hz * (float)span_ms * 0.001f + 0.5f);
    const int32_t lost = want - (int32_t)cycles;
    return lost <= 0 ? (uint16_t)0 : lost > 65535 ? (uint16_t)65535 : (uint16_t)lost;
}

// The loop holds its rate when it ran at least 80 % of the configured cycles in the last full second. (Logging to the internal
// flash costs a few percent: every page write stops both cores for a moment. A loop that is simply too slow for its setting
// reaches a half or a third.)
inline bool loop_rate_held(uint32_t measured_hz, float target_hz) { return (float)measured_hz >= 0.8f * target_hz; }

// ---- the flight task and the timer ticks
// The task has the highest priority on its core: whatever it does not leave is not there for the thread that reads the
// receiver, the sensors and the app's requests. A task that simply runs whenever a tick is waiting would, with cycles longer
// than the period, start again at once and never let that thread run - a craft that still stabilises but no longer hears its
// pilot. A task that gives a tick away after every long cycle loses rate it does not have to lose: on the first board a plain
// cycle took 120 - 145 us and the one in eight that runs the estimator 240 - 300 us, of a 250 us period.
//
// So the task keeps a budget. The other thread is owed a quarter of every cycle's length as idle time - a fifth of the core.
// Idle time beyond that is credit, up to half a period: a cycle that overruns its tick is followed by the next one at once
// (one tick at most is made up, the others are lost and counted), as long as the credit lasts. When the balance is negative
// the task stays away until the debt is paid, in whole ticks.
//   cycles of 135 us with one of 290 us in eight (4 kHz):  every tick is used
//   cycles of 240 us:                                       four in five ticks       (a cycle and its pause need 300 us)
//   cycles of 800 us:                                       one in four ticks        (the loop-rate blocker says so)
// The credit limit bounds how long the other thread can be kept waiting: two periods and two cycles (1 ms at 4 kHz with
// cycles that fit their period).
class TickBudget {
public:
    // A cycle ran for took_us after the task had been idle for idle_us. Returns the idle time that has to pass before the
    // next cycle may start (0: with the next tick, also one that is already waiting).
    WB_FAST uint32_t after_cycle(uint32_t idle_us, uint32_t took_us, uint32_t period_us) {
        int64_t b = (int64_t)balance_ + (int64_t)idle_us - (int64_t)(((uint64_t)took_us + 3) / 4);    // (rounded up: a fifth, not a hair less)
        const int64_t credit = period_us / 2;
        if (b > credit) b = credit;
        balance_ = (int32_t)(b < -0x40000000 ? -0x40000000 : b);
        return balance_ < 0 ? (uint32_t)(-balance_) : 0u;
    }
    int32_t balance() const { return balance_; }
private:
    int32_t balance_ = 0;
};

// One turn of the task's loop: wait for a tick, run a cycle, settle the budget. P is the platform (flight_task.h: FreeRTOS and
// the hardware timer; test_app: a simulated timer):
//   void wait_tick()            blocks until the timer has ticked; returns at once when ticks are waiting, and consumes them all
//   uint32_t now()              microseconds
//   void cycle(uint32_t now)    one control cycle
template <class P> struct TickLoop {
    TickBudget budget;
    uint32_t end_us = 0; bool started = false, have_tick = false;
    WB_FAST void turn(P& p, uint32_t period_us) {
        if (!have_tick) p.wait_tick();
        have_tick = false;
        const uint32_t t0 = p.now();
        const uint32_t idle = started ? t0 - end_us : period_us;
        p.cycle(t0);
        end_us = p.now(); started = true;
        const uint32_t need = budget.after_cycle(idle, end_us - t0, period_us);
        if (need == 0) return;
        do p.wait_tick(); while ((uint32_t)(p.now() - end_us) < need);       // (a tick that was waiting already has brought no idle time: it is used up here)
        have_tick = true;
    }
};

// Which UART an optical-flow sensor that talks over one (MSP) gets: 1, 2, 0 - or -1 when none is left. UART1 belongs to the GPS
// and UART2 to a serial receiver when those are in use; UART0 is free only when the app's link is on USB.
inline int flow_uart(bool gps_on_uart1, bool receiver_on_uart2, bool uart0_free) {
    if (!gps_on_uart1) return 1;
    if (!receiver_on_uart2) return 2;
    return uart0_free ? 0 : -1;
}
// Which UART (0..2, -1 = none) serves the GPS, the serial receiver and an MSP flow sensor.
//   AUTO:  GPS on UART1, receiver on UART2 (fixed, as before); the flow sensor gets what is left (flow_uart(), decided after the
//          other two have started - pass gps_on / rx_on then).
//   TABLE: the ports section's function per UART (port_function: NONE, SERIAL_RX, GPS, TELEMETRY = unused, FLOW_MSP).
//          UART0 only when the app link does not use it (native USB). A device with no UART in the table gets a reason.
enum PortFn : uint8_t { PF_NONE, PF_SERIAL_RX, PF_GPS, PF_TELEMETRY, PF_FLOW_MSP };     // == schema enum port_function
struct UartPlan { int gps = -1, rx = -1, flow = -1; const char* why = ""; };
inline UartPlan plan_uarts(bool table, const uint8_t fn[3], bool want_gps, bool want_rx, bool want_flow, bool uart0_free, bool gps_on, bool rx_on) {
    UartPlan p;
    if (!table) {
        p.gps = want_gps ? 1 : -1; p.rx = want_rx ? 2 : -1;
        if (want_flow) p.flow = flow_uart(gps_on, rx_on, uart0_free);
        return p;
    }
    bool uart0_wanted = false;
    for (int u = 0; u < 3; u++) {
        if (fn[u] != PF_GPS && fn[u] != PF_SERIAL_RX && fn[u] != PF_FLOW_MSP) continue;
        if (u == 0 && !uart0_free) { uart0_wanted = true; continue; }
        int& slot = fn[u] == PF_GPS ? p.gps : fn[u] == PF_SERIAL_RX ? p.rx : p.flow;
        if (slot < 0) slot = u;
    }
    if (!want_gps) p.gps = -1;
    if (!want_rx) p.rx = -1;
    if (!want_flow) p.flow = -1;
    if (uart0_wanted) p.why = "UART0 carries the app link on this board (USB-UART bridge): it cannot serve a device";
    return p;
}

// Whether the range sensor is the lidar inside an MSP flow sensor: asked for (range model MSP), or the model is AUTO and no
// other range sensor answered at start-up. Never when range sensing is switched off or the flow sensor is not an MSP one.
inline bool lidar_of_flow(bool range_enabled, bool flow_is_msp, bool model_msp, bool model_auto, bool other_found) {
    return range_enabled && flow_is_msp && (model_msp || (model_auto && !other_found));
}

} // namespace app
