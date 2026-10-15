// The firmware with its real threads, under ThreadSanitizer (app/sync.h).
//
//   flight thread   FlightApp::flight_cycle()  4000 times per simulated second
//   sensor thread   FlightApp::slow_update()   once per simulated millisecond: receiver, GPS, barometer, compass, battery, app requests
//   link thread     wbp::Server                reads the streams, changes settings, sends commands - like the app does
//
// The three run concurrently on real OS threads with a real mutex as the lock. ThreadSanitizer watches every memory access: one
// field that the two firmware threads share without the lock ends the program with a report (and `make test` fails).
// The plant stands still (nobody steps it), so the simulated chips are only touched by the thread whose driver reads them.
#include <stdio.h>
#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include "virtual_board.h"
#include "sim_flash.h"
#include "../src/blackbox/bb_header.h"
#include "../src/proto/wbp_server.h"
#include "../src/app/link_host.h"

// ThreadSanitizer settings, built into the program so that it behaves the same however it is started.
// app::Exchange hands the telemetry from the flight task to the link task through a sequence lock: the reader copies while the
// writer may be writing, and throws the copy away if the sequence number moved. By the letter that is a data race; it is also
// the whole point of the mechanism (the flight task never waits) and has its own test with real threads in test_app.
extern "C" const char* __tsan_default_suppressions() { return "race:app::Exchange::publish\nrace:app::Exchange::snapshot\n"; }
extern "C" const char* __tsan_default_options() { return "halt_on_error=1:exitcode=66"; }

static std::atomic<int> g_pass{0}, g_fail{0};
#define CHECK(cond, ...) do { if (cond) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)
using namespace wbp;

struct StdSync : app::Sync {
    std::mutex m; std::atomic<uint32_t> locks{0}; std::atomic<std::thread::id> owner{std::thread::id()};
    void lock() override { m.lock(); owner = std::this_thread::get_id(); locks++; }
    void unlock() override { owner = std::thread::id(); m.unlock(); }
    bool mine() const { return owner.load() == std::this_thread::get_id(); }
};
static StdSync* g_sync = nullptr;
static std::atomic<uint32_t> g_io_total{0}, g_io_locked{0};
#include <execinfo.h>
#include <unistd.h>
static void io_hook() { g_io_total++; if (g_sync && g_sync->mine()) { if (g_io_locked++ == 0 && getenv("WB_TRACE")) { void* bt[24]; const int n = backtrace(bt, 24); backtrace_symbols_fd(bt, n, 2); } } }
struct MemStorage : wbp::Storage {
    std::map<std::string, std::vector<uint8_t>> kv;
    int read(const char* k, uint8_t* b, size_t cap) override { auto it = kv.find(k); if (it == kv.end()) return -1; const size_t n = it->second.size() < cap ? it->second.size() : cap; memcpy(b, it->second.data(), n); return (int)n; }
    bool write(const char* k, const uint8_t* d, size_t n) override { kv[k] = std::vector<uint8_t>(d, d + n); return true; }
};
struct MemTransport : wbp::Transport {
    std::vector<uint8_t> out;
    size_t write(const uint8_t* d, size_t n) override { out.insert(out.end(), d, d + n); return n; }
    size_t writable() override { return 100000; }
};

static void wait_1ms(void*) { std::this_thread::sleep_for(std::chrono::microseconds(200)); }
static uint32_t now_ms() { return millis(); }

int main() {
    printf("[two firmware threads + the link thread, ThreadSanitizer]\n");
    g_stub_us = 0; g_stub_delay_advances = true;
    VirtualBoard board; MemStorage flash; MemTransport tx;
    wbp::Settings settings; wbp::SettingsStore store(flash);
    store.load(settings); VirtualBoard::wire(settings);
    settings.modes.mode[0] = MODE_ARM; settings.modes.aux[0] = 0; settings.modes.lo[0] = 1700; settings.modes.hi[0] = 2100;
    settings.motors.bidirectional = 1;
    settings.blackbox.device = BB_DEVICE_FLASH; settings.blackbox.trigger = BB_TRIGGER_ARMED; settings.blackbox.rate_hz = 1000;
    store.save(settings);
    MemFlash logmem{1024 * 1024};

    FlightApp app;
    app::LinkHost host(app.link, &wait_1ms, nullptr, &now_ms);
    wbp::Server server(settings, &store, host, tx, wbp::ROLE_RX, 0, 7, 0, "threads", wbp::FEAT_STREAMS);
    app.configure(settings);
    CHECK(app.begin(), "the firmware starts on the virtual board");
    bb::Service blackbox(app.blackbox, tx);                           // the flight log: recorded by the flight task, written by the link task
    bb::FirmwareHeader bb_header{&settings, "threads", 0, 7, 0};
    blackbox.attach_store(&logmem); blackbox.set_header_source(&bb::FirmwareHeader::write, &bb_header);
    blackbox.set_loop(app.core.cfg.pid_hz(), app.motors.cfg().count, app.core.cfg.loop_hz);   // a record per control cycle at most
    host.set_blackbox(&blackbox, &settings);
    host.attach(&server); host.set_report(app.report(), app.report_lines());
    StdSync sync; app.set_sync(&sync); g_sync = &sync; g_stub_io_hook = &io_hook;

    const int kSeconds = 7; const uint32_t kCycles = 4000u * kSeconds;
    std::atomic<bool> stop{false};
    std::atomic<uint32_t> slow_ms{0}, slow_passes{0}, link_ticks{0};
    std::atomic<int> armed_at_4s{-1}, armed_at_end{-1}, motor_at_4s{0};
    std::atomic<int> pid_ack{-1}, trim_err{-1}, test_err{-1}, armed_set_ack{-1}, status_frames{0}, loop_hz_seen{0}, target_seen{0};

    // ---------------- flight thread: the clock is its timer
    std::thread flight([&] {
        std::mt19937 rng(777);
        for (uint32_t i = 0; i < kCycles && !stop; i++) {
            g_stub_us += 250;
            board.refresh_imu(rng);
            { HostMotorHw* esc = static_cast<HostMotorHw*>(app.motors.hw()); if (esc) for (int k = 0; k < 4; k++) { esc->reply[k] = VirtualBoard::esc_gcr(6000.0f, 14); esc->has_reply[k] = true; } }
            app.flight_cycle(micros());
            const uint32_t ms = millis();
            // the sensor thread is "1 kHz": do not run more than a millisecond ahead of it (the simulated clock is this thread's)
            while (!stop && (int32_t)(ms - slow_ms.load()) > 1) std::this_thread::yield();
            if (i == 4000u * 4) { armed_at_4s = app.output().armed ? 1 : 0; const HostMotorHw* hw = static_cast<HostMotorHw*>(app.motors.hw()); motor_at_4s = hw ? hw->last.value[0] : 0; }
        }
        armed_at_end = app.output().armed ? 1 : 0;
        stop = true;
    });

    // ---------------- sensor thread, and the pilot
    std::thread sensors([&] {
        uint32_t last = 0xFFFFFFFFu;
        while (!stop) {
            const uint32_t ms = millis();
            if (ms == last) { std::this_thread::yield(); continue; }
            last = ms;
            if (ms >= 2500) board.rc[4] = 2000;                       // arm
            if (ms >= 3000 && ms < 4500) board.rc[2] = 1400; else board.rc[2] = 1000;
            if (ms >= 5200) board.rc[4] = 1000;                       // disarm
            board.refresh_slow(ms);
            app.slow_update();
            slow_ms = ms; slow_passes++;
        }
    });

    // ---------------- link thread: what the app does
    std::thread link([&] {
        wbp::Parser parser; uint8_t seq = 1; uint32_t last = 0xFFFFFFFFu; int step = 0;
        auto send = [&](uint8_t type, const std::vector<uint8_t>& p) -> std::vector<uint8_t> {      // request -> reply payload (type in [0])
            const uint8_t sq = seq++; if (seq == 0) seq = 1;
            uint8_t buf[wbp::kMaxPayload + wbp::kOverhead];
            const size_t n = wbp::encode(buf, sizeof(buf), type, sq, p.data(), (uint16_t)p.size());
            server.on_bytes(buf, n);
            std::vector<uint8_t> reply; wbp::Frame f;
            for (uint8_t c : tx.out) if (parser.feed(c, f)) { do {
                if (f.type == MSG_STREAM && f.len == sizeof(T_status) + 5 && f.payload[0] == STR_STATUS) { T_status st; memcpy(&st, f.payload + 5, sizeof(st)); status_frames++; if (st.loop_hz) loop_hz_seen = st.loop_hz; target_seen = st.loop_target; }
                if (f.seq == sq && f.type != MSG_STREAM && f.type != MSG_LOG) { reply.assign(1, f.type); reply.insert(reply.end(), f.payload, f.payload + f.len); }
            } while (parser.pump(f)); }
            tx.out.clear();
            return reply;
        };
        auto ack = [](const std::vector<uint8_t>& r) { return r.size() == 4 && r[0] == MSG_ACK ? (int)r[3] : 255; };
        while (!stop) {
            const uint32_t ms = millis();
            if (ms == last) { std::this_thread::yield(); continue; }
            last = ms;
            server.tick(ms); link_ticks++;
            if (step == 0 && ms >= 300) { step++; for (uint8_t id : {STR_ATTITUDE, STR_STATUS, STR_MOTORS, STR_SENSORS, STR_RC, STR_ESTIMATE, STR_TUNING, STR_BATTERY, STR_GPS, STR_CALIBRATION}) send(MSG_STREAM_CFG, {id, 50, 0}); }
            if (step == 1 && ms >= 1600) { step++;                    // a live section: new PID numbers while the loop runs
                S_pid p = settings.pid; p.iterm_windup = 77;
                std::vector<uint8_t> pl; pl.push_back(SEC_PID); pl.insert(pl.end(), (const uint8_t*)&p, (const uint8_t*)&p + sizeof(p));
                const auto r = send(MSG_SECTION_SET, pl); pid_ack = r.empty() ? 254 : (r[0] == MSG_SECTION ? 0 : ack(r)); }
            if (step == 2 && ms >= 1800) { step++; const auto r = send(MSG_COMMAND, {CMD_LEVEL_TRIM}); trim_err = r.size() >= 3 ? r[2] : 254; }
            if (step == 3 && ms >= 2000) { step++; const auto r = send(MSG_COMMAND, {CMD_MOTOR_TEST, 0, 100, 0}); test_err = r.size() >= 3 ? r[2] : 254; }
            if (step == 4 && ms >= 2200) { step++; send(MSG_COMMAND, {CMD_MOTOR_STOP}); }
            if (step == 5 && ms >= 3500) { step++;                    // armed: a settings change must be refused, and nothing may break
                S_rates rt = settings.rates;
                std::vector<uint8_t> pl; pl.push_back(SEC_RATES); pl.insert(pl.end(), (const uint8_t*)&rt, (const uint8_t*)&rt + sizeof(rt));
                armed_set_ack = ack(send(MSG_SECTION_SET, pl)); }
            if (step >= 6 && ms % 50 == 0) send(MSG_COMMAND, {CMD_BEEP, 255});      // keeps asking for something harmless
            else if (ms % 10 == 0) send(MSG_HELLO, {});
        }
    });

    flight.join(); sensors.join(); link.join();

    CHECK(app.loops() == kCycles, "every control cycle ran (%u of %u)", (unsigned)app.loops(), (unsigned)kCycles);
    // (the control thread owns the test's clock and lets the sensor thread fall one millisecond behind: a PC that is busy with
    //  something else shows about half of the passes, a free one nearly all of them. Less than that: the thread was held up.)
    CHECK(slow_passes > 1000u * kSeconds * 45 / 100, "the sensor thread ran next to the control loop the whole time (%u passes in %d s)", (unsigned)slow_passes.load(), kSeconds);
    CHECK(sync.locks > kCycles + slow_passes, "both threads used the lock (%u times)", (unsigned)sync.locks.load());
    CHECK(pid_ack == 0, "a live settings section was applied while the loop ran (%d)", pid_ack.load());
    CHECK(settings.pid.iterm_windup == 77 && app.core.cfg.pid.iterm_windup == 77, "the new value reached the core (stored %d, core %d)", (int)settings.pid.iterm_windup, (int)app.core.cfg.pid.iterm_windup);
    CHECK(trim_err == ERR_OK || trim_err == ERR_FAILED, "a command ran through the sensor thread (%d)", trim_err.load());
    CHECK(test_err == ERR_OK, "motor test accepted while disarmed (%d)", test_err.load());
    CHECK(armed_at_4s == 1 && motor_at_4s > 100, "armed by the receiver, throttle up: motors run (DShot value %d)", motor_at_4s.load());
    CHECK(armed_set_ack == ERR_ARMED, "settings change refused while armed (%d)", armed_set_ack.load());
    CHECK(armed_at_end == 0, "disarmed by the switch");
    CHECK(status_frames > 100 && loop_hz_seen >= 3900 && loop_hz_seen <= 4100 && target_seen == 4000, "the link saw the loop rate: %d Hz of %d (%d status frames)", loop_hz_seen.load(), target_seen.load(), status_frames.load());
    CHECK(app.telemetry().esc_answers == 0x0F && app.telemetry().bidir_state == BIDIR_STATE_ACTIVE, "ESC replies were taken in the flight thread (answers 0x%x)", app.telemetry().esc_answers);
    CHECK(app.imuErrors() == 0, "no IMU read failed");
    CHECK(g_io_total > 10000 && g_io_locked == 0, "no I2C / UART / ADC access while the lock is held: a control cycle never waits for a bus (%u of %u)", (unsigned)g_io_locked.load(), (unsigned)g_io_total.load());
    g_stub_io_hook = nullptr; g_sync = nullptr;
    CHECK(app.blackbox.ring().dropped() == 0, "no log record was dropped on the way (%u)", (unsigned)app.blackbox.ring().dropped());
    CHECK(blackbox.log().sessions() == 1 && blackbox.log().used() > 100000 && logmem.bad_writes == 0, "the armed flight was logged to the flash by the link thread (%d session, %u bytes)", blackbox.log().sessions(), (unsigned)blackbox.log().used());
    printf("\n%d passed, %d failed\n", g_pass.load(), g_fail.load());
    return g_fail ? 1 : 0;
}
