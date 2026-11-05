/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../proto/wbp_server.h"
#include "exchange.h"
#include "tx_queue.h"
#include "msp_bridge.h"
#include "msp_config.h"
#include "msp_cli.h"
#include "../blackbox/bb_service.h"
#include "../proto/settings_apply.h"

// ============================================================================
// LinkHost: what the WBP server sees of the firmware (wbp::Host), living in the LINK task.
// It never touches a driver or the flight core: telemetry comes from the Exchange snapshot, and every command / live settings
// change is handed to the flight loop through the Exchange and executed there (see exchange.h).
//
// check_request() is the gate the flight loop runs before executing a request: unknown command, wrong argument size and
// "not while armed" are decided in one place, with the armed state of that very control cycle (no race with the link task).
// Portable: no Arduino includes.
// ============================================================================
namespace app {

// What an MSP peer is told about the craft (msp::Bridge): the ESC tools on the app link, the video transmitter on the OSD's UART.
inline void msp_info_of(const Telemetry& t, const wbp::Settings* s, msp::Info& in) {
    in.armed = t.armed; in.motors = t.motor_count; in.cycle_us = t.pid_target_hz ? (uint16_t)(1000000u / t.pid_target_hz) : 250;   // (Betaflight's cycle time is the PID loop's)
    in.batt_v = t.batt_v; in.amps = t.batt_amps; in.cells = t.batt_cells; in.mah = (uint16_t)(t.batt_mah > 65535.0f ? 65535.0f : t.batt_mah > 0 ? t.batt_mah : 0.0f);
    if (s) {                                             // (read only)
        static const uint8_t bf[] = {0, 1, 2, 3, 5, 6, 7, 7, 8, 4};   // esc_protocol -> Betaflight (DShot1200: none there, said 600)
        const uint8_t pr = s->motors.protocol;
        in.bf_motor_protocol = pr < sizeof(bf) ? bf[pr] : 6;
        in.pwm_rate = s->motors.pulse_hz; in.idle_x100 = (uint16_t)(s->motors.idle_x10 * 10u);
        in.pulse_disarmed = s->motors.pulse_disarmed; in.pulse_armed = s->motors.pulse_armed; in.pulse_max = s->motors.pulse_max;
        in.pole_count = s->motors.pole_count; in.bidir = s->motors.bidirectional != 0;
        in.airmode = s->general.airmode != 0;
        const uint8_t rp = s->receiver.protocol;
        in.rx_kind = rp == wbp::RX_PROTOCOL_PPM ? 1 : rp == wbp::RX_PROTOCOL_PWM ? 2 : 0;
        in.gps = s->sensors.gps_enabled != 0; in.range = s->sensors.range_enabled != 0;
        in.identity = s->msp.identity;
        in.modes_set = 0;
        for (int i = 0; i < 16; i++) if (s->modes.hi[i] > s->modes.lo[i] && s->modes.mode[i] < 32) in.modes_set |= 1u << s->modes.mode[i];
    }
    in.health = t.health; in.modes_on = t.modes; in.blockers = t.blockers; in.cpu_pct = t.cpu_pct; if (t.loop_hz) in.gyro_hz = t.loop_hz;      // (the gyro is read every loop)
    for (int i = 0; i < 8; i++) in.motor_out[i] = t.motor_cmd[i];
}

// Flight-loop side. Returns ERR_OK if the request may be executed now.
inline wbp::Err check_request(const Request& r, bool armed) {
    if (r.cmd == REQ_APPLY_SECTION) return armed ? wbp::ERR_ARMED : (r.n == 1 && wbp::find_section(r.args[0]) ? wbp::ERR_OK : wbp::ERR_BAD_SECTION);
    if (r.cmd == REQ_REBOOT || r.cmd == REQ_ESC_PASSTHROUGH) return armed ? wbp::ERR_ARMED : wbp::ERR_OK;
    const wbp::CommandInfo* ci = wbp::find_command(r.cmd);
    if (!ci) return wbp::ERR_UNKNOWN;
    if (r.n != ci->arg_bytes) return wbp::ERR_BAD_ARG;
    if (armed && !ci->armed_ok) return wbp::ERR_ARMED;
    return wbp::ERR_OK;
}

class LinkHost : public wbp::Host {
public:
    using NowFn = uint32_t (*)();
    // wait/ctx: how the link task sleeps ~1 ms while the flight loop executes a request (vTaskDelay(1)); now: millis()
    LinkHost(Exchange& x, Exchange::WaitFn wait, void* ctx, NowFn now) : x_(x), wait_(wait), ctx_(ctx), now_(now) { boot_ms_ = now_(); }

    void attach(wbp::Server* s) { srv_ = s; }
    // Boot report: text lines that never change after setup() (detected sensors, failure reasons). Sent as LOG frames on CMD_REPORT.
    void set_report(const char* const* lines, int n) { report_ = lines; report_n_ = n; }
    void set_ports(const PortUse* p) { ports_ = p; }                 // what each UART serves (FlightApp::port_use(): set once begin() is done)
    const PortUse* ports() const { return ports_; }

    // The blackbox service lives in the link task too. `settings` = the Settings object the server edits (for the blackbox section).
    void set_blackbox(bb::Service* svc, const wbp::Settings* settings) { bb_ = svc; settings_ = settings; if (bb_ && settings_) { bb::Config c; wbp::apply_blackbox(*settings_, c); bb_->set_config(c); } }

    bool armed() override { x_.snapshot(tel_); return tel_.armed; }
    void on_frame() override { x_.link_touch(now_()); }
    // the transmit queue and the USB kick counter (link_arduino.h), for the link stream; either may be null
    void set_port(const QueuedTransport* tx, const volatile uint32_t* kicks) { tx_ = tx; kicks_ = kicks; }
    // a number that differs from one start to the next (esp_random() on the board): the app tells a restart by it
    void set_boot_id(uint32_t id) { boot_id_ = id; }
    void tick(uint32_t now_ms) override {
        // how regularly the link task runs: the longest pause between two passes, per second (a pass that waits for a request counts)
        if (tick_seen_) { const uint32_t gap = now_ms - last_tick_ms_; if (gap > gap_cur_) gap_cur_ = gap; }
        tick_seen_ = true; last_tick_ms_ = now_ms;
        if ((uint32_t)(now_ms - win_ms_) >= 1000u) { win_ms_ = now_ms; gap_last_ = gap_cur_; gap_cur_ = 0; cmd_last_ = cmd_cur_; cmd_cur_ = 0; }
        x_.snapshot(tel_);
        if (tel_.fw_saved_seq != saved_seq_) { saved_seq_ = tel_.fw_saved_seq; autosave_ |= (1u << tel_.fw_saved_section) | tel_.fw_saved_mask; }       // the flight loop changed a section (or several, 0.43: MAVLink parameters) and wants it kept
        if (!bb_) return;
        bb::Inputs in; in.now_ms = now_ms; in.armed = tel_.armed; in.switch_on = ((tel_.modes >> wbp::MODE_BLACKBOX) & 1u) != 0; in.app_active = x_.link_active(now_ms);
        in.erase_switch = ((tel_.modes >> wbp::MODE_BLACKBOX_ERASE) & 1u) != 0;
        bb_->poll(in);
    }
    uint32_t now_ms() override { return now_(); }

    uint16_t fill_stream(uint8_t id, uint8_t* out, uint16_t cap) override {
        if (id == wbp::STR_BLACKBOX) return bb_ ? bb_->pack_status(out, cap) : 0;          // the one stream that does not come from the flight loop
        x_.snapshot(tel_);
        if (id == wbp::STR_LINK) fill_link(tel_);
        return pack_stream(id, tel_, srv_ ? srv_->flags() : 0, out, cap);
    }

    wbp::Err command(uint8_t cmd, const uint8_t* args, uint16_t n, uint8_t* result, uint16_t& rlen) override {
        rlen = 0;
        const wbp::CommandInfo* ci = wbp::find_command(cmd);
        if (!ci) return wbp::ERR_UNKNOWN;
        if (n != ci->arg_bytes) return wbp::ERR_BAD_ARG;
        if (cmd == wbp::CMD_REPORT) {                          // answered by the link task itself: static text, no flight-loop involvement
            if (srv_) for (int i = 0; i < report_n_; i++) srv_->log(1, report_[i]);
            return wbp::ERR_OK;
        }
        if (bb::Service::handles(cmd)) {                       // log memory and stream: link-task business, the flight loop is not asked
            if (!bb_) return wbp::ERR_NOT_SUPPORTED;
            if (!ci->armed_ok && armed()) return wbp::ERR_ARMED;
            return bb_->command(cmd, args, result, rlen);
        }
        if (n > sizeof(Request::args)) return wbp::ERR_BAD_ARG;
        Request r; r.cmd = cmd; r.n = (uint8_t)n; if (n) memcpy(r.args, args, n);
        const Reply rep = timed_call(r);
        if (rep.n) { memcpy(result, rep.data, rep.n); rlen = rep.n; }
        if (rep.err == wbp::ERR_OK && ci->writes_section) changed_ = true;
        return rep.err;
    }

    bool apply_live(uint8_t section) override {
        const wbp::SectionInfo* si = wbp::find_section(section);
        // needs a reboot: the server raises SF_REBOOT_REQUIRED. (motors: the flight loop takes a new order / spin direction at once
        // and says no to anything else - FlightApp::apply_motor_order, the motor wizard)
        if (!si || (!si->live && section != wbp::SEC_MOTORS)) return false;
        if (section == wbp::SEC_BLACKBOX) {                    // read by the blackbox service only; applies to the next session
            if (bb_ && settings_) { bb::Config c; wbp::apply_blackbox(*settings_, c); bb_->set_config(c); }
            return true;
        }
        Request r; r.cmd = REQ_APPLY_SECTION; r.n = 1; r.args[0] = section;
        return timed_call(r).err == wbp::ERR_OK;
    }
    bool take_settings_changed() override { const bool c = changed_; changed_ = false; return c; }
    // flash writes stall the flight loop for milliseconds: the section waits here until the craft is disarmed
    uint8_t take_autosave() override {
        if (!autosave_ || tel_.armed) return 0;
        for (uint8_t s = 1; s < 32; s++) if (autosave_ & (1u << s)) { autosave_ &= ~(1u << s); return s; }      // (one section a call; bit 0: no section 0)
        autosave_ = 0; return 0;
    }
    void reboot() override { Request r; r.cmd = REQ_REBOOT; timed_call(r); }
    // ESC passthrough: the flight side stops the motor output for good. Returns the number of ESCs, 0 = refused (armed, no motors).
    uint8_t esc_passthrough() { Request r; r.cmd = REQ_ESC_PASSTHROUGH; const Reply rep = timed_call(r); return rep.err == wbp::ERR_OK && rep.n ? rep.data[0] : 0; }

    // the link task's own counters, into a telemetry copy (link stream)
    void fill_link(Telemetry& t) const {
        const uint32_t now = now_();
        t.up_ms = now - boot_ms_;                              // (since this object was made: the firmware's start on the board, a restart in the simulator)
        t.boot_id = boot_id_; t.link_busy = sat_u16(x_.timeouts());
        if (srv_) { t.link_rx = srv_->frames(); t.link_bad = sat_u16(srv_->bad_frames()); t.link_dropped = srv_->dropped_streams(); }
        if (tx_) { t.link_tx_lost = sat_u16(tx_->frames_lost()); t.link_queued = sat_u16((uint32_t)tx_->queued()); }
        if (kicks_) t.link_kicks = sat_u16(*kicks_);
        t.link_cmd_max_ms = sat_u16(cmd_last_ > cmd_cur_ ? cmd_last_ : cmd_cur_); t.link_gap_ms = sat_u16(gap_last_ > gap_cur_ ? gap_last_ : gap_cur_);
    }

    // what the MSP side (ESC tools) is told: from the telemetry snapshot
    void msp_info(msp::Info& in) {
        x_.snapshot(tel_);
        msp_info_of(tel_, settings_, in);
        memcpy(in.uid, uid_, 12);
    }
    void set_uid(const uint8_t uid[12]) { memcpy(uid_, uid, 12); }
    void msp_arming(bool disable) { x_.msp_arming(disable, now_()); }      // (MSP_SET_ARMING_DISABLED, app/exchange.h)
    void msp_touch() { x_.msp_touch(now_()); }
    const Telemetry& telemetry() const { return tel_; }       // the snapshot msp_info() took
    wbp::Server* server() const { return srv_; }
    Reply call(const Request& r) { return timed_call(r); }          // a request to the flight loop (MSP_SET_MOTOR's motor test)

private:
    uint8_t uid_[12] = {};
    Reply timed_call(const Request& r) {
        const uint32_t t0 = now_(); const Reply rep = x_.call(r, wait_, ctx_);
        const uint32_t d = now_() - t0; if (d > cmd_cur_) cmd_cur_ = d;
        return rep;
    }
    const QueuedTransport* tx_ = nullptr; const volatile uint32_t* kicks_ = nullptr; uint32_t boot_ms_ = 0, boot_id_ = 0;
    bool tick_seen_ = false; uint32_t last_tick_ms_ = 0, win_ms_ = 0, gap_cur_ = 0, gap_last_ = 0, cmd_cur_ = 0, cmd_last_ = 0;
    Exchange& x_; Exchange::WaitFn wait_; void* ctx_; NowFn now_;
    wbp::Server* srv_ = nullptr; Telemetry tel_; bool changed_ = false; uint8_t saved_seq_ = 0; uint32_t autosave_ = 0;      // autosave_: a bit per section to keep
    const char* const* report_ = nullptr; int report_n_ = 0; const PortUse* ports_ = nullptr;
    bb::Service* bb_ = nullptr; const wbp::Settings* settings_ = nullptr;
};

// MSP next to WBP on the app link (msp_bridge.h): the link task gives every byte it received to both. on_byte() answers the ESC
// tools' questions itself and returns the number of ESCs once a passthrough was granted (the flight side has let go of the motor
// pins): the caller then runs the 4-way interface (hal/motor/esc4way.h) on this port and restarts the firmware when it ends.
class MspLink {
public:
    // settings: the object the app's server edits - given, the configurators may read and (msp.writes) change it (app/msp_config.h)
    MspLink(LinkHost& h, wbp::Transport& tx, wbp::Settings* settings = nullptr) : h_(h), tx_(tx), settings_(settings), port_(h), cfg_(settings ? *settings : spare(), port_) {
        if (settings) br_.set_handler(&msp::Config::handler, &cfg_);
    }
    void set_firmware(uint8_t maj, uint8_t min, uint8_t patch, const char* date, const char* time) { info_.fw_major = maj; info_.fw_minor = min; info_.fw_patch = patch; info_.build_date = date; info_.build_time = time; info_.target = WB_MCU_NAME; }
    uint8_t on_byte(uint8_t c) {
        // the Betaflight App's CLI over MSP (app/msp_cli.h): STX, a printable line, ETX - only between frames of both protocols
        if (cli_) {
            if (c == 0x03) { cli_ = false; cli_buf_[cli_n_] = 0; run_cli(); return 0; }
            if (c == 0x0A || c == 0x0D) return 0;
            if (c < 0x20 || c > 0x7E || cli_n_ >= sizeof(cli_buf_) - 1) { cli_ = false; return 0; }       // (not a command line: dropped)
            cli_buf_[cli_n_++] = (char)c; return 0;
        }
        if (c == 0x02 && settings_ && h_.server() && !br_.mid_frame() && !h_.server()->rx_mid_frame()) { cli_ = true; cli_n_ = 0; return 0; }
        if (c == '$' && !info_fresh_) { h_.msp_info(info_); cfg_.set_telemetry(&h_.telemetry()); info_fresh_ = true; }      // (a fresh snapshot per frame, not per byte)
        uint8_t out[msp::kMaxReply + 9]; size_t n = 0;
        const msp::Bridge::Result r = br_.feed(c, info_, out, sizeof(out), n);
        if (r == msp::Bridge::NONE) return 0;
        info_fresh_ = false;
        if (r == msp::Bridge::REPLY) { h_.msp_touch(); tx_.write(out, n); if (cfg_.reboot_pending()) h_.reboot(); return 0; }
        const uint8_t escs = h_.esc_passthrough();          // PASSTHROUGH: ask the flight side (refused armed, without motors)
        n = br_.passthrough_reply(escs, out, sizeof(out)); tx_.write(out, n);
        passthroughs_ += escs ? 1 : 0;
        return escs;
    }
    const msp::Bridge& bridge() const { return br_; }
    const msp::Config& config() const { return cfg_; }
    bool mid_frame() const { return br_.mid_frame(); }
    uint32_t passthroughs() const { return passthroughs_; }
private:
    // writes go the way the app's own go: through the server (clamp, rules, apply live / reboot required; save)
    struct Port : msp::ConfigPort {
        explicit Port(LinkHost& h) : h(h) {}
        LinkHost& h;
        bool armed() override { return h.armed(); }
        void commit(uint8_t section) override { if (wbp::Server* s = h.server()) s->commit_section(section); }
        bool save() override { wbp::Server* s = h.server(); return s && s->save_all(); }
        void arming_disabled(bool on) override { h.msp_arming(on); }
        // MSP_SET_MOTOR: the same requests as the app's Motor tab (the flight loop's gate: disarmed; each one keeps the motor 600 ms)
        bool motor_test(const uint16_t* us, int n) override {
            bool any = false, ok = true;
            for (int i = 0; i < n && i < 8; i++) {
                const uint16_t v = us[i] <= 1000 ? 0 : us[i] >= 2000 ? 1000 : (uint16_t)(us[i] - 1000);     // (0.1 %)
                if (v == 0 && !was[i]) continue;
                Request r; r.cmd = wbp::CMD_MOTOR_TEST; r.n = 3; r.args[0] = (uint8_t)i; r.args[1] = (uint8_t)v; r.args[2] = (uint8_t)(v >> 8);
                const Reply rep = h.call(r);
                if (rep.err != wbp::ERR_OK && v) ok = false;                // (a motor this frame has not: refused, fine when asked to stand)
                was[i] = v != 0; any = any || v != 0;
            }
            if (!any) { Request r; r.cmd = wbp::CMD_MOTOR_STOP; h.call(r); }
            return ok;
        }
        bool was[8] = {};
    };
    static wbp::Settings& spare() { static wbp::Settings s; return s; }      // (no settings given: the handler is not installed)
    void run_cli() {
        h_.msp_touch();
        const uint8_t stx = 0x02, etx = 0x03; tx_.write(&stx, 1);
        app::BfCli bf(*settings_, *h_.server(), [](const char* t, void* ctx) { ((wbp::Transport*)ctx)->write((const uint8_t*)t, strlen(t)); }, &tx_, h_.ports());
        bf.run(cli_buf_);
        tx_.write(&etx, 1);
    }
    LinkHost& h_; wbp::Transport& tx_; wbp::Settings* settings_ = nullptr; msp::Bridge br_; msp::Info info_; bool info_fresh_ = false; uint32_t passthroughs_ = 0;
    Port port_; msp::Config cfg_;
    bool cli_ = false; char cli_buf_[128] = {}; uint8_t cli_n_ = 0;
};

} // namespace app
