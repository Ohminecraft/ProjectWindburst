/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "../proto/wbp_server.h"
#include "exchange.h"
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

// Flight-loop side. Returns ERR_OK if the request may be executed now.
inline wbp::Err check_request(const Request& r, bool armed) {
    if (r.cmd == REQ_APPLY_SECTION) return armed ? wbp::ERR_ARMED : (r.n == 1 && wbp::find_section(r.args[0]) ? wbp::ERR_OK : wbp::ERR_BAD_SECTION);
    if (r.cmd == REQ_REBOOT) return armed ? wbp::ERR_ARMED : wbp::ERR_OK;
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
    LinkHost(Exchange& x, Exchange::WaitFn wait, void* ctx, NowFn now) : x_(x), wait_(wait), ctx_(ctx), now_(now) {}

    void attach(wbp::Server* s) { srv_ = s; }
    // Boot report: text lines that never change after setup() (detected sensors, failure reasons). Sent as LOG frames on CMD_REPORT.
    void set_report(const char* const* lines, int n) { report_ = lines; report_n_ = n; }

    // The blackbox service lives in the link task too. `settings` = the Settings object the server edits (for the blackbox section).
    void set_blackbox(bb::Service* svc, const wbp::Settings* settings) { bb_ = svc; settings_ = settings; if (bb_ && settings_) { bb::Config c; wbp::apply_blackbox(*settings_, c); bb_->set_config(c); } }

    bool armed() override { x_.snapshot(tel_); return tel_.armed; }
    void on_frame() override { x_.link_touch(now_()); }
    void tick(uint32_t now_ms) override {
        x_.snapshot(tel_);
        if (tel_.fw_saved_seq != saved_seq_) { saved_seq_ = tel_.fw_saved_seq; autosave_ = tel_.fw_saved_section; }       // the flight loop changed a section and wants it kept
        if (!bb_) return;
        bb::Inputs in; in.now_ms = now_ms; in.armed = tel_.armed; in.switch_on = ((tel_.modes >> wbp::MODE_BLACKBOX) & 1u) != 0; in.app_active = x_.link_active(now_ms);
        bb_->poll(in);
    }
    uint32_t now_ms() override { return now_(); }

    uint16_t fill_stream(uint8_t id, uint8_t* out, uint16_t cap) override {
        if (id == wbp::STR_BLACKBOX) return bb_ ? bb_->pack_status(out, cap) : 0;          // the one stream that does not come from the flight loop
        x_.snapshot(tel_);
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
        const Reply rep = x_.call(r, wait_, ctx_);
        if (rep.n) { memcpy(result, rep.data, rep.n); rlen = rep.n; }
        if (rep.err == wbp::ERR_OK && ci->writes_section) changed_ = true;
        return rep.err;
    }

    bool apply_live(uint8_t section) override {
        const wbp::SectionInfo* si = wbp::find_section(section);
        if (!si || !si->live) return false;                    // needs a reboot: the server raises SF_REBOOT_REQUIRED
        if (section == wbp::SEC_BLACKBOX) {                    // read by the blackbox service only; applies to the next session
            if (bb_ && settings_) { bb::Config c; wbp::apply_blackbox(*settings_, c); bb_->set_config(c); }
            return true;
        }
        Request r; r.cmd = REQ_APPLY_SECTION; r.n = 1; r.args[0] = section;
        return x_.call(r, wait_, ctx_).err == wbp::ERR_OK;
    }
    bool take_settings_changed() override { const bool c = changed_; changed_ = false; return c; }
    // flash writes stall the flight loop for milliseconds: the section waits here until the craft is disarmed
    uint8_t take_autosave() override { if (!autosave_ || tel_.armed) return 0; const uint8_t s = autosave_; autosave_ = 0; return s; }
    void reboot() override { Request r; r.cmd = REQ_REBOOT; x_.call(r, wait_, ctx_); }

private:
    Exchange& x_; Exchange::WaitFn wait_; void* ctx_; NowFn now_;
    wbp::Server* srv_ = nullptr; Telemetry tel_; bool changed_ = false; uint8_t saved_seq_ = 0, autosave_ = 0;
    const char* const* report_ = nullptr; int report_n_ = 0;
    bb::Service* bb_ = nullptr; const wbp::Settings* settings_ = nullptr;
};

} // namespace app
