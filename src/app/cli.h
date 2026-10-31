/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../proto/wbp_server.h"
#include "../proto/wbp_fields.h"
#include "../config/version.h"

namespace app {

// ============================================================================
// A command line on the app's serial port, like Betaflight's: a terminal (PuTTY, screen, the Arduino monitor at the link's baud)
// sends '#' while no protocol frame is under way and gets a prompt. Every setting by its name (section.field), in the units
// the app shows (the schema's scale applied), enums and bit sets by name:
//     get [part of a name]       set <section.field> = <value>      diff [section]      dump
//     defaults [section]         save      reboot      status      version      help      exit
// `diff` / `dump` print `set` lines that can be pasted back.
// Changes go through the same steps as the app's (wbp::Server: clamp, derived values, live apply or "reboot required");
// nothing is changed while armed. Runs in the link task, as the server; out() writes raw text to the port.
// ============================================================================
class Cli {
public:
    using OutFn = void (*)(const char* text, void* ctx);
    Cli(wbp::Settings& s, wbp::Server& srv, OutFn out, void* ctx) : s_(s), srv_(srv), out_(out), ctx_(ctx) {}

    // The byte that opens the command line: '#' between two protocol frames.
    static bool opens(uint8_t c, bool frame_under_way) { return c == '#' && !frame_under_way; }
    void enter() { active_ = true; n_ = 0; cr_ = false; say("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n"); prompt(); }
    bool active() const { return active_; }
    // One byte from the terminal. false once the command line was left (exit).
    bool feed(uint8_t c) {
        if (!active_) return false;
        if (c == '\n' && cr_) { cr_ = false; return true; }               // CR LF: one end of line
        cr_ = c == '\r';
        if (c == '\r' || c == '\n') { say("\r\n"); line_[n_] = 0; run(line_); n_ = 0; if (active_) prompt(); return active_; }
        if (c == 8 || c == 127) { if (n_) { n_--; say("\b \b"); } return true; }
        if (c == 3) { n_ = 0; say("^C\r\n"); prompt(); return true; }      // Ctrl-C: the line is dropped
        if (c < 32 || n_ >= sizeof(line_) - 1) return true;
        line_[n_++] = (char)c; char e[2] = {(char)c, 0}; say(e);          // (the terminal shows what is typed: we echo)
        return true;
    }
    // A whole line, for the tests and for scripts.
    void run(const char* text) {
        char l[256]; strncpy(l, text, sizeof(l) - 1); l[sizeof(l) - 1] = 0; trim(l);
        if (!*l || *l == '#') return;
        char* arg = l; while (*arg && *arg != ' ') arg++;
        if (*arg) { *arg++ = 0; while (*arg == ' ') arg++; }
        if (!strcmp(l, "help")) help();
        else if (!strcmp(l, "get")) get(arg);
        else if (!strcmp(l, "set")) set(arg);
        else if (!strcmp(l, "diff")) dump(true, strcmp(arg, "all") ? arg : "");
        else if (!strcmp(l, "dump")) dump(false, "");
        else if (!strcmp(l, "defaults")) defaults(arg);
        else if (!strcmp(l, "save")) save();
        else if (!strcmp(l, "reboot")) { say("Rebooting\r\n"); srv_.reboot_now(); }
        else if (!strcmp(l, "status")) status();
        else if (!strcmp(l, "version")) { char b[64]; snprintf(b, sizeof(b), "# Windburst %d.%d.%d, schema 0x%08lX\r\n", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, (unsigned long)wbp::kSchemaHash); say(b); }
        else if (!strcmp(l, "exit")) { active_ = false; say(srv_.flags() & wbp::SF_REBOOT_REQUIRED ? "Leaving CLI mode - some changes need a reboot (save, then reboot)\r\n" : "Leaving CLI mode\r\n"); }
        else { say("Unknown command, try 'help'\r\n"); }
    }

private:
    void say(const char* t) { out_(t, ctx_); }
    void prompt() { say("# "); }
    static void trim(char* s) { size_t n = strlen(s); while (n && (s[n - 1] == ' ' || s[n - 1] == '\t')) s[--n] = 0; size_t i = 0; while (s[i] == ' ' || s[i] == '\t') i++; if (i) memmove(s, s + i, n - i + 1); }
    static const char* section_name(uint8_t id) { const wbp::SectionInfo* si = wbp::find_section(id); return si ? si->name : "?"; }
    // a section of this device (the server's role): the other device's settings are not listed, set or reset here
    bool mine(uint8_t id) const { const wbp::SectionInfo* si = wbp::find_section(id); return si && si->role == srv_.role(); }
    static int tsize(uint8_t t) { switch (t) { case wbp::FT_U16: case wbp::FT_I16: return 2; case wbp::FT_U32: case wbp::FT_I32: case wbp::FT_F32: return 4; default: return 1; } }
    uint8_t* addr(const wbp::FieldInfo& f, int i) const { return wbp::section_ptr(s_, f.section) + f.offset + i * tsize(f.type); }
    double raw(const wbp::FieldInfo& f, int i) const {
        const uint8_t* p = addr(f, i);
        switch (f.type) {
        case wbp::FT_U16: { uint16_t v; memcpy(&v, p, 2); return v; } case wbp::FT_I16: { int16_t v; memcpy(&v, p, 2); return v; }
        case wbp::FT_U32: { uint32_t v; memcpy(&v, p, 4); return v; } case wbp::FT_I32: { int32_t v; memcpy(&v, p, 4); return v; }
        case wbp::FT_F32: { float v; memcpy(&v, p, 4); return v; } default: return *p;
        }
    }
    void put_raw(const wbp::FieldInfo& f, int i, double v) {
        uint8_t* p = addr(f, i);
        switch (f.type) {
        case wbp::FT_U16: { const uint16_t x = (uint16_t)v; memcpy(p, &x, 2); break; } case wbp::FT_I16: { const int16_t x = (int16_t)v; memcpy(p, &x, 2); break; }
        case wbp::FT_U32: { const uint32_t x = (uint32_t)v; memcpy(p, &x, 4); break; } case wbp::FT_I32: { const int32_t x = (int32_t)v; memcpy(p, &x, 4); break; }
        case wbp::FT_F32: { const float x = (float)v; memcpy(p, &x, 4); break; } default: *p = (uint8_t)v;
        }
    }
    static int decimals(float scale) { int d = 0; float s = scale; while (d < 6 && fabsf(s - roundf(s)) > 1e-6f) { s *= 10.0f; d++; } return d; }
    // one value as text (the way `set` takes it back)
    void value_text(const wbp::FieldInfo& f, int i, char* b, size_t cap) const {
        const double v = raw(f, i);
        if (f.type == wbp::FT_BOOL) { snprintf(b, cap, "%s", v ? "ON" : "OFF"); return; }
        if (f.flags & wbp::FF_BITS) {
            size_t n = 0; b[0] = 0; const uint32_t bits = (uint32_t)v;
            for (int k = 0; k < f.n_names && k < 32; k++) if ((bits >> k) & 1u) n += (size_t)snprintf(b + n, n < cap ? cap - n : 0, "%s%s", n ? "," : "", f.names[k]);
            if (!n) snprintf(b, cap, "NONE");
            return;
        }
        if (f.names) { const int k = (int)v; if ((f.flags & wbp::FF_OPTIONAL) && k == 255) snprintf(b, cap, "NONE"); else if (k >= 0 && k < f.n_names) snprintf(b, cap, "%s", f.names[k]); else snprintf(b, cap, "%d", k); return; }
        if (f.type == wbp::FT_F32) { snprintf(b, cap, "%.6g", v); return; }
        if (f.scale != 1.0f) { snprintf(b, cap, "%.*f", decimals(f.scale), v * f.scale); return; }
        snprintf(b, cap, "%.0f", v);
    }
    void field_text(const wbp::FieldInfo& f, char* b, size_t cap) const {
        if (f.flags & wbp::FF_TEXT) {
            const uint8_t* p = addr(f, 0); size_t n = 0; b[n++] = '"';
            for (int i = 0; i < f.count && p[i] && n + 2 < cap; i++) b[n++] = (char)p[i];
            b[n++] = '"'; b[n] = 0; return;
        }
        size_t n = 0; b[0] = 0;
        for (int i = 0; i < f.count && n + 1 < cap; i++) { if (i) b[n++] = ','; value_text(f, i, b + n, cap - n); n += strlen(b + n); }
    }
    void line_of(const wbp::FieldInfo& f, char* b, size_t cap, bool as_set) const {
        char v[200]; field_text(f, v, sizeof(v));
        snprintf(b, cap, as_set ? "set %s.%s = %s\r\n" : "%s.%s = %s\r\n", section_name(f.section), f.name, v);
    }
    // ---- commands
    void help() {
        say("get [name]            settings whose name contains it (all without one)\r\n"
            "set <name> = <value>  e.g. set general.loop_hz = 8000 | set pins.motor = 4,5,6,7 | set pins.motor[2] = 6 | set general.usb_name = \"My quad\"\r\n"
            "diff [section]        set lines of what differs from the defaults\r\n"
            "dump                  every setting as set lines\r\n"
            "defaults [section]    back to the defaults (in RAM: save to keep)\r\n"
            "save                  write the settings to flash\r\n"
            "reboot | status | version | exit\r\n");
    }
    void get(const char* part) {
        char b[260]; int n = 0;
        for (const wbp::FieldInfo& f : wbp::kFields) {
            if (!mine(f.section)) continue;
            char full[64]; snprintf(full, sizeof(full), "%s.%s", section_name(f.section), f.name);
            if (*part && !strstr(full, part)) continue;
            line_of(f, b, sizeof(b), false); say(b); n++;
            if (*part && (f.names || (f.flags & wbp::FF_RANGE))) {                       // with a filter: what the field takes
                if ((f.flags & wbp::FF_RANGE) && !f.names) { char r[96]; const int d = decimals(f.scale); snprintf(r, sizeof(r), "  allowed: %.*f .. %.*f\r\n", d, f.min * f.scale, d, f.max * f.scale); say(r); }
                else if (f.names) { say(f.flags & wbp::FF_BITS ? "  bits: " : "  allowed: "); for (int k = 0; k < f.n_names; k++) { say(f.names[k]); say(k + 1 < f.n_names ? ", " : "\r\n"); } }
            }
        }
        if (!n) say("Nothing matches\r\n");
    }
    const wbp::FieldInfo* find(const char* name) const {
        for (const wbp::FieldInfo& f : wbp::kFields) { if (!mine(f.section)) continue; char full[64]; snprintf(full, sizeof(full), "%s.%s", section_name(f.section), f.name); if (!strcmp(full, name)) return &f; }
        return nullptr;
    }
    // a decimal number: [-+]digits[.digits][e[-+]digits] (not strtod: on the STM32 it pulls the C library's whole stdio in)
    static bool number(const char* t, double& out) {
        const char* p = t; bool neg = false; if (*p == '-' || *p == '+') neg = *p++ == '-';
        double v = 0; int digits = 0; while (*p >= '0' && *p <= '9') { v = v * 10 + (*p++ - '0'); digits++; }
        if (*p == '.') { p++; double f = 0.1; while (*p >= '0' && *p <= '9') { v += (*p++ - '0') * f; f *= 0.1; digits++; } }
        if (!digits) return false;
        if (*p == 'e' || *p == 'E') { p++; bool en = false; if (*p == '-' || *p == '+') en = *p++ == '-'; int e = 0, ed = 0; while (*p >= '0' && *p <= '9') { e = e * 10 + (*p++ - '0'); ed++; if (e > 60) return false; } if (!ed) return false; while (e--) v = en ? v / 10 : v * 10; }
        if (*p) return false;
        out = neg ? -v : v; return true;
    }
    // the next comma-separated item of `rest` (cut out in place), nullptr at the end (not strtok: it drags newlib's reentrancy / stdio in on the STM32)
    static char* next_item(char*& rest) {
        if (!rest) return nullptr;
        char* item = rest; char* c = strchr(rest, ',');
        if (c) { *c = 0; rest = c + 1; } else rest = nullptr;
        return item;
    }
    static bool ieq(const char* a, const char* b) { while (*a && *b) { char x = *a++, y = *b++; if (x >= 'a' && x <= 'z') x -= 32; if (y >= 'a' && y <= 'z') y -= 32; if (x != y) return false; } return !*a && !*b; }
    // one value from text into raw units; false = not a value this field takes (why says what)
    bool parse_value(const wbp::FieldInfo& f, const char* t, double& out, const char*& why) const {
        if (f.type == wbp::FT_BOOL) { if (ieq(t, "on") || ieq(t, "1") || ieq(t, "true")) { out = 1; return true; } if (ieq(t, "off") || ieq(t, "0") || ieq(t, "false")) { out = 0; return true; } why = "ON or OFF"; return false; }
        if (f.flags & wbp::FF_BITS) {
            uint32_t bits = 0; char tmp[200]; strncpy(tmp, t, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
            if (!ieq(tmp, "none")) for (char* rest = tmp, *tok; (tok = next_item(rest)) != nullptr; ) {
                char* x = tok; while (*x == ' ') x++; trim(x); int k = 0; while (k < f.n_names && !ieq(f.names[k], x)) k++;
                if (k == f.n_names) { why = "one of the names 'get' lists, separated by commas"; return false; }
                bits |= 1u << k;
            }
            out = bits; return true;
        }
        if (f.names) {
            if ((f.flags & wbp::FF_OPTIONAL) && ieq(t, "none")) { out = 255; return true; }
            for (int k = 0; k < f.n_names; k++) if (ieq(f.names[k], t)) { out = k; return true; }
            why = "one of the names 'get <name>' lists"; return false;
        }
        double v = 0; if (!number(t, v)) { why = "a number"; return false; }
        double r = f.type == wbp::FT_F32 ? v : v / (double)f.scale;
        if (f.type != wbp::FT_F32) r = floor(r + 0.5);
        if (f.flags & wbp::FF_RANGE) { if (r < f.min - 1e-9 || r > f.max + 1e-9) { why = "out of range (see 'get <name>')"; return false; } }
        else {
            static const double lo[] = {0, 0, 0, 0, -32768, -2147483648.0, -3.4e38}, hi[] = {255, 1, 65535, 4294967295.0, 32767, 2147483647.0, 3.4e38};
            if (r < lo[f.type] || r > hi[f.type]) { why = "out of range for this field"; return false; }
        }
        out = r; return true;
    }
    void set(char* arg) {
        if (srv_.armed()) { say("Not while armed\r\n"); return; }
        char* eq = strchr(arg, '='); if (!eq) { say("Usage: set <section.field> = <value>\r\n"); return; }
        *eq = 0; char* name = arg; char* val = eq + 1; trim(name); trim(val);
        int index = -1; char* br = strchr(name, '[');
        if (br) { index = 0; for (const char* q = br + 1; *q >= '0' && *q <= '9'; q++) index = index * 10 + (*q - '0'); *br = 0; }
        const wbp::FieldInfo* fp = find(name); if (!fp) { say("No such setting (try 'get'): "); say(name); say("\r\n"); return; }
        const wbp::FieldInfo& f = *fp;
        if (index >= f.count || (index >= 0 && (f.flags & wbp::FF_TEXT))) { say("Index out of range\r\n"); return; }
        if (f.flags & wbp::FF_TEXT) {
            size_t n = strlen(val); if (n >= 2 && val[0] == '"' && val[n - 1] == '"') { val[n - 1] = 0; val++; n -= 2; }
            if (n > (size_t)f.count - 1) { say("Too long\r\n"); return; }
            uint8_t* p = addr(f, 0); memset(p, 0, f.count); memcpy(p, val, n);
        } else {
            double v[64]; int k = 0; const char* why = "";
            if (index >= 0 || f.count == 1 || (f.flags & wbp::FF_BITS)) { if (!parse_value(f, val, v[0], why)) { say("Invalid value: "); say(why); say("\r\n"); return; } k = 1; }
            else {
                char tmp[256]; strncpy(tmp, val, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
                for (char* rest = tmp, *tok; k < 64 && (tok = next_item(rest)) != nullptr; ) { trim(tok); if (!parse_value(f, tok, v[k], why)) { say("Invalid value: "); say(why); say("\r\n"); return; } k++; }
                if (k != f.count) { char b[64]; snprintf(b, sizeof(b), "Needs %d values (or name[i] = one)\r\n", f.count); say(b); return; }
            }
            if (index >= 0) put_raw(f, index, v[0]); else for (int i = 0; i < k; i++) put_raw(f, i, v[i]);
        }
        srv_.commit_section(f.section);
        char b[260]; line_of(f, b, sizeof(b), false); say(b);                   // what it is now (after the firmware's own limits)
    }
    void dump(bool only_changed, const char* section) {
        static wbp::Settings d; wbp::defaults(d);                              // (static: it is big for a task's stack)
        char b[260]; uint8_t last = 0;
        say("# Windburst "); { char v[24]; snprintf(v, sizeof(v), "%d.%d.%d", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH); say(v); } say(only_changed ? " diff\r\n" : " dump\r\n");
        for (const wbp::FieldInfo& f : wbp::kFields) {
            if (!mine(f.section) || (*section && strcmp(section_name(f.section), section))) continue;
            if (only_changed) { const size_t n = (size_t)tsize(f.type) * f.count; if (!memcmp(addr(f, 0), wbp::section_ptr(d, f.section) + f.offset, n)) continue; }
            if (f.section != last) { last = f.section; say("\r\n# "); say(section_name(f.section)); say("\r\n"); }
            line_of(f, b, sizeof(b), true); say(b);
        }
        say("\r\n");
    }
    void defaults(const char* section) {
        if (srv_.armed()) { say("Not while armed\r\n"); return; }
        int n = 0;
        for (const wbp::SectionInfo& si : wbp::kSections) if (mine(si.id) && (!*section || !strcmp(section, si.name))) { wbp::defaults_section(s_, si.id); srv_.commit_section(si.id); n++; }
        say(n ? "Defaults loaded (in RAM: 'save' to keep them)\r\n" : "No such section\r\n");
    }
    void save() {
        if (srv_.armed()) { say("Not while armed\r\n"); return; }
        if (!srv_.save_all()) { say("Save FAILED\r\n"); return; }
        say(srv_.flags() & wbp::SF_REBOOT_REQUIRED ? "Saved - some changes take effect after 'reboot'\r\n" : "Saved\r\n");
    }
    void status() {
        char b[96]; snprintf(b, sizeof(b), "Windburst %d.%d.%d, %s, settings %s%s\r\n", WB_FW_MAJOR, WB_FW_MINOR, WB_FW_PATCH, srv_.armed() ? "ARMED" : "disarmed",
                             srv_.flags() & wbp::SF_DIRTY ? "not saved" : "saved", srv_.flags() & wbp::SF_REBOOT_REQUIRED ? ", reboot required" : "");
        say(b);
    }

    wbp::Settings& s_; wbp::Server& srv_; OutFn out_; void* ctx_;
    bool active_ = false, cr_ = false; char line_[200] = {}; size_t n_ = 0;
};

} // namespace app
