/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cli.h"
#include "sync.h"
#include "../proto/wbp_schema.h"

// ============================================================================
// The Betaflight App's CLI over MSP (0.44): the App sends STX, a command line, LF, ETX on the app link (useMspCliSession.ts) and
// reads the lines that come back between STX and ETX. The Ports tab (API 1.49) is read-only and assigns serial ports with it.
// Betaflight's own commands here, the rest goes to this firmware's CLI (app/cli.h: get / set section.field, diff, dump, ...):
//   peripherals                 one line per UART: "serial UART1: gps*" - what each one serves (the claims), * = opened
//   get <x>_uart                "gps_uart = UART1" and "Allowed values: NONE, UART0, UART1, UART2"
//   set <x>_uart = UARTn|NONE   the serial ports table (ports.assign TABLE; the other assignments as they were)
//   save                        save, then restart (Betaflight's save restarts; the App reconnects)
//   exit                        restart without saving (Betaflight's)
// x: gps (GPS), rx (SERIAL_RX), opticalflow (FLOW_MSP: this firmware's MSP port), esc_sensor (ESC_SENSOR), osd (DISPLAYPORT),
// telemetry (TELEMETRY: MAVLink). The AUTO assignment is worked out as at start-up (app/sync.h plan_uarts / flow_uart / esc_uart).
// Portable (test_msp_config, test_fw).
// ============================================================================
namespace app {

class BfCli {
public:
    using OutFn = Cli::OutFn;
    // use: what the UARTs really serve (FlightApp::port_use(), via LinkHost::ports()); nullptr: worked out from the settings
    BfCli(wbp::Settings& s, wbp::Server& srv, OutFn out, void* ctx, const PortUse* use) : s_(s), srv_(srv), out_(out), ctx_(ctx), use_(use && use->known ? use : nullptr), u0_(use && use->known && !use->uart0_link) {}

    // a line of either CLI: Betaflight's commands here, the rest to this firmware's CLI
    void run(const char* line) { if (!handle(line)) { Cli c(s_, srv_, out_, ctx_); c.run(line); } }
    // Betaflight's commands only (the '#' terminal's hook, app/cli.h set_ext): true when the line was one of them
    bool handle(const char* line) {
        char l[128]; strncpy(l, line, sizeof(l) - 1); l[sizeof(l) - 1] = 0;
        size_t n = strlen(l); while (n && (l[n - 1] == '\r' || l[n - 1] == '\n' || l[n - 1] == ' ')) l[--n] = 0;
        const char* p = l; while (*p == ' ') p++;
        if (!strcmp(p, "peripherals")) { peripherals(); return true; }
        if (!strcmp(p, "save")) { if (srv_.save_all()) { say("Saving\r\nRebooting\r\n"); srv_.reboot_now(); } else say("###ERROR: SAVE FAILED (armed?)###\r\n"); return true; }
        if (!strcmp(p, "exit")) { say("Leaving CLI mode, unsaved changes lost.\r\nRebooting\r\n"); srv_.reboot_now(); return true; }
        if (!strncmp(p, "get ", 4) && fn_of(p + 4) >= 0) { get_uart(p + 4); return true; }
        if (!strncmp(p, "set ", 4)) { char name[32]; const char* eq = strchr(p + 4, '=');
            if (eq) { size_t k = (size_t)(eq - (p + 4)); if (k >= sizeof(name)) k = sizeof(name) - 1; memcpy(name, p + 4, k); name[k] = 0; trim(name);
                      if (fn_of(name) >= 0) { char v[16]; strncpy(v, eq + 1, sizeof(v) - 1); v[sizeof(v) - 1] = 0; trim(v); set_uart(name, v); return true; } } }
        return false;
    }

    // what each UART is set to serve (the settings; AUTO worked out as at start-up): the functions in the PortFn order
    void uarts(int8_t fn_of_uart[3][7]) const {
        for (int u = 0; u < 3; u++) for (int f = 0; f < 7; f++) fn_of_uart[u][f] = 0;
        const UartPlan p = plan();
        if (p.gps >= 0) fn_of_uart[p.gps][PF_GPS] = 1;
        if (p.rx >= 0) fn_of_uart[p.rx][PF_SERIAL_RX] = 1;
        if (p.flow >= 0) fn_of_uart[p.flow][PF_FLOW_MSP] = 1;
        if (p.esc >= 0) fn_of_uart[p.esc][PF_ESC_SENSOR] = 1;
        if (p.osd >= 0) fn_of_uart[p.osd][PF_DISPLAYPORT] = 1;
        if (p.tlm >= 0) fn_of_uart[p.tlm][PF_TELEMETRY] = 1;
    }

private:
    struct Name { const char* bf; uint8_t fn; const char* claim; };
    static const Name* names() {
        static const Name k[] = {{"gps_uart", PF_GPS, "gps"}, {"rx_uart", PF_SERIAL_RX, "rx"}, {"opticalflow_uart", PF_FLOW_MSP, "opticalflow"},
                                 {"esc_sensor_uart", PF_ESC_SENSOR, "esc_sensor"}, {"osd_uart", PF_DISPLAYPORT, "osd"}, {"telemetry_uart", PF_TELEMETRY, "telemetry"}, {nullptr, 0, nullptr}};
        return k;
    }
    static int fn_of(const char* bf) { for (const Name* n = names(); n->bf; n++) if (!strcmp(n->bf, bf)) return n->fn; return -1; }
    static const char* claim_of(int fn) { for (const Name* n = names(); n->bf; n++) if (n->fn == fn) return n->claim; return "?"; }

    // the devices that want a UART, and where they are (or would be, AUTO) - as FlightApp::begin() decides at start-up
    UartPlan plan() const {
        const uint8_t pr = s_.receiver.protocol;
        const bool want_rx = pr != wbp::RX_PROTOCOL_PPM && pr != wbp::RX_PROTOCOL_PWM && pr != wbp::RX_PROTOCOL_MSP && pr != wbp::RX_PROTOCOL_NRF24;
        const bool want_gps = s_.sensors.gps_enabled && (s_.gps.provider == wbp::GPS_PROVIDER_NMEA || s_.gps.provider == wbp::GPS_PROVIDER_UBLOX);
        const bool want_flow = s_.flow.type == wbp::FLOW_TYPE_MSP || pr == wbp::RX_PROTOCOL_MSP || (s_.sensors.gps_enabled && s_.gps.provider == wbp::GPS_PROVIDER_MSP)
                            || (s_.sensors.mag_enabled && s_.sensors.mag_model == wbp::MAG_MODEL_MSP) || (s_.sensors.baro_enabled && s_.sensors.baro_model == wbp::BARO_MODEL_MSP);
        const bool table = s_.ports.assign == wbp::PORT_ASSIGN_TABLE;
        uint8_t fn[3]; for (int i = 0; i < 3; i++) fn[i] = s_.ports.function[i];
        UartPlan p = plan_uarts(table, fn, want_gps, want_rx, want_flow, u0_, want_gps, want_rx);
        if (!table) {
            p.esc = s_.motors.esc_sensor ? esc_uart(want_gps, want_rx, u0_, p.flow) : -1;
            p.osd = s_.osd.type != wbp::OSD_TYPE_NONE ? telemetry_uart(want_gps, want_rx, u0_, p.flow, p.esc) : -1;
        } else {
            if (!s_.motors.esc_sensor) p.esc = -1;
            if (s_.osd.type == wbp::OSD_TYPE_NONE) p.osd = -1;
        }
        return p;
    }
    void peripherals() {
        int8_t f[3][7]; uarts(f);
        for (int u = 0; u < 3; u++) {
            char b[96]; int k = snprintf(b, sizeof(b), "serial UART%d:", u); bool any = false;
            if (u == 0 && !u0_) { k += snprintf(b + k, sizeof(b) - k, " msp*"); any = true; }        // (the app link)
            for (int fn = 1; fn < 7; fn++) {
                const bool here = use_ ? use_->uart[fn] == u : f[u][fn] != 0;          // what started (FlightApp::begin), else the settings
                if (!here) continue;
                k += snprintf(b + k, sizeof(b) - k, "%s %s%s", any ? "," : "", claim_of(fn), !use_ || use_->open[fn] ? "*" : ""); any = true;
            }
            snprintf(b + k, sizeof(b) - k, "\r\n"); say(b);
        }
    }
    void get_uart(const char* bf) {
        int8_t f[3][7]; uarts(f); const int fn = fn_of(bf); int at = -1;
        for (int u = 0; u < 3; u++) if (f[u][fn]) at = u;
        char b[64];
        if (at < 0) snprintf(b, sizeof(b), "%s = NONE\r\n", bf); else snprintf(b, sizeof(b), "%s = UART%d\r\n", bf, at);
        say(b); say(u0_ ? "Allowed values: NONE, UART0, UART1, UART2\r\n" : "Allowed values: NONE, UART1, UART2\r\n");
    }
    void set_uart(const char* bf, const char* v) {
        int to = -2;
        if (!strcmp(v, "NONE")) to = -1; else if (!strncmp(v, "UART", 4) && v[4] >= '0' && v[4] <= '2' && !v[5]) to = v[4] - '0';
        if (to == -2 || (to == 0 && !u0_)) { say("###ERROR: INVALID VALUE###\r\n"); return; }
        if (srv_.armed()) { say("###ERROR: ARMED###\r\n"); return; }
        int8_t f[3][7]; uarts(f); const int fn = fn_of(bf);
        // the table as things are now (AUTO written out), then this one moved
        uint8_t t[3] = {0, 0, 0};
        for (int u = 0; u < 3; u++) for (int k = 1; k < 7; k++) if (f[u][k]) t[u] = (uint8_t)k;
        if (s_.ports.assign == wbp::PORT_ASSIGN_TABLE) for (int u = 0; u < 3; u++) t[u] = s_.ports.function[u];      // (a table entry of a device switched off stays)
        for (int u = 0; u < 3; u++) if (t[u] == fn) t[u] = PF_NONE;
        if (to >= 0) t[to] = (uint8_t)fn;
        for (int u = 0; u < 3; u++) s_.ports.function[u] = t[u];
        s_.ports.assign = wbp::PORT_ASSIGN_TABLE;
        srv_.commit_section(wbp::SEC_PORTS);
        char b[64]; snprintf(b, sizeof(b), "%s set to %s\r\n", bf, v); say(b);
    }
    void say(const char* t) { out_(t, ctx_); }
    static void trim(char* s) { size_t n = strlen(s); while (n && s[n - 1] == ' ') s[--n] = 0; size_t i = 0; while (s[i] == ' ') i++; if (i) memmove(s, s + i, n - i + 1); }
    wbp::Settings& s_; wbp::Server& srv_; OutFn out_; void* ctx_; const PortUse* use_; bool u0_;
};

} // namespace app
