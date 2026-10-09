// Simulated u-blox module behind a serial link for test_gps.cpp. Independent of src/hal/gps: own UBX/NMEA encoders and a strict
// per-generation command set (M9/M10 answer legacy CFG-* with NACK, M5-M8 answer CFG-VALSET with NACK, M5/M6 have no NAV-PVT / NAV-SAT ...),
// so a driver that sends the wrong dialect for the detected module cannot configure it.
#pragma once
#include <deque>
#include <vector>
#include <string>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "../src/hal/gps/gps_engine.h"

struct SimTruth {
    double lat = 47.3977419, lon = 8.5455938; double alt_m = 488.2;      // degrees, MSL
    double vn = 0.0, ve = 0.0, vd = 0.0;                                 // m/s
    int sats = 14; bool fix3d = false; uint32_t fix_after_ms = 0; bool fix_ok_flag = true;   // fix_ok_flag=false: fixType says 3D but gnssFixOK is 0
    double hacc_m = 1.2, vacc_m = 1.8, sacc_mps = 0.12, pdop = 1.3;
};

struct CfgRecord { uint8_t cls, id; std::vector<uint8_t> p; };

class SimGps : public gps::Port {
public:
    gps::UbxVersion version = gps::UBX_M8;
    uint32_t module_baud = 9600, engine_baud = 57600;
    SimTruth truth;
    bool responds = true;                                                 // false: dead / unplugged module
    bool silent_acks = false;                                             // true: module executes commands but never ACKs
    bool mute_after_config = false;
    // module configuration state (what the driver actually achieved)
    bool nmea[6] = {true, true, true, true, true, true};                 // GGA GLL GSA GSV RMC VTG
    bool pvt = false, posllh = false, velned = false, sol = false, status = false, dop = false;
    int sat_rate = 0; bool navsat = false, svinfo = false; uint32_t rate_ms = 1000;
    uint8_t dynmodel = 0; bool sbas_enabled = true; uint32_t sbas_scan_lo = 0; uint64_t sbas_mask = 0; bool gal = false, sbas_gnss = true;
    uint8_t powermode = 1; uint8_t utc = 0; bool nav5_seen = false, rate_seen = false; int nacks = 0, acks = 0; uint32_t baud_changes = 0;
    std::vector<CfgRecord> log; std::vector<std::string> nmea_in;       // everything the module received
    uint32_t mon_ver_seen = 0;

    // ---------------- Port (engine side)
    int read() override { if (rx_.empty()) return -1; uint8_t c = rx_.front(); rx_.pop_front(); return c; }
    void write(const uint8_t* d, size_t n) override { if (!responds || engine_baud != module_baud) return; for (size_t i = 0; i < n; i++) module_byte(d[i]); }
    void set_baud(uint32_t b) override { engine_baud = b; }

    // ---------------- time
    void tick(uint32_t now_ms) {
        now_ = now_ms;
        if (!responds) return;
        if (truth.fix_after_ms && now_ms >= truth.fix_after_ms) truth.fix3d = true;
        if (now_ms >= next_epoch_) { next_epoch_ = now_ms + rate_ms; epoch(); }
    }
    uint32_t epochs = 0;

private:
    // ---------------- encoders
    void out(const std::vector<uint8_t>& b) { if (engine_baud == module_baud) for (uint8_t c : b) rx_.push_back(c); else for (size_t i = 0; i < b.size(); i++) rx_.push_back((uint8_t)(0x80 | (i * 37 + now_))); }
    static void le(std::vector<uint8_t>& v, uint64_t x, int n) { for (int i = 0; i < n; i++) v.push_back((uint8_t)(x >> (8 * i))); }
    void ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t>& p) {
        std::vector<uint8_t> f = {0xB5, 0x62, cls, id, (uint8_t)p.size(), (uint8_t)(p.size() >> 8)};
        f.insert(f.end(), p.begin(), p.end());
        uint8_t a = 0, b = 0; for (size_t i = 2; i < f.size(); i++) { a = (uint8_t)(a + f[i]); b = (uint8_t)(b + a); }
        f.push_back(a); f.push_back(b); out(f);
    }
    void nmea_line(const std::string& body) { uint8_t x = 0; for (char c : body) x ^= (uint8_t)c; char t[8]; snprintf(t, sizeof(t), "*%02X\r\n", x); std::string s = "$" + body + t; out(std::vector<uint8_t>(s.begin(), s.end())); }
    int32_t e7(double deg) const { return (int32_t)llround(deg * 1e7); }
    uint8_t fixtype() const { return truth.fix3d ? 3 : 0; }

    void send_pvt() {
        std::vector<uint8_t> p; le(p, now_, 4); le(p, 2026, 2); p.push_back(10); p.push_back(1); p.push_back(12); p.push_back(30); p.push_back(15); p.push_back(truth.fix3d ? 0x07 : 0x00);
        le(p, 50, 4); le(p, 0, 4); p.push_back(fixtype()); p.push_back((truth.fix3d && truth.fix_ok_flag) ? 1 : 0); p.push_back(0x20); p.push_back((uint8_t)truth.sats);
        le(p, (uint32_t)e7(truth.lon), 4); le(p, (uint32_t)e7(truth.lat), 4); le(p, (uint32_t)llround((truth.alt_m + 47.0) * 1000), 4); le(p, (uint32_t)llround(truth.alt_m * 1000), 4);
        le(p, (uint32_t)llround(truth.hacc_m * 1000), 4); le(p, (uint32_t)llround(truth.vacc_m * 1000), 4);
        le(p, (uint32_t)llround(truth.vn * 1000), 4); le(p, (uint32_t)llround(truth.ve * 1000), 4); le(p, (uint32_t)llround(truth.vd * 1000), 4);
        const double gs = sqrt(truth.vn * truth.vn + truth.ve * truth.ve); le(p, (uint32_t)llround(gs * 1000), 4);
        double hd = atan2(truth.ve, truth.vn) * 180.0 / M_PI; if (hd < 0) hd += 360; le(p, (uint32_t)llround(hd * 1e5), 4);
        le(p, (uint32_t)llround(truth.sacc_mps * 1000), 4); le(p, 2000000, 4); le(p, (uint32_t)llround(truth.pdop * 100), 2);
        while (p.size() < 92) p.push_back(0);
        ubx(0x01, 0x07, p);
    }
    void send_legacy() {
        if (posllh) { std::vector<uint8_t> p; le(p, now_, 4); le(p, (uint32_t)e7(truth.lon), 4); le(p, (uint32_t)e7(truth.lat), 4); le(p, (uint32_t)llround((truth.alt_m + 47) * 1000), 4); le(p, (uint32_t)llround(truth.alt_m * 1000), 4); le(p, 1500, 4); le(p, 2200, 4); ubx(0x01, 0x02, p); }
        if (velned) { std::vector<uint8_t> p; le(p, now_, 4); le(p, (uint32_t)llround(truth.vn * 100), 4); le(p, (uint32_t)llround(truth.ve * 100), 4); le(p, (uint32_t)llround(truth.vd * 100), 4);
            const double gs = sqrt(truth.vn * truth.vn + truth.ve * truth.ve); le(p, (uint32_t)llround(sqrt(gs * gs + truth.vd * truth.vd) * 100), 4); le(p, (uint32_t)llround(gs * 100), 4);
            double hd = atan2(truth.ve, truth.vn) * 180.0 / M_PI; if (hd < 0) hd += 360; le(p, (uint32_t)llround(hd * 1e5), 4); le(p, 15, 4); le(p, 200000, 4); ubx(0x01, 0x12, p); }
        if (sol) { std::vector<uint8_t> p(52, 0); p[10] = fixtype(); p[11] = truth.fix3d ? 1 : 0; p[47] = (uint8_t)truth.sats; const uint16_t pd = (uint16_t)llround(truth.pdop * 100); p[44] = (uint8_t)pd; p[45] = (uint8_t)(pd >> 8); ubx(0x01, 0x06, p); }
        if (status) { std::vector<uint8_t> p(16, 0); p[4] = fixtype(); p[5] = truth.fix3d ? 1 : 0; ubx(0x01, 0x03, p); }
        if (dop) { std::vector<uint8_t> p(18, 0); const uint16_t pd = (uint16_t)llround(truth.pdop * 100), hd = (uint16_t)(pd * 0.8), vd = (uint16_t)(pd * 0.9); memcpy(&p[6], &pd, 2); memcpy(&p[10], &vd, 2); memcpy(&p[12], &hd, 2); ubx(0x01, 0x04, p); }
    }
    void send_sats() {
        const int n = 6;
        if (navsat) { std::vector<uint8_t> p; le(p, now_, 4); p.push_back(1); p.push_back((uint8_t)n); p.push_back(0); p.push_back(0);
            for (int i = 0; i < n; i++) { p.push_back((uint8_t)(i % 2 ? 0 : 2)); p.push_back((uint8_t)(10 + i)); p.push_back((uint8_t)(30 + i)); p.push_back((uint8_t)(20 + i)); le(p, (uint16_t)(40 * i), 2); le(p, 0, 2); le(p, 0x1D, 4); } ubx(0x01, 0x35, p); }
        if (svinfo) { std::vector<uint8_t> p; le(p, now_, 4); p.push_back((uint8_t)n); p.push_back(0); le(p, 0, 2);
            for (int i = 0; i < n; i++) { p.push_back((uint8_t)i); p.push_back((uint8_t)(10 + i)); p.push_back(0x0D); p.push_back(7); p.push_back((uint8_t)(30 + i)); p.push_back((uint8_t)(20 + i)); le(p, (uint16_t)(40 * i), 2); le(p, 0, 4); } ubx(0x01, 0x30, p); }
    }
    static std::string coord(double v, bool lat) { const double a = fabs(v); const int d = (int)a; const double m = (a - d) * 60.0; char b[24]; snprintf(b, sizeof(b), lat ? "%02d%07.4f" : "%03d%07.4f", d, m); return b; }
    void send_nmea() {
        char b[160]; const double gs = sqrt(truth.vn * truth.vn + truth.ve * truth.ve);
        if (nmea[0]) { snprintf(b, sizeof(b), "GNGGA,123015.00,%s,%c,%s,%c,%d,%02d,0.9,%.1f,M,47.0,M,,", coord(truth.lat, true).c_str(), truth.lat >= 0 ? 'N' : 'S', coord(truth.lon, false).c_str(), truth.lon >= 0 ? 'E' : 'W', truth.fix3d ? 1 : 0, truth.sats, truth.alt_m); nmea_line(b); }
        if (nmea[2]) { snprintf(b, sizeof(b), "GNGSA,A,%d,01,02,03,04,05,06,07,08,,,,,%.1f,%.1f,%.1f", truth.fix3d ? 3 : 1, truth.pdop, truth.pdop * 0.8, truth.pdop * 0.9); nmea_line(b); }
        if (nmea[4]) { double hd = atan2(truth.ve, truth.vn) * 180.0 / M_PI; if (hd < 0) hd += 360; snprintf(b, sizeof(b), "GNRMC,123015.00,%c,%s,%c,%s,%c,%.2f,%.1f,121026,,,A", truth.fix3d ? 'A' : 'V', coord(truth.lat, true).c_str(), truth.lat >= 0 ? 'N' : 'S', coord(truth.lon, false).c_str(), truth.lon >= 0 ? 'E' : 'W', gs / 0.514444, hd); nmea_line(b); }
        if (nmea[3]) { nmea_line("GPGSV,2,1,07,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45"); nmea_line("GPGSV,2,2,07,15,31,150,40,17,56,201,42,19,10,040,38"); }
    }
    void epoch() {
        epochs++;
        if (mute_after_config && !nmea[0] && !pvt && !posllh) return;
        send_nmea();
        if (pvt) send_pvt();
        send_legacy();
        if (sat_rate > 0 && (epochs % sat_rate) == 0) send_sats();
    }
    void ack(bool ok, uint8_t cls, uint8_t id) {
        ok ? acks++ : nacks++;
        if (silent_acks) return;
        ubx(0x05, ok ? 0x01 : 0x00, {cls, id});
    }

    // ---------------- module-side parsing
    enum { W_SYNC1, W_SYNC2, W_CLS, W_ID, W_L1, W_L2, W_PAY, W_CKA, W_CKB } w_ = W_SYNC1;
    uint8_t cls_ = 0, id_ = 0, ca_ = 0, cb_ = 0, cka_ = 0; uint16_t len_ = 0; std::vector<uint8_t> pay_; std::string line_; bool in_line_ = false;
    void ck(uint8_t c) { ca_ = (uint8_t)(ca_ + c); cb_ = (uint8_t)(cb_ + ca_); }
    void module_byte(uint8_t c) {
        if (w_ == W_SYNC1 && c == '$') { in_line_ = true; line_.clear(); return; }
        if (in_line_) { if (c == '\n') { in_line_ = false; handle_nmea(line_); } else if (c != '\r') line_.push_back((char)c); return; }
        switch (w_) {
        case W_SYNC1: if (c == 0xB5) w_ = W_SYNC2; break;
        case W_SYNC2: w_ = (c == 0x62) ? W_CLS : W_SYNC1; ca_ = cb_ = 0; break;
        case W_CLS: cls_ = c; ck(c); w_ = W_ID; break;
        case W_ID: id_ = c; ck(c); w_ = W_L1; break;
        case W_L1: len_ = c; ck(c); w_ = W_L2; break;
        case W_L2: len_ |= (uint16_t)(c << 8); ck(c); pay_.clear(); w_ = len_ ? W_PAY : W_CKA; break;
        case W_PAY: ck(c); pay_.push_back(c); if (pay_.size() >= len_) w_ = W_CKA; break;
        case W_CKA: cka_ = c; w_ = W_CKB; break;
        case W_CKB: w_ = W_SYNC1; if (cka_ == ca_ && c == cb_) handle_ubx(); break;
        }
    }
    void handle_nmea(const std::string& s) {
        nmea_in.push_back(s);
        unsigned long b; unsigned a, b2, c2; if (sscanf(s.c_str(), "PUBX,41,%u,%x,%x,%lu", &a, &b2, &c2, &b) == 4) { module_baud = (uint32_t)b; baud_changes++; }
    }
    void handle_ubx() {
        using namespace gps;
        log.push_back({cls_, id_, pay_});
        const bool m9 = version >= UBX_M9, m8 = version >= UBX_M8, m7 = version >= UBX_M7;
        if (cls_ == 0x0A && id_ == 0x04) {                                    // MON-VER
            mon_ver_seen++;
            std::vector<uint8_t> p(30 + 10 + 30, 0); snprintf((char*)&p[0], 30, "ROM CORE 3.01"); static const char* hw[] = {"", "00040005", "00040007", "00070000", "00080000", "00190000", "000A0000"};
            memcpy(&p[30], hw[version], 8); ubx(0x0A, 0x04, p); return; }
        if (cls_ != 0x06) return;
        bool ok = true;
        switch (id_) {
        case 0x01: {                                                          // CFG-MSG
            if (m9 || pay_.size() != 3) { ok = false; break; }
            const uint8_t c = pay_[0], i = pay_[1], r = pay_[2];
            if (c == 0xF0 && i <= 5) nmea[i] = r != 0;
            else if (c == 0x01 && i == 0x07) { if (!m7) ok = false; else pvt = r != 0; }
            else if (c == 0x01 && i == 0x02) posllh = r != 0;
            else if (c == 0x01 && i == 0x03) status = r != 0;
            else if (c == 0x01 && i == 0x04) dop = r != 0;
            else if (c == 0x01 && i == 0x06) sol = r != 0;
            else if (c == 0x01 && i == 0x12) velned = r != 0;
            else if (c == 0x01 && i == 0x30) { if (m8) ok = false; else { svinfo = r != 0; sat_rate = r; } }
            else if (c == 0x01 && i == 0x35) { if (!m8) ok = false; else { navsat = r != 0; sat_rate = r; } }
            else ok = false;
            break; }
        case 0x08: if (m9 || pay_.size() != 6) ok = false; else { rate_ms = pay_[0] | (pay_[1] << 8); rate_seen = true; } break;                    // CFG-RATE
        case 0x24: if (m9 || pay_.size() != 36) ok = false; else { dynmodel = pay_[2]; utc = pay_[30]; nav5_seen = true; } break;                          // CFG-NAV5
        case 0x16: if (m9 || pay_.size() != 8) ok = false; else { sbas_enabled = pay_[0] & 1; memcpy(&sbas_scan_lo, &pay_[4], 4); } break;                 // CFG-SBAS
        case 0x86: if (!m8 || m9 || pay_.size() != 8) ok = false; else powermode = pay_[1]; break;                                                        // CFG-PMS
        case 0x3E:                                                            // CFG-GNSS
            if (!m7 || m9) { ok = false; break; }
            if (pay_.empty()) {                                              // poll -> answer with SBAS + Galileo + GPS blocks, then ACK
                std::vector<uint8_t> p = {0, 32, 32, 3}; auto blk = [&](uint8_t g, bool en) { p.push_back(g); p.push_back(4); p.push_back(16); p.push_back(0); le(p, en ? 0x01010001 : 0x01000000, 4); };
                blk(0, true); blk(1, sbas_gnss); blk(2, gal); ubx(0x06, 0x3E, p); ack(true, 0x06, 0x3E); return;
            } else { const uint8_t n = pay_[3]; for (int k = 0; k < n && 4 + 8 * k + 8 <= (int)pay_.size(); k++) { const uint8_t g = pay_[4 + 8 * k]; const bool en = pay_[8 + 8 * k] & 1; if (g == 1) sbas_gnss = en; if (g == 2) gal = en; } }
            break;
        case 0x8A: {                                                          // CFG-VALSET
            if (!m9 || pay_.size() < 8) { ok = false; break; }
            size_t i = 4;
            while (i + 4 <= pay_.size()) {
                const uint32_t key = pay_[i] | (pay_[i + 1] << 8) | (pay_[i + 2] << 16) | ((uint32_t)pay_[i + 3] << 24); i += 4;
                const int sz = ((key >> 24) & 0xFF) == 0x10 || ((key >> 24) & 0xFF) == 0x20 ? 1 : (((key >> 24) & 0xFF) == 0x30 ? 2 : (((key >> 24) & 0xFF) == 0x40 ? 4 : 8));
                if (i + sz > pay_.size()) { ok = false; break; }
                uint64_t v = 0; for (int k = 0; k < sz; k++) v |= (uint64_t)pay_[i + k] << (8 * k); i += sz;
                switch (key) {
                case 0x30210001: rate_ms = (uint32_t)v; rate_seen = true; break;
                case 0x20110021: dynmodel = (uint8_t)v; nav5_seen = true; break;
                case 0x2011001C: utc = (uint8_t)v; break;
                case 0x209100BB: nmea[0] = v; break; case 0x209100B1: nmea[5] = v; break; case 0x209100C5: nmea[3] = v; break;
                case 0x209100CA: nmea[1] = v; break; case 0x209100C0: nmea[2] = v; break; case 0x209100AC: nmea[4] = v; break;
                case 0x20910007: pvt = v; break; case 0x20910016: navsat = v; sat_rate = (int)v; break; case 0x20910039: dop = v; break;
                case 0x10360002: sbas_enabled = v; break; case 0x50360006: sbas_mask = v; break;
                case 0x10310020: sbas_gnss = v; break; case 0x10310021: gal = v; break;
                case 0x20D00001: powermode = (uint8_t)v; break;
                default: break; }
            }
            break; }
        default: ok = false; break;
        }
        ack(ok, cls_, id_);
    }
    std::deque<uint8_t> rx_; uint32_t now_ = 0, next_epoch_ = 0;
};
