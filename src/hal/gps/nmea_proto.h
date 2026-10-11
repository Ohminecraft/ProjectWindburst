/*
 * This file is part of Windburst Project
 */

#pragma once

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "gps_types.h"

// ============================================================================
// NMEA 0183 parser (Betaflight "NMEA" provider): GGA (position, fix quality, sats, hdop, altitude), RMC (speed, course, date/time, validity),
// GSA (fix type, p/h/vdop), GSV (satellites in view). Checksum is verified before anything is applied. Any talker id (GP/GN/GL/GA/BD) works.
// Portable and unit-tested. A solution update is reported once per GGA after the matching RMC arrived (BF reports on GGA).
// ============================================================================
namespace gps {
namespace nmea {

inline uint8_t hexval(char c) { return (c >= 'A') ? (uint8_t)(c - 'A' + 10) : (uint8_t)(c >= 'a' ? c - 'a' + 10 : c - '0'); }

// "ddmm.mmmm" / "dddmm.mmmm" -> 1e-7 degrees (BF GPS_coord_to_degrees keeps minute fraction digits exactly)
inline int32_t coord(const char* s, bool negative) {
    if (!*s) return 0;
    const char* dot = strchr(s, '.');
    const int int_len = dot ? (int)(dot - s) : (int)strlen(s);
    if (int_len < 3) return 0;
    const int deg_digits = int_len - 2;
    int32_t deg = 0; for (int i = 0; i < deg_digits; i++) deg = deg * 10 + (s[i] - '0');
    int64_t min1e7 = 0;                                       // minutes * 1e7
    int64_t whole = (s[deg_digits] - '0') * 10 + (s[deg_digits + 1] - '0');
    min1e7 = whole * 10000000LL;
    if (dot) { int64_t scale = 1000000; for (const char* q = dot + 1; *q && scale > 0; q++, scale /= 10) min1e7 += (int64_t)(*q - '0') * scale; }
    const int64_t v = (int64_t)deg * 10000000LL + min1e7 / 60;
    return (int32_t)(negative ? -v : v);
}
inline int32_t fixed(const char* s, int mult) {               // "123.45" -> 12345 for mult 100 (no float)
    bool neg = false; if (*s == '-') { neg = true; s++; }
    int64_t i = 0; while (*s >= '0' && *s <= '9') i = i * 10 + (*s++ - '0');
    int64_t f = 0, scale = mult;
    if (*s == '.') { s++; while (*s >= '0' && *s <= '9' && scale > 1) { scale /= 10; f += (*s++ - '0') * scale; } }
    int64_t v = i * mult + f; return (int32_t)(neg ? -v : v);
}

class Parser {
public:
    Solution sol;                      // accumulated solution
    SatInfo sats[kMaxSats]; int num_sats_in_view = 0;
    uint32_t frames = 0, errors = 0;

    // Returns true when a new position solution was completed (a valid GGA).
    bool feed(char c) {
        if (c == '$') { len_ = 0; started_ = true; return false; }
        if (!started_) return false;
        if (c == '\r' || c == '\n') { started_ = false; return len_ ? sentence() : false; }
        if (len_ < sizeof(buf_) - 1) buf_[len_++] = c; else { started_ = false; errors++; }
        return false;
    }
private:
    bool sentence() {
        buf_[len_] = 0;
        char* star = strchr(buf_, '*');
        if (!star || star[1] == 0 || star[2] == 0) { errors++; return false; }
        uint8_t x = 0; for (char* p = buf_; p < star; p++) x ^= (uint8_t)*p;
        if (x != (uint8_t)((hexval(star[1]) << 4) | hexval(star[2]))) { errors++; return false; }
        *star = 0;
        char* f[24]; int nf = 0; char* p = buf_;
        f[nf++] = p;
        for (; *p && nf < 24; p++) if (*p == ',') { *p = 0; f[nf++] = p + 1; }
        if (strlen(f[0]) != 5) return false;
        const char* t = f[0] + 2; frames++;
        if (!strcmp(t, "GGA")) return gga(f, nf);
        if (!strcmp(t, "RMC")) { rmc(f, nf); return false; }
        if (!strcmp(t, "GSA")) { gsa(f, nf); return false; }
        if (!strcmp(t, "GSV")) { gsv(f, nf); return false; }
        return false;
    }
    bool gga(char** f, int nf) {
        if (nf < 10) return false;
        const int quality = atoi(f[6]);
        sol.fix = quality >= 1;                                // 1 GPS, 2 DGPS, 4/5 RTK ...
        sol.num_sat = (uint8_t)atoi(f[7]);
        if (sol.fix) {
            sol.lat = coord(f[2], f[3][0] == 'S'); sol.lon = coord(f[4], f[5][0] == 'W');
            sol.alt_cm = fixed(f[9], 100);
            if (f[8][0]) sol.hdop = (uint16_t)fixed(f[8], 100);
            if (sol.fix_type < 3) sol.fix_type = 3;
        } else sol.fix_type = 0;
        return sol.fix;
    }
    void rmc(char** f, int nf) {
        if (nf < 10) return;
        if (f[2][0] == 'V') { return; }                          // void: no valid navigation data
        sol.ground_speed_cms = (uint16_t)((int64_t)fixed(f[7], 100) * 5144 / 10000);     // knots -> cm/s (1 kn = 51.444 cm/s)
        if (f[8][0]) { int32_t c = fixed(f[8], 10); if (c < 0) c += 3600; sol.course_ddeg = (uint16_t)(c % 3600); }
        // north / east velocity from the speed and the course over the ground (UBX gives them directly; the horizontal estimator
        // and Position Hold need them). Below 0.5 knot the course means nothing, but the velocity is about 0 then anyway.
        { const float r = sol.course_ddeg * (3.14159265f / 1800.0f), v = (float)sol.ground_speed_cms;
          sol.vel_n_cms = (int16_t)lroundf(v * cosf(r)); sol.vel_e_cms = (int16_t)lroundf(v * sinf(r)); sol.vel_d_cms = 0; sol.has_vel_d = false; }
        const char* tm = f[1]; const char* dt = f[9];
        if (strlen(tm) >= 6 && strlen(dt) >= 6) {
            sol.date.hour = (uint8_t)((tm[0] - '0') * 10 + tm[1] - '0'); sol.date.minute = (uint8_t)((tm[2] - '0') * 10 + tm[3] - '0'); sol.date.second = (uint8_t)((tm[4] - '0') * 10 + tm[5] - '0');
            sol.date.day = (uint8_t)((dt[0] - '0') * 10 + dt[1] - '0'); sol.date.month = (uint8_t)((dt[2] - '0') * 10 + dt[3] - '0'); sol.date.year = (uint16_t)(2000 + (dt[4] - '0') * 10 + dt[5] - '0');
            sol.date.valid = true;
        }
    }
    void gsa(char** f, int nf) {
        if (nf < 18) return;
        const int ft = atoi(f[2]); if (ft >= 1 && ft <= 3) sol.fix_type = (uint8_t)(ft == 1 ? 0 : ft);
        if (f[15][0]) sol.pdop = (uint16_t)fixed(f[15], 100);
        if (f[16][0]) sol.hdop = (uint16_t)fixed(f[16], 100);
        if (f[17][0]) sol.vdop = (uint16_t)fixed(f[17], 100);
    }
    void gsv(char** f, int nf) {
        if (nf < 4) return;
        const int total = atoi(f[1]), msg = atoi(f[2]); num_sats_in_view = atoi(f[3]);
        const int base = (msg - 1) * 4; if (msg == 1) for (auto& s : sats) s = SatInfo();
        for (int k = 0; k < 4 && 4 + 4 * k + 3 < nf + 0 + 1; k++) {
            const int at = base + k; if (at >= kMaxSats || 4 + 4 * k + 3 >= nf) break;
            sats[at].sv_id = (uint8_t)atoi(f[4 + 4 * k]); sats[at].elev = (int8_t)atoi(f[5 + 4 * k]); sats[at].azim = (int16_t)atoi(f[6 + 4 * k]); sats[at].cno = (uint8_t)atoi(f[7 + 4 * k]); sats[at].gnss_id = 0;
        }
        (void)total;
    }
    char buf_[120]; size_t len_ = 0; bool started_ = false;
};

} // namespace nmea
} // namespace gps
