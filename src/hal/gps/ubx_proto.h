/*
 * This file is part of Windburst Project
 */

#pragma once

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "gps_types.h"

// ============================================================================
// u-blox UBX protocol: frame builder + byte-wise parser + decoders. Portable, unit-tested against a simulated module.
// Message ids, payload layouts, CFG-VALSET key ids and the per-generation (M5/M6 | M7/M8 | M9/M10) configuration split follow
// Betaflight io/gps.c.
// ============================================================================
namespace gps {
namespace ubx {

enum : uint8_t { SYNC1 = 0xB5, SYNC2 = 0x62 };
enum : uint8_t { CLS_NAV = 0x01, CLS_ACK = 0x05, CLS_CFG = 0x06, CLS_MON = 0x0A, CLS_NMEA = 0xF0 };
enum : uint8_t { NAV_POSLLH = 0x02, NAV_STATUS = 0x03, NAV_DOP = 0x04, NAV_SOL = 0x06, NAV_PVT = 0x07, NAV_VELNED = 0x12, NAV_SVINFO = 0x30, NAV_SAT = 0x35,
                 ACK_NACK = 0x00, ACK_ACK = 0x01,
                 CFG_PRT = 0x00, CFG_MSG = 0x01, CFG_RATE = 0x08, CFG_SBAS = 0x16, CFG_NAV5 = 0x24, CFG_GNSS = 0x3E, CFG_PMS = 0x86, CFG_VALSET = 0x8A,
                 MON_VER = 0x04,
                 NMEA_GGA = 0x00, NMEA_GLL = 0x01, NMEA_GSA = 0x02, NMEA_GSV = 0x03, NMEA_RMC = 0x04, NMEA_VTG = 0x05 };

// CFG-VALSET keys (gps.h). Top byte encodes the value size: 0x1x/0x2x = 1 byte, 0x3x = 2, 0x4x = 4, 0x5x = 8.
enum : uint32_t {
    K_RATE_MEAS = 0x30210001, K_RATE_NAV = 0x30210002, K_RATE_TIMEREF = 0x20210003,
    K_NAVSPG_DYNMODEL = 0x20110021, K_NAVSPG_UTCSTANDARD = 0x2011001C,
    K_MSGOUT_NMEA_GGA_UART1 = 0x209100BB, K_MSGOUT_NMEA_VTG_UART1 = 0x209100B1, K_MSGOUT_NMEA_GSV_UART1 = 0x209100C5,
    K_MSGOUT_NMEA_GLL_UART1 = 0x209100CA, K_MSGOUT_NMEA_GSA_UART1 = 0x209100C0, K_MSGOUT_NMEA_RMC_UART1 = 0x209100AC,
    K_MSGOUT_NAV_PVT_UART1 = 0x20910007, K_MSGOUT_NAV_SAT_UART1 = 0x20910016, K_MSGOUT_NAV_DOP_UART1 = 0x20910039,
    K_SBAS_USE_TESTMODE = 0x10360002, K_SBAS_USE_RANGING = 0x10360003, K_SBAS_USE_DIFFCORR = 0x10360004, K_SBAS_USE_INTEGRITY = 0x10360005,
    K_SBAS_PRNSCANMASK = 0x50360006,
    K_SIGNAL_GPS_ENA = 0x1031001F, K_SIGNAL_SBAS_ENA = 0x10310020, K_SIGNAL_GAL_ENA = 0x10310021, K_SIGNAL_BDS_ENA = 0x10310022,
    K_SIGNAL_QZSS_ENA = 0x10310024, K_SIGNAL_GLO_ENA = 0x10310025,
    K_PM_OPERATEMODE = 0x20D00001,
};
enum : uint8_t { LAYER_RAM = 1 };                        // BF writes to RAM only (not flash): the module is re-configured on every power-up

constexpr size_t kMaxFrame = 8 + 160;                    // outgoing frames are small; incoming parse buffer is separate

// -------------------------------------------------------------------------------------------------------------------
// Frame builder: header + payload + Fletcher checksum. Returns total length (or 0 if it does not fit).
// -------------------------------------------------------------------------------------------------------------------
inline void checksum(const uint8_t* p, size_t n, uint8_t& a, uint8_t& b) { a = 0; b = 0; for (size_t i = 0; i < n; i++) { a = (uint8_t)(a + p[i]); b = (uint8_t)(b + a); } }

inline size_t build(uint8_t* out, size_t cap, uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    if (cap < (size_t)len + 8) return 0;
    out[0] = SYNC1; out[1] = SYNC2; out[2] = cls; out[3] = id; out[4] = (uint8_t)len; out[5] = (uint8_t)(len >> 8);
    if (len) memcpy(out + 6, payload, len);
    checksum(out + 2, (size_t)len + 4, out[6 + len], out[7 + len]);
    return (size_t)len + 8;
}

// CFG-VALSET payload builder: version 0, layer, 2 reserved, then {key u32 LE, value} items.
struct ValSet {
    uint8_t buf[128 + 4]; size_t n = 4;
    explicit ValSet(uint8_t layer = LAYER_RAM) { memset(buf, 0, sizeof(buf)); buf[1] = layer; }
    static size_t value_len(uint32_t key) { switch ((key >> 24) & 0xFF) { case 0x10: case 0x20: return 1; case 0x30: return 2; case 0x40: return 4; case 0x50: return 8; default: return 0; } }
    bool add(uint32_t key, uint64_t value) {
        const size_t l = value_len(key);
        if (!l || n + 4 + l > sizeof(buf)) return false;
        for (int i = 0; i < 4; i++) buf[n++] = (uint8_t)(key >> (8 * i));
        for (size_t i = 0; i < l; i++) buf[n++] = (uint8_t)(value >> (8 * i));
        return true;
    }
};

// ---- the configuration messages Betaflight sends (payload only; wrap with build(CLS_CFG, id, ...)) ----
inline size_t nav5_legacy(uint8_t* p, uint8_t model, UtcStandard utc) {            // CFG-NAV5, 36 bytes (M5..M8)
    static const uint8_t utc_map[5] = {0, 3, 5, 6, 7};                            // BF ubloxUTCStandardConfig_int
    memset(p, 0, 36);
    p[0] = 0xFF; p[1] = 0xFF;                                                      // mask: apply everything
    p[2] = (uint8_t)(model == 0 ? 0 : model + 1);                                  // dynModel: "no model with value 1"
    p[3] = 3;                                                                      // fixMode auto 2D/3D
    const int32_t fixed_alt_var = 10000; memcpy(p + 8, &fixed_alt_var, 4);
    p[12] = 5;                                                                     // minElev
    const uint16_t pdop = 250, tdop = 250, pacc = 100, tacc = 300; memcpy(p + 14, &pdop, 2); memcpy(p + 16, &tdop, 2); memcpy(p + 18, &pacc, 2); memcpy(p + 20, &tacc, 2);
    p[22] = 0;                                                                     // staticHoldThresh
    p[23] = 60;                                                                    // dgnssTimeout
    const uint16_t hold_dist = 200; memcpy(p + 28, &hold_dist, 2);
    p[30] = utc_map[(int)utc > 4 ? 0 : (int)utc];
    return 36;
}
inline size_t cfg_msg_rate(uint8_t* p, uint8_t cls, uint8_t id, uint8_t rate) { p[0] = cls; p[1] = id; p[2] = rate; return 3; }
inline size_t cfg_rate_legacy(uint8_t* p, uint16_t hz) { const uint16_t ms = (uint16_t)(1000 / (hz ? hz : 1)); p[0] = (uint8_t)ms; p[1] = (uint8_t)(ms >> 8); p[2] = 1; p[3] = 0; p[4] = 1; p[5] = 0; return 6; }
inline size_t cfg_pms(uint8_t* p) { memset(p, 0, 8); return 8; }                  // M8: full power
inline size_t cfg_sbas_legacy(uint8_t* p, SbasMode m, bool integrity) {            // CFG-SBAS, 8 bytes
    memset(p, 0, 8);
    p[0] = 0x02 | (m != SBAS_NONE ? 0x01 : 0);                                     // test mode | enabled
    p[1] = 0x01 | 0x02 | (integrity ? 0x04 : 0);                                   // range | diffcorr | integrity
    p[2] = 3;                                                                      // maxSBAS
    uint32_t scan = 0;
    switch (m) { case SBAS_EGNOS: scan = 0x00010048; break; case SBAS_WAAS: scan = 0x0004A800; break; case SBAS_MSAS: scan = 0x00020200; break; case SBAS_GAGAN: scan = 0x00001180; break; default: break; }
    memcpy(p + 4, &scan, 4);
    return 8;
}
inline uint64_t sbas_prn_mask(SbasMode m) {
    auto prn_bit = [](int prn) { return (uint64_t)1 << (prn - 120); };   // not named "bit": Arduino defines a bit() macro                 // SBAS_SEARCH_PRN(n): bit (n-120)
    switch (m) { case SBAS_EGNOS: return prn_bit(123) | prn_bit(126) | prn_bit(136); case SBAS_WAAS: return prn_bit(131) | prn_bit(133) | prn_bit(135) | prn_bit(138);
                 case SBAS_MSAS: return prn_bit(129) | prn_bit(137); case SBAS_GAGAN: return prn_bit(127) | prn_bit(128) | prn_bit(132); default: return 0; }
}
inline size_t cfg_valset_nav(ValSet& v, uint8_t model, UtcStandard utc) {        // M9/M10 equivalent of NAV5
    static const uint8_t utc_map[5] = {0, 3, 5, 6, 7};
    v.add(K_NAVSPG_DYNMODEL, model == 0 ? 0 : model + 1);
    v.add(K_NAVSPG_UTCSTANDARD, utc_map[(int)utc > 4 ? 0 : (int)utc]);
    return v.n;
}
inline size_t cfg_valset_sbas(ValSet& v, SbasMode m, bool integrity) {
    v.add(K_SBAS_USE_TESTMODE, m != SBAS_NONE ? 1 : 0); v.add(K_SBAS_USE_RANGING, 1); v.add(K_SBAS_USE_DIFFCORR, 1);
    if (integrity) v.add(K_SBAS_USE_INTEGRITY, 1);
    v.add(K_SBAS_PRNSCANMASK, sbas_prn_mask(m));
    return v.n;
}
inline size_t cfg_valset_rate(ValSet& v, uint16_t hz) { v.add(K_RATE_MEAS, (uint16_t)(1000 / (hz ? hz : 1))); v.add(K_RATE_NAV, 1); v.add(K_RATE_TIMEREF, 1); return v.n; }
inline size_t cfg_valset_disable_nmea(ValSet& v) {
    v.add(K_MSGOUT_NMEA_GGA_UART1, 0); v.add(K_MSGOUT_NMEA_VTG_UART1, 0); v.add(K_MSGOUT_NMEA_GSV_UART1, 0);
    v.add(K_MSGOUT_NMEA_GLL_UART1, 0); v.add(K_MSGOUT_NMEA_GSA_UART1, 0); v.add(K_MSGOUT_NMEA_RMC_UART1, 0);
    return v.n;
}
// $PUBX,41: change the module's UART baud (and protocols) - the only way to change rate while still on an unknown one.
inline size_t pubx41(char* out, size_t cap, uint32_t baud) {
    char body[48]; int n = snprintf(body, sizeof(body), "PUBX,41,1,0003,0001,%lu,0", (unsigned long)baud);
    uint8_t x = 0; for (int i = 0; i < n; i++) x ^= (uint8_t)body[i];
    return (size_t)snprintf(out, cap, "$%s*%02X\r\n", body, x);
}

// -------------------------------------------------------------------------------------------------------------------
// Parser: byte-wise, checksum-verified. Payload kept up to kPayload bytes (rest is checksummed and dropped), like BF.
// -------------------------------------------------------------------------------------------------------------------
constexpr size_t kPayload = 8 + 12 * kMaxSats + 8;       // NAV-SAT with 40 satellites

struct Message { uint8_t cls = 0, id = 0; uint16_t len = 0; const uint8_t* payload = nullptr; };

class Parser {
public:
    // Returns true when a complete, checksum-valid frame has just been received; `msg` then points into the parser's buffer.
    bool feed(uint8_t c, Message& msg) {
        switch (st_) {
        case S_SYNC1: if (c == SYNC1) st_ = S_SYNC2; break;
        case S_SYNC2: st_ = (c == SYNC2) ? S_CLS : ((c == SYNC1) ? S_SYNC2 : S_SYNC1); ca_ = cb_ = 0; break;
        case S_CLS: cls_ = c; ck(c); st_ = S_ID; break;
        case S_ID: id_ = c; ck(c); st_ = S_LEN1; break;
        case S_LEN1: len_ = c; ck(c); st_ = S_LEN2; break;
        case S_LEN2: len_ |= (uint16_t)(c << 8); ck(c); n_ = 0;
            if (len_ > 2048) { errors_++; st_ = S_SYNC1; } else st_ = len_ ? S_PAY : S_CKA; break;
        case S_PAY: ck(c); if (n_ < kPayload) buf_[n_] = c; if (++n_ >= len_) st_ = S_CKA; break;
        case S_CKA: ck_a_ = c; st_ = S_CKB; break;
        case S_CKB: st_ = S_SYNC1;
            if (ck_a_ == ca_ && c == cb_) { msg.cls = cls_; msg.id = id_; msg.len = len_ < kPayload ? len_ : (uint16_t)kPayload; msg.payload = buf_; frames_++; return true; }
            errors_++; break;
        }
        return false;
    }
    uint32_t frames() const { return frames_; }
    uint32_t errors() const { return errors_; }
    void reset() { st_ = S_SYNC1; }
private:
    void ck(uint8_t c) { ca_ = (uint8_t)(ca_ + c); cb_ = (uint8_t)(cb_ + ca_); }
    enum S : uint8_t { S_SYNC1, S_SYNC2, S_CLS, S_ID, S_LEN1, S_LEN2, S_PAY, S_CKA, S_CKB } st_ = S_SYNC1;
    uint8_t cls_ = 0, id_ = 0, ca_ = 0, cb_ = 0, ck_a_ = 0; uint16_t len_ = 0; size_t n_ = 0; uint32_t frames_ = 0, errors_ = 0;
    uint8_t buf_[kPayload];
};

// -------------------------------------------------------------------------------------------------------------------
// Decoders
// -------------------------------------------------------------------------------------------------------------------
inline uint32_t u32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
inline int32_t i32(const uint8_t* p) { return (int32_t)u32(p); }
inline uint16_t u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline int16_t i16(const uint8_t* p) { return (int16_t)u16(p); }

inline uint16_t sat16(int64_t v) { return v < 0 ? 0 : (v > 65535 ? 65535 : (uint16_t)v); }
inline int16_t sat_i16(int64_t v) { return v < -32768 ? -32768 : (v > 32767 ? 32767 : (int16_t)v); }

// NAV-PVT (92 bytes, M7+). Returns true; fills sol. A frame is a complete position+velocity update by itself.
inline bool decode_pvt(const Message& m, Solution& s) {
    if (m.len < 92) return false;
    const uint8_t* p = m.payload;
    s.time_ms = u32(p);
    s.date.year = u16(p + 4); s.date.month = p[6]; s.date.day = p[7]; s.date.hour = p[8]; s.date.minute = p[9]; s.date.second = p[10];
    s.date.valid = (p[11] & 0x03) == 0x03;                          // validDate | validTime
    s.fix_type = p[20];
    const bool fix_ok = (p[21] & 0x01) && s.fix_type == 3;           // gnssFixOK && 3D
    s.fix = fix_ok;
    s.num_sat = p[23];
    s.lon = i32(p + 24); s.lat = i32(p + 28);
    s.alt_cm = i32(p + 36) / 10;                                     // hMSL mm -> cm
    s.h_acc_mm = u32(p + 40); s.v_acc_mm = u32(p + 44);
    const int32_t velN = i32(p + 48), velE = i32(p + 52), velD = i32(p + 56), gspeed = i32(p + 60), head = i32(p + 64);
    s.s_acc_mm = u32(p + 68); s.head_acc = u32(p + 72);
    s.pdop = u16(p + 76); s.hdop = s.pdop; s.vdop = s.pdop;          // NAV-PVT has pDOP only; BF uses it for all three (conservative)
    s.vel_n_cms = sat_i16(velN / 10); s.vel_e_cms = sat_i16(velE / 10); s.vel_d_cms = sat_i16(velD / 10); s.has_vel_d = true;
    s.ground_speed_cms = sat16(gspeed / 10);
    const double g = (double)gspeed, d = (double)velD;
    double sp = __builtin_sqrt(g * g + d * d) * 0.1; s.speed3d_cms = sat16((int64_t)sp);
    int32_t hd = head / 10000; if (hd < 0) hd += 3600; s.course_ddeg = (uint16_t)(hd % 3600);
    return true;
}

// Legacy (M5/M6) split messages. Betaflight emits a solution once POSLLH (position) AND VELNED (speed) have both arrived.
inline void decode_posllh(const Message& m, Solution& s) {
    if (m.len < 28) return;
    const uint8_t* p = m.payload;
    s.time_ms = u32(p); s.lon = i32(p + 4); s.lat = i32(p + 8); s.alt_cm = i32(p + 16) / 10; s.h_acc_mm = u32(p + 20); s.v_acc_mm = u32(p + 24);
}
inline void decode_sol(const Message& m, Solution& s) {
    if (m.len < 52) return;
    const uint8_t* p = m.payload;
    s.fix_type = p[10]; s.fix = (p[11] & 0x01) && p[10] == 3; s.num_sat = p[47]; s.pdop = u16(p + 44);
    if (!s.hdop) { s.hdop = s.pdop; s.vdop = s.pdop; }
}
inline void decode_status(const Message& m, Solution& s) {
    if (m.len < 16) return;
    s.fix_type = m.payload[4]; s.fix = (m.payload[5] & 0x01) && m.payload[4] == 3;
}
inline void decode_dop(const Message& m, Solution& s) { if (m.len >= 18) { s.pdop = u16(m.payload + 6); s.vdop = u16(m.payload + 10); s.hdop = u16(m.payload + 12); } }
inline void decode_velned(const Message& m, Solution& s) {
    if (m.len < 36) return;
    const uint8_t* p = m.payload;
    s.vel_n_cms = sat_i16(i32(p + 4)); s.vel_e_cms = sat_i16(i32(p + 8)); s.vel_d_cms = sat_i16(i32(p + 12)); s.has_vel_d = true;
    s.speed3d_cms = sat16(u32(p + 16)); s.ground_speed_cms = sat16(u32(p + 20));
    int32_t hd = i32(p + 24) / 10000; if (hd < 0) hd += 3600; s.course_ddeg = (uint16_t)(hd % 3600);
    s.s_acc_mm = u32(p + 28) * 10; s.head_acc = u32(p + 32);
}
// NAV-SVINFO (M7 and older): header 8 bytes then 12 per channel {chn, svid, flags, quality, cno, elev, azim i16, prRes i32}.
inline int decode_svinfo(const Message& m, SatInfo* out, int cap) {
    if (m.len < 8) return 0;
    int n = m.payload[4]; const int avail = (m.len - 8) / 12; if (n > avail) n = avail; if (n > cap) n = cap;
    for (int i = 0; i < n; i++) {
        const uint8_t* q = m.payload + 8 + 12 * i;
        out[i].gnss_id = 0; out[i].sv_id = q[1]; out[i].flags = q[3]; out[i].cno = q[4]; out[i].elev = (int8_t)q[5]; out[i].azim = i16(q + 6);
    }
    return n;
}
// NAV-SAT (M8+): header 8 bytes then 12 per satellite.
inline int decode_sat(const Message& m, SatInfo* out, int cap) {
    if (m.len < 8) return 0;
    int n = m.payload[5]; const int avail = (m.len - 8) / 12; if (n > avail) n = avail; if (n > cap) n = cap;
    for (int i = 0; i < n; i++) {
        const uint8_t* q = m.payload + 8 + 12 * i;
        out[i].gnss_id = q[0]; out[i].sv_id = q[1]; out[i].cno = q[2]; out[i].elev = (int8_t)q[3]; out[i].azim = i16(q + 4); out[i].flags = u32(q + 8);
    }
    return n;
}
// MON-VER: swVersion[30], hwVersion[10] (ASCII hex-ish string such as "00080000"). Mapped like Betaflight's ubloxVersionMap.
inline UbxVersion decode_hw_version(const Message& m) {
    if (m.len < 40) return UBX_UNDEF;
    char hw[11]; memcpy(hw, m.payload + 30, 10); hw[10] = 0;
    const unsigned long v = strtoul(hw, nullptr, 16);
    switch (v) { case 0x00040005: return UBX_M5; case 0x00040007: return UBX_M6; case 0x00070000: return UBX_M7; case 0x00080000: return UBX_M8; case 0x00190000: return UBX_M9; case 0x000A0000: return UBX_M10; default: return UBX_UNDEF; }
}
inline const char* version_name(UbxVersion v) { static const char* n[] = {"UNKNOWN", "M5", "M6", "M7", "M8", "M9", "M10"}; return n[v > 6 ? 0 : v]; }

} // namespace ubx
} // namespace gps
