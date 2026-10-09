// A reader for blackbox logs used by the tests. It knows the format ONLY from the text of the log header (field names, types,
// scales), like the app's reader (app/wbclient/wblog.py) - it shares no code with the firmware's packing.
#pragma once
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "../src/proto/wbp_codec.h"
#include "../src/blackbox/bb_format.h"

// ------------------------------------------------------------------------------------------------------------------------------
// An independent reader: frames -> sessions -> records, driven by the header text only.
// ------------------------------------------------------------------------------------------------------------------------------
struct RField { std::string name, type, unit; double scale; int size; int offset; };
struct RSession {
    std::string text; std::map<std::string, std::string> meta; std::vector<RField> fast, slow; int fast_size = 0, slow_size = 0;
    std::vector<std::vector<double>> F, S;                 // decoded rows, column 0 = t (raw microseconds)
    struct Ev { uint32_t t; int code; std::vector<uint8_t> data; };
    std::vector<Ev> events; int data_frames = 0, seq_gaps = 0, bad_records = 0; bool header_done = false;
    int col(const std::vector<RField>& f, const std::string& n) const { for (size_t i = 0; i < f.size(); i++) if (f[i].name == n) return (int)i; return -1; }
    double fv(size_t row, const std::string& n) const { const int c = col(fast, n); return c < 0 ? NAN : F[row][(size_t)c]; }
    double sv(size_t row, const std::string& n) const { const int c = col(slow, n); return c < 0 ? NAN : S[row][(size_t)c]; }
    int count_events(int code) const { int n = 0; for (const Ev& e : events) n += e.code == code; return n; }
};
static std::vector<RField> parse_fields(const std::string& line, int& size) {
    std::vector<RField> out; std::istringstream is(line); std::string tok; is >> tok; size = 0;
    while (is >> tok) {
        RField f; size_t a = tok.find(':'), b = tok.find(':', a + 1), c = tok.find(':', b + 1);
        f.name = tok.substr(0, a); f.type = tok.substr(a + 1, b - a - 1); f.scale = atof(tok.substr(b + 1, c - b - 1).c_str()); f.unit = tok.substr(c + 1);
        f.size = f.type == "u8" ? 1 : (f.type == "i16" || f.type == "u16") ? 2 : 4; f.offset = size; size += f.size;
        out.push_back(f);
    }
    return out;
}
static double rd(const uint8_t* p, const RField& f) {
    if (f.type == "u8") return p[0] * f.scale;
    if (f.type == "i16") { int16_t v; memcpy(&v, p, 2); return v * f.scale; }
    if (f.type == "u16") { uint16_t v; memcpy(&v, p, 2); return v * f.scale; }
    if (f.type == "i32") { int32_t v; memcpy(&v, p, 4); return v * f.scale; }
    uint32_t v; memcpy(&v, p, 4); return f.name == "t" ? (double)v : v * f.scale;
}
static void finish_header(RSession& s) {
    std::istringstream is(s.text); std::string line;
    while (std::getline(is, line)) {
        const size_t sp = line.find(' ');
        const std::string key = line.substr(0, sp), val = sp == std::string::npos ? "" : line.substr(sp + 1);
        if (key == "fast") s.fast = parse_fields(line, s.fast_size);
        else if (key == "slow") s.slow = parse_fields(line, s.slow_size);
        else if (key == "set") s.meta["set." + val.substr(0, val.find(' '))] = val.substr(val.find(' ') + 1);
        else s.meta[key] = val;
    }
    s.header_done = true;
}
// Reads a byte stream (a flash region, a download, a serial capture) into sessions.
static std::vector<RSession> read_log(const uint8_t* data, size_t n) {
    std::vector<RSession> out; wbp::Parser p; wbp::Frame f; int expect_seq = -1;
    auto on_frame = [&]() {
        if (f.type == bb::kMsgHeader) {
            if (f.seq == 0) { out.emplace_back(); expect_seq = -1; }
            if (!out.empty()) out.back().text.append((const char*)f.payload, f.len);
        } else if (f.type == bb::kMsgData && !out.empty()) {
            RSession& s = out.back();
            if (!s.header_done) finish_header(s);
            if (expect_seq >= 0 && f.seq != (uint8_t)expect_seq) s.seq_gaps++;
            expect_seq = (f.seq + 1) & 0xFF; s.data_frames++;
            size_t i = 0;
            while (i < f.len) {
                const uint8_t k = f.payload[i++];
                if (k == 'F' && i + (size_t)s.fast_size <= f.len) { std::vector<double> row; for (const RField& fd : s.fast) row.push_back(rd(f.payload + i + fd.offset, fd)); s.F.push_back(row); i += (size_t)s.fast_size; }
                else if (k == 'S' && i + (size_t)s.slow_size <= f.len) { std::vector<double> row; for (const RField& fd : s.slow) row.push_back(rd(f.payload + i + fd.offset, fd)); s.S.push_back(row); i += (size_t)s.slow_size; }
                else if (k == 'E' && i + 6 <= f.len && i + 6 + f.payload[i + 5] <= f.len) { RSession::Ev e; memcpy(&e.t, f.payload + i, 4); e.code = f.payload[i + 4]; const uint8_t dn = f.payload[i + 5]; e.data.assign(f.payload + i + 6, f.payload + i + 6 + dn); s.events.push_back(e); i += 6u + dn; }
                else { s.bad_records++; break; }
            }
        }
    };
    for (size_t i = 0; i < n; i++) if (p.feed(data[i], f)) { do on_frame(); while (p.pump(f)); }
    for (RSession& s : out) if (!s.header_done) finish_header(s);
    return out;
}

