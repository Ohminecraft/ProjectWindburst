// The MSP port (hal/msp_sensors.h) on a PC: frames built here from the senders' own layout (INAV msp_protocol_v2_sensor_msg.h,
// Betaflight MSP_SET_RAW_RC), not with the driver's code, fed through the byte stream like a UART.
// Build: g++ -std=c++17 -O2 test/test_msp_sensors.cpp -o build/test_msp_sensors
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <deque>
#include <vector>
#include "../src/hal/msp_sensors.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, ...) do { const bool ok_ = (cond); if (getenv("V")) { printf("    "); printf(__VA_ARGS__); printf("\n"); } if (ok_) g_pass++; else { g_fail++; printf("  FAIL line %d: %s   ", __LINE__, #cond); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint8_t crc_ref(const std::vector<uint8_t>& f, size_t from) {
    uint8_t c = 0;
    for (size_t i = from; i < f.size(); i++) { c ^= f[i]; for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
    return c;
}
static std::vector<uint8_t> v2(uint16_t cmd, const std::vector<uint8_t>& p) {
    std::vector<uint8_t> f = {'$', 'X', '<', 0, (uint8_t)cmd, (uint8_t)(cmd >> 8), (uint8_t)p.size(), (uint8_t)(p.size() >> 8)};
    f.insert(f.end(), p.begin(), p.end()); f.push_back(crc_ref(f, 3)); return f;
}
static std::vector<uint8_t> v1(uint8_t cmd, const std::vector<uint8_t>& p) {
    std::vector<uint8_t> f = {'$', 'M', '<', (uint8_t)p.size(), cmd}; uint8_t x = (uint8_t)(p.size() ^ cmd);
    for (uint8_t b : p) { f.push_back(b); x ^= b; }
    f.push_back(x); return f;
}
template <class T> static void put(std::vector<uint8_t>& p, T v) { uint8_t b[sizeof(T)]; memcpy(b, &v, sizeof(T)); p.insert(p.end(), b, b + sizeof(T)); }

struct Wire : flow::BytePort {
    std::deque<uint8_t> q;
    int read() override { if (q.empty()) return -1; const uint8_t c = q.front(); q.pop_front(); return c; }
    void send(const std::vector<uint8_t>& f) { q.insert(q.end(), f.begin(), f.end()); }
};

// mspSensorGpsDataMessage_t
static std::vector<uint8_t> gps_payload(uint8_t fix, uint8_t sats, int32_t lat, int32_t lon, int32_t alt_cm, int32_t vn, int32_t ve, int32_t vd, uint16_t course_cdeg) {
    std::vector<uint8_t> p; put<uint8_t>(p, 0); put<uint16_t>(p, 2300); put<uint32_t>(p, 123456789u); put<uint8_t>(p, fix); put<uint8_t>(p, sats);
    put<uint16_t>(p, 150); put<uint16_t>(p, 250); put<uint16_t>(p, 30); put<uint16_t>(p, 120);
    put<int32_t>(p, lon); put<int32_t>(p, lat); put<int32_t>(p, alt_cm); put<int32_t>(p, vn); put<int32_t>(p, ve); put<int32_t>(p, vd);
    put<uint16_t>(p, course_cdeg); put<uint16_t>(p, 65535); put<uint16_t>(p, 2026); put<uint8_t>(p, 10); put<uint8_t>(p, 23); put<uint8_t>(p, 14); put<uint8_t>(p, 5); put<uint8_t>(p, 9);
    return p;
}

struct FlowSink { int n = 0; uint16_t last = 0; static bool fn(void* c, uint16_t cmd, const uint8_t*, uint16_t, uint32_t) { auto* s = (FlowSink*)c; s->n++; s->last = cmd; return true; } };

int main() {
    printf("[msp port]\n");
    {   // GPS
        Wire w; mspdev::Port port; port.attach(&w);
        const auto p = gps_payload(3, 14, 107654321, 1066543210, 12345, 300, -400, 50, 31250);
        CHECK(p.size() == 52, "the GPS message is 52 bytes (%zu)", p.size());
        w.send(v2(0x1F03, p)); port.poll(1000, 1);
        gps::Solution s; const bool got = port.take_gps(s);
        CHECK(got && s.fix && s.fix_type == 3 && s.num_sat == 14 && s.lat == 107654321 && s.lon == 1066543210 && s.alt_cm == 12345,
              "a 3D fix: %d sats, %ld / %ld, %ld cm", s.num_sat, (long)s.lat, (long)s.lon, (long)s.alt_cm);
        CHECK(s.vel_n_cms == 300 && s.vel_e_cms == -400 && s.vel_d_cms == 50 && s.has_vel_d && s.ground_speed_cms == 500 && s.course_ddeg == 3125 && s.hdop == 120,
              "velocity NED %d / %d / %d cm/s, ground speed %u, course %u ddeg", s.vel_n_cms, s.vel_e_cms, s.vel_d_cms, s.ground_speed_cms, s.course_ddeg);
        CHECK(s.h_acc_mm == 1500 && s.v_acc_mm == 2500 && s.s_acc_mm == 300 && s.time_ms == 123456789u && s.date.valid && s.date.year == 2026 && s.date.hour == 14,
              "accuracies in mm, time of week, date");
        CHECK(!port.take_gps(s), "taken once");
        w.send(v2(0x1F03, gps_payload(2, 5, 1, 2, 3, 0, 0, 0, 0))); port.poll(2000, 2); port.take_gps(s);
        CHECK(!s.fix && s.fix_type == 2, "fix type 2: 2D, not a fix to fly on");
        w.send(v2(0x1F03, gps_payload(1, 3, 1, 2, 3, 0, 0, 0, 0))); port.poll(3000, 3); port.take_gps(s);
        CHECK(!s.fix && s.fix_type == 0, "fix type 1: none");
        w.send(v2(0x1F03, gps_payload(6, 3, 1, 2, 3, 0, 0, 0, 0))); port.poll(4000, 4); port.take_gps(s);
        CHECK(s.fix && s.fix_type == 3, "fix type above 3 (RTK): 3D");
        auto short_one = gps_payload(3, 9, 1, 2, 3, 0, 0, 0, 0); short_one.resize(40); w.send(v2(0x1F03, short_one)); port.poll(5000, 5);
        CHECK(!port.take_gps(s) && port.unused() == 1, "a GPS message cut short: not used");
        w.send(v2(0x1F03, gps_payload(3, 9, 1, 2, 3, 40000, 0, 0, 36000))); port.poll(6000, 6); port.take_gps(s);
        CHECK(s.vel_n_cms == 32767 && s.course_ddeg == 0, "a speed past 327 m/s is clamped; course 360.00 is 0");
    }
    {   // compass and barometer
        Wire w; mspdev::Port port; port.attach(&w);
        std::vector<uint8_t> m; put<uint8_t>(m, 0); put<uint32_t>(m, 1000); put<int16_t>(m, 250); put<int16_t>(m, -120); put<int16_t>(m, 410);
        std::vector<uint8_t> b; put<uint8_t>(b, 0); put<uint32_t>(b, 1000); put<float>(b, 101325.0f); put<int16_t>(b, 2537);
        w.send(v2(0x1F04, m)); w.send(v2(0x1F05, b)); port.poll(1000, 1);
        mspdev::MagSample ms; mspdev::BaroSample bs;
        CHECK(port.take_mag(ms) && fabsf(ms.frd_ga[0] - 0.25f) < 1e-6f && fabsf(ms.frd_ga[1] + 0.12f) < 1e-6f && fabsf(ms.frd_ga[2] - 0.41f) < 1e-6f,
              "compass: milligauss, front / right / down -> %.3f %.3f %.3f G", ms.frd_ga[0], ms.frd_ga[1], ms.frd_ga[2]);
        CHECK(port.take_baro(bs) && bs.pa == 101325.0f && fabsf(bs.temp_c - 25.37f) < 1e-4f, "barometer: %.0f Pa, %.2f C", bs.pa, bs.temp_c);
        CHECK(!port.take_mag(ms) && !port.take_baro(bs), "each taken once");
        std::vector<uint8_t> bad; put<uint8_t>(bad, 0); put<uint32_t>(bad, 1000); put<float>(bad, NAN); put<int16_t>(bad, 0);
        w.send(v2(0x1F05, bad)); port.poll(2000, 2);
        CHECK(!port.take_baro(bs), "a pressure that is not a number: dropped");
    }
    {   // MSP_SET_RAW_RC, v1 and v2
        Wire w; mspdev::Port port; port.attach(&w);
        CHECK(!port.rc().ok(0), "no frame yet: no link");
        std::vector<uint8_t> p; for (uint16_t v : {1500, 1600, 1000, 1400, 2000, 1000, 1700, 1300}) put<uint16_t>(p, v);
        w.send(v1(200, p)); port.poll(1000, 10);
        CHECK(port.take_rc() && port.rc().ok(10) && port.rc().channels()[1] == 1600 && port.rc().channels()[4] == 2000, "v1: 8 channels, link up");
        CHECK(port.rc().channels()[8] == 1000 && !port.take_rc(), "channels it does not send keep their default; taken once");
        std::vector<uint8_t> q; for (int i = 0; i < 18; i++) put<uint16_t>(q, (uint16_t)(1100 + i * 10));
        w.send(v2(200, q)); port.poll(2000, 20);
        CHECK(port.take_rc() && port.rc().channels()[15] == 1250, "v2: 18 channels, the first 16 kept (%u)", port.rc().channels()[15]);
        std::vector<uint8_t> r; put<uint16_t>(r, 50); put<uint16_t>(r, 4000); w.send(v1(200, r)); port.poll(3000, 30);
        CHECK(port.rc().channels()[0] == 750 && port.rc().channels()[1] == 2250 && port.rc().channels()[2] == 1120, "values far out are clamped; channels it leaves out keep the last ones");
        CHECK(port.rc().ok(30 + 300) && !port.rc().ok(30 + 301), "no frame for more than 300 ms: the link is lost (failsafe)");
    }
    {   // flow / lidar go to the flow sensor; garbage and damage
        Wire w; mspdev::Port port; port.attach(&w); FlowSink fs;
        std::vector<uint8_t> fl(9, 0), rg(5, 0);
        w.send(v2(0x1F02, fl)); port.poll(1000, 1);
        CHECK(port.unused() == 1 && fs.n == 0, "no flow sensor on this port: its frames are unused");
        port.setFlow(&FlowSink::fn, &fs);
        w.send(v2(0x1F02, fl)); w.send(v2(0x1F01, rg)); port.poll(2000, 2);
        CHECK(fs.n == 2 && fs.last == 0x1F01 && port.flow_frames() == 2, "flow and range frames go to the flow sensor");
        auto f = v2(0x1F04, std::vector<uint8_t>(11, 0)); f.back() ^= 0x20; w.send({0x00, 0xFF, 'x'}); w.send(f); port.poll(3000, 3);
        mspdev::MagSample ms;
        CHECK(port.crc_errors() == 1 && !port.take_mag(ms), "a damaged frame after noise: counted, not used");
        w.send(v2(0x1F06, std::vector<uint8_t>(9, 0))); port.poll(4000, 4);
        CHECK(port.unused() == 2, "airspeed (0x1F06): a sensor this firmware has no use for");
        for (int i = 0; i < 100; i++) w.send(v2(0x1F04, std::vector<uint8_t>(11, 0)));
        const size_t before = w.q.size(); port.poll(5000, 5);
        CHECK(before - w.q.size() == (size_t)mspdev::Port::kMaxBytesPerCall, "one poll reads %d bytes at most", mspdev::Port::kMaxBytesPerCall);
    }
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
