// wb_sim - the Windburst RX firmware running on a PC, for developing and testing the app without hardware.
//
// It is the REAL firmware body (FlightApp, drivers, receiver decoder, flight core, settings store, WBP server) on a virtual board:
// simulated ICM-42688-P, BMP280, QMC5883L, u-blox GPS, CRSF receiver and a quadcopter that really flies (see test/virtual_board.h).
// Two optical flow sensors are fitted as well (the settings choose which one is used: flow.type): a MicoAir MTF-01 on UART0
// (the app talks to this board over USB, so UART0 is free) and a PMW3901 on the second SPI bus (without a lidar).
//
//   build:  make sim            ->  build/wb_sim
//   run:    build/wb_sim [--port 5760] [--flash sim_flash.bin] [--fresh] [--speed 1.0] [--no-baro] [--no-mag] [--no-imu] [--no-flow] [--stm32] [--logmem KB]
//           --logmem: size of the blackbox flash partition (default 1024 kB, 0 = a board without one); its contents are kept in <flash>.log
//   app:    connect to   socket://127.0.0.1:5760      (python -m wbclient --port socket://127.0.0.1:5760 info)
//
// A second TCP port (port + 1) is the "hand of the tester": one text command per line, answered with "ok" / "err ...":
//   rc <channel 1..16> <us>      move a stick / switch of the simulated transmitter      tx on|off      switch the transmitter
//   rclink crsf|spektrum2048|spektrum1024|ppm|pwm   the receiver on the board (PPM on GPIO40; PWM on GPIO39, 40, 41, 42, 1, 3, 45, 46)
//   pose <roll> <nose_down> <yaw>  put the craft on the ground in this attitude (degrees)   shake 0|1      move it by hand
//   mount <roll> <pitch>         the board sits crooked in the frame (for level trim)    accerr ox oy oz sx sy sz   accelerometer offset / scale error
//   magerr hx hy hz              compass hard-iron offset (gauss)                        batt <volts>   resting pack voltage (0 = USB power only)
//   state                        one line: armed, height, attitude, sim time, motors, battery, buzzer (buzz = sounding now, buzz_ms = total),
//                                tones = beacon tones the ESCs received (the motors beeping), sd_files / sd_writes = the SD card
//   vib <axis 1..3> <Hz> <dps>   a vibration on one gyro axis      weak <motor> <0.3..1>   a motor that gives less thrust
//   esc <motor|0 = all> <0|1>    an ESC stops / resumes answering (bidirectional DShot)
//   adc <gpio 1..20> <mV>        the voltage at an ADC pin (a current sensor's output)  escamps <motor> <A>   the current that ESC reports (extended DShot telemetry)
//   swap <motor> <motor>         two ESC plugs in each other's place (the settings do not know): what runaway take-off prevention is for
//   logmem garbage               fill the start of the blackbox partition with foreign data (a board that had a file system there)
//   sdcard pull|push             take the SD card out / push it in          sdcard slow <n>   every n-th card write takes 180 ms (0 = never)
//   sdcard put <number> <bytes>  a file wb_<number>.wblog of that size that is already on the card (the card lives in RAM: empty at every start)
//   carry <z> <vn> <ve>          somebody holds the craft level, z m above the floor, and walks at vn / ve m/s (north / east)     carry off   puts it down
//   wind <n> <e>                 the air moves at this speed (m/s towards north / east)
//   tumble <wx> <wy> <wz>        somebody holds the craft up and turns it at these body rates (rad/s); tumble off puts it down level
//   linkpause <s>                the link stops sending its streams for s (simulated) seconds but still answers requests
//   flow quality <0..255>        what the MSP flow sensor reports about the surface      flow dark 0|1   the PMW3901 sees nothing
//   flow plug 0|1                pull / push the plug of the MSP sensor                  flow mount <deg> <mirror 0|1> <scale>   how the sensor sits on the craft
//   gps fix 0|1                  the receiver loses / has its fix                        gps dead 0|1    the module stops talking
//   quit                         stop the simulator
//
// --fresh starts with empty flash (no pins: like a brand-new board). Otherwise the board's pins and a default mode setup
// (ARM = AUX1 high, ANGLE = AUX2 high, ALTHOLD = AUX3 high) are stored on first start, like a user would do once.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <chrono>
#include <thread>
#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef SOCKET sock_t;
  static void sock_init() { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); }
  static void sock_close(sock_t s) { closesocket(s); }
  static void sock_nonblock(sock_t s) { u_long on = 1; ioctlsocket(s, FIONBIO, &on); }
  static bool sock_would_block() { return WSAGetLastError() == WSAEWOULDBLOCK; }
  #define BAD_SOCK INVALID_SOCKET
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <signal.h>
  typedef int sock_t;
  static void sock_init() { signal(SIGPIPE, SIG_IGN); }
  static void sock_close(sock_t s) { close(s); }
  static void sock_nonblock(sock_t s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
  static bool sock_would_block() { return errno == EAGAIN || errno == EWOULDBLOCK; }
  #define BAD_SOCK (-1)
#endif
#include "../test/fw_rig.h"

static sock_t listen_on(int port) {
    sock_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == BAD_SOCK) return BAD_SOCK;
    int yes = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes));
    sockaddr_in a; memset(&a, 0, sizeof(a)); a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   // local machine only
    if (bind(s, (sockaddr*)&a, sizeof(a)) != 0 || listen(s, 1) != 0) { sock_close(s); return BAD_SOCK; }
    sock_nonblock(s);
    return s;
}
static sock_t accept_on(sock_t l) {
    sock_t c = accept(l, nullptr, nullptr);
    if (c == BAD_SOCK) return BAD_SOCK;
    sock_nonblock(c); int yes = 1; setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&yes, sizeof(yes));
    return c;
}
// >0 bytes, 0 nothing right now, -1 closed
static int recv_some(sock_t c, uint8_t* buf, int cap) {
    const int n = (int)recv(c, (char*)buf, cap, 0);
    if (n > 0) return n;
    if (n == 0) return -1;
    return sock_would_block() ? 0 : -1;
}
static bool send_all(sock_t c, const uint8_t* d, size_t n) {
    size_t off = 0; int spins = 0;
    while (off < n) {
        const int k = (int)send(c, (const char*)d + off, (int)(n - off), 0);
        if (k > 0) { off += (size_t)k; spins = 0; continue; }
        if (k < 0 && sock_would_block() && ++spins < 2000) { std::this_thread::sleep_for(std::chrono::microseconds(200)); continue; }
        return false;
    }
    return true;
}

// ---- flash image in a file, so settings survive a restart of the simulator like they survive a power cycle
static void flash_load(MemStorage& f, const std::string& path) {
    FILE* fp = fopen(path.c_str(), "rb"); if (!fp) return;
    for (;;) {
        uint8_t kl; uint32_t n; if (fread(&kl, 1, 1, fp) != 1) break;
        std::string k(kl, '\0'); if (fread(&k[0], 1, kl, fp) != kl || fread(&n, 4, 1, fp) != 1 || n > 4096) break;
        std::vector<uint8_t> v(n); if (n && fread(v.data(), 1, n, fp) != n) break;
        f.kv[k] = v;
    }
    fclose(fp);
}
static void flash_save(const MemStorage& f, const std::string& path) {
    FILE* fp = fopen(path.c_str(), "wb"); if (!fp) return;
    for (const auto& kv : f.kv) { const uint8_t kl = (uint8_t)kv.first.size(); const uint32_t n = (uint32_t)kv.second.size(); fwrite(&kl, 1, 1, fp); fwrite(kv.first.data(), 1, kl, fp); fwrite(&n, 4, 1, fp); if (n) fwrite(kv.second.data(), 1, n, fp); }
    fclose(fp);
}

// ---- the blackbox partition in a file next to it
static void logmem_load(MemFlash& m, const std::string& path) {
    FILE* fp = fopen(path.c_str(), "rb"); if (!fp) return;
    const size_t n = fread(m.mem.data(), 1, m.mem.size(), fp); (void)n;
    fclose(fp);
}
static void logmem_save(const MemFlash& m, const std::string& path) {
    FILE* fp = fopen(path.c_str(), "wb"); if (!fp) return;
    fwrite(m.mem.data(), 1, m.mem.size(), fp); fclose(fp);
}

static std::string control(Rig& r, const std::string& line, bool& quit) {
    char cmd[32] = ""; double a = 0, b = 0, c = 0; char word[16] = "";
    if (sscanf(line.c_str(), "%31s", cmd) != 1) return "err empty";
    if (!strcmp(cmd, "rc") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 1 && a <= 16 && b >= 800 && b <= 2200) { r.board.rc[(int)a - 1] = (uint16_t)b; return "ok"; }
    if (!strcmp(cmd, "tx") && sscanf(line.c_str(), "%*s %15s", word) == 1) { r.board.tx_on = !strcmp(word, "on"); return "ok"; }
    if (!strcmp(cmd, "rclink") && sscanf(line.c_str(), "%*s %15s", word) == 1) {          // what the receiver on the board speaks
        static const char* const k[5] = {"crsf", "spektrum2048", "spektrum1024", "ppm", "pwm"};
        for (int i = 0; i < 5; i++) if (!strcmp(word, k[i])) { r.board.rc_link = (VirtualBoard::RcLink)i; return "ok"; }
        return "err rclink crsf|spektrum2048|spektrum1024|ppm|pwm";
    }
    if (!strcmp(cmd, "pose") && sscanf(line.c_str(), "%*s %lf %lf %lf", &a, &b, &c) == 3) { r.board.plant.set_tilt((float)a, (float)b, (float)c); r.board.plant.w[0] = r.board.plant.w[1] = r.board.plant.w[2] = 0; r.board.plant.z = 0; r.board.plant.vz = 0; r.board.plant.ground = true; return "ok"; }
    if (!strcmp(cmd, "shake") && sscanf(line.c_str(), "%*s %lf", &a) == 1) { r.board.shaking = a != 0; return "ok"; }
    if (!strcmp(cmd, "mount") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2) { r.board.mount_roll_deg = (float)a; r.board.mount_pitch_deg = (float)b; return "ok"; }
    { double o[3], k[3]; if (!strcmp(cmd, "accerr") && sscanf(line.c_str(), "%*s %lf %lf %lf %lf %lf %lf", &o[0], &o[1], &o[2], &k[0], &k[1], &k[2]) == 6) { for (int i = 0; i < 3; i++) { r.board.acc_offset[i] = (float)o[i]; r.board.acc_scale[i] = (float)k[i]; } return "ok"; } }
    { double h[3]; if (!strcmp(cmd, "magerr") && sscanf(line.c_str(), "%*s %lf %lf %lf", &h[0], &h[1], &h[2]) == 3) { for (int i = 0; i < 3; i++) r.board.mag_hard[i] = h[i]; return "ok"; } }
    if (!strcmp(cmd, "logmem") && sscanf(line.c_str(), "%*s %15s", word) == 1 && !strcmp(word, "garbage")) { r.logmem.fill_garbage(0, 20000); r.logmem.writes++; return "ok"; }
    if (!strcmp(cmd, "batt") && sscanf(line.c_str(), "%*s %lf", &a) == 1 && a >= 0 && a <= 60) { r.board.batt_v = a; return "ok"; }
    if (!strcmp(cmd, "adc") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 1 && a <= 20 && a != VirtualBoard::kBattAdc && b >= 0 && b <= 3300) { g_stub_adc_mv[(int)a] = (uint32_t)b; return "ok"; }   // e.g. a current sensor's output
    if (!strcmp(cmd, "escamps") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 1 && a <= 8 && b >= 0 && b <= 255) { r.board.esc_amps[(int)a - 1] = (float)b; return "ok"; }   // extended DShot telemetry
    // a vibration on one gyro axis (1 roll, 2 pitch, 3 yaw); a motor that gives less thrust; an ESC whose telemetry wire is broken (0 = all of them)
    if (!strcmp(cmd, "vib") && sscanf(line.c_str(), "%*s %lf %lf %lf", &a, &b, &c) == 3 && a >= 1 && a <= 3 && b >= 0 && b <= 2000 && c >= 0 && c <= 500) { r.board.vib_hz[(int)a - 1] = (float)b; r.board.vib_dps[(int)a - 1] = (float)c; return "ok"; }
    if (!strcmp(cmd, "weak") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 1 && a <= 8 && b >= 0.3 && b <= 1.0) { r.board.plant.gain[(int)a - 1] = (float)b; return "ok"; }
    // two ESC plugs in each other's place (a wiring mistake the settings do not know about)
    if (!strcmp(cmd, "swap") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 1 && a <= r.board.plant.n && b >= 1 && b <= r.board.plant.n && a != b) { r.board.plant.motor_swap((int)a - 1, (int)b - 1); return "ok"; }
    if (!strcmp(cmd, "esc") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && a >= 0 && a <= 8) { if (a == 0) r.board.esc_replies = b != 0; else r.board.esc_silent[(int)a - 1] = b == 0; return "ok"; }
    if (!strcmp(cmd, "sdcard") && sscanf(line.c_str(), "%*s %15s", word) == 1) {
        if (!strcmp(word, "pull")) { r.sdcard.pull(); return "ok"; }
        if (!strcmp(word, "push")) { r.sdcard.push(); return "ok"; }
        if (!strcmp(word, "slow") && sscanf(line.c_str(), "%*s %*s %lf", &a) == 1 && a >= 0) { r.sdcard.stall_every = (int)a; return "ok"; }
        if (!strcmp(word, "put") && sscanf(line.c_str(), "%*s %*s %lf %lf", &a, &b) == 2 && a >= 1 && b >= 0 && b <= 8e6) { r.sdcard.put((uint32_t)a, (size_t)b); return "ok"; }
        return "err sdcard pull|push|slow <n>|put <number> <bytes>";
    }
    if (!strcmp(cmd, "carry")) {
        Plant& p = r.board.plant;
        if (sscanf(line.c_str(), "%*s %lf %lf %lf", &a, &b, &c) == 3 && a >= 0.05 && a <= 50 && fabs(b) <= 20 && fabs(c) <= 20) { p.held = true; p.set_tilt(0, 0, p.yaw_deg()); p.w[0] = p.w[1] = p.w[2] = 0; p.z = (float)a; p.hold_vn = (float)b; p.hold_ve = (float)c; return "ok"; }
        if (sscanf(line.c_str(), "%*s %15s", word) == 1 && !strcmp(word, "off")) { const float yaw = p.yaw_deg(); p.held = false; p.hold_vn = p.hold_ve = 0; p.vn = p.ve = 0; r.board.rest(0, 0); p.set_tilt(0, 0, yaw); return "ok"; }
        return "err carry <z> <vn> <ve> | carry off";
    }
    if (!strcmp(cmd, "tumble")) {
        Plant& p = r.board.plant;
        if (sscanf(line.c_str(), "%*s %lf %lf %lf", &a, &b, &c) == 3 && fabs(a) <= 10 && fabs(b) <= 10 && fabs(c) <= 10) { p.held = true; p.hold_vn = p.hold_ve = 0; p.z = 0.3f; p.w[0] = (float)a; p.w[1] = (float)b; p.w[2] = (float)c; return "ok"; }
        if (sscanf(line.c_str(), "%*s %15s", word) == 1 && !strcmp(word, "off")) { p.held = false; r.board.rest(0, 0); return "ok"; }
        return "err tumble <wx> <wy> <wz> | tumble off";
    }
    if (!strcmp(cmd, "wind") && sscanf(line.c_str(), "%*s %lf %lf", &a, &b) == 2 && fabs(a) <= 30 && fabs(b) <= 30) { r.board.plant.wind_n = (float)a; r.board.plant.wind_e = (float)b; return "ok"; }
    if (!strcmp(cmd, "flow") && sscanf(line.c_str(), "%*s %15s", word) == 1) {
        if (!strcmp(word, "quality") && sscanf(line.c_str(), "%*s %*s %lf", &a) == 1 && a >= 0 && a <= 255) { r.board.mtf.quality = (uint8_t)a; return "ok"; }
        if (!strcmp(word, "dark") && sscanf(line.c_str(), "%*s %*s %lf", &a) == 1) { r.board.pmw.shutter_upper = a != 0 ? 0x1F : 0x05; return "ok"; }
        if (!strcmp(word, "plug") && sscanf(line.c_str(), "%*s %*s %lf", &a) == 1) { r.board.mtf.present = a != 0; return "ok"; }
        if (!strcmp(word, "mount") && sscanf(line.c_str(), "%*s %*s %lf %lf %lf", &a, &b, &c) == 3 && c >= 0.2 && c <= 5) { r.board.flow_truth.mount_cw_deg = (float)a; r.board.flow_truth.mirrored = b != 0; r.board.mtf.scale = (float)c; r.board.pmw.scale = (float)c; return "ok"; }
        return "err flow quality <0..255> | dark 0|1 | plug 0|1 | mount <deg> <mirror> <scale>";
    }
    if (!strcmp(cmd, "gps") && sscanf(line.c_str(), "%*s %15s %lf", word, &a) == 2) {
        if (!strcmp(word, "fix")) { r.board.gps.truth.fix3d = a != 0; r.board.gps.truth.fix_after_ms = 0; return "ok"; }
        if (!strcmp(word, "dead")) { r.board.gps.responds = a == 0; return "ok"; }
        return "err gps fix 0|1 | gps dead 0|1";
    }
    if (!strcmp(cmd, "state")) {
        char out[600]; const app::Telemetry& t = r.tel(); int tones = 0; for (const VirtualBoard::EscCmd& e : r.board.esc_cmds) if (e.cmd >= 1 && e.cmd <= 5) tones++;          // m0..m3: what the virtual ESCs really received (decoded from the firmware's DShot frames)
        snprintf(out, sizeof(out), "ok armed=%d z=%.2f roll=%.1f pitch=%.1f yaw=%.1f t_ms=%u boots=%d m0=%.3f m1=%.3f m2=%.3f m3=%.3f batt=%.2f buzz=%d buzz_ms=%u stall_ms=%u flash_writes=%u bad_writes=%u tones=%d sd_files=%u sd_writes=%d pn=%.2f pe=%.2f vn=%.2f ve=%.2f", t.armed ? 1 : 0, (double)r.board.plant.z, (double)t.roll, (double)t.pitch, (double)t.yaw, (unsigned)millis(), r.boots,
                 (double)r.board.motor_frac[0], (double)r.board.motor_frac[1], (double)r.board.motor_frac[2], (double)r.board.motor_frac[3],
                 r.board.batt_v, r.board.buzzer_on() ? 1 : 0, (unsigned)r.board.buzz_ms(0, millis()), (unsigned)(r.board.stalled_us / 1000u), (unsigned)r.logmem.writes, (unsigned)r.logmem.bad_writes, tones, (unsigned)r.sdcard.files.size(), r.sdcard.writes,
                 (double)r.board.plant.pn, (double)r.board.plant.pe, (double)r.board.plant.vn, (double)r.board.plant.ve);
        return out;
    }
    if (!strcmp(cmd, "linkpause") && sscanf(line.c_str(), "%*s %lf", &a) == 1 && a >= 0 && a <= 30) { r.streams_off_until_ms = millis() + (uint32_t)(a * 1000.0); return "ok"; }   // streams stop, requests are still answered
    if (!strcmp(cmd, "quit")) { quit = true; return "ok"; }
    return "err unknown command";
}

int main(int argc, char** argv) {
    int port = 5760; std::string flash_path = "sim_flash.bin"; bool fresh = false; double speed = 1.0; bool no_baro = false, no_mag = false, no_imu = false, no_flow = false, stm32 = false; int logmem_kb = 1024;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        if (a == "--port" && i + 1 < argc) port = atoi(argv[++i]);
        else if (a == "--flash" && i + 1 < argc) flash_path = argv[++i];
        else if (a == "--fresh") fresh = true;
        else if (a == "--speed" && i + 1 < argc) speed = atof(argv[++i]);
        else if (a == "--no-baro") no_baro = true;
        else if (a == "--no-mag") no_mag = true;
        else if (a == "--no-imu") no_imu = true;
        else if (a == "--no-flow") no_flow = true;
        else if (a == "--stm32") stm32 = true;      // says it is an STM32 board (FEAT_STM32: the app names the pins PA0 .. PK15); the virtual board stays the same
        else if (a == "--logmem" && i + 1 < argc) logmem_kb = atoi(argv[++i]);
        else { fprintf(stderr, "usage: wb_sim [--port N] [--flash FILE] [--fresh] [--speed X] [--no-baro] [--no-mag] [--no-imu] [--no-flow] [--stm32] [--logmem KB]\n"); return 2; }
    }
    sock_init();
    const sock_t l_app = listen_on(port), l_ctl = listen_on(port + 1);
    if (l_app == BAD_SOCK || l_ctl == BAD_SOCK) { fprintf(stderr, "cannot listen on 127.0.0.1:%d / %d\n", port, port + 1); return 1; }

    g_stub_us = 0; g_stub_delay_advances = true;
    Rig rig;
    ESP.psram_size = 8192u * 1024u; ESP.psram_free = ESP.psram_size - 512u * 1024u;      // the simulated module is an N16R8: 8 MB of PSRAM, 512 kB of it the SD card's write buffer
    rig.board.has_baro = !no_baro; rig.board.has_mag = !no_mag; rig.board.has_imu = !no_imu;
    rig.usb_link = true;                          // the app is on the chip's USB socket: UART0 is free for a sensor
    rig.board.mtf.present = !no_flow; rig.board.has_pmw = !no_flow;
    rig.board.plant.ground_holds = true;          // a craft on the bench: a motor test does not roll it over
    flash_load(rig.flash, flash_path);
    const std::string log_path = flash_path + ".log";
    if (logmem_kb < 16) rig.has_logmem = false;
    else { rig.logmem.mem.assign((size_t)(logmem_kb / 4 * 4) * 1024u, 0xFF); if (!fresh) logmem_load(rig.logmem, log_path); }
    if (!fresh && rig.flash.kv.empty()) {
        wbp::Settings s; wbp::SettingsStore st(rig.flash); st.load(s);
        VirtualBoard::wire(s);
        s.pins.flow_cs = VirtualBoard::kFlowCs; s.pins.flow_sck = VirtualBoard::kFlowSck; s.pins.flow_miso = VirtualBoard::kFlowMiso; s.pins.flow_mosi = VirtualBoard::kFlowMosi; s.pins.flow_rx = VirtualBoard::kFlowRx;
        s.modes.mode[0] = wbp::MODE_ARM; s.modes.aux[0] = 0; s.modes.lo[0] = 1700; s.modes.hi[0] = 2100;
        s.modes.mode[1] = wbp::MODE_ANGLE; s.modes.aux[1] = 1; s.modes.lo[1] = 1700; s.modes.hi[1] = 2100;
        s.modes.mode[2] = wbp::MODE_ALTHOLD; s.modes.aux[2] = 2; s.modes.lo[2] = 1700; s.modes.hi[2] = 2100;
        st.save(s);
    }
    int flash_writes = rig.flash.writes; uint32_t log_ops = rig.logmem.writes + rig.logmem.erases; double log_saved = 0;
    rig.boot();
    if (stm32) rig.fw->server.set_features(wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | wbp::FEAT_STM32);
    printf("wb_sim: firmware %s, app on socket://127.0.0.1:%d, control on %d, flash '%s'%s\n", rig.fw->began ? "running" : "started WITHOUT a working IMU/motor setup", port, port + 1, flash_path.c_str(), fresh ? " (fresh)" : "");
    for (int i = 0; i < rig.app().report_lines(); i++) printf("  %s\n", rig.app().report()[i]);
    fflush(stdout);

    sock_t app = BAD_SOCK, ctl = BAD_SOCK; std::string ctl_line; bool quit = false;
    const auto t0 = std::chrono::steady_clock::now(); const uint64_t sim0 = g_stub_us; uint64_t last_rx_us = g_stub_us; int boots = rig.boots;
    while (!quit) {
        if (app == BAD_SOCK) { app = accept_on(l_app); if (app != BAD_SOCK) { rig.tx.out.clear(); last_rx_us = g_stub_us; } }
        else { sock_t extra = accept_on(l_app); if (extra != BAD_SOCK) sock_close(extra); }       // one app at a time, like one USB cable
        if (ctl == BAD_SOCK) ctl = accept_on(l_ctl);

        if (app != BAD_SOCK) {
            uint8_t buf[2048]; const int n = recv_some(app, buf, sizeof(buf));
            if (n > 0) { rig.fw->server.on_bytes(buf, (size_t)n); last_rx_us = g_stub_us; }
            else if (n < 0) { sock_close(app); app = BAD_SOCK; }
            else if (g_stub_us - last_rx_us > 20000) { rig.fw->server.on_idle(); last_rx_us = g_stub_us; }
        }
        if (ctl != BAD_SOCK) {
            uint8_t buf[512]; const int n = recv_some(ctl, buf, sizeof(buf));
            if (n < 0) { sock_close(ctl); ctl = BAD_SOCK; ctl_line.clear(); }
            for (int i = 0; i < n; i++) {
                if (buf[i] == '\n') { const std::string rep = control(rig, ctl_line, quit) + "\n"; send_all(ctl, (const uint8_t*)rep.data(), rep.size()); ctl_line.clear(); }
                else if (buf[i] != '\r' && ctl_line.size() < 200) ctl_line.push_back((char)buf[i]);
            }
        }

        // simulated time follows the wall clock (x speed); never more than 50 ms of catching up in one go
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const uint64_t target = sim0 + (uint64_t)(wall * speed * 1e6);
        int budget = 50;
        while (g_stub_us < target && budget-- > 0) rig.run(0.001);
        if (rig.boots != boots) { boots = rig.boots; if (stm32) rig.fw->server.set_features(wbp::FEAT_STREAMS | wbp::FEAT_CALIBRATION | wbp::FEAT_MOTOR_TEST | wbp::FEAT_GPS | wbp::FEAT_BLACKBOX | wbp::FEAT_BATTERY | wbp::FEAT_BEEPER | wbp::FEAT_STM32); printf("wb_sim: firmware restarted (boot %d)\n", boots); fflush(stdout); }
        if (rig.flash.writes != flash_writes) { flash_writes = rig.flash.writes; flash_save(rig.flash, flash_path); }
        if (rig.has_logmem && rig.logmem.writes + rig.logmem.erases != log_ops && wall - log_saved > 0.5) { log_ops = rig.logmem.writes + rig.logmem.erases; log_saved = wall; logmem_save(rig.logmem, log_path); }

        if (!rig.tx.out.empty()) {
            if (app != BAD_SOCK && !send_all(app, rig.tx.out.data(), rig.tx.out.size())) { sock_close(app); app = BAD_SOCK; }
            rig.tx.out.clear();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    if (rig.has_logmem) logmem_save(rig.logmem, log_path);
    if (app != BAD_SOCK) sock_close(app);
    if (ctl != BAD_SOCK) sock_close(ctl);
    sock_close(l_app); sock_close(l_ctl);
    return 0;
}
