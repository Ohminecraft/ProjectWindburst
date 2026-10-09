/*
 * This file is part of Windburst Project
 */

#pragma once

#include "gps_types.h"
#include "ubx_proto.h"
#include "nmea_proto.h"

// ============================================================================
// GPS engine: serial GPS driver state machine, hardware independent (talks through gps::Port) so the whole bring-up -
// baud sweep, module detection, per-generation auto-configuration, data parsing, timeouts - is unit-tested against a simulated u-blox.
// Follows Betaflight io/gps.c:
//   UNKNOWN -> DETECT_BAUD -> CHANGE_BAUD -> CONFIGURE -> RECEIVING_DATA -> (2.5 s without a solution) LOST_COMMUNICATION -> DETECT_BAUD
//   DETECT_BAUD : send MON-VER 3x every 330 ms; answered => $PUBX,41 to the wanted baud, else step the FC baud (auto_baud sweep)
//   CHANGE_BAUD : wait 990 ms for the module's UART to settle (M8 needs it), switch the FC to the wanted baud
//   CONFIGURE   : after 1 s, one step per 110 ms, each waiting for UBX-ACK (retry, then skip & count)
//   RECEIVING   : once a 3D fix exists, switch the dynamic model from "acquire" to "flight" (BF acquire/flight model)
// ============================================================================
namespace gps {

class Port {
public:
    virtual ~Port() = default;
    virtual int read() = 0;                                  // next received byte or -1
    virtual void write(const uint8_t* d, size_t n) = 0;
    virtual void set_baud(uint32_t baud) = 0;
    virtual bool tx_empty() { return true; }
};

class Engine {
public:
    Config cfg;

    void begin(Port& port, uint32_t now_ms) {
        port_ = &port; now_ = now_ms; state_ts_ = now_ms; last_nav_ms_ = now_ms;
        sol_ = Solution(); version_ = UBX_UNDEF; version_assumed_ = false; flight_model_ = false; armed_ = false; sat_n_ = 0; timeouts_ = 0;
        config_nacks_ = config_timeouts_ = 0; nav_dt_ms_ = 0; sweep_steps_ = 0;
        user_baud_idx_ = baud_index(cfg.baud);
        temp_baud_idx_ = user_baud_idx_;
        if (cfg.provider == PROV_MSP || cfg.provider == PROV_VIRTUAL) { set_state(ST_INITIALIZED); return; }
        port_->set_baud(kBauds[temp_baud_idx_]);
        set_state(ST_DETECT_BAUD);
    }

    // Call often (>= 100 Hz is plenty). Returns true when a NEW complete solution was received during this call.
    bool update(uint32_t now_ms) {
        now_ = now_ms;
        bool fresh = false;
        if (port_ && cfg.provider != PROV_MSP && cfg.provider != PROV_VIRTUAL) {
            int c, budget = 512;
            while (budget-- > 0 && (c = port_->read()) >= 0) fresh |= feed((uint8_t)c);
        }
        switch (state_) {
        case ST_UNKNOWN: case ST_INITIALIZED: break;
        case ST_DETECT_BAUD: case ST_CHANGE_BAUD: case ST_CONFIGURE:
            if (cfg.provider == PROV_UBLOX) configure_ublox(); else if (cfg.provider == PROV_NMEA) configure_nmea();
            break;
        case ST_LOST_COMMUNICATION:
            timeouts_++; sol_.num_sat = 0; sol_.fix = false; sol_.fix_type = 0;
            set_state(ST_DETECT_BAUD); break;
        case ST_RECEIVING_DATA:
            if (cfg.provider == PROV_UBLOX && cfg.auto_config && !flight_model_ && sol_.fix && port_->tx_empty()) {
                flight_model_ = true; send_nav_model(cfg.flight_model);
            }
            if (cfg.provider == PROV_UBLOX && cfg.auto_config && sat_dirty_ && port_->tx_empty()) apply_sat_rate();
            if (now_ - last_nav_ms_ > kTimeoutMs) { sol_.num_sat = 0; sol_.fix = false; sol_.fix_type = 0; set_state(ST_LOST_COMMUNICATION); }
            break;
        }
        return fresh;
    }

    // MSP / VIRTUAL providers: another subsystem supplies the solution (BF: GPS_MSP, GPS_VIRTUAL).
    void inject(const Solution& s, uint32_t now_ms) {
        now_ = now_ms; sol_ = s; note_solution();
        if (state_ == ST_INITIALIZED || state_ == ST_LOST_COMMUNICATION) set_state(ST_RECEIVING_DATA);
    }
    // BF turns NAV-SAT off while armed to reduce in-flight serial traffic (and back on after landing).
    void set_armed(bool armed) { if (armed != armed_) { armed_ = armed; sat_dirty_ = true; } }

    const Solution& solution() const { return sol_; }
    State state() const { return state_; }
    bool healthy() const { return state_ == ST_RECEIVING_DATA; }
    UbxVersion version() const { return version_; }
    bool version_assumed() const { return version_assumed_; }
    uint32_t baud() const { return kBauds[temp_baud_idx_]; }
    const SatInfo* sats() const { return sats_; }
    int sat_count() const { return sat_n_; }
    uint32_t timeouts() const { return timeouts_; }
    uint32_t config_nacks() const { return config_nacks_; }
    uint32_t config_timeouts() const { return config_timeouts_; }
    uint32_t frames() const { return ubx_.frames() + nmea_.frames; }
    uint32_t errors() const { return ubx_.errors() + nmea_.errors; }
    float rate_hz() const { return nav_dt_ms_ ? 1000.0f / (float)nav_dt_ms_ : 0.0f; }
    bool configured() const { return cfg_pos_ >= kStepCount; }
    uint32_t last_solution_ms() const { return last_nav_ms_; }

    static constexpr uint32_t kBauds[6] = {230400, 115200, 57600, 38400, 19200, 9600};   // BF gpsInitData order

private:
    // ---- timings from Betaflight ----
    static constexpr uint32_t kTimeoutMs = 2500, kBaudTestMs = 330, kStepMs = 110, kTestCount = 3, kAckTimeoutMs = 400, kSweepCycles = 2, kSweepBackoffMs = 30000;
    enum Step : uint8_t { S_DETECT_UNIT, S_SLOW_RATE, S_DISABLE_NMEA, S_VTG, S_GSV, S_GLL, S_GGA, S_GSA, S_RMC, S_ACQUIRE, S_ANA, S_SBAS, S_PMS, S_PVT,
                          S_SOL, S_POSLLH, S_STATUS, S_VELNED, S_DOP, S_SATINFO, S_NAV_RATE, S_GNSS, kStepCount };
    enum Ack : uint8_t { ACK_IDLE, ACK_WAITING, ACK_GOT, ACK_NACK };

    // the standard rate nearest to b (a value typed by hand, 100000 say, is not silently replaced by 57600)
    static int baud_index(uint32_t b) {
        int best = 0; uint32_t d_best = 0xFFFFFFFFu;
        for (int i = 0; i < 6; i++) { const uint32_t d = kBauds[i] > b ? kBauds[i] - b : b - kBauds[i]; if (d < d_best) { d_best = d; best = i; } }
        return best;
    }
    void set_state(State s) { state_ = s; state_ts_ = now_; cfg_pos_ = 0; if (s == ST_DETECT_BAUD) { test_count_ = 0; test_sent_ = false; } if (s == ST_CONFIGURE) { ack_ = ACK_IDLE; retries_ = 0; step_ts_ = now_; gnss_set_sent_ = false; } }
    void note_solution() {
        if (last_nav_ms_ && sol_.time_ms != prev_itow_ ) { const uint32_t dt = now_ - last_nav_ms_; if (dt > 0 && dt < 2000) nav_dt_ms_ = nav_dt_ms_ ? (nav_dt_ms_ * 7 + dt) / 8 : dt; }
        prev_itow_ = sol_.time_ms; last_nav_ms_ = now_;
    }

    // ---------------------------------------------------------------- receive path
    bool feed(uint8_t c) {
        if (cfg.provider == PROV_NMEA) {
            if (nmea_.feed((char)c)) { sol_ = nmea_.sol; copy_nmea_sats(); note_solution(); return true; }
            return false;
        }
        ubx::Message m;
        if (!ubx_.feed(c, m)) return false;
        return handle(m);
    }
    void copy_nmea_sats() { sat_n_ = nmea_.num_sats_in_view < kMaxSats ? nmea_.num_sats_in_view : kMaxSats; for (int i = 0; i < sat_n_; i++) sats_[i] = nmea_.sats[i]; }
    bool handle(const ubx::Message& m) {
        using namespace ubx;
        bool complete = false;
        const uint16_t key = (uint16_t)((m.cls << 8) | m.id);
        switch (key) {
        case (CLS_NAV << 8) | NAV_PVT: decode_pvt(m, sol_); complete = true; break;
        case (CLS_NAV << 8) | NAV_POSLLH: decode_posllh(m, sol_); have_pos_ = true; break;
        case (CLS_NAV << 8) | NAV_VELNED: decode_velned(m, sol_); have_vel_ = true; break;
        case (CLS_NAV << 8) | NAV_SOL: decode_sol(m, sol_); break;
        case (CLS_NAV << 8) | NAV_STATUS: decode_status(m, sol_); break;
        case (CLS_NAV << 8) | NAV_DOP: decode_dop(m, sol_); break;
        case (CLS_NAV << 8) | NAV_SAT: sat_n_ = decode_sat(m, sats_, kMaxSats); break;
        case (CLS_NAV << 8) | NAV_SVINFO: sat_n_ = decode_svinfo(m, sats_, kMaxSats); break;
        case (CLS_MON << 8) | MON_VER: {
            UbxVersion v = decode_hw_version(m);
            if (v == UBX_UNDEF && m.len >= 40) { v = UBX_M8; version_assumed_ = true; }       // unknown hw string: assume a modern module rather than never configuring
            version_ = v; break; }
        case (CLS_CFG << 8) | CFG_GNSS: if (state_ == ST_CONFIGURE && cfg_pos_ == S_GNSS) answer_gnss(m); break;
        case (CLS_ACK << 8) | ACK_ACK: case (CLS_ACK << 8) | ACK_NACK:
            if (ack_ == ACK_WAITING && m.len >= 2 && m.payload[0] == ack_cls_ && m.payload[1] == ack_id_) {
                if (cfg_pos_ == S_GNSS && !gnss_set_sent_) break;                               // ACK of the poll, not of our CFG-GNSS write
                ack_ = (m.id == ACK_ACK) ? ACK_GOT : ACK_NACK;
            }
            break;
        default: return false;
        }
        if (have_pos_ && have_vel_) { have_pos_ = have_vel_ = false; complete = true; }
        if (complete) { note_solution(); }
        return complete;
    }

    // ---------------------------------------------------------------- transmit helpers
    void send(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, bool wait_ack) {
        uint8_t buf[ubx::kMaxFrame];
        const size_t n = ubx::build(buf, sizeof(buf), cls, id, payload, len);
        if (!n) return;
        port_->write(buf, n);
        ack_cls_ = cls; ack_id_ = id; ack_ = wait_ack ? ACK_WAITING : ACK_GOT; sent_ts_ = now_;
    }
    void send_valset(ubx::ValSet& v, bool wait_ack) { send(ubx::CLS_CFG, ubx::CFG_VALSET, v.buf, (uint16_t)v.n, wait_ack); }
    void send_cfg_msg(uint8_t cls, uint8_t id, uint8_t rate, bool wait = true) { uint8_t p[3]; ubx::cfg_msg_rate(p, cls, id, rate); send(ubx::CLS_CFG, ubx::CFG_MSG, p, 3, wait); }
    void send_nav_model(AcquireModel model) {
        if (version_ >= UBX_M9) { ubx::ValSet v; ubx::cfg_valset_nav(v, model, cfg.utc_standard); send_valset(v, false); }
        else { uint8_t p[36]; ubx::nav5_legacy(p, model, cfg.utc_standard); send(ubx::CLS_CFG, ubx::CFG_NAV5, p, 36, false); }
        ack_ = ACK_IDLE;
    }
    void send_rate(uint16_t hz) {
        if (version_ >= UBX_M9) { ubx::ValSet v; ubx::cfg_valset_rate(v, hz); send_valset(v, true); }
        else { uint8_t p[6]; ubx::cfg_rate_legacy(p, hz); send(ubx::CLS_CFG, ubx::CFG_RATE, p, 6, true); }
    }
    void apply_sat_rate() {                                                                  // armed: off, disarmed: every 5th solution
        sat_dirty_ = false; const uint8_t rate = armed_ ? 0 : 5;
        if (version_ >= UBX_M9) { ubx::ValSet v; v.add(ubx::K_MSGOUT_NAV_SAT_UART1, rate); send_valset(v, false); }
        else if (version_ >= UBX_M8) send_cfg_msg(ubx::CLS_NAV, ubx::NAV_SAT, rate, false);
        else send_cfg_msg(ubx::CLS_NAV, ubx::NAV_SVINFO, rate, false);
        ack_ = ACK_IDLE;
        if (armed_) sat_n_ = 0;
    }
    void answer_gnss(const ubx::Message& m) {                                                // BF: modify the polled CFG-GNSS and write it back
        if (m.len < 4) return;
        const uint8_t nblocks = m.payload[3]; const uint16_t size = (uint16_t)(4 + nblocks * 8);
        if (nblocks > 7 || m.len < size) return;
        uint8_t p[4 + 7 * 8]; memcpy(p, m.payload, size);
        for (int i = 0; i < nblocks; i++) {
            uint8_t* b = p + 4 + 8 * i; const uint8_t gnss = b[0]; uint32_t flags; memcpy(&flags, b + 4, 4);
            if (gnss == 1 /*SBAS*/ && cfg.sbas == SBAS_NONE) flags &= ~1u;
            if (gnss == 2 /*Galileo*/) { if (cfg.use_galileo) flags |= 1u; else flags &= ~1u; }
            memcpy(b + 4, &flags, 4);
        }
        send(ubx::CLS_CFG, ubx::CFG_GNSS, p, size, true); gnss_set_sent_ = true;
    }

    // ---------------------------------------------------------------- state machine: u-blox
    void configure_ublox() {
        if (!port_->tx_empty()) return;
        switch (state_) {
        case ST_DETECT_BAUD: {
            if (version_ != UBX_UNDEF) {                                                      // module answered: move it to the wanted baud
                char b[40]; const size_t n = ubx::pubx41(b, sizeof(b), kBauds[user_baud_idx_]);
                port_->write((const uint8_t*)b, n);
                temp_baud_idx_ = user_baud_idx_; sweep_steps_ = 0;
                set_state(ST_CHANGE_BAUD); return;
            }
            if (test_count_ < kTestCount) {
                if (!test_sent_) { send(ubx::CLS_MON, ubx::MON_VER, nullptr, 0, false); ack_ = ACK_IDLE; test_sent_ = true; }
                if (now_ - state_ts_ > kBaudTestMs) { state_ts_ = now_; test_sent_ = false; test_count_++; }
                return;
            }
            test_count_ = 0; state_ts_ = now_;
            if (!cfg.auto_baud) return;                                                       // stay on the configured baud, keep asking
            if (sweep_steps_ >= kSweepCycles * 6 && now_ - last_sweep_ms_ < kSweepBackoffMs) return;
            last_sweep_ms_ = now_;
            temp_baud_idx_ = temp_baud_idx_ == 0 ? 5 : temp_baud_idx_ - 1;                   // 9600 first from the top, then upwards
            port_->set_baud(kBauds[temp_baud_idx_]);
            if (sweep_steps_ < kSweepCycles * 6) sweep_steps_++;
            break; }
        case ST_CHANGE_BAUD:
            if (now_ - state_ts_ < 3 * kBaudTestMs) return;
            port_->set_baud(kBauds[user_baud_idx_]); temp_baud_idx_ = user_baud_idx_;
            set_state(ST_CONFIGURE); break;
        case ST_CONFIGURE:
            if (!cfg.auto_config) { set_state(ST_RECEIVING_DATA); last_nav_ms_ = now_; return; }
            if (now_ - state_ts_ < 1000) return;
            run_config_step(); break;
        default: break;
        }
    }
    void run_config_step() {
        if (cfg_pos_ >= kStepCount) { set_state(ST_RECEIVING_DATA); last_nav_ms_ = now_; return; }
        if (ack_ == ACK_WAITING) {
            if (now_ - sent_ts_ < kAckTimeoutMs) return;
            config_timeouts_++;
            ack_ = ACK_IDLE;
            if (++retries_ > 2) { cfg_pos_++; retries_ = 0; step_ts_ = now_; return; }       // give up on this step after 3 tries
        } else if (ack_ == ACK_GOT) {
            cfg_pos_++; retries_ = 0; ack_ = ACK_IDLE; step_ts_ = now_; return;
        } else if (ack_ == ACK_NACK) {
            config_nacks_++; cfg_pos_++; retries_ = 0; ack_ = ACK_IDLE; step_ts_ = now_; return;
        }
        if (now_ - step_ts_ < kStepMs) return;
        step_ts_ = now_;
        const bool m9 = version_ >= UBX_M9, m8 = version_ >= UBX_M8, m7 = version_ >= UBX_M7;
        switch (cfg_pos_) {
        case S_DETECT_UNIT:
            if (version_ == UBX_UNDEF) { send(ubx::CLS_MON, ubx::MON_VER, nullptr, 0, false); ack_ = ACK_IDLE; } else { cfg_pos_++; }
            break;
        case S_SLOW_RATE: send_rate(1); break;                                                // throttle to 1 Hz while configuring
        case S_DISABLE_NMEA:
            if (m9) { ubx::ValSet v; ubx::cfg_valset_disable_nmea(v); send_valset(v, false); cfg_pos_ = S_RMC; }   // BF: one VALSET, then skip the per-message steps
            else { cfg_pos_++; }
            break;
        case S_VTG: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_VTG, 0); break;
        case S_GSV: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_GSV, 0); break;
        case S_GLL: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_GLL, 0); break;
        case S_GGA: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_GGA, 0); break;
        case S_GSA: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_GSA, 0); break;
        case S_RMC: send_cfg_msg(ubx::CLS_NMEA, ubx::NMEA_RMC, 0); break;
        case S_ACQUIRE: send_nav_model(cfg.acquire_model); ack_ = ACK_GOT; break;
        case S_ANA: cfg_pos_++; break;                                                        // AssistNow Autonomous: BF default off, not supported
        case S_SBAS:
            if (m9) { ubx::ValSet v; ubx::cfg_valset_sbas(v, cfg.sbas, cfg.sbas_integrity); send_valset(v, false); }
            else { uint8_t p[8]; ubx::cfg_sbas_legacy(p, cfg.sbas, cfg.sbas_integrity); send(ubx::CLS_CFG, ubx::CFG_SBAS, p, 8, true); }
            break;
        case S_PMS:
            if (m9) { ubx::ValSet v; v.add(ubx::K_PM_OPERATEMODE, 0); send_valset(v, false); }
            else if (m8) { uint8_t p[8]; ubx::cfg_pms(p); send(ubx::CLS_CFG, ubx::CFG_PMS, p, 8, true); }
            else { cfg_pos_++; }
            break;
        case S_PVT:
            if (m9) { ubx::ValSet v; v.add(ubx::K_MSGOUT_NAV_PVT_UART1, 1); send_valset(v, false); }
            else if (m7) { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_PVT, 1); }
            else { cfg_pos_++; }
            break;
        // From M7 on NAV-PVT carries everything, so the legacy NAV messages are switched off; M5/M6 need them (BF logic).
        case S_SOL:    if (m9) { cfg_pos_++; } else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_SOL, m7 ? 0 : 1); } break;
        case S_POSLLH: if (m9) { cfg_pos_++; } else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_POSLLH, m7 ? 0 : 1); } break;   // M9+: already off by default
        case S_STATUS: if (m9) { cfg_pos_++; } else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_STATUS, m7 ? 0 : 1); } break;
        case S_VELNED: if (m9) { cfg_pos_++; } else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_VELNED, m7 ? 0 : 1); } break;
        case S_DOP:
            if (m9) { ubx::ValSet v; v.add(ubx::K_MSGOUT_NAV_DOP_UART1, 0); send_valset(v, false); }
            else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_DOP, m7 ? 0 : 1); }
            break;
        case S_SATINFO:
            sat_dirty_ = false;
            if (m9) { ubx::ValSet v; v.add(ubx::K_MSGOUT_NAV_SAT_UART1, 5); send_valset(v, false); }
            else if (m8) { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_SAT, 5); }
            else { send_cfg_msg(ubx::CLS_NAV, ubx::NAV_SVINFO, 5); }
            break;
        case S_NAV_RATE: send_rate(cfg.update_rate_hz); break;
        case S_GNSS:
            if (m9) { ubx::ValSet v; v.add(ubx::K_SIGNAL_SBAS_ENA, cfg.sbas != SBAS_NONE ? 1 : 0); v.add(ubx::K_SIGNAL_GAL_ENA, cfg.use_galileo ? 1 : 0); gnss_set_sent_ = true; send_valset(v, true); }
            else if (m7 && (cfg.sbas == SBAS_NONE || cfg.use_galileo)) { gnss_set_sent_ = false; send(ubx::CLS_CFG, ubx::CFG_GNSS, nullptr, 0, true); }
            else { cfg_pos_++; }
            break;
        default: cfg_pos_++; break;
        }
    }

    // ---------------------------------------------------------------- state machine: NMEA (BF gpsConfigureNmea: set baud, send custom commands)
    void configure_nmea() {
        if (!port_->tx_empty()) return;
        if (now_ - state_ts_ < 500) return;                                                  // BF: 500 ms between steps
        if (state_ == ST_DETECT_BAUD) { set_state(ST_CHANGE_BAUD); return; }                  // BF does not try to detect the NMEA module's baud
        if (state_ == ST_CHANGE_BAUD) {
            if (cfg_pos_ == 0) { port_->set_baud(kBauds[user_baud_idx_]); temp_baud_idx_ = user_baud_idx_; cfg_pos_ = 1; state_ts_ = now_; return; }
            send_custom_commands(); set_state(ST_RECEIVING_DATA); last_nav_ms_ = now_;
        }
    }
    void send_custom_commands() {
        const char* p = cfg.nmea_custom_commands;
        while (*p) {
            while (*p == ' ') p++;
            const char* e = p; while (*e && *e != ' ') e++;
            if (e > p) { port_->write((const uint8_t*)p, (size_t)(e - p)); port_->write((const uint8_t*)"\r\n", 2); }
            p = e;
        }
    }

    Port* port_ = nullptr; ubx::Parser ubx_; nmea::Parser nmea_;
    Solution sol_; State state_ = ST_UNKNOWN;
    uint32_t now_ = 0, state_ts_ = 0, step_ts_ = 0, sent_ts_ = 0, last_nav_ms_ = 0, last_sweep_ms_ = 0, prev_itow_ = 0, nav_dt_ms_ = 0, timeouts_ = 0, config_nacks_ = 0, config_timeouts_ = 0;
    int user_baud_idx_ = 2, temp_baud_idx_ = 2; uint8_t test_count_ = 0, retries_ = 0, ack_cls_ = 0, ack_id_ = 0; uint16_t sweep_steps_ = 0;
    Ack ack_ = ACK_IDLE; uint8_t cfg_pos_ = 0;
    UbxVersion version_ = UBX_UNDEF; bool version_assumed_ = false, flight_model_ = false, armed_ = false, sat_dirty_ = false;
    bool test_sent_ = false, have_pos_ = false, have_vel_ = false, gnss_set_sent_ = false;
    SatInfo sats_[kMaxSats]; int sat_n_ = 0;
};

constexpr uint32_t Engine::kBauds[6];

} // namespace gps
