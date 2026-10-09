/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "bb_format.h"
#include "bb_recorder.h"
#include "bb_flash.h"
#include "bb_storage.h"
#include "../proto/wbp_schema.h"
#include "../proto/wbp_codec.h"
#include "../proto/wbp_server.h"

// ============================================================================
// Service: the link-task end of the blackbox. Called once per millisecond (poll), it
//   * decides when a session starts and ends (settings: device, trigger; the app's "stream to me" request),
//   * writes the session header, then moves records from the Recorder's queue into frames and the frames to the sink:
//       storage : the Storage the settings select (bb_storage.h) -
//                 FlashLog: one flash page per call at most -> one short stall of the flight loop per millisecond at most
//                 CardLog : bytes go into a queue, a separate task writes them to the SD card
//       serial  : the app link; a frame that does not fit the transmit buffer is dropped, the stream never blocks
//   * erases, verifies and reads back the log memory for the app.
// The flight loop is involved only through the Recorder (queue + two atomics).
//
// Rules for the two destinations (the spec: "connected to the app -> stop storing, stream instead"):
//   * the app asks for the stream (BLACKBOX_STREAM). On the ground a running storage session is closed and the stream starts.
//     In the air the storage session is NOT cut: the request is refused with BUSY until the craft is disarmed.
//   * while the app streams, no storage session starts. When the stream ends (the app says so, or it has been silent for 3 s -
//     cable pulled), the trigger decides again, also in flight.
// Portable. Everything here runs in ONE task.
// ============================================================================
namespace bb {

enum State : uint8_t { ST_OFF, ST_NO_STORAGE, ST_NEEDS_ERASE, ST_IDLE, ST_LOGGING, ST_STREAMING, ST_ERASING, ST_FULL };
enum Device : uint8_t { DEV_NONE, DEV_FLASH, DEV_SDCARD };
enum Trigger : uint8_t { TRIG_SWITCH, TRIG_ARMED, TRIG_ALWAYS };

struct Config {
    uint8_t device = DEV_NONE, trigger = TRIG_ARMED;
    uint8_t sd_mode = 0, sd_mhz = 20;                // SD card wiring (SPI / SDMMC 1-bit) and clock
    uint16_t rate_hz = 250;
    uint32_t groups = 0;
};
struct Inputs { uint32_t now_ms = 0; bool armed = false, switch_on = false, app_active = false; };

// The live stream shares the link with the telemetry and the replies: it gets half of what the link carries.
// -> the highest record rate that fits (records of `record_bytes` incl. their type byte; frames add about 3 %); 0 = no limit.
inline uint16_t stream_rate_limit(uint32_t link_bytes_per_s, uint16_t record_bytes) {
    if (link_bytes_per_s == 0 || record_bytes == 0) return 0;
    const uint32_t per_record = (uint32_t)record_bytes * 103u / 100u + 1u;
    const uint32_t hz = link_bytes_per_s / 2u / per_record;
    return (uint16_t)(hz < 1u ? 1u : hz > 65535u ? 65535u : hz);
}

// fast records per second -> every how many control cycles
inline uint8_t rate_divider(float loop_hz, uint16_t rate_hz) {
    if (rate_hz == 0 || loop_hz <= 0) return 1;
    const int d = (int)(loop_hz / (float)rate_hz + 0.5f);
    return (uint8_t)(d < 1 ? 1 : d > 255 ? 255 : d);
}

class Service {
public:
    // extra(text, ctx): the firmware's own header lines (name, version, schema hash, settings as hex)
    using ExtraFn = void (*)(TextOut&, void*);
    Service(Recorder& rec, wbp::Transport& tx) : rec_(rec), tx_(tx) {}

    void attach_store(BlockStore* store) { flash_.begin(store); select(); }       // the on-board flash region (device FLASH)
    void attach_card(Storage* card) { card_ = card; if (card_) card_->configure_device(cfg_.device == DEV_SDCARD, cfg_.sd_mode, cfg_.sd_mhz); select(); }   // the SD card (device SDCARD)
    void set_header_source(ExtraFn fn, void* ctx) { extra_ = fn; extra_ctx_ = ctx; }
    void set_loop(float loop_hz, uint8_t motors) { loop_hz_ = loop_hz; motors_ = motors; }
    void set_config(const Config& c) {                             // takes effect for the next session
        cfg_ = c;
        if (card_) card_->configure_device(cfg_.device == DEV_SDCARD, cfg_.sd_mode, cfg_.sd_mhz);
        select();
    }
    const Config& config() const { return cfg_; }
    Storage& log() { return *cur_; }                               // the storage the settings select (NONE: the flash, for reading back)
    FlashLog& flash() { return flash_; }
    uint8_t device() const { return dev_; }                        // the device log() belongs to
    State state() const { return state_; }

    // ------------------------------------------------------------------------------------------------ once per millisecond
    void poll(const Inputs& in) {
        now_ms_ = in.now_ms; armed_ = in.armed;
        if (in.armed) { rd_pos_ = rd_end_ = 0; reading_held_ = false; }                       // armed: no flash reads for a download, and nothing holds the flight log back
        if ((uint32_t)(in.now_ms - sec_ms_) >= 1000u) { sec_ms_ = in.now_ms; rate_hz_ = (uint16_t)(sec_fast_ > 65535u ? 65535u : sec_fast_); byte_rate_ = sec_bytes_; sec_fast_ = 0; sec_bytes_ = 0; buffer_pct_ = cur_->take_buffer_peak(); }

        // ---- erase: one block per call; the craft stays on the ground meanwhile (Recorder::busy -> arming blocker)
        if (state_ == ST_ERASING) {
            if (!cur_->erase_step()) { rec_.set_busy(false); rec_.post_erase_done(); state_ = idle_state(); }
            return;
        }
        const bool trigger = cfg_.device != DEV_NONE && dev_ == cfg_.device && (cfg_.trigger == TRIG_ALWAYS || (cfg_.trigger == TRIG_ARMED && in.armed) || (cfg_.trigger == TRIG_SWITCH && in.switch_on));

        switch (state_) {
        case ST_LOGGING:
            if (closing_) { if (drain_and_close()) state_ = idle_state(); break; }
            if (!trigger) { begin_close(END_TRIGGER_OFF); break; }
            pump_records(in.now_ms);
            if (cur_->io_error() || !cur_->writing()) { rec_.stop(); rec_.ring().clear(); state_ = idle_state(); break; }
            if (cur_->writable() < 3 * kPage) { begin_close(END_FULL); break; }
            { const bool flush = (uint32_t)(in.now_ms - flush_ms_) >= kFlushMs; if (cur_->pump(flush) && flush) flush_ms_ = in.now_ms; }
            break;
        case ST_STREAMING:
            if (closing_) { if (drain_and_close()) state_ = idle_state(); break; }
            if (!stream_wanted_ || !in.app_active) { stream_wanted_ = false; begin_close(END_STREAM_STOPPED); break; }
            pump_records(in.now_ms);
            break;
        default:
            select();
            state_ = idle_state();
            if (!in.armed && cfg_.device != DEV_NONE) for (int i = 0; i < 4 && cur_->verify_step(); i++) {}   // check the free space while nothing depends on timing (not at all when logging is off)
            if (stream_wanted_ && in.app_active) { start(ST_STREAMING, stream_rate(stream_rate_ ? stream_rate_ : cfg_.rate_hz)); break; }
            stream_wanted_ = false;
            // (no new session while the app is reading the memory: the list it asked for must stay true until it has what it wants.
            //  Arming ends that - see the top of poll(): a flight is always logged.)
            if (trigger && state_ == ST_IDLE && !reading() && cur_->writable() >= kSector && cur_->start_session()) start(ST_LOGGING, cfg_.rate_hz);
            break;
        }
        if (state_ == ST_LOGGING && !in.armed) cur_->verify_step();                         // a session that began right after boot: keep clearing space ahead of it
        send_blobs();
    }

    // ------------------------------------------------------------------------------------------------ commands from the app (link task)
    static bool handles(uint8_t cmd) { return cmd == wbp::CMD_BLACKBOX_ERASE || cmd == wbp::CMD_BLACKBOX_STREAM || cmd == wbp::CMD_BLACKBOX_SESSION || cmd == wbp::CMD_BLACKBOX_READ; }
    wbp::Err command(uint8_t cmd, const uint8_t* a, uint8_t* result, uint16_t& rlen) {
        rlen = 0;
        switch (cmd) {
        case wbp::CMD_BLACKBOX_ERASE:
            if (!cur_->ok()) return wbp::ERR_NO_SENSOR;
            if (state_ == ST_LOGGING || state_ == ST_ERASING) return wbp::ERR_BUSY;
            rd_pos_ = rd_end_ = 0; reading_held_ = false;
            if (!cur_->erase_start()) return wbp::ERR_FAILED;
            if (state_ == ST_STREAMING) { rec_.stop(); rec_.ring().clear(); stream_wanted_ = false; closing_ = false; }
            rec_.set_busy(true); state_ = ST_ERASING;
            return wbp::ERR_OK;
        case wbp::CMD_BLACKBOX_STREAM: {
            const bool enable = a[0] != 0; const uint16_t rate = (uint16_t)(a[1] | (a[2] << 8));
            if (!enable) { stream_wanted_ = false; return wbp::ERR_OK; }
            if (state_ == ST_ERASING || (state_ == ST_LOGGING && armed_)) return wbp::ERR_BUSY;       // a flight log is not cut short
            if (state_ == ST_LOGGING && !closing_) begin_close(END_APP_TOOK_OVER);
            stream_wanted_ = true; stream_rate_ = rate;
            return wbp::ERR_OK; }
        case wbp::CMD_BLACKBOX_SESSION: {
            const int i = a[0] | (a[1] << 8);
            if (!cur_->ok()) return wbp::ERR_NO_SENSOR;
            if (const wbp::Err e = free_for_reading()) return e;
            if (i >= cur_->listed()) return wbp::ERR_BAD_ARG;
            const uint32_t st = cur_->session_start(i), sz = cur_->session_size(i);
            memcpy(result, &st, 4); memcpy(result + 4, &sz, 4); rlen = 8;
            return wbp::ERR_OK; }
        case wbp::CMD_BLACKBOX_READ: {
            uint32_t off, len; memcpy(&off, a, 4); memcpy(&len, a + 4, 4);
            if (!cur_->ok()) return wbp::ERR_NO_SENSOR;
            if (len == 0) { rd_pos_ = rd_end_ = 0; return wbp::ERR_OK; }
            if (const wbp::Err e = free_for_reading()) return e;
            if (off >= cur_->used() || len > cur_->used() - off) return wbp::ERR_BAD_ARG;
            rd_pos_ = off; rd_end_ = off + len;
            return wbp::ERR_OK; }
        default: return wbp::ERR_NOT_SUPPORTED;
        }
    }

    // T_blackbox for the telemetry stream
    uint16_t pack_status(uint8_t* out, uint16_t cap) const {
        wbp::T_blackbox o;
        if (cap < sizeof(o)) return 0;
        Plan p; p.build(cfg_.groups, motors_);
        o.state = state_; o.capacity = cur_->capacity(); o.used = cur_->used(); o.sessions = (uint16_t)cur_->sessions();
        o.rate_hz = rate_hz_; o.record_bytes = (uint16_t)(p.fast_size + 1); o.dropped = rec_.ring().dropped(); o.erase_pct = cur_->erase_percent(); o.byte_rate = byte_rate_;
        o.device = dev_; o.capacity_kb = cur_->capacity_kb(); o.used_kb = cur_->used_kb(); o.buffer_pct = buffer_pct_;
        memcpy(out, &o, sizeof(o));
        return (uint16_t)sizeof(o);
    }
    uint32_t stream_frames_dropped() const { return stream_dropped_; }
    uint32_t session_number() const { return session_no_; }

private:
    static constexpr uint32_t kFlushMs = 500;        // a page that is not full is written anyway after this long (bounds what a power cut can lose)

    // The app wants to list or read the log memory. A session that is being written on the ground (trigger ALWAYS, or the
    // switch) is closed for it - otherwise such a craft could never give its logs back - and none starts while the app is at
    // it (kReadHoldMs after its last request). In the air the flight log wins: BUSY.
    static constexpr uint32_t kReadHoldMs = 3000;
    wbp::Err free_for_reading() {
        if (state_ == ST_ERASING) return wbp::ERR_BUSY;
        if (state_ == ST_LOGGING) {
            if (armed_) return wbp::ERR_BUSY;
            if (!closing_) begin_close(END_APP_TOOK_OVER);
            read_hold_ms_ = now_ms_; reading_held_ = true;
            return wbp::ERR_BUSY;                                    // closed within a few milliseconds: the app asks again
        }
        read_hold_ms_ = now_ms_; reading_held_ = true;
        return wbp::ERR_OK;
    }
    bool reading() {
        if (rd_pos_ < rd_end_) { read_hold_ms_ = now_ms_; reading_held_ = true; return true; }
        if (reading_held_ && (uint32_t)(now_ms_ - read_hold_ms_) < kReadHoldMs) return true;
        reading_held_ = false;
        return false;
    }
    uint16_t stream_rate(uint16_t wanted) const {
        Plan p; p.build(cfg_.groups, motors_);
        const uint16_t lim = stream_rate_limit(tx_.bytes_per_second(), (uint16_t)(p.fast_size + 1));
        return lim && wanted > lim ? lim : wanted;
    }

    State idle_state() const {
        if (cfg_.device == DEV_NONE || dev_ != cfg_.device) return ST_OFF;
        if (!cur_->ok()) return ST_NO_STORAGE;
        if (cur_->needs_erase() || cur_->io_error()) return ST_NEEDS_ERASE;
        if (cur_->full()) return ST_FULL;
        return ST_IDLE;
    }

    // ---- session start: header text, then the recorder is switched on
    void start(State s, uint16_t rate_hz) {
        const uint8_t div = rate_divider(loop_hz_, rate_hz);
        Plan plan; plan.build(cfg_.groups, motors_);
        session_no_++;
        TextOut t(hdr_, sizeof(hdr_));
        t.add("WBLOG %d\n", kFormatVersion);
        t.add("session %lu\n", (unsigned long)(s == ST_LOGGING ? (uint32_t)cur_->sessions() + 1u : 0u));
        t.add("to %s\n", s == ST_LOGGING ? "storage" : "stream");
        t.add("boot_ms %lu\n", (unsigned long)now_ms_);
        t.add("loop_hz %g\n", (double)loop_hz_);
        t.add("rate_div %u\n", (unsigned)div);
        t.add("rate_hz %g\n", (double)(loop_hz_ / (float)div));
        t.add("slow_hz %u\n", (unsigned)kSlowHz);
        t.add("motors %u\n", (unsigned)plan.motors);
        describe_fields(t, "fast", plan, kFast, kFastDefs);
        describe_fields(t, "slow", plan, kSlow, kSlowDefs);
        t.add("events %s\n", kEventNames);
        if (extra_) extra_(t, extra_ctx_);
        t.add("end\n");
        hdr_len_ = t.overflow ? 0 : t.n; hdr_pos_ = 0; hdr_seq_ = 0;
        if (t.overflow) { TextOut m(hdr_, sizeof(hdr_)); m.add("WBLOG %d\nerror header too long\nend\n", kFormatVersion); hdr_len_ = m.n; }
        data_seq_ = 0; pay_n_ = 0; out_n_ = 0; closing_ = false; flush_ms_ = now_ms_; frame_ms_ = now_ms_; dropped_seen_ = rec_.ring().dropped();
        state_ = s;
        rec_.start(plan, div);
    }

    // Stop producing records; what is queued is still written, then an END event, then the session is closed.
    void begin_close(uint8_t reason) {
        rec_.stop(); closing_ = true; close_reason_ = reason; close_polls_ = 0; end_written_ = false;
        if (reason == END_FULL) rec_.ring().clear();             // no room for the backlog: the last bytes are kept for the END marker
    }
    // True when finished. Bounded: a sink that takes nothing any more (region full, dead link) ends the session after 2 s anyway.
    bool drain_and_close() {
        const bool storage = state_ == ST_LOGGING;
        if (++close_polls_ > 2000 || (storage && (cur_->io_error() || !cur_->writing()))) return finish();
        pump_records(now_ms_);
        if (storage) { cur_->pump(true); if (out_n_ && cur_->writable() < out_n_) return finish(); }      // the region is full to the last page
        if (hdr_pos_ < hdr_len_ || rec_.ring().peek()) return false;
        if (!end_written_) {
            if (pay_n_ + 8u > kFramePayload) { close_frame(); flush_out(); return false; }
            uint8_t* p = pay_ + pay_n_; *p++ = 'E'; put_u32(p, now_ms_ * 1000u); *p++ = EV_END; *p++ = 1; *p++ = close_reason_;
            pay_n_ = (uint16_t)(pay_n_ + 8); end_written_ = true;
        }
        if (out_n_ && !flush_out()) return false;
        if (pay_n_) { close_frame(); out_droppable_ = false; if (!flush_out()) return false; }
        if (storage && cur_->staged()) return false;                // pages still to be programmed, one per call
        return finish();
    }
    bool finish() {
        if (state_ == ST_LOGGING) cur_->end_session();
        closing_ = false; end_written_ = false; pay_n_ = 0; out_n_ = 0; hdr_pos_ = hdr_len_ = 0;
        rec_.ring().clear();
        return true;
    }

    // ---- queue -> frames -> sink
    void pump_records(uint32_t now_ms) {
        // header first, a frame at a time, never dropped
        while (hdr_pos_ < hdr_len_) {
            if (out_n_ && !flush_out()) return;
            const size_t n = hdr_len_ - hdr_pos_ > kFramePayload ? kFramePayload : hdr_len_ - hdr_pos_;
            out_n_ = (uint16_t)wbp::encode(out_, sizeof(out_), kMsgHeader, hdr_seq_++, (const uint8_t*)hdr_ + hdr_pos_, (uint16_t)n);
            out_droppable_ = false; hdr_pos_ += n;
        }
        if (out_n_ && !flush_out()) return;
        // records lost in the queue are part of the record
        const uint32_t dropped = rec_.ring().dropped();
        if (dropped != dropped_seen_ && pay_n_ + 11 <= kFramePayload) {
            const uint32_t n = dropped - dropped_seen_; dropped_seen_ = dropped;
            uint8_t* p = pay_ + pay_n_; *p++ = 'E'; put_u32(p, now_ms * 1000u); *p++ = EV_DROPPED; *p++ = 4; put_u32(p, n); pay_n_ = (uint16_t)(pay_n_ + 11);
        }
        for (int budget = 0; budget < 64; budget++) {
            const uint8_t n = rec_.ring().peek();
            if (!n) break;
            if (pay_n_ + n > kFramePayload) { close_frame(); if (out_n_ && !flush_out()) return; }
            if (pay_n_ == 0) frame_ms_ = now_ms;
            rec_.ring().pop(pay_ + pay_n_);
            if (pay_[pay_n_] == 'F') sec_fast_++;
            pay_n_ = (uint16_t)(pay_n_ + n);
        }
        // a frame that is not full leaves anyway after a short time: the app sees the stream live, and storage flushes do not wait for it
        if (pay_n_ && (uint32_t)(now_ms - frame_ms_) >= (state_ == ST_STREAMING ? 20u : 250u)) { close_frame(); flush_out(); }
    }
    void close_frame() {
        if (!pay_n_ || out_n_) return;
        out_n_ = (uint16_t)wbp::encode(out_, sizeof(out_), kMsgData, data_seq_++, pay_, pay_n_);
        out_droppable_ = true; pay_n_ = 0;
    }
    // Hands the encoded frame to the sink. False = the sink cannot take it now (storage: buffer full; serial header: transmit buffer full).
    bool flush_out() {
        if (!out_n_) return true;
        if (state_ == ST_LOGGING) {
            if (!cur_->append(out_, out_n_)) return false;
        } else {
            if (tx_.writable() < out_n_) {
                if (!out_droppable_) return false;
                stream_dropped_++; out_n_ = 0; return true;      // the app is too slow / the link too thin: this frame is lost, the next one may fit
            }
            tx_.write(out_, out_n_);
        }
        sec_bytes_ += out_n_; out_n_ = 0;
        return true;
    }

    // ---- download: BB_BLOB frames [offset u32][data], as fast as the transmit buffer takes them
    void send_blobs() {
        for (int k = 0; k < 8 && rd_pos_ < rd_end_; k++) {
            const uint32_t n = rd_end_ - rd_pos_ > 480u ? 480u : rd_end_ - rd_pos_;
            if (tx_.writable() < n + 4 + wbp::kOverhead) return;
            uint8_t pay[484], frame[484 + wbp::kOverhead];
            memcpy(pay, &rd_pos_, 4);
            if (!cur_->read(rd_pos_, pay + 4, n)) { rd_pos_ = rd_end_ = 0; return; }
            const size_t len = wbp::encode(frame, sizeof(frame), kMsgBlob, blob_seq_++, pay, (uint16_t)(n + 4));
            tx_.write(frame, len);
            rd_pos_ += n;
        }
    }

    // The storage the settings select. It is only switched while no session runs on it and nothing is being erased or read back.
    void select() {
        if (state_ == ST_LOGGING || state_ == ST_ERASING) return;
        // NONE keeps the flash selected: nothing is logged, but what is stored there can still be listed, downloaded and erased
        Storage* want = cfg_.device != DEV_SDCARD ? static_cast<Storage*>(&flash_) : card_ ? card_ : static_cast<Storage*>(&none_);
        if (want == cur_ && dev_ == cfg_.device) return;
        cur_ = want; dev_ = cfg_.device; rd_pos_ = rd_end_ = 0;
    }
    // SDCARD on a firmware without a card interface: a storage that is never there
    struct NoStorage : Storage {
        bool ok() const override { return false; } uint32_t capacity() const override { return 0; } uint32_t used() const override { return 0; }
        uint32_t capacity_kb() const override { return 0; } uint32_t used_kb() const override { return 0; }
        bool needs_erase() const override { return false; } bool io_error() const override { return false; } bool full() const override { return false; } bool writing() const override { return false; }
        int sessions() const override { return 0; } int listed() const override { return 0; } uint32_t session_start(int) const override { return 0; } uint32_t session_size(int) const override { return 0; }
        bool verify_step() override { return false; } uint32_t writable() const override { return 0; } bool start_session() override { return false; }
        bool append(const uint8_t*, uint32_t) override { return false; } uint32_t staged() const override { return 0; } uint32_t pump(bool) override { return 0; } void end_session() override {}
        bool erase_start() override { return false; } bool erase_step() override { return false; } uint8_t erase_percent() const override { return 100; }
        bool read(uint32_t, uint8_t*, uint32_t) override { return false; } const char* store_name() const override { return "none"; }
    };

    Recorder& rec_; wbp::Transport& tx_; FlashLog flash_; NoStorage none_; Storage* card_ = nullptr; Storage* cur_ = &none_; uint8_t dev_ = DEV_NONE, buffer_pct_ = 0;
    Config cfg_; State state_ = ST_OFF;
    ExtraFn extra_ = nullptr; void* extra_ctx_ = nullptr; float loop_hz_ = 4000.0f; uint8_t motors_ = 4;
    bool stream_wanted_ = false, closing_ = false, end_written_ = false, out_droppable_ = false, armed_ = false;
    uint8_t close_reason_ = 0, hdr_seq_ = 0, data_seq_ = 0, blob_seq_ = 0; uint16_t close_polls_ = 0;
    uint16_t stream_rate_ = 0, pay_n_ = 0, out_n_ = 0, rate_hz_ = 0;
    uint32_t now_ms_ = 0, flush_ms_ = 0, frame_ms_ = 0, sec_ms_ = 0, sec_fast_ = 0, sec_bytes_ = 0, byte_rate_ = 0, dropped_seen_ = 0, stream_dropped_ = 0, session_no_ = 0;
    uint32_t rd_pos_ = 0, rd_end_ = 0, read_hold_ms_ = 0; bool reading_held_ = false;
    size_t hdr_len_ = 0, hdr_pos_ = 0;
    uint8_t pay_[kFramePayload], out_[kFramePayload + wbp::kOverhead];
    char hdr_[6144];
};

} // namespace bb
