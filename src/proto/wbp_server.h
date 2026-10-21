#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "wbp_schema.h"
#include "wbp_codec.h"
#include "settings_store.h"

// ============================================================================
// WBP server: the firmware end of the app <-> firmware link. Portable; talks through Transport (USB CDC / UART) and Host (the flight app).
//
//  request                         reply
//  HELLO                           DEVICE_INFO  [proto u8][role u8][maj][min][patch][schema_hash u32][features u32][name...]
//                                               and, when set_chip() was called: [0][mcu name...][0][pin mask, 32 bytes: bit n = pin n exists]
//  SECTION_GET [id]                SECTION      [id][size u16][data]      (ACK with an error if the id is unknown)
//  SECTION_SET [id][data]          SECTION      the values ACTUALLY stored (range-validated, and with the rules of settings_rules.h
//                                               applied: the app sees what was clamped or computed)
//                                  ACK ERR_ARMED while armed, ERR_SIZE_MISMATCH / ERR_BAD_SECTION otherwise
//  SETTINGS_SAVE                   ACK (ERR_STORAGE if the flash write failed)       SETTINGS_DEFAULTS [id|0xFF]  ACK
//  REBOOT                          ACK, then Host::reboot()
//  STREAM_CFG [id][rate u16]       ACK          rate 0 = off, id 0xFF = all off; clamped to the stream's max
//  COMMAND [cmd][args...]          CMD_RESULT   [cmd][err][result...]
//  (unsolicited)                   STREAM [id][t_ms u32][data]   and   LOG [level][text]
//  ACK payload = [for_type][for_seq][err]
// Settings are edited in RAM; `flags()` tells the app (via the status stream) if they are unsaved / need a reboot. Nothing configuration-
// changing is accepted while armed.
// ============================================================================
namespace wbp {

enum Role : uint8_t { ROLE_RX = 0, ROLE_TX = 1 };
enum Feature : uint32_t { FEAT_STREAMS = 1u << 0, FEAT_CALIBRATION = 1u << 1, FEAT_MOTOR_TEST = 1u << 2, FEAT_GPS = 1u << 3, FEAT_BLACKBOX = 1u << 4, FEAT_BATTERY = 1u << 5, FEAT_BEEPER = 1u << 6,
                         FEAT_STM32 = 1u << 7, FEAT_WIFI = 1u << 8, FEAT_USB_NAME = 1u << 9 };     // USB_NAME: general.usb_name is the board's name on USB   // STM32: pin numbers are port * 16 + pin (PA0 = 0, PB3 = 19: src/port/pin_names.h), not GPIO numbers
enum SettingsFlags : uint8_t { SF_DIRTY = SETTINGS_FLAGS_DIRTY, SF_REBOOT_REQUIRED = SETTINGS_FLAGS_REBOOT_REQUIRED };

class Transport {
public:
    virtual ~Transport() = default;
    virtual size_t write(const uint8_t* d, size_t n) = 0;
    virtual size_t writable() { return 4096; }            // free space in the TX buffer; frames that do not fit are skipped, never blocked on
    virtual uint32_t bytes_per_second() { return 0; }     // what the link carries (a UART: baud / 10); 0 = so fast that nobody has to budget (USB)
};

class Host {                                               // implemented by the flight app (and by a fake in the tests)
public:
    virtual ~Host() = default;
    virtual bool armed() = 0;
    virtual uint32_t now_ms() = 0;
    // write the T_<stream> struct for stream `id` into out, return its size, 0 if the source is not available right now
    virtual uint16_t fill_stream(uint8_t id, uint8_t* out, uint16_t cap) = 0;
    virtual Err command(uint8_t cmd, const uint8_t* args, uint16_t n, uint8_t* result, uint16_t& rlen) = 0;
    // a section changed in RAM: apply it to the running firmware if that is possible without a reboot (PID, rates, modes ...)
    virtual bool apply_live(uint8_t /*section*/) { return false; }
    // true once after the firmware itself changed a section in RAM (a finished calibration): the settings become 'unsaved'
    virtual bool take_settings_changed() { return false; }
    // a section the firmware changed itself and wants kept across a power cycle (0 = none): written to flash by tick(), never while armed
    virtual uint8_t take_autosave() { return 0; }
    // a valid frame arrived from the app (any type): the app is there
    virtual void on_frame() {}
    // called from Server::tick(), i.e. every millisecond in the link task: background work of the link side (blackbox)
    virtual void tick(uint32_t /*now_ms*/) {}
    virtual void reboot() {}
};

class Server {
public:
    Server(Settings& live, SettingsStore* store, Host& host, Transport& tx, Role role, uint8_t maj, uint8_t min, uint8_t patch, const char* name, uint32_t features)
        : s_(live), store_(store), host_(host), tx_(tx), role_(role), maj_(maj), min_(min), patch_(patch), name_(name), features_(features) {}
    void set_features(uint32_t f) { features_ = f; }                 // (the simulator plays an STM32 board with wb_sim --stm32)
    // The chip by name (port/chip.h WB_MCU_NAME) and the pin numbers it has (32 bytes, bit n = pin n): sent with DEVICE_INFO, so
    // the app shows which MCU it talks to and offers only pins that exist on it.
    void set_chip(const char* mcu, const uint8_t* pin_mask) { mcu_ = mcu; memcpy(pins_, pin_mask, sizeof(pins_)); }

    void on_byte(uint8_t c) { Frame f; if (parser_.feed(c, f)) { handle(f); while (parser_.pump(f)) handle(f); } }
    void on_bytes(const uint8_t* d, size_t n) { for (size_t i = 0; i < n; i++) on_byte(d[i]); }
    // call when no byte arrived for ~20 ms
    void on_idle() { Frame f; if (parser_.idle(f)) { handle(f); while (parser_.pump(f)) handle(f); } }

    // call every few ms: emits due streams
    void tick(uint32_t now_ms) {
        host_.tick(now_ms);
        // a section the firmware changed by itself (in-flight level trim): kept without anybody pressing Save. If the flash refuses,
        // the settings show as unsaved instead.
        if (const uint8_t sec = host_.take_autosave()) {
            const SectionInfo* si = find_section(sec);
            if (si && store_ && store_->save_section(s_, sec)) { autosaved_++; char t[64]; snprintf(t, sizeof(t), "settings: section %s written by the firmware", si->name); log(1, t, true); }
            else if (si) dirty_ = true;
        }
        int sent = 0;
        for (int i = 0; i < kStreamCount && sent < 4; i++) {
            if (rate_[i] == 0) continue;
            if ((int32_t)(now_ms - next_[i]) < 0) continue;
            const uint32_t period = 1000u / rate_[i];
            next_[i] += period; if ((int32_t)(now_ms - next_[i]) > (int32_t)(period * 4)) next_[i] = now_ms + period;     // never burst to catch up
            uint8_t buf[kMaxPayload]; buf[0] = kStreams[i].id; buf[1] = (uint8_t)now_ms; buf[2] = (uint8_t)(now_ms >> 8); buf[3] = (uint8_t)(now_ms >> 16); buf[4] = (uint8_t)(now_ms >> 24);
            const uint16_t n = host_.fill_stream(kStreams[i].id, buf + 5, (uint16_t)(sizeof(buf) - 5));
            if (!n) { dropped_streams_++; continue; }
            if (!send(MSG_STREAM, stream_seq_++, buf, (uint16_t)(n + 5), true)) dropped_streams_++; else sent++;
        }
    }
    // droppable: skipped when the transmit buffer has no room (a message nobody asked for must never block on an app that is not there)
    void log(uint8_t level, const char* text, bool droppable = false) { uint8_t b[200]; const size_t n = strlen(text) > 190 ? 190 : strlen(text); b[0] = level; memcpy(b + 1, text, n); send(MSG_LOG, log_seq_++, b, (uint16_t)(n + 1), droppable); }

    uint8_t flags() const { return (dirty_ ? SF_DIRTY : 0) | (reboot_required_ ? SF_REBOOT_REQUIRED : 0); }
    // ---- the CLI (src/app/cli.h) edits the settings in place, in the same task; these are the steps a SECTION_SET / SAVE takes
    bool rx_mid_frame() const { return parser_.mid_frame(); }
    bool armed() const { return host_.armed(); }
    void commit_section(uint8_t id) {                      // a section was changed in RAM: clamp it, derive what follows, apply or ask for a reboot
        validate_section(s_, id); apply_rules(s_, id); dirty_ = true;
        if (!host_.apply_live(id)) reboot_required_ = true;
    }
    bool save_all() { if (host_.armed() || !store_ || !store_->save(s_)) return false; dirty_ = false; return true; }
    void reboot_now() { host_.reboot(); }
    void mark_reboot_required() { reboot_required_ = true; }
    void mark_dirty() { dirty_ = true; }
    uint32_t frames() const { return parser_.frames(); }
    uint32_t bad_frames() const { return parser_.errors(); }
    uint32_t autosaved() const { return autosaved_; }               // sections written to flash on the firmware's own account
    uint32_t dropped_streams() const { return dropped_streams_; }
    uint16_t stream_rate(uint8_t id) const { for (int i = 0; i < kStreamCount; i++) if (kStreams[i].id == id) return rate_[i]; return 0; }

private:
    bool send(uint8_t type, uint8_t seq, const uint8_t* p, uint16_t n, bool droppable) {
        uint8_t out[kMaxPayload + kOverhead];
        const size_t len = encode(out, sizeof(out), type, seq, p, n);
        if (!len) return false;
        if (droppable && tx_.writable() < len) return false;
        tx_.write(out, len); return true;
    }
    void ack(uint8_t for_type, uint8_t for_seq, Err e) { const uint8_t p[3] = {for_type, for_seq, (uint8_t)e}; send(MSG_ACK, for_seq, p, 3, false); }
    void send_section(uint8_t seq, uint8_t id) {
        const SectionInfo* si = find_section(id); uint8_t b[kMaxPayload];
        b[0] = id; b[1] = (uint8_t)si->size; b[2] = (uint8_t)(si->size >> 8); memcpy(b + 3, section_ptr(s_, id), si->size);
        send(MSG_SECTION, seq, b, (uint16_t)(si->size + 3), false);
    }

    void handle(const Frame& f) {
        host_.on_frame();
        switch (f.type) {
        case MSG_HELLO: {
            uint8_t b[128]; size_t n = 0; b[n++] = kProtocolVersion; b[n++] = role_; b[n++] = maj_; b[n++] = min_; b[n++] = patch_;
            for (int i = 0; i < 4; i++) b[n++] = (uint8_t)(kSchemaHash >> (8 * i));
            for (int i = 0; i < 4; i++) b[n++] = (uint8_t)(features_ >> (8 * i));
            for (const char* c = name_; *c && n < 13 + 40; c++) b[n++] = (uint8_t)*c;
            if (mcu_) {
                b[n++] = 0;
                for (const char* c = mcu_; *c && n < sizeof(b) - 33; c++) b[n++] = (uint8_t)*c;
                b[n++] = 0; memcpy(b + n, pins_, sizeof(pins_)); n += sizeof(pins_);
            }
            send(MSG_DEVICE_INFO, f.seq, b, (uint16_t)n, false); break; }
        case MSG_SECTION_GET:
            if (f.len != 1 || !find_section(f.payload[0])) { ack(f.type, f.seq, ERR_BAD_SECTION); break; }
            send_section(f.seq, f.payload[0]); break;
        case MSG_SECTION_SET: {
            if (f.len < 1) { ack(f.type, f.seq, ERR_BAD_ARG); break; }
            const SectionInfo* si = find_section(f.payload[0]);
            if (!si) { ack(f.type, f.seq, ERR_BAD_SECTION); break; }
            if (f.len != si->size + 1) { ack(f.type, f.seq, ERR_SIZE_MISMATCH); break; }
            if (host_.armed()) { ack(f.type, f.seq, ERR_ARMED); break; }
            memcpy(section_ptr(s_, si->id), f.payload + 1, si->size);
            validate_section(s_, si->id);
            apply_rules(s_, si->id);                           // e.g. PID gains computed from the tuning sliders
            dirty_ = true;
            if (!host_.apply_live(si->id)) reboot_required_ = true;
            send_section(f.seq, si->id); break; }
        case MSG_SETTINGS_SAVE:
            if (host_.armed()) { ack(f.type, f.seq, ERR_ARMED); break; }
            if (!store_ || !store_->save(s_)) { ack(f.type, f.seq, ERR_STORAGE); break; }
            dirty_ = false; ack(f.type, f.seq, ERR_OK); break;
        case MSG_SETTINGS_DEFAULTS: {
            if (f.len != 1) { ack(f.type, f.seq, ERR_BAD_ARG); break; }
            if (host_.armed()) { ack(f.type, f.seq, ERR_ARMED); break; }
            if (f.payload[0] == 0xFF) { defaults(s_); for (const SectionInfo& si : kSections) if (!host_.apply_live(si.id)) reboot_required_ = true; }
            else { if (!defaults_section(s_, f.payload[0])) { ack(f.type, f.seq, ERR_BAD_SECTION); break; } if (!host_.apply_live(f.payload[0])) reboot_required_ = true; }
            dirty_ = true; ack(f.type, f.seq, ERR_OK); break; }
        case MSG_REBOOT:
            if (host_.armed()) { ack(f.type, f.seq, ERR_ARMED); break; }
            ack(f.type, f.seq, ERR_OK); host_.reboot(); break;
        case MSG_STREAM_CFG: {
            if (f.len != 3) { ack(f.type, f.seq, ERR_BAD_ARG); break; }
            const uint16_t rate = (uint16_t)(f.payload[1] | (f.payload[2] << 8));
            if (f.payload[0] == 0xFF) { for (auto& r : rate_) r = 0; ack(f.type, f.seq, ERR_OK); break; }
            int idx = -1; for (int i = 0; i < kStreamCount; i++) if (kStreams[i].id == f.payload[0]) idx = i;
            if (idx < 0) { ack(f.type, f.seq, ERR_BAD_ARG); break; }
            rate_[idx] = rate > kStreams[idx].max_rate ? kStreams[idx].max_rate : rate;
            next_[idx] = host_.now_ms(); ack(f.type, f.seq, ERR_OK); break; }
        case MSG_COMMAND: {
            if (f.len < 1) { ack(f.type, f.seq, ERR_BAD_ARG); break; }
            uint8_t res[200]; uint16_t rlen = 0;
            const Err e = host_.command(f.payload[0], f.payload + 1, (uint16_t)(f.len - 1), res + 2, rlen);
            res[0] = f.payload[0]; res[1] = (uint8_t)e;
            if (host_.take_settings_changed()) dirty_ = true;
            send(MSG_CMD_RESULT, f.seq, res, (uint16_t)(rlen + 2), false); break; }
        default: ack(f.type, f.seq, ERR_UNKNOWN); break;
        }
    }

    Settings& s_; SettingsStore* store_; Host& host_; Transport& tx_; Role role_; uint8_t maj_, min_, patch_; const char* name_; uint32_t features_;
    const char* mcu_ = nullptr; uint8_t pins_[32] = {};
    Parser parser_; uint16_t rate_[kStreamCount] = {}; uint32_t next_[kStreamCount] = {}; uint8_t stream_seq_ = 0, log_seq_ = 0;
    bool dirty_ = false, reboot_required_ = false; uint32_t dropped_streams_ = 0, autosaved_ = 0;
};

} // namespace wbp
