/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// ============================================================================
// ESC passthrough: the "4-way interface" that BLHeliSuite / BLHeliSuite32 / ESC Configurator talk to through a flight
// controller, and the BLHeli bootloader protocol it speaks on the motor signal wires. Written from the protocol as Betaflight
// implements it (io/serial_4way.c, io/serial_4way_avrootloader.c - interface version 20.0.06, protocol 108), portable: it runs
// on a PC against a simulated ESC (test/test_esc4way.cpp).
//
// How a session goes:
//   the PC tool asks over MSP (app/msp_bridge.h) for MSP_SET_PASSTHROUGH; the firmware stops the motor output, answers with
//   the number of ESCs and hands the link to Esc4Way::run(). From then on the link carries 4-way frames:
//     PC -> FC   0x2F cmd addr_hi addr_lo n param[n (0 = 256)] crc16_hi crc16_lo        (CRC16-XMODEM over everything before)
//     FC -> PC   0x2E cmd addr_hi addr_lo n param[n] ack crc16_hi crc16_lo
//   and the FC turns each request into bootloader commands on ONE motor wire: a half-duplex serial line at 19200 baud, 8N1,
//   idle high, that the FC and the ESC drive in turn (the FC bit-bangs it; between its bytes the pin is an input with pull-up).
//   cmd_InterfaceExit ends the session: run() returns, the caller restarts the firmware (the motor output starts afresh).
//
// Bootloader frames (FC -> ESC): command bytes + CRC16 (poly 0xA001, reflected; low byte first) once the ESC is connected.
// Answers: one ACK byte (0x30 = success), read data + CRC16 + ACK. Supported: the BLHeli bootloaders of SiLabs and Atmel ESCs
// (BLHeli / BLHeli_S / Bluejay) and of ARM ESCs (BLHeli_32, AM32). Not supported: the STK500v2 "SK" bootloader of old Atmel
// ESCs (Betaflight only with USE_SERIAL_4WAY_SK_BOOTLOADER).
//
// Port (all ESC calls take the ESC index 0..count-1):
//   int  host_read();                     next byte from the PC, -1 = none yet (never waits)
//   void host_write(const uint8_t*, size_t);
//   void host_flush();                    everything written so far goes out
//   void host_wait();                     nothing from the PC: sleep about a millisecond (other tasks run)
//   bool host_alive();                    false: the link is gone, run() gives up
//   void esc_input(int), esc_output(int), esc_hi(int), esc_lo(int); bool esc_is_hi(int);
//   uint32_t micros();
//   void lock(); void unlock();           around one byte on the ESC wire, and slices of 200 us of waiting for one (on the board:
//                                         a critical section - the bit timing must not be interrupted for more than a few us)
// ============================================================================
namespace esc4way {

// ---- 4-way interface (PC side)
enum Cmd : uint8_t {
    CMD_REMOTE_ESCAPE = 0x2E, CMD_LOCAL_ESCAPE = 0x2F,
    CMD_TEST_ALIVE = 0x30, CMD_PROTOCOL_GET_VERSION = 0x31, CMD_INTERFACE_GET_NAME = 0x32, CMD_INTERFACE_GET_VERSION = 0x33,
    CMD_INTERFACE_EXIT = 0x34, CMD_DEVICE_RESET = 0x35, CMD_DEVICE_INIT_FLASH = 0x37, CMD_DEVICE_ERASE_ALL = 0x38,
    CMD_DEVICE_PAGE_ERASE = 0x39, CMD_DEVICE_READ = 0x3A, CMD_DEVICE_WRITE = 0x3B, CMD_DEVICE_C2CK_LOW = 0x3C,
    CMD_DEVICE_READ_EEPROM = 0x3D, CMD_DEVICE_WRITE_EEPROM = 0x3E, CMD_INTERFACE_SET_MODE = 0x3F, CMD_DEVICE_VERIFY = 0x40,
};
enum Ack : uint8_t { ACK_OK = 0x00, ACK_I_INVALID_CMD = 0x02, ACK_I_INVALID_CRC = 0x03, ACK_I_VERIFY_ERROR = 0x04,
                     ACK_I_INVALID_CHANNEL = 0x08, ACK_I_INVALID_PARAM = 0x09, ACK_D_GENERAL_ERROR = 0x0F };
enum Mode : uint8_t { MODE_SIL_BLB = 1, MODE_ATM_BLB = 2, MODE_SK = 3, MODE_ARM_BLB = 4 };
constexpr uint8_t kProtocolVersion = 108;
constexpr uint8_t kVersionHi = 200, kVersionLo = 6;          // 20.0.06 = 20006 -> 200, 06 (Betaflight's numbers: BLHeliSuite checks them)
constexpr char kInterfaceName[] = "m4wFCIntf";

// ---- bootloader (ESC side)
enum BlCmd : uint8_t { BL_RUN = 0x00, BL_PROG_FLASH = 0x01, BL_ERASE_FLASH = 0x02, BL_READ_FLASH_SIL = 0x03, BL_VERIFY_FLASH_ARM = 0x04,
                       BL_READ_EEPROM = 0x04, BL_PROG_EEPROM = 0x05, BL_READ_FLASH_ATM = 0x07, BL_KEEP_ALIVE = 0xFD,
                       BL_SET_BUFFER = 0xFE, BL_SET_ADDRESS = 0xFF };
enum BlAck : uint8_t { BR_SUCCESS = 0x30, BR_ERROR_VERIFY = 0xC0, BR_ERROR_COMMAND = 0xC1, BR_ERROR_CRC = 0xC2, BR_NONE = 0xFF };
constexpr uint32_t kBitUs = 52;                              // 19200 baud
constexpr uint32_t kStartBitTimeoutUs = 2000;                // one wait for an answer byte (Betaflight START_BIT_TIMEOUT_MS)
constexpr uint32_t kSliceUs = 200;                           // the longest stretch the lock is held while waiting for a start bit

inline uint16_t crc_xmodem(uint16_t crc, uint8_t b) {        // the PC frames
    crc ^= (uint16_t)b << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}
inline uint16_t crc_bl(uint16_t crc, uint8_t b) {            // the bootloader frames
    for (int i = 0; i < 8; i++) { const bool x = ((b ^ crc) & 1) != 0; crc >>= 1; if (x) crc ^= 0xA001; b >>= 1; }
    return crc;
}

// What a device's 4 info bytes say (DeviceInfo: [0] signature low, [1] signature high, [2] boot version, [3] interface mode)
inline bool is_silabs(const uint8_t* d) { const uint16_t w = (uint16_t)(d[1] << 8 | d[0]); return w > 0xE800 && w < 0xF900; }
inline bool is_atmel(const uint8_t* d) { const uint16_t w = (uint16_t)(d[1] << 8 | d[0]); return w == 0x9307 || w == 0x930A || w == 0x930F || w == 0x940B; }
inline bool is_arm(const uint8_t* d) { return d[1] > 0x00 && d[1] < 0x90 && d[0] == 0x06; }      // BLHeli_32 / AM32: id high 0x01..0x8F, low always 0x06

template <class Port> class Interface {
public:
    struct Stats { uint32_t frames = 0, bad_crc = 0, esc_errors = 0; };
    Interface(Port& p, int esc_count) : p_(p), count_(esc_count) {}

    // The ESC lines become inputs, pulled high (the bootloader's idle level), and stay so between commands.
    void begin() { for (int c = 0; c < count_; c++) { p_.esc_input(c); p_.esc_hi(c); } }
    // Serves the PC until cmd_InterfaceExit (true) or until the link is gone (false).
    bool run() {
        for (;;) {
            Frame f;
            if (!read_frame(f)) return false;
            const bool exit = handle(f);
            if (exit) { p_.host_flush(); return true; }
        }
    }
    const Stats& stats() const { return stats_; }
    int selected() const { return esc_; }

private:
    struct Frame { uint8_t cmd = 0, ah = 0, al = 0, n = 0; uint8_t param[256]; uint16_t len = 0; bool crc_ok = false; };
    Port& p_; int count_; int esc_ = 0; uint8_t mode_ = MODE_ARM_BLB; uint8_t info_[4] = {0, 0, 0, 0}; Stats stats_;
    bool connected() const { return info_[0] > 0; }
    void disconnect() { info_[0] = info_[1] = info_[2] = info_[3] = 0; }

    // ---------------------------------------------------------------- PC link
    bool host_byte(uint8_t& b) {
        for (;;) { const int c = p_.host_read(); if (c >= 0) { b = (uint8_t)c; return true; } if (!p_.host_alive()) return false; p_.host_wait(); }
    }
    bool read_frame(Frame& f) {
        uint8_t b;
        do { if (!host_byte(b)) return false; } while (b != CMD_LOCAL_ESCAPE);
        uint16_t crc = crc_xmodem(0, b);
        uint8_t hdr[4]; for (int i = 0; i < 4; i++) { if (!host_byte(hdr[i])) return false; crc = crc_xmodem(crc, hdr[i]); }
        f.cmd = hdr[0]; f.ah = hdr[1]; f.al = hdr[2]; f.n = hdr[3]; f.len = f.n ? f.n : 256;
        for (uint16_t i = 0; i < f.len; i++) { if (!host_byte(f.param[i])) return false; crc = crc_xmodem(crc, f.param[i]); }
        uint8_t ch, cl; if (!host_byte(ch) || !host_byte(cl)) return false;
        f.crc_ok = (uint16_t)(ch << 8 | cl) == crc;
        return true;
    }
    void reply(uint8_t cmd, uint8_t ah, uint8_t al, const uint8_t* out, uint16_t n_out, uint8_t ack) {
        uint8_t hdr[5] = {CMD_REMOTE_ESCAPE, cmd, ah, al, (uint8_t)(n_out & 0xFF)};       // n_out 256 is sent as 0
        uint16_t crc = 0; for (uint8_t v : hdr) crc = crc_xmodem(crc, v);
        p_.host_write(hdr, 5);
        uint8_t dummy = 0; if (n_out == 0) { out = &dummy; n_out = 1; hdr[4] = 1; }        // (never happens: every answer carries at least one byte)
        for (uint16_t i = 0; i < n_out; i++) crc = crc_xmodem(crc, out[i]);
        p_.host_write(out, n_out);
        crc = crc_xmodem(crc, ack);
        const uint8_t tail[3] = {ack, (uint8_t)(crc >> 8), (uint8_t)crc};
        p_.host_write(tail, 3);
        p_.host_flush();
    }

    // ---------------------------------------------------------------- one request
    bool handle(Frame& f) {
        stats_.frames++;
        uint8_t ack = ACK_OK, small[2] = {0, 0}; const uint8_t* out = small; uint16_t n_out = 1;
        uint8_t ah = f.ah, al = f.al; bool exit = false;
        if (!f.crc_ok) { ack = ACK_I_INVALID_CRC; stats_.bad_crc++; }
        else switch (f.cmd) {
        case CMD_TEST_ALIVE:
            if (connected() && !bl_keep_alive()) { ack = ACK_D_GENERAL_ERROR; disconnect(); }
            break;
        case CMD_PROTOCOL_GET_VERSION: small[0] = kProtocolVersion; break;
        case CMD_INTERFACE_GET_NAME: out = (const uint8_t*)kInterfaceName; n_out = (uint16_t)strlen(kInterfaceName); break;
        case CMD_INTERFACE_GET_VERSION: small[0] = kVersionHi; small[1] = kVersionLo; n_out = 2; break;
        case CMD_INTERFACE_EXIT: exit = true; break;
        case CMD_INTERFACE_SET_MODE:
            if (f.param[0] == MODE_SIL_BLB || f.param[0] == MODE_ATM_BLB || f.param[0] == MODE_ARM_BLB) mode_ = f.param[0]; else ack = ACK_I_INVALID_PARAM;
            break;
        case CMD_DEVICE_RESET:
            if (f.param[0] >= count_) { ack = ACK_I_INVALID_CHANNEL; break; }
            esc_ = f.param[0];
            bl_run();                                                      // the bootloader starts the ESC firmware
            if (f.al == 1) {                                               // ... and holding the line low 300 ms resets it outright
                p_.esc_output(esc_); p_.esc_lo(esc_); const uint32_t t0 = p_.micros(); while ((uint32_t)(p_.micros() - t0) < 300000u) p_.host_wait();
                p_.esc_hi(esc_); p_.esc_input(esc_);
            }
            disconnect();
            break;
        case CMD_DEVICE_INIT_FLASH:
            disconnect();
            if (f.param[0] >= count_) { ack = ACK_I_INVALID_CHANNEL; break; }
            esc_ = f.param[0];
            if (connect()) info_[3] = mode_; else { disconnect(); ack = ACK_D_GENERAL_ERROR; }
            out = info_; n_out = 4;
            break;
        case CMD_DEVICE_PAGE_ERASE:
            if (mode_ != MODE_SIL_BLB && mode_ != MODE_ARM_BLB) { ack = ACK_I_INVALID_CMD; break; }
            ah = mode_ == MODE_ARM_BLB ? (uint8_t)(f.param[0] << 2) : (uint8_t)(f.param[0] << 1);   // page * 1024 (ARM) / * 512 (SiLabs)
            al = 0;
            if (!bl_page_erase(ah, al)) ack = ACK_D_GENERAL_ERROR;
            small[0] = f.param[0];
            break;
        case CMD_DEVICE_READ: case CMD_DEVICE_READ_EEPROM: {
            const uint16_t n = f.param[0] ? f.param[0] : 256;
            const bool eeprom = f.cmd == CMD_DEVICE_READ_EEPROM;
            if (eeprom && mode_ != MODE_ATM_BLB) { ack = ACK_I_INVALID_CMD; break; }
            const uint8_t bl = eeprom ? BL_READ_EEPROM : mode_ == MODE_ATM_BLB ? BL_READ_FLASH_ATM : BL_READ_FLASH_SIL;
            if (bl_read(bl, ah, al, f.param, n)) { out = f.param; n_out = n; } else ack = ACK_D_GENERAL_ERROR;
            break; }
        case CMD_DEVICE_WRITE: case CMD_DEVICE_WRITE_EEPROM: {
            const bool eeprom = f.cmd == CMD_DEVICE_WRITE_EEPROM;
            if (eeprom && mode_ != MODE_ATM_BLB) { ack = mode_ == MODE_SIL_BLB ? ACK_I_INVALID_CMD : ACK_D_GENERAL_ERROR; break; }
            if (!bl_write(eeprom ? BL_PROG_EEPROM : BL_PROG_FLASH, ah, al, f.param, f.len, eeprom ? 3000000u : 500000u)) ack = ACK_D_GENERAL_ERROR;
            break; }
        case CMD_DEVICE_VERIFY: {
            if (mode_ != MODE_ARM_BLB) { ack = ACK_I_INVALID_CMD; break; }
            const uint8_t r = bl_verify(ah, al, f.param, f.len);
            ack = r == BR_SUCCESS ? ACK_OK : r == BR_ERROR_VERIFY ? ACK_I_VERIFY_ERROR : ACK_D_GENERAL_ERROR;
            break; }
        default: ack = ACK_I_INVALID_CMD; break;
        }
        if (ack == ACK_D_GENERAL_ERROR) stats_.esc_errors++;
        reply(f.cmd, ah, al, out, n_out, ack);
        return exit;
    }

    // ---------------------------------------------------------------- the ESC wire: one byte at 19200 baud
    // Answer byte: wait up to 2 ms for the start bit, sample each bit 3/4 into it (Betaflight's suart_getc_). false: nothing came,
    // or a broken frame. The wait is cut into slices of kSliceUs with the lock held: on the board the lock keeps interrupts away,
    // and a whole bootloader answer can take seconds of such waits (an erase) - interrupts get a chance between two slices.
    bool getc(uint8_t& b) {
        const uint32_t t0 = p_.micros();
        for (;;) {
            p_.lock();
            const uint32_t s0 = p_.micros();
            bool start = false;
            while (!(start = !p_.esc_is_hi(esc_))) {
                const uint32_t now = p_.micros();
                if ((uint32_t)(now - t0) >= kStartBitTimeoutUs || (uint32_t)(now - s0) >= kSliceUs) break;
            }
            if (start) {
                uint32_t t = p_.micros() + kBitUs * 3 / 4;                     // bit 0 is the start bit itself, sampled 39 us into it
                uint16_t bits = 0;
                for (int i = 0; i < 10; i++) {
                    while ((int32_t)(p_.micros() - t) < 0) {}
                    if (p_.esc_is_hi(esc_)) bits |= (uint16_t)(1u << i);
                    t += kBitUs;
                }
                p_.unlock();
                if ((bits & 1) || !(bits & (1u << 9))) return false;           // start bit high or stop bit low
                b = (uint8_t)(bits >> 1);
                return true;
            }
            p_.unlock();
            if ((uint32_t)(p_.micros() - t0) >= kStartBitTimeoutUs) return false;
        }
    }
    // One byte out: an idle (high) bit first, the start bit, 8 data bits LSB first, the stop bit (not waited for: the next byte's
    // leading idle bit, or the turn-around, completes it)
    void putc(uint8_t v) {
        uint16_t bits = (uint16_t)((uint16_t)v << 2) | 1u | (1u << 10);
        p_.lock();
        uint32_t t = p_.micros();
        for (;;) {
            if (bits & 1) p_.esc_hi(esc_); else p_.esc_lo(esc_);
            t += kBitUs; bits >>= 1;
            if (!bits) break;
            while ((int32_t)(p_.micros() - t) < 0) {}
        }
        p_.unlock();
    }
    void send(const uint8_t* d, size_t n) {
        p_.esc_output(esc_);
        uint16_t crc = 0;
        for (size_t i = 0; i < n; i++) { putc(d[i]); crc = crc_bl(crc, d[i]); }
        if (connected()) { putc((uint8_t)crc); putc((uint8_t)(crc >> 8)); }
        p_.esc_input(esc_);                                                // (at once, as Betaflight does: the pull-up holds the stop bit, the ESC may answer right after it)
    }
    // n data bytes (+ CRC + ACK once connected, + ACK only before). true when the ACK says success and the CRC fits.
    bool receive(uint8_t* d, uint16_t n) {
        uint16_t crc = 0;
        for (uint16_t i = 0; i < n; i++) { if (!getc(d[i])) return false; crc = crc_bl(crc, d[i]); }
        uint8_t ack = BR_NONE;
        if (connected()) {
            uint8_t lo, hi; if (!getc(lo) || !getc(hi) || !getc(ack)) return false;
            if ((uint16_t)(hi << 8 | lo) != crc) ack = BR_ERROR_CRC;
        } else if (!getc(ack)) return false;
        return ack == BR_SUCCESS;
    }
    uint8_t get_ack(uint32_t tries) {                                      // each try waits up to 2 ms for the start bit
        uint8_t a = BR_NONE;
        for (uint32_t i = 0; i < tries; i++) if (getc(a)) return a;
        return BR_NONE;
    }

    // ---------------------------------------------------------------- bootloader commands
    bool connect() {
        for (int attempt = 0; attempt < 3; attempt++) {
            static const uint8_t init[17] = {0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};   // (carries its own CRC)
            send(init, sizeof(init));
            uint8_t bi[8];
            if (!receive(bi, 8)) continue;
            if (bi[0] != '4' || bi[1] != '7' || bi[2] != '1') continue;   // "471x": a BLHeli bootloader
            uint8_t d[4] = {bi[5], bi[4], bi[3], 0};                      // signature low, high, boot version
            if (is_silabs(d)) mode_ = MODE_SIL_BLB; else if (is_atmel(d)) mode_ = MODE_ATM_BLB; else if (is_arm(d)) mode_ = MODE_ARM_BLB; else continue;
            info_[0] = d[0]; info_[1] = d[1]; info_[2] = d[2];
            return true;
        }
        return false;
    }
    bool bl_keep_alive() { const uint8_t c[2] = {BL_KEEP_ALIVE, 0}; send(c, 2); return get_ack(1) == BR_ERROR_COMMAND; }   // (the bootloader does not know it - that answer proves it is there)
    void bl_run() { const uint8_t c[2] = {BL_RUN, 0}; info_[0] = 1; send(c, 2); }                                         // (sent with a CRC, as Betaflight does)
    bool bl_set_address(uint8_t ah, uint8_t al) {
        if (ah == 0xFF && al == 0xFF) return true;
        const uint8_t c[4] = {BL_SET_ADDRESS, 0, ah, al}; send(c, 4);
        return get_ack(2) == BR_SUCCESS;
    }
    bool bl_set_buffer(const uint8_t* d, uint16_t n) {
        const uint8_t c[4] = {BL_SET_BUFFER, 0, (uint8_t)(n == 256 ? 1 : 0), (uint8_t)(n & 0xFF)}; send(c, 4);
        if (get_ack(2) != BR_NONE) return false;                          // (no answer to this one: the data follows)
        send(d, n);
        return get_ack(40) == BR_SUCCESS;
    }
    bool bl_read(uint8_t cmd, uint8_t ah, uint8_t al, uint8_t* d, uint16_t n) {
        if (!bl_set_address(ah, al)) return false;
        const uint8_t c[2] = {cmd, (uint8_t)(n & 0xFF)}; send(c, 2);
        return receive(d, n);
    }
    bool bl_write(uint8_t cmd, uint8_t ah, uint8_t al, const uint8_t* d, uint16_t n, uint32_t timeout_us) {
        if (!bl_set_address(ah, al) || !bl_set_buffer(d, n)) return false;
        const uint8_t c[2] = {cmd, 0x01}; send(c, 2);
        return get_ack(timeout_us / kStartBitTimeoutUs) == BR_SUCCESS;
    }
    bool bl_page_erase(uint8_t ah, uint8_t al) {
        if (!bl_set_address(ah, al)) return false;
        const uint8_t c[2] = {BL_ERASE_FLASH, 0x01}; send(c, 2);
        return get_ack(3000000u / kStartBitTimeoutUs) == BR_SUCCESS;
    }
    uint8_t bl_verify(uint8_t ah, uint8_t al, const uint8_t* d, uint16_t n) {
        if (!bl_set_address(ah, al) || !bl_set_buffer(d, n)) return BR_NONE;
        const uint8_t c[2] = {BL_VERIFY_FLASH_ARM, 0x01}; send(c, 2);
        return get_ack(40000u / kStartBitTimeoutUs);
    }
};

} // namespace esc4way
