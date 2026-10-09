/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stdint.h>

// ============================================================================
// Storage: what bb::Service needs from a place that keeps flight logs. Two exist:
//   FlashLog (bb_flash.h) - sessions in a raw region of the on-board flash
//   CardLog  (bb_card.h)  - one file per session on an SD card
//
// The app downloads by ADDRESS: every storage presents its stored sessions as one byte range [0, used()), and a session is
// (session_start(i), session_size(i)) inside it. For the flash that is the region itself; for the card it is the listed files
// placed one after the other.
// All calls are made by ONE task (the link task).
// ============================================================================
namespace bb {

class Storage {
public:
    virtual ~Storage() = default;
    virtual bool ok() const = 0;                         // there and usable
    virtual uint32_t capacity() const = 0;               // bytes, saturated at 4 GB - 1
    virtual uint32_t used() const = 0;                   // end of the download address range
    virtual uint32_t capacity_kb() const = 0;
    virtual uint32_t used_kb() const = 0;
    virtual bool needs_erase() const = 0;                // holds something that is not ours: nothing is written until the user erases
    virtual bool io_error() const = 0;
    virtual bool full() const = 0;
    virtual bool writing() const = 0;                    // a session is open

    // ---- sessions (index 0 = oldest listed)
    virtual int sessions() const = 0;                    // all of them
    virtual int listed() const = 0;                      // those that can be downloaded (the newest ones)
    virtual uint32_t session_start(int i) const = 0;
    virtual uint32_t session_size(int i) const = 0;

    // ---- writing
    virtual bool verify_step() = 0;                      // background check of the free space, one small step; true while more remains
    virtual uint32_t writable() const = 0;               // bytes the running session (or a new one) can still take
    virtual bool start_session() = 0;
    virtual bool append(const uint8_t* d, uint32_t n) = 0;   // false = cannot take it right now (nothing of the call was taken)
    virtual uint32_t staged() const = 0;                 // appended but not yet on the medium
    virtual uint32_t pump(bool all) = 0;                 // one bounded step of moving staged bytes to the medium
    virtual void end_session() = 0;

    // ---- erase everything (on the ground), one bounded step per call
    virtual bool erase_start() = 0;
    virtual bool erase_step() = 0;                       // true while more remains
    virtual uint8_t erase_percent() const = 0;

    // ---- reading back
    virtual bool read(uint32_t addr, uint8_t* buf, uint32_t n) = 0;
    virtual const char* store_name() const = 0;
    // highest fill level (percent) of the write buffer since the last call; 0 for a storage without one
    virtual uint8_t take_buffer_peak() { return 0; }
    // The settings changed: is this storage the selected one, and (SD card) how is it wired. May be called at any time.
    virtual void configure_device(bool /*selected*/, uint8_t /*mode*/, uint8_t /*mhz*/) {}
};

} // namespace bb
