/*
 * This file is part of Windburst Project
 */

#pragma once

#include <stddef.h>
#include <math.h>
#include "wbp_schema.h"

// ============================================================================
// Settings whose unit changed: a record of an older firmware is converted when it is loaded (SettingsStore::load, right after
// the record is copied in, before the range check). Only a record that HAS the old field and not yet the new one is converted;
// a shorter record (neither) keeps the new defaults, today's record is taken as it is.
//
// 0.44 - Betaflight's microseconds instead of percent (the old fields stay in their place as old_*, unused):
//   receiver.deadband (% of the half travel)  -> deadband, yaw_deadband (us)       x 5
//   tx_sticks.deadband (% of the half travel) -> deadband (us)                     x 5
//   althold mid / low / max stick (%)         -> us                                1000 + x 10;  deadband (% of the travel) x 10
//   general.fs_fallback_throttle (% of hover) -> fs_throttle (us)                  1000 + % x 3.7 (the hover guess 37 %)
// Portable: unit-tested in test/test_proto.cpp.
// ============================================================================
namespace wbp {

// the record (`stored` bytes) holds the old field [off_old, off_old + 1) and ends before the new one
inline bool has_old_only(uint16_t stored, size_t off_old, size_t off_new) { return stored >= off_old + 1 && stored <= off_new; }

inline void upgrade_section(Settings& s, uint8_t id, uint16_t stored) {
    switch (id) {
    case SEC_GENERAL:
        if (has_old_only(stored, offsetof(S_general, old_fs_fallback), offsetof(S_general, fs_throttle)))
            s.general.fs_throttle = (uint16_t)(1000 + lroundf(s.general.old_fs_fallback * 3.7f));
        break;
    case SEC_RECEIVER:
        if (has_old_only(stored, offsetof(S_receiver, old_deadband), offsetof(S_receiver, deadband))) {
            const uint16_t us = (uint16_t)(s.receiver.old_deadband * 5u);
            s.receiver.deadband = (uint8_t)(us > 255 ? 255 : us); s.receiver.yaw_deadband = s.receiver.deadband;
        }
        break;
    case SEC_ALTHOLD:
        if (has_old_only(stored, offsetof(S_althold, old_max_stick), offsetof(S_althold, mid_stick))) {
            S_althold& h = s.althold;
            h.mid_stick = (uint16_t)(1000 + h.old_mid_stick * 10); h.deadband = (uint8_t)(h.old_deadband * 10 > 255 ? 255 : h.old_deadband * 10);
            h.low_stick = (uint16_t)(1000 + h.old_low_stick * 10); h.max_stick = (uint16_t)(1000 + h.old_max_stick * 10);
        }
        break;
    case SEC_TX_STICKS:
        if (has_old_only(stored, offsetof(S_tx_sticks, old_deadband), offsetof(S_tx_sticks, deadband))) {
            const uint16_t us = (uint16_t)(s.tx_sticks.old_deadband * 5u); s.tx_sticks.deadband = (uint8_t)(us > 255 ? 255 : us);
        }
        break;
    default: break;
    }
}

} // namespace wbp
