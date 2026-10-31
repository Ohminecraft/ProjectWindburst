/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../port/platform.h"
#include "../proto/settings_store.h"

// The settings in the ESP32's NVS (Preferences), one key per section record (proto/settings_store.h). The flight controller
// (main.cpp) and the DIY transmitter (main_tx.cpp) both keep theirs here.
#if WB_PLATFORM_ESP32
#include <Preferences.h>
class NvsStorage : public wbp::Storage {
public:
    bool begin(const char* ns = "windburst") { ok_ = prefs_.begin(ns, false); return ok_; }
    int read(const char* key, uint8_t* buf, size_t cap) override {
        if (!ok_ || !prefs_.isKey(key)) return -1;
        const size_t len = prefs_.getBytesLength(key);
        if (len == 0 || len > cap) return -1;
        return (int)prefs_.getBytes(key, buf, len);
    }
    bool write(const char* key, const uint8_t* data, size_t n) override { return ok_ && prefs_.putBytes(key, data, n) == n; }
private:
    Preferences prefs_; bool ok_ = false;
};
#endif  // WB_PLATFORM_ESP32
