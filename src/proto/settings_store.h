#pragma once
#include <stdint.h>
#include <string.h>
#include "wbp_schema.h"
#include "wbp_codec.h"
#include "settings_rules.h"

// ============================================================================
// Persistent settings. One storage record per section ("wbs01".."wbs12"), each = [format u8][size u16][data][crc16 of the rest].
// A record that is missing, has a different size (new firmware changed that section) or a bad CRC leaves ONLY that section at its
// defaults - the rest of the configuration survives a firmware update. Loaded values are range-validated (a corrupted flash can never
// produce an out-of-range PID or an invalid motor protocol).
// A section that only GREW AT ITS END (or was cut there) since an older firmware names the older sizes in the schema ("compat",
// up to two): a record of such a size is loaded as far as it goes and the new fields keep their defaults - a pin map survives new
// pins being added, also across two firmware versions.
// ============================================================================
namespace wbp {

class Storage {                                           // NVS on the ESP32 (Preferences), a std::map in the tests
public:
    virtual ~Storage() = default;
    virtual int read(const char* key, uint8_t* buf, size_t cap) = 0;      // bytes read, -1 if the key does not exist
    virtual bool write(const char* key, const uint8_t* data, size_t n) = 0;
};

class SettingsStore {
public:
    explicit SettingsStore(Storage& st) : st_(st) {}

    struct LoadReport { uint32_t loaded_mask = 0, defaulted_mask = 0, clamped_fields = 0, migrated_mask = 0; };   // bit = section id; migrated = loaded from an older, shorter / longer record

    LoadReport load(Settings& s) {
        LoadReport r; defaults(s);
        for (const SectionInfo& si : kSections) {
            uint8_t buf[kMaxPayload + 8]; char key[8]; make_key(key, si.id);
            const int n = st_.read(key, buf, sizeof(buf));
            uint16_t stored = 0;                              // the size the record was written with, if it is one we can read
            const uint16_t sizes[4] = {si.size, si.compat, si.compat2, si.compat3};
            for (const uint16_t want : sizes) {
                if (!want || stored) continue;
                if (n == (int)want + 5 && buf[0] == kFormat && (uint16_t)(buf[1] | (buf[2] << 8)) == want &&
                    crc16(buf, (size_t)want + 3) == (uint16_t)(buf[want + 3] | (buf[want + 4] << 8))) stored = want;
            }
            if (stored) {
                memcpy(section_ptr(s, si.id), buf + 3, stored < si.size ? stored : si.size);      // what an older record does not have keeps its default
                const int fx = validate_section(s, si.id); if (fx > 0) r.clamped_fields += (uint32_t)fx;
                apply_rules(s, si.id);
                r.loaded_mask |= 1u << si.id; if (stored != si.size) r.migrated_mask |= 1u << si.id;
            } else { defaults_section(s, si.id); r.defaulted_mask |= 1u << si.id; }
        }
        return r;
    }
    bool save(Settings& s) {
        bool all = true;
        for (const SectionInfo& si : kSections) all &= save_section(s, si.id);
        return all;
    }
    bool save_section(Settings& s, uint8_t id) {
        const SectionInfo* si = find_section(id); if (!si) return false;
        uint8_t buf[kMaxPayload + 8]; buf[0] = kFormat; buf[1] = (uint8_t)si->size; buf[2] = (uint8_t)(si->size >> 8);
        memcpy(buf + 3, section_ptr(s, id), si->size);
        const uint16_t c = crc16(buf, (size_t)si->size + 3); buf[si->size + 3] = (uint8_t)c; buf[si->size + 4] = (uint8_t)(c >> 8);
        char key[8]; make_key(key, id);
        return st_.write(key, buf, (size_t)si->size + 5);
    }

private:
    static constexpr uint8_t kFormat = 1;
    static void make_key(char* k, uint8_t id) { k[0] = 'w'; k[1] = 'b'; k[2] = 's'; k[3] = (char)('0' + id / 10); k[4] = (char)('0' + id % 10); k[5] = 0; }
    Storage& st_;
};

} // namespace wbp
