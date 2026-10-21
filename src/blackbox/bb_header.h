/*
 * This file is part of Windburst Project
 */

#pragma once

#include "bb_format.h"
#include "../proto/wbp_schema.h"

// The firmware's own lines of a log header: who wrote it and the complete settings (each section as hex of its wire image,
// to be decoded with the schema whose hash is given). Called by bb::Service in the link task, which owns the Settings object.
namespace bb {

struct FirmwareHeader {
    wbp::Settings* settings; const char* name; uint8_t maj, min, patch;
    static void write(TextOut& t, void* ctx) {
        const FirmwareHeader& h = *static_cast<const FirmwareHeader*>(ctx);
        t.add("firmware %s %u.%u.%u\n", h.name, (unsigned)h.maj, (unsigned)h.min, (unsigned)h.patch);
        t.add("schema 0x%08lX\n", (unsigned long)wbp::kSchemaHash);
        for (const wbp::SectionInfo& si : wbp::kSections) {
            if (si.id == wbp::SEC_WIFI) continue;            // (the WiFi password stays out of a file that is passed around)
            if (si.id == wbp::SEC_OSD) continue;             // (the screen layout says nothing about the flight; 150 bytes less per session start)
            t.add("set %s ", si.name); t.hex(wbp::section_ptr(*h.settings, si.id), si.size); t.add("\n");
        }
    }
};

} // namespace bb
