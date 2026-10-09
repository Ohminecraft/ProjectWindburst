/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <esp_partition.h>
#include "bb_flash.h"

// ============================================================================
// The blackbox's flash region on the ESP32: a data partition of the on-board flash, used RAW (see bb_flash.h for why not a
// file system).
//   1. a partition labelled "wblog"  (partitions/wblog_*.csv in the repository: one application slot, the rest for the log)
//   2. otherwise the partition the stock Arduino tables call "spiffs" (1.4 .. 3.4 MB): the firmware uses no file system, so it
//      is free. A region that does not look like ours or blank is NOT written to until the user erases it from the app.
// Every call blocks the caller and, while it runs, the other CPU core as well (flash cache off): reads ~0.1 ms, a 256-byte
// program well under 1 ms, an erase tens of ms per 4 kB sector. bb::Service only erases and reads on the ground.
// ============================================================================
class EspPartitionStore : public bb::BlockStore {
public:
    bool begin() {
        part_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "wblog");
        if (!part_) part_ = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, nullptr);
        return part_ != nullptr;
    }
    uint32_t size() const override { return part_ ? (uint32_t)(part_->size / bb::kSector) * bb::kSector : 0; }
    bool read(uint32_t addr, uint8_t* buf, uint32_t n) override { return part_ && esp_partition_read(part_, addr, buf, n) == ESP_OK; }
    bool write(uint32_t addr, const uint8_t* data, uint32_t n) override { return part_ && esp_partition_write(part_, addr, data, n) == ESP_OK; }
    bool erase(uint32_t addr, uint32_t n) override { return part_ && esp_partition_erase_range(part_, addr, n) == ESP_OK; }
    const char* name() const override { return part_ ? part_->label : "none"; }

private:
    const esp_partition_t* part_ = nullptr;
};
