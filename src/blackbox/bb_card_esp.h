/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <SD_MMC.h>          // unconditional: PlatformIO's library finder does not know SOC_* macros; the header guards itself on chips without SDMMC
#include "bb_card.h"
#include "../config/hardware_config.h"
#include "../hal/bus/bus_spi.h"

// ============================================================================
// The SD card on the ESP32 (Arduino SD / SD_MMC libraries = ESP-IDF FatFs), as the bb::FileStore the flight log writes to.
//
//   SPI mode    4 wires: CS, SCK, MOSI (card DI), MISO (card DO). Works with every card socket / breakout.
//               Uses the SECOND general-purpose SPI bus (the first one belongs to the IMU): the card never shares wires with the gyro.
//   SDMMC 1-bit 3 wires: CLK, CMD, D0 (the same three card pads as SCK, DI, DO). Faster and does not busy-wait.
//               The card needs pull-ups (10k to 3.3 V) on CMD and D0, and its DAT3 / CS pad held HIGH. ESP32-S3 / ESP32 only.
//
// Pins: HardwareConfig::getSdCardPins() (app: Pins tab, sd_cs / sd_clk / sd_cmd / sd_d0).
// Files: FAT16 / FAT32 (the Arduino-ESP32 build of FatFs has no exFAT: a 64 GB+ card must be re-formatted as FAT32), directory /wblog,
// long file names.
//
// SdCardTask is the writer task (see bb_card.h): core 0, lowest application priority, calls CardLog::work().
// ============================================================================
class SdFileStore : public bb::FileStore {
public:
    enum Mode : uint8_t { MODE_SPI, MODE_SDMMC_1BIT };
    void configure(uint8_t mode, uint8_t mhz) override { mode_ = mode; mhz_ = mhz < 1 ? 1 : (mhz > 40 ? 40 : mhz); }

    bool mount() override {
        unmount();
        uint8_t pin[4]; hw_.getSdCardPins(pin);
        if (pin[1] == GPIO_UNUSED || pin[2] == GPIO_UNUSED || pin[3] == GPIO_UNUSED) { why_ = "SD card pins not set (app: Pins tab, sd_clk / sd_cmd / sd_d0)"; return false; }
        if (mode_ == MODE_SPI) {
            if (pin[0] == GPIO_UNUSED) { why_ = "SD card CS pin not set (app: Pins tab, sd_cs)"; return false; }
            if (!spi_started_) { if (!spi_.begin((int8_t)pin[1], (int8_t)pin[3], (int8_t)pin[2], (int8_t)pin[0])) { why_ = "the second SPI bus did not start (already used by a barometer on SPI?)"; return false; } spi_started_ = true; }
            if (!SD.begin(pin[0], spi_, (uint32_t)mhz_ * 1000000u, "/sd", 4, false)) { why_ = "no usable SD card on SPI (card inserted? FAT32 formatted - not exFAT? wiring? lower blackbox.sd_mhz)"; return false; }
            fs_ = &SD; which_ = 1;
        } else {
#if defined(SOC_SDMMC_HOST_SUPPORTED)
            if (!SD_MMC.setPins(pin[1], pin[2], pin[3])) { why_ = "SDMMC does not accept these pins"; return false; }
            if (!SD_MMC.begin("/sdcard", true, false, (int)mhz_ * 1000, 4)) { why_ = "no usable SD card on SDMMC (card inserted? FAT32 formatted - not exFAT? pull-ups on CMD / D0, DAT3 high?)"; return false; }
            fs_ = &SD_MMC; which_ = 2;
#else
            why_ = "this chip has no SDMMC interface: use SPI mode"; return false;
#endif
        }
        if (!fs_->exists(bb::kCardDir) && !fs_->mkdir(bb::kCardDir)) { why_ = "cannot create the log directory on the card (write protected?)"; unmount(); return false; }
        why_ = "";
        return true;
    }
    void unmount() override {
        close_write(); if (rd_) rd_.close();
        rd_number_ = 0;
        if (which_ == 1) SD.end();
#if defined(SOC_SDMMC_HOST_SUPPORTED)
        if (which_ == 2) SD_MMC.end();
#endif
        which_ = 0; fs_ = nullptr;
    }
    bool alive() override { return fs_ && fs_->exists(bb::kCardDir); }
    uint64_t total_bytes() override {
        if (which_ == 1) return SD.totalBytes();
#if defined(SOC_SDMMC_HOST_SUPPORTED)
        if (which_ == 2) return SD_MMC.totalBytes();
#endif
        return 0;
    }
    uint64_t free_bytes() override {
        uint64_t t = 0, u = 0;
        if (which_ == 1) { t = SD.totalBytes(); u = SD.usedBytes(); }
#if defined(SOC_SDMMC_HOST_SUPPORTED)
        if (which_ == 2) { t = SD_MMC.totalBytes(); u = SD_MMC.usedBytes(); }
#endif
        return t > u ? t - u : 0;
    }
    bool list(ListFn fn, void* ctx) override {
        if (!fs_) return false;
        File dir = fs_->open(bb::kCardDir);
        if (!dir || !dir.isDirectory()) return false;
        for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
            if (!f.isDirectory()) { const uint32_t n = bb::card_file_number(f.name()); if (n) fn(ctx, n, (uint32_t)f.size()); }
            f.close();
        }
        dir.close();
        return true;
    }
    bool open_write(uint32_t number) override {
        if (!fs_) return false;
        close_write(); if (rd_) { rd_.close(); rd_number_ = 0; }
        char p[40]; path(number, p, sizeof(p));
        wr_ = fs_->open(p, FILE_WRITE);
        return (bool)wr_;
    }
    bool write(const uint8_t* d, uint32_t n) override { return wr_ && wr_.write(d, n) == n; }
    bool sync() override { if (!wr_) return false; wr_.flush(); return true; }    // flush() = fflush + fsync; a dead card shows up as a short write()
    void close_write() override { if (wr_) wr_.close(); }
    bool read(uint32_t number, uint32_t offset, uint8_t* buf, uint32_t n) override {
        if (!fs_) return false;
        if (!rd_ || rd_number_ != number) {                       // a download reads one file from start to end: keep it open
            if (rd_) rd_.close();
            char p[40]; path(number, p, sizeof(p));
            rd_ = fs_->open(p, FILE_READ); rd_number_ = number; rd_pos_ = 0;
            if (!rd_) { rd_number_ = 0; return false; }
        }
        if (rd_pos_ != offset && !rd_.seek(offset)) return false;
        const size_t got = rd_.read(buf, n);
        rd_pos_ = offset + (uint32_t)got;
        return got == n;
    }
    bool remove(uint32_t number) override {
        if (!fs_) return false;
        if (rd_ && rd_number_ == number) { rd_.close(); rd_number_ = 0; }
        char p[40]; path(number, p, sizeof(p));
        return fs_->remove(p);
    }
    const char* name() const override { return mode_ == MODE_SPI ? "sd-spi" : "sd-mmc"; }
    const char* why() const override { return why_; }

private:
    static void path(uint32_t number, char* out, size_t cap) {
        const int n = snprintf(out, cap, "%s/", bb::kCardDir);
        bb::card_file_name(number, out + n, cap - (size_t)n);
    }
    HardwareConfig hw_;
    SPIClass spi_{(int)SPI_DEVICE_1};
    fs::FS* fs_ = nullptr; File wr_, rd_;
    uint32_t rd_number_ = 0, rd_pos_ = 0;
    uint8_t mode_ = MODE_SPI, mhz_ = 20, which_ = 0; bool spi_started_ = false; const char* why_ = "";
};

class RtosLock : public bb::Lock {
public:
    RtosLock() { m_ = xSemaphoreCreateMutex(); }
    void lock() override { if (m_) xSemaphoreTake(m_, portMAX_DELAY); }
    void unlock() override { if (m_) xSemaphoreGive(m_); }
private:
    SemaphoreHandle_t m_ = nullptr;
};

class SdCardTask {
public:
    explicit SdCardTask(bb::CardLog& card) : card_(card) {}
    // Below the link task in importance, but on the same core 0: the flight loop on core 1 never waits for the card.
    bool start(int core = 0, int priority = 1) { return xTaskCreatePinnedToCore(&SdCardTask::entry, "wb_sd", 8192, this, priority, &handle_, core) == pdPASS; }

    // The write buffer: 512 kB of PSRAM when the module has some, otherwise 32 kB of internal RAM.
    static uint32_t give_buffer(bb::CardLog& card) {
        uint8_t* mem = nullptr; uint32_t size = 0;
        if (psramFound()) { size = 512u * 1024u; mem = (uint8_t*)ps_malloc(size); }
        if (!mem) { size = 32u * 1024u; mem = (uint8_t*)malloc(size); }
        if (!mem) size = 0;
        card.set_buffer(mem, size);
        return size;
    }

private:
    static void entry(void* self) { static_cast<SdCardTask*>(self)->run(); }
    void run() {
        for (;;) {
            if (card_.enabled() && !card_.buffer_size()) give_buffer(card_);     // the card was selected in the app after boot
            const bool busy = card_.work(millis());
            vTaskDelay(busy ? 1 : 4);
        }
    }
    bb::CardLog& card_; TaskHandle_t handle_ = nullptr;
};
