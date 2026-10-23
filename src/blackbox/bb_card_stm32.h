/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include "bb_card.h"
#include "../config/hardware_config.h"
#include "../config/ram_profile.h"
#include "../port/rtos.h"

// ============================================================================
// The SD card on an STM32 (SdFat by Bill Greiman: FAT16 / FAT32 / exFAT), as the bb::FileStore the flight log writes to.
//
//   SPI mode only: CS, SCK, MOSI (card DI), MISO (card DO) - the pins of one SPI peripheral (SPI1 / SPI2 / SPI3: the STM32's
//   pin map decides which; app Pins tab sd_cs / sd_clk / sd_cmd / sd_d0). SDMMC (SDIO on the F4, SDMMC on the H5) is not used
//   yet: the setting blackbox.sd_mode = SDMMC is refused with a reason.
//   Files: directory /wblog, the same names as on the ESP32 (bb_card.h); exFAT cards work as they are.
//
// SdCardTaskStm32 is the writer task (bb_card.h: CardLog::work), the lowest priority of the four (below wb_link).
// NOT confirmed on hardware.
// ============================================================================
class SdFatFileStore : public bb::FileStore {
public:
    void configure(uint8_t mode, uint8_t mhz) override { mode_ = mode; mhz_ = mhz < 1 ? 1 : (mhz > 50 ? 50 : mhz); }

    bool mount() override {
        unmount();
        uint8_t pin[4]; hw_.getSdCardPins(pin);
        if (mode_ != 0) { why_ = "SDMMC is not done on the STM32 yet: set the card to SPI (blackbox.sd_mode) and wire CS, SCK, MOSI, MISO"; return false; }
        if (pin[0] == GPIO_UNUSED || pin[1] == GPIO_UNUSED || pin[2] == GPIO_UNUSED || pin[3] == GPIO_UNUSED) { why_ = "SD card pins not set (app: Pins tab, sd_cs / sd_clk / sd_cmd / sd_d0)"; return false; }
        if (!spi_) {
            const PinName mosi = (PinName)pin[2], miso = (PinName)pin[3], sck = (PinName)pin[1];
            if (!pinmap_peripheral(mosi, PinMap_SPI_MOSI) || pinmap_peripheral(mosi, PinMap_SPI_MOSI) != pinmap_peripheral(miso, PinMap_SPI_MISO)
                || pinmap_peripheral(mosi, PinMap_SPI_MOSI) != pinmap_peripheral(sck, PinMap_SPI_SCLK)) { why_ = "the SD card's SCK / MOSI / MISO are not the pins of one SPI peripheral"; return false; }
            spi_ = new SPIClass(pinNametoDigitalPin(mosi), pinNametoDigitalPin(miso), pinNametoDigitalPin(sck));
        }
        if (!f_) f_ = new Fs();                                        // (the file system's state: only when the card is used)
        const SdSpiConfig cfg(pinNametoDigitalPin((PinName)pin[0]), DEDICATED_SPI, SD_SCK_MHZ(mhz_), spi_);
        if (!f_->sd.begin(cfg)) { why_ = "no usable SD card on SPI (card inserted? FAT32 / exFAT formatted? wiring? lower blackbox.sd_mhz)"; return false; }
        if (!f_->sd.exists(bb::kCardDir) && !f_->sd.mkdir(bb::kCardDir)) { why_ = "cannot create the log directory on the card (write protected?)"; unmount(); return false; }
        mounted_ = true; why_ = "";
        return true;
    }
    void unmount() override {
        if (!f_) return;
        close_write(); if (f_->rd.isOpen()) f_->rd.close();
        rd_number_ = 0;
        if (mounted_) f_->sd.end();
        mounted_ = false;
    }
    bool alive() override { return mounted_ && f_->sd.exists(bb::kCardDir); }
    uint64_t total_bytes() override { return mounted_ ? (uint64_t)f_->sd.clusterCount() * f_->sd.bytesPerCluster() : 0; }
    uint64_t free_bytes() override {
        if (!mounted_) return 0;
        const int32_t f = f_->sd.freeClusterCount();                    // (scans the FAT on FAT32: a second or two on a big card)
        return f > 0 ? (uint64_t)f * f_->sd.bytesPerCluster() : 0;
    }
    bool list(ListFn fn, void* ctx) override {
        if (!mounted_) return false;
        FsFile dir = f_->sd.open(bb::kCardDir);
        if (!dir || !dir.isDirectory()) return false;
        FsFile f; char name[48];
        while (f.openNext(&dir, O_RDONLY)) {
            if (!f.isDirectory() && f.getName(name, sizeof(name))) { const uint32_t n = bb::card_file_number(name); if (n) fn(ctx, n, (uint32_t)f.fileSize()); }
            f.close();
        }
        dir.close();
        return true;
    }
    bool open_write(uint32_t number) override {
        if (!mounted_) return false;
        close_write(); if (f_->rd.isOpen()) { f_->rd.close(); rd_number_ = 0; }
        char p[40]; path(number, p, sizeof(p));
        return f_->wr.open(p, O_WRONLY | O_CREAT | O_TRUNC);
    }
    bool write(const uint8_t* d, uint32_t n) override { return f_ && f_->wr.isOpen() && f_->wr.write(d, n) == n; }
    bool sync() override { return f_ && f_->wr.isOpen() && f_->wr.sync(); }
    void close_write() override { if (f_ && f_->wr.isOpen()) f_->wr.close(); }
    bool read(uint32_t number, uint32_t offset, uint8_t* buf, uint32_t n) override {
        if (!mounted_) return false;
        if (!f_->rd.isOpen() || rd_number_ != number) {                  // a download reads one file from start to end: keep it open
            if (f_->rd.isOpen()) f_->rd.close();
            char p[40]; path(number, p, sizeof(p));
            if (!f_->rd.open(p, O_RDONLY)) { rd_number_ = 0; return false; }
            rd_number_ = number;
        }
        if (f_->rd.curPosition() != offset && !f_->rd.seekSet(offset)) return false;
        return f_->rd.read(buf, n) == (int)n;
    }
    bool remove(uint32_t number) override {
        if (!mounted_) return false;
        if (f_->rd.isOpen() && rd_number_ == number) { f_->rd.close(); rd_number_ = 0; }
        char p[40]; path(number, p, sizeof(p));
        return f_->sd.remove(p);
    }
    const char* name() const override { return "sd-spi"; }
    const char* why() const override { return why_; }

private:
    static void path(uint32_t number, char* out, size_t cap) {
        const int n = snprintf(out, cap, "%s/", bb::kCardDir);
        bb::card_file_name(number, out + n, cap - (size_t)n);
    }
    HardwareConfig hw_;
    SPIClass* spi_ = nullptr;
    struct Fs { SdFs sd; FsFile wr, rd; };
    Fs* f_ = nullptr;
    uint32_t rd_number_ = 0;
    uint8_t mode_ = 0, mhz_ = 20; bool mounted_ = false; const char* why_ = "";
};

// Before the scheduler runs (setup(): one thread) the lock does nothing - a FreeRTOS call there would mask SysTick and stop
// delay() for the rest of setup() (main_stm32.cpp). The mutex is made by the first lock() once the tasks run.
class RtosLockStm32 : public bb::Lock {
public:
    void lock() override {
        if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
        if (!m_) { taskENTER_CRITICAL(); if (!m_) m_ = xSemaphoreCreateMutex(); taskEXIT_CRITICAL(); }
        if (m_) xSemaphoreTake(m_, portMAX_DELAY);
    }
    void unlock() override { if (m_ && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) xSemaphoreGive(m_); }
private:
    SemaphoreHandle_t m_ = nullptr;
};

class SdCardTaskStm32 {
public:
    explicit SdCardTaskStm32(bb::CardLog& card) : card_(card) {}
    bool start() { return port::task_create(&SdCardTaskStm32::entry, "wb_sd", WB_LOW_RAM ? 3072 : 6144, this, (UBaseType_t)1, &handle_, 0); }
    // the write buffer: 32 kB, 8 kB on a low-RAM chip (config/ram_profile.h)
    static uint32_t give_buffer(bb::CardLog& card) {
        uint32_t size = WB_LOW_RAM ? 8u * 1024u : 32u * 1024u;
        uint8_t* mem = (uint8_t*)malloc(size);
        if (!mem) size = 0;
        card.set_buffer(mem, size);
        return size;
    }
private:
    static void entry(void* self) { static_cast<SdCardTaskStm32*>(self)->run(); }
    void run() {
        for (;;) {
            if (card_.enabled() && !card_.buffer_size()) give_buffer(card_);
            const bool busy = card_.work(millis());
            vTaskDelay(busy ? 1 : 4);
        }
    }
    bb::CardLog& card_; TaskHandle_t handle_ = nullptr;
};
