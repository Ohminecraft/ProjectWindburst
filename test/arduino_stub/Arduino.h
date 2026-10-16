// Minimal Arduino stand-in so src/hal/bus/*.h and IMUAccGyro compile and run on a PC (test_manager.cpp).
#pragma once
#include <functional>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
// Function-like macros the real Arduino core defines globally; any identifier with these names breaks silently on the device.
#define bit(b) (1UL << (b))
#define bitRead(v, b) (((v) >> (b)) & 0x01)
#define bitSet(v, b) ((v) |= (1UL << (b)))
#define bitClear(v, b) ((v) &= ~(1UL << (b)))
#define sq(x) ((x) * (x))
#define radians(d) ((d) * 0.017453292519943295)
#define degrees(r) ((r) * 57.29577951308232)
#define word(h, l) (((h) << 8) | (l))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define constrain(a, lo, hi) ((a) < (lo) ? (lo) : ((a) > (hi) ? (hi) : (a)))
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define MSBFIRST 1
#define SPI_MODE3 3
// Simulated clock. Default: frozen at 0 (the unit tests pass their own time stamps). The firmware-in-the-loop tests advance it and
// set g_stub_delay_advances so that delay() really takes time, exactly like on the board.
#if defined(WB_STUB_ATOMIC_CLOCK)                 // test_threads: several threads read the clock
#include <atomic>
inline std::atomic<uint64_t> g_stub_us{0};
#else
inline uint64_t g_stub_us = 0;
#endif
inline bool g_stub_delay_advances = false;
inline void delay(uint32_t ms) { if (g_stub_delay_advances) g_stub_us += (uint64_t)ms * 1000u; }
inline void delayMicroseconds(uint32_t us) { if (g_stub_delay_advances) g_stub_us += us; }
inline uint32_t micros() { return (uint32_t)g_stub_us; }
inline uint32_t g_stub_millis_cost_us = 0;                      // a test can let every millis() call take time (the control cycle reads it once, between the IMU and the core)
inline uint32_t millis() { if (g_stub_millis_cost_us) g_stub_us += g_stub_millis_cost_us; return (uint32_t)(g_stub_us / 1000u); }
inline void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val);   // defined by each test program: CS edges delimit SPI transactions

// Called for every access to a slow device (I2C transaction, UART byte, ADC conversion): test_threads uses it to prove that
// the firmware never waits for a device while it holds the lock of the flight task.
inline void (*g_stub_io_hook)() = nullptr;
inline void stub_io() { if (g_stub_io_hook) g_stub_io_hook(); }

// ---- ADC, PWM (LEDC) and plain outputs: the test sets what a pin reads and looks at what the firmware drove
inline uint32_t g_stub_adc_mv[64] = {};                         // analogReadMilliVolts(pin)
inline uint32_t g_stub_adc_reads = 0;
inline uint32_t analogReadMilliVolts(uint8_t pin) { stub_io(); g_stub_adc_reads++; return pin < 64 ? g_stub_adc_mv[pin] : 0; }
inline int8_t digitalPinToAnalogChannel(uint8_t pin) { return (pin >= 1 && pin <= 20) ? (int8_t)(pin - 1) : (int8_t)-1; }   // ESP32-S3: GPIO1..20
struct StubPwm { bool attached = false; uint32_t freq = 0, duty = 0; uint8_t bits = 0; };
inline StubPwm g_stub_pwm[64];
inline int g_stub_pwm_free = 8;                                 // LEDC channels left (the ESP32-S3 has 8)
inline bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t bits) {
    if (pin >= 64 || g_stub_pwm_free <= 0) return false;
    if (!g_stub_pwm[pin].attached) g_stub_pwm_free--;
    g_stub_pwm[pin] = StubPwm{true, freq, 0, bits}; return true;
}
inline bool ledcWrite(uint8_t pin, uint32_t duty) { if (pin >= 64 || !g_stub_pwm[pin].attached) return false; g_stub_pwm[pin].duty = duty; return true; }
inline bool ledcDetach(uint8_t pin) { if (pin >= 64 || !g_stub_pwm[pin].attached) return false; g_stub_pwm[pin] = StubPwm(); g_stub_pwm_free++; return true; }
inline uint8_t g_stub_gpio[64] = {};                            // last level written with digitalWrite (filled by the program's digitalWrite)
// ---- GPIO inputs with an interrupt (PPM / PWM receivers): the test changes a pin's level at a given time, the handler runs
//      with the clock set to that time (as if the interrupt fired then)
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define INPUT_PULLUP 0x05
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
struct StubIsr { void (*fn)(void*) = nullptr; void* arg = nullptr; int mode = 0; };
inline StubIsr g_stub_isr[64];
inline uint8_t g_stub_pin_in[64] = {};
inline void attachInterruptArg(uint8_t pin, void (*fn)(void*), void* arg, int mode) { if (pin < 64) g_stub_isr[pin] = StubIsr{fn, arg, mode}; }
inline void detachInterrupt(uint8_t pin) { if (pin < 64) g_stub_isr[pin] = StubIsr(); }
inline int digitalRead(uint8_t pin) { return pin < 64 ? g_stub_pin_in[pin] : 0; }
inline void stub_pin_edge(uint8_t pin, uint8_t level, uint64_t at_us) {
    if (pin >= 64 || g_stub_pin_in[pin] == level) return;
    g_stub_pin_in[pin] = level;
    const StubIsr& h = g_stub_isr[pin];
    if (!h.fn || !((level && (h.mode & RISING)) || (!level && (h.mode & FALLING)))) return;
    const uint64_t now = g_stub_us; g_stub_us = at_us; h.fn(h.arg); g_stub_us = now;
}

// ---- UART: a HardwareSerial is wired to a simulated device (GPS module, receiver) by the test
#define SERIAL_8N1 0x800001c
#define SERIAL_8E2 0x800003e
struct StubUartDevice {
    virtual ~StubUartDevice() = default;
    virtual int dev_read() = 0;                                  // next byte the device sends, -1 = nothing
    virtual int dev_pending() { return 0; }                      // bytes waiting after that one, if the device knows (what available() reports)
    virtual void dev_write(const uint8_t* d, size_t n) = 0;      // bytes the firmware sends to the device
    virtual void dev_config(uint32_t /*baud*/, uint32_t /*format*/, bool /*inverted*/) {}
};
typedef enum { UART_NO_ERROR, UART_BREAK_ERROR, UART_BUFFER_FULL_ERROR, UART_FIFO_OVF_ERROR, UART_FRAME_ERROR, UART_PARITY_ERROR } hardwareSerial_error_t;
class HardwareSerial {
public:
    StubUartDevice* dev = nullptr; uint32_t baud = 0, format = 0; int rx_pin = -1, tx_pin = -1; bool inverted = false, started = false;
    size_t rx_buffer = 256; uint8_t rx_fifo_full = 0; std::function<void(hardwareSerial_error_t)> on_error;
    void setRxBufferSize(size_t n) { rx_buffer = n; }
    bool setRxFIFOFull(uint8_t n) { rx_fifo_full = n; return true; }
    void onReceiveError(std::function<void(hardwareSerial_error_t)> f) { on_error = f; }
    void stub_error(hardwareSerial_error_t e) { if (on_error) on_error(e); }      // (tests: what the driver's event task would report)
    void setTxBufferSize(size_t) {}
    void begin(uint32_t b, uint32_t fmt = SERIAL_8N1, int rx = -1, int tx = -1, bool inv = false) { baud = b; format = fmt; rx_pin = rx; tx_pin = tx; inverted = inv; started = true; have_ = false; if (dev) dev->dev_config(b, fmt, inv); }
    void updateBaudRate(uint32_t b) { baud = b; if (dev) dev->dev_config(b, format, inverted); }
    void flush() {}
    int available() { stub_io(); if (!have_ && dev && started) { const int c = dev->dev_read(); if (c >= 0) { peek_ = (uint8_t)c; have_ = true; } } return have_ ? 1 + (dev ? dev->dev_pending() : 0) : 0; }
    int read() { single_reads++; if (!available()) return -1; have_ = false; return peek_; }
    size_t read(uint8_t* buf, size_t n) { block_reads++; size_t k = 0; while (k < n && available()) { have_ = false; buf[k++] = peek_; } return k; }
    uint32_t single_reads = 0, block_reads = 0;                   // (tests: how the firmware reads)
    size_t write(const uint8_t* d, size_t n) { stub_io(); if (dev && started) dev->dev_write(d, n); return n; }
    size_t write(uint8_t c) { return write(&c, 1); }
    int tx_room = 1024;                                           // (tests: what availableForWrite() says - a slow UART is a small number)
    int availableForWrite() { return started ? tx_room : 0; }
private:
    bool have_ = false; uint8_t peek_ = 0;
};
inline HardwareSerial Serial, Serial0, Serial1, Serial2;      // (Serial0: UART0 on a board whose `Serial` is the USB port)
struct StubEsp {
    int restarts = 0; void restart() { restarts++; }
    // memory as the board reports it (the tests set these): internal heap and PSRAM, bytes
    uint32_t heap_size = 320u * 1024u, heap_free = 240u * 1024u, psram_size = 0, psram_free = 0;
    uint32_t getHeapSize() const { return heap_size; } uint32_t getFreeHeap() const { return heap_free; }
    uint32_t getPsramSize() const { return psram_size; } uint32_t getFreePsram() const { return psram_free; }
};
inline StubEsp ESP;
