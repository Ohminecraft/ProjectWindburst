/*
 * This file is part of Windburst Project
 */

#pragma once
#include "platform.h"

#include <Arduino.h>                // (on a PC: test/arduino_stub, which plays the ESP32's HardwareSerial)
#if WB_PLATFORM_STM32
#include "pinmap.h"
#endif

// ============================================================================
// A UART of the flight controller (receiver, GPS, optical flow) with its pins chosen at run time.
//
//   port::SerialPort       ESP32: the core's HardwareSerial (Serial1 ...); STM32: a Uart whose instance (USART1, UART4 ...)
//                          is whatever peripheral the two pins belong to - on an STM32 a UART cannot be put on just any pin.
//   uart_begin(...)        pins, baud, frame format, inverted lines (SBUS)
//   uart_set_baud(...)     another baud on the same pins (GPS auto baud)
//   uart_read(...)         up to n bytes that are already there, never waits
//
// STM32 notes (STM32duino 3.0):
//   * the receive buffer is SERIAL_RX_BUFFER_SIZE for every port (platformio.ini sets 1024: 24 ms of CRSF at 420 kbaud);
//   * the driver needs a TX pin. A receiver wired with RX only gets a TX pin of the same UART that nothing else uses (not USB,
//     not SWD) for the start, then the transmitter is switched off and that pin is an input again;
//   * there is no error callback: uartOverflows() / uartLineErrors() stay 0 on this chip.
// ============================================================================
namespace port {

#if !WB_PLATFORM_STM32
using SerialPort = HardwareSerial;

inline bool uart_begin(SerialPort& u, uint32_t baud, uint32_t config, uint8_t rx, uint8_t tx, bool invert, size_t rx_buffer) {
    if (rx_buffer) u.setRxBufferSize(rx_buffer);
    u.begin(baud, config, rx == 255 ? -1 : (int)rx, tx == 255 ? -1 : (int)tx, invert);
    return true;
}
inline void uart_set_baud(SerialPort& u, uint32_t baud) { u.flush(); u.updateBaudRate(baud); }
inline size_t uart_read(SerialPort& u, uint8_t* buf, size_t n) { return u.read(buf, n); }
inline const char* uart_name(SerialPort&) { return ""; }

#else
class SerialPort : public Uart {
public:
    SerialPort() : Uart(NC, NC) { _rx_invert = _tx_invert = _data_invert = false; }
    void set_invert(bool on) { _rx_invert = _tx_invert = on; }
    bool started = false;
    uint32_t config = SERIAL_8N1;
    bool tx_borrowed = false;                                 // TX pin only lent for the start (see above)
    PinName tx_pin = NC;
};

inline const char* uart_name(SerialPort& u) {
    if (!u.started) return "";
    const USART_TypeDef* i = u.getHandle()->Instance;
#define WB_UART_NAME(X) if (i == X) return #X;
#if defined(USART1)
    WB_UART_NAME(USART1)
#endif
#if defined(USART2)
    WB_UART_NAME(USART2)
#endif
#if defined(USART3)
    WB_UART_NAME(USART3)
#endif
#if defined(UART4)
    WB_UART_NAME(UART4)
#endif
#if defined(UART5)
    WB_UART_NAME(UART5)
#endif
#if defined(USART6)
    WB_UART_NAME(USART6)
#endif
#if defined(UART7)
    WB_UART_NAME(UART7)
#endif
#if defined(UART8)
    WB_UART_NAME(UART8)
#endif
#if defined(UART9)
    WB_UART_NAME(UART9)
#endif
#if defined(USART10)
    WB_UART_NAME(USART10)
#endif
#if defined(USART11)
    WB_UART_NAME(USART11)
#endif
#if defined(UART12)
    WB_UART_NAME(UART12)
#endif
#if defined(LPUART1)
    WB_UART_NAME(LPUART1)
#endif
#undef WB_UART_NAME
    return "UART";
}

// Who else uses a pin (FlightApp sets it from the "pins" settings): the TX pin lent to an RX-only UART must not be one of them,
// nor USB (PA11 / PA12) or SWD (PA13 / PA14).
inline bool (*g_pin_taken)(uint8_t pin) = nullptr;
inline PinName uart_free_tx(void* inst) {
    for (const PinMap* m = PinMap_UART_TX; m->pin != NC; m++) {
        if (m->peripheral != inst || (uint32_t)m->pin > 0xFF) continue;      // (ALT entries do not fit a pin byte)
        const uint8_t p = (uint8_t)m->pin;
        if (p >= 11 && p <= 14) continue;
        if (g_pin_taken && g_pin_taken(p)) continue;
        return m->pin;
    }
    return NC;
}

inline bool uart_begin(SerialPort& u, uint32_t baud, uint32_t config, uint8_t rx, uint8_t tx, bool invert, size_t /*rx_buffer: fixed size here*/) {
    if (u.started) { u.end(); u.started = false; }
    if (rx == 255) return false;
    const PinName prx = (PinName)rx;
    PinName ptx = tx == 255 ? NC : (PinName)tx;
    u.tx_borrowed = false;
    if (ptx == NC) {                                          // RX only: lend the UART's own TX pin for the start
        void* inst = pinmap_peripheral(prx, PinMap_UART_RX);
        if (!inst) return false;
        ptx = uart_free_tx(inst);
        if (ptx == NC) return false;                          // every TX pin of that UART is used by something else
        u.tx_borrowed = true;
    }
    u.setRx(prx); u.setTx(ptx); u.tx_pin = ptx;
    u.set_invert(invert);
    u.config = config;
    u.begin(baud, (uint16_t)config);
    if (!(bool)u) return false;
    if (u.tx_borrowed) {                                      // transmitter off, the lent pin back to an input
        UART_HandleTypeDef* h = u.getHandle();
        h->Instance->CR1 &= ~USART_CR1_TE;
        pinMode(pinNametoDigitalPin(ptx), INPUT);
    }
    u.started = true;
    return true;
}
inline void uart_set_baud(SerialPort& u, uint32_t baud) {
    if (!u.started) return;
    u.flush(); u.end();
    u.begin(baud, (uint16_t)u.config);
    if (u.tx_borrowed) { u.getHandle()->Instance->CR1 &= ~USART_CR1_TE; pinMode(pinNametoDigitalPin(u.tx_pin), INPUT); }
}
inline size_t uart_read(SerialPort& u, uint8_t* buf, size_t n) {
    size_t k = 0;
    while (k < n) { const int c = u.read(); if (c < 0) break; buf[k++] = (uint8_t)c; }
    return k;
}
#endif

} // namespace port
