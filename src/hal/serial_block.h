/*
 * This file is part of Windburst Project
 */

#pragma once

#include <Arduino.h>
#include "../port/uart.h"

// ============================================================================
// Bytes from a HardwareSerial in blocks. On Arduino-ESP32 3.x every read() of ONE byte takes the port's mutex, then the
// driver's, walks the ring buffer and checks it for overflow (and available() takes the mutex again): several microseconds of
// flash code per byte. A CRSF receiver at 500 Hz is 13 000 bytes a second - read one by one that is a good part of what the
// sensor thread has, the driver's buffer fills up, the sticks arrive late and bytes are lost (what the Receiver tab showed as
// "UART buffer overflows"). read(buf, n) costs the same once for the whole block.
//   SerialBlock<64> in; ... int c = in.get(serial);   // -1 = nothing
// ============================================================================
template <size_t N> class SerialBlock {
public:
    int get(port::SerialPort& s) {
        if (i_ >= n_) {
            int a = s.available(); if (a <= 0) return -1;
            if (a > (int)N) a = (int)N;
            n_ = port::uart_read(s, buf_, (size_t)a); i_ = 0;
            if (n_ == 0) return -1;
        }
        return buf_[i_++];
    }
    void clear() { i_ = n_ = 0; }
    size_t pending() const { return n_ - i_; }
private:
    uint8_t buf_[N]; size_t i_ = 0, n_ = 0;
};
