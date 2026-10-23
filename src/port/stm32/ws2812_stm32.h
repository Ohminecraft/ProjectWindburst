/*
 * This file is part of Windburst Project
 */

#pragma once
#include <Arduino.h>
#include "HardwareTimer.h"
#include "dshot_stm32.h"

// ============================================================================
// WS2812 / SK6812 on an STM32 (Betaflight's ws2811 driver): pins.led must be a timer channel (CH1 .. CH4). The timer runs at
// 800 kHz; at each compare event of that channel a DMA writes the next bit's compare value (led::timer_rows in hal/led/ws2812.h,
// portable and tested). A picture of 32 LEDs takes 0.96 ms on the wire and no CPU after the start. The timer is the strip's alone
// (port::pwm_reserve_timer): not the one of a motor, a servo or the buzzer. The DMA is the same per-channel request (TIMx_CHy)
// bidirectional DShot uses (port/stm32/dshot_stm32.h).
// NOT confirmed on hardware.
// ============================================================================
namespace led {

class TimStrip {
public:
    static constexpr int kMaxLeds = 32;
    // why begin() said no (nullptr: it did not)
    const char* why() const { return why_; }
    bool begin(uint8_t pin) {
        uint32_t ch = 0;
        TIM_TypeDef* t = stm32dshot::timer_of(pin, ch);
        if (!t) { why_ = "the LED pin has no timer channel (STM32: pick a TIMx_CHy pin)"; return false; }
        if (!port::pwm_reserve_timer(t)) { why_ = "the LED pin's timer is taken (motors, a servo or the buzzer)"; return false; }
        if (!dma_.claim(t, (int)ch)) { why_ = "no DMA for the LED pin's timer channel"; return false; }
        ht_ = new HardwareTimer(t);
        period_ = (ht_->getTimerClkFreq() + kTimerHz / 2u) / kTimerHz;
        t->CR1 = 0; t->DIER = 0; t->PSC = 0; t->ARR = period_ - 1u;
        stm32dshot::channel_output(t, (int)ch, false);
        (&t->CCR1)[ch - 1] = 0;
        const PinName pn = (PinName)pin;
        pin_function(pn, pinmap_function(pn, PinMap_TIM));
#if defined(TIM_BDTR_MOE)
        if (IS_TIM_BREAK_INSTANCE(t)) t->BDTR |= TIM_BDTR_MOE;
#endif
        t->CR1 = TIM_CR1_ARPE; t->EGR = TIM_EGR_UG; t->CR1 |= TIM_CR1_CEN;
        t->DIER |= 1u << (8 + ch);                                     // CCyDE: a DMA request each bit
        if (!rows_) rows_ = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)rows_for(kMaxLeds));      // (3 kB, only with a strip)
        if (!rows_) { why_ = "no memory for the LED strip's rows"; return false; }
        tim_ = t; ch_ = (int)ch; why_ = nullptr;
        return true;
    }
    void show(const app::Rgb* c, int n) {
        if (!tim_ || n <= 0 || dma_.busy()) return;                    // (the previous picture is still going out: skip this one)
        if (n > kMaxLeds) n = kMaxLeds;
        const int k = timer_rows(c, n, period_, rows_, rows_for(kMaxLeds));
        if (k) dma_.go(&(&tim_->CCR1)[ch_ - 1], rows_, (uint16_t)k, true);
    }
private:
    TIM_TypeDef* tim_ = nullptr; HardwareTimer* ht_ = nullptr; stm32dshot::Dma dma_; int ch_ = 0; uint32_t period_ = 0;
    uint32_t* rows_ = nullptr;
    const char* why_ = nullptr;
};
using Strip = TimStrip;

} // namespace led
