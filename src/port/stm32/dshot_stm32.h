/*
 * This file is part of Windburst Project
 */

#pragma once
#include <Arduino.h>
#include "HardwareTimer.h"
#include "../../config/fast_code.h"
#include "../../hal/motor/motor_hw.h"
#include "../../hal/motor/dshot_burst.h"
#include "../../hal/motor/dshot_direct.h"
#include "../pwm.h"

// ============================================================================
// DShot150 / 300 / 600 / 1200 on an STM32. Every motor pin must be a timer channel (CH1 .. CH4, not CHxN). Two ways:
//
// ONE-WAY: Betaflight's "DMA burst" (DSHOT_DMAR). The motors on one timer share one DMA transfer: the timer runs at the bit
//   rate, its update request (TIMx_UP) lets the DMA write the next row of compare values into TIMx->DMAR, which the timer
//   spreads over the motors' CCRs (hal/motor/dshot_burst.h: 16 bit rows + 2 low rows). The CPU fills 18 rows and starts the
//   transfers: no interrupt.
//
// BIDIRECTIONAL: the same sequence as the ESP32's (hal/motor/dshot_direct.h, tested on the PC), on a port made of the timer:
//   frame     each channel its own DMA (TIMx_CHy request), memory -> CCRy, the output inverted (idle HIGH, a bit is a low pulse)
//   alarm     TIM6, one-shot, a few microseconds after the frame: its interrupt hands the line to the ESC
//   release   the channels become input captures on both edges, the timer free-running (ARR 0xFFFF); the same DMA channel now
//             copies every captured counter value (CCRy -> memory)
//   next send the captures are turned into runs (time between two edges, levels alternating from the start bit's falling
//             edge) and decoded like the ESP32's (reply_from_runs); the channels become outputs again
//   This is Betaflight's way with bidirectional DShot on timers (pwm_output_dshot.c: dshotEnableChannels / input capture DMA).
//
// DMA by family:
//   F4  fixed streams / channels (RM0090 tables 42 / 43): TIMx_UP for the burst, TIMx_CHy for bidirectional, the first free
//       stream of each request's alternatives (TIM4_CH4 has none: not bidirectional on that pin)
//   G4  DMA1 channels 1 .. 4 through DMAMUX (DMA_REQUEST_TIMx_UP / _CHy)
//   H5  GPDMA1 channels 0 .. 3 (GPDMA1_REQUEST_TIMx_UP / _CHy)
//
// The motor timers are DShot's alone: a servo or the buzzer on another channel of the same timer is refused
// (port::pwm_reserve_timer). Bidirectional and the start-up check: none here (an STM32 pin cannot capture while it drives); a
// pin that does not answer shows as "not answering" in the Motor tab as on the ESP32.
// NOT confirmed on hardware.
// ============================================================================
namespace stm32dshot {

// ---- one DMA channel / stream: mem -> peripheral register (tx) or peripheral register -> mem (rx), 32-bit words
#if defined(STM32F4xx)
struct DmaRow { TIM_TypeDef* tim; uint8_t req; DMA_TypeDef* dma; uint8_t stream, channel; };      // req 0 = UP, 1 .. 4 = CHy
inline const DmaRow* dma_rows(int& n) {
    static const DmaRow rows[] = {
#if defined(TIM1)
        {TIM1, 0, DMA2, 5, 6}, {TIM1, 1, DMA2, 1, 6}, {TIM1, 1, DMA2, 3, 6}, {TIM1, 1, DMA2, 6, 0}, {TIM1, 2, DMA2, 2, 6}, {TIM1, 2, DMA2, 6, 0},
        {TIM1, 3, DMA2, 6, 6}, {TIM1, 3, DMA2, 6, 0}, {TIM1, 4, DMA2, 4, 6},
#endif
#if defined(TIM2)
        {TIM2, 0, DMA1, 1, 3}, {TIM2, 0, DMA1, 7, 3}, {TIM2, 1, DMA1, 5, 3}, {TIM2, 2, DMA1, 6, 3}, {TIM2, 3, DMA1, 1, 3}, {TIM2, 4, DMA1, 7, 3}, {TIM2, 4, DMA1, 6, 3},
#endif
#if defined(TIM3)
        {TIM3, 0, DMA1, 2, 5}, {TIM3, 1, DMA1, 4, 5}, {TIM3, 2, DMA1, 5, 5}, {TIM3, 3, DMA1, 7, 5}, {TIM3, 4, DMA1, 2, 5},
#endif
#if defined(TIM4)
        {TIM4, 0, DMA1, 6, 2}, {TIM4, 1, DMA1, 0, 2}, {TIM4, 2, DMA1, 3, 2}, {TIM4, 3, DMA1, 7, 2},
#endif
#if defined(TIM5)
        {TIM5, 0, DMA1, 0, 6}, {TIM5, 0, DMA1, 6, 6}, {TIM5, 1, DMA1, 2, 6}, {TIM5, 2, DMA1, 4, 6}, {TIM5, 3, DMA1, 0, 6}, {TIM5, 4, DMA1, 1, 6}, {TIM5, 4, DMA1, 3, 6},
#endif
#if defined(TIM8)
        {TIM8, 0, DMA2, 1, 7}, {TIM8, 1, DMA2, 2, 0}, {TIM8, 1, DMA2, 2, 7}, {TIM8, 2, DMA2, 3, 7}, {TIM8, 2, DMA2, 2, 0}, {TIM8, 3, DMA2, 4, 7}, {TIM8, 3, DMA2, 2, 0},
        {TIM8, 4, DMA2, 7, 7},
#endif
    };
    n = (int)(sizeof(rows) / sizeof(rows[0])); return rows;
}
struct Dma {
    DMA_Stream_TypeDef* s = nullptr; DMA_TypeDef* dma = nullptr; uint8_t stream = 0, channel = 0; uint16_t n_ = 0;
    static uint8_t& taken(DMA_TypeDef* d) { static uint8_t t1 = 0, t2 = 0; return d == DMA1 ? t1 : t2; }   // streams in use, a bit each
    bool claim(TIM_TypeDef* tim, int req) {
        int n = 0; const DmaRow* rows = dma_rows(n);
        for (int i = 0; i < n; i++) {
            const DmaRow& r = rows[i];
            if (r.tim != tim || r.req != req || (taken(r.dma) >> r.stream) & 1u) continue;
            dma = r.dma; stream = r.stream; channel = r.channel; taken(dma) |= (uint8_t)(1u << stream);
            if (dma == DMA1) __HAL_RCC_DMA1_CLK_ENABLE(); else __HAL_RCC_DMA2_CLK_ENABLE();
            s = (DMA_Stream_TypeDef*)((uint32_t)(uintptr_t)dma + 0x10u + 0x18u * stream);
            return true;
        }
        return false;
    }
    WB_FAST void stop() { s->CR &= ~DMA_SxCR_EN; while (s->CR & DMA_SxCR_EN) {} static const uint8_t sh[4] = {0, 6, 16, 22}; const uint32_t m = 0x3Du << sh[stream & 3u]; if (stream < 4) dma->LIFCR = m; else dma->HIFCR = m; }
    WB_FAST void go(volatile uint32_t* reg, uint32_t* mem, uint16_t n, bool to_reg) {
        stop(); n_ = n;
        s->PAR = (uint32_t)(uintptr_t)reg; s->M0AR = (uint32_t)(uintptr_t)mem; s->NDTR = n; s->FCR = 0;                  // direct mode
        s->CR = ((uint32_t)channel << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | (to_reg ? DMA_SxCR_DIR_0 : 0u) | DMA_SxCR_PL_1;
        s->CR |= DMA_SxCR_EN;
    }
    WB_FAST bool busy() const { return (s->CR & DMA_SxCR_EN) && s->NDTR; }
    WB_FAST uint16_t done() const { return (uint16_t)(n_ - s->NDTR); }                      // words moved so far
};
#elif defined(STM32G4xx) || defined(STM32H5xx)
inline uint32_t dma_request(TIM_TypeDef* tim, int req) {
#if defined(STM32G4xx)
#if defined(TIM1)
    if (tim == TIM1) { static const uint32_t r[5] = {DMA_REQUEST_TIM1_UP, DMA_REQUEST_TIM1_CH1, DMA_REQUEST_TIM1_CH2, DMA_REQUEST_TIM1_CH3, DMA_REQUEST_TIM1_CH4}; return r[req]; }
#endif
#if defined(TIM2)
    if (tim == TIM2) { static const uint32_t r[5] = {DMA_REQUEST_TIM2_UP, DMA_REQUEST_TIM2_CH1, DMA_REQUEST_TIM2_CH2, DMA_REQUEST_TIM2_CH3, DMA_REQUEST_TIM2_CH4}; return r[req]; }
#endif
#if defined(TIM3)
    if (tim == TIM3) { static const uint32_t r[5] = {DMA_REQUEST_TIM3_UP, DMA_REQUEST_TIM3_CH1, DMA_REQUEST_TIM3_CH2, DMA_REQUEST_TIM3_CH3, DMA_REQUEST_TIM3_CH4}; return r[req]; }
#endif
#if defined(TIM4)
    if (tim == TIM4) { static const uint32_t r[5] = {DMA_REQUEST_TIM4_UP, DMA_REQUEST_TIM4_CH1, DMA_REQUEST_TIM4_CH2, DMA_REQUEST_TIM4_CH3, DMA_REQUEST_TIM4_CH4}; return r[req]; }
#endif
#if defined(TIM5)
    if (tim == TIM5) { static const uint32_t r[5] = {DMA_REQUEST_TIM5_UP, DMA_REQUEST_TIM5_CH1, DMA_REQUEST_TIM5_CH2, DMA_REQUEST_TIM5_CH3, DMA_REQUEST_TIM5_CH4}; return r[req]; }
#endif
#if defined(TIM8)
    if (tim == TIM8) { static const uint32_t r[5] = {DMA_REQUEST_TIM8_UP, DMA_REQUEST_TIM8_CH1, DMA_REQUEST_TIM8_CH2, DMA_REQUEST_TIM8_CH3, DMA_REQUEST_TIM8_CH4}; return r[req]; }
#endif
#endif
#if defined(STM32H5xx)
#if defined(TIM1)
    if (tim == TIM1) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM1_UP, GPDMA1_REQUEST_TIM1_CH1, GPDMA1_REQUEST_TIM1_CH2, GPDMA1_REQUEST_TIM1_CH3, GPDMA1_REQUEST_TIM1_CH4}; return r[req]; }
#endif
#if defined(TIM2)
    if (tim == TIM2) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM2_UP, GPDMA1_REQUEST_TIM2_CH1, GPDMA1_REQUEST_TIM2_CH2, GPDMA1_REQUEST_TIM2_CH3, GPDMA1_REQUEST_TIM2_CH4}; return r[req]; }
#endif
#if defined(TIM3)
    if (tim == TIM3) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM3_UP, GPDMA1_REQUEST_TIM3_CH1, GPDMA1_REQUEST_TIM3_CH2, GPDMA1_REQUEST_TIM3_CH3, GPDMA1_REQUEST_TIM3_CH4}; return r[req]; }
#endif
#if defined(TIM4)
    if (tim == TIM4) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM4_UP, GPDMA1_REQUEST_TIM4_CH1, GPDMA1_REQUEST_TIM4_CH2, GPDMA1_REQUEST_TIM4_CH3, GPDMA1_REQUEST_TIM4_CH4}; return r[req]; }
#endif
#if defined(TIM5)
    if (tim == TIM5) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM5_UP, GPDMA1_REQUEST_TIM5_CH1, GPDMA1_REQUEST_TIM5_CH2, GPDMA1_REQUEST_TIM5_CH3, GPDMA1_REQUEST_TIM5_CH4}; return r[req]; }
#endif
#if defined(TIM8)
    if (tim == TIM8) { static const uint32_t r[5] = {GPDMA1_REQUEST_TIM8_UP, GPDMA1_REQUEST_TIM8_CH1, GPDMA1_REQUEST_TIM8_CH2, GPDMA1_REQUEST_TIM8_CH3, GPDMA1_REQUEST_TIM8_CH4}; return r[req]; }
#endif
#endif
    return 0xFFFFFFFFu;
}
#if defined(STM32G4xx)
struct Dma {
    DMA_Channel_TypeDef* c = nullptr; uint8_t index = 0; uint16_t n_ = 0;
    static uint8_t& used() { static uint8_t u = 0; return u; }       // DMA1 channels taken so far (from channel 1 up)
    bool claim(TIM_TypeDef* tim, int req) {
        const uint32_t r = dma_request(tim, req);
        if (r == 0xFFFFFFFFu || used() >= 6) return false;
        __HAL_RCC_DMAMUX1_CLK_ENABLE(); __HAL_RCC_DMA1_CLK_ENABLE();
        index = used()++;
        static DMA_Channel_TypeDef* const ch[6] = {DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6};
        static DMAMUX_Channel_TypeDef* const mx[6] = {DMAMUX1_Channel0, DMAMUX1_Channel1, DMAMUX1_Channel2, DMAMUX1_Channel3, DMAMUX1_Channel4, DMAMUX1_Channel5};
        c = ch[index]; mx[index]->CCR = r;
        return true;
    }
    WB_FAST void stop() { c->CCR &= ~DMA_CCR_EN; DMA1->IFCR = 0xFu << (4u * index); }
    WB_FAST void go(volatile uint32_t* reg, uint32_t* mem, uint16_t n, bool to_reg) {
        stop(); n_ = n;
        c->CPAR = (uint32_t)(uintptr_t)reg; c->CMAR = (uint32_t)(uintptr_t)mem; c->CNDTR = n;
        c->CCR = (to_reg ? DMA_CCR_DIR : 0u) | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL_1;
        c->CCR |= DMA_CCR_EN;
    }
    WB_FAST bool busy() const { return (c->CCR & DMA_CCR_EN) && c->CNDTR; }
    WB_FAST uint16_t done() const { return (uint16_t)(n_ - c->CNDTR); }
};
#else
struct Dma {
    DMA_Channel_TypeDef* c = nullptr; uint32_t req = 0; uint16_t n_ = 0;
    static uint8_t& used() { static uint8_t u = 0; return u; }       // GPDMA1 channels taken so far (from channel 0 up)
    bool claim(TIM_TypeDef* tim, int r) {
        req = dma_request(tim, r);
        if (req == 0xFFFFFFFFu || used() >= 6) return false;
        __HAL_RCC_GPDMA1_CLK_ENABLE();
        static DMA_Channel_TypeDef* const ch[6] = {GPDMA1_Channel0, GPDMA1_Channel1, GPDMA1_Channel2, GPDMA1_Channel3, GPDMA1_Channel4, GPDMA1_Channel5};
        c = ch[used()++];
        return true;
    }
    WB_FAST void stop() {
        if (c->CCR & DMA_CCR_EN) { c->CCR |= DMA_CCR_SUSP; for (int i = 0; i < 1000 && !(c->CSR & DMA_CSR_SUSPF); i++) {} c->CCR |= DMA_CCR_RESET; }
        c->CFCR = DMA_CFCR_TCF | DMA_CFCR_HTF | DMA_CFCR_DTEF | DMA_CFCR_ULEF | DMA_CFCR_USEF | DMA_CFCR_SUSPF | DMA_CFCR_TOF;
    }
    WB_FAST void go(volatile uint32_t* reg, uint32_t* mem, uint16_t n, bool to_reg) {
        stop(); n_ = n;
        if (to_reg) {                                                // memory -> the timer, paced by the timer (destination request)
            c->CTR1 = (2u << DMA_CTR1_SDW_LOG2_Pos) | DMA_CTR1_SINC | (2u << DMA_CTR1_DDW_LOG2_Pos);
            c->CTR2 = (req << DMA_CTR2_REQSEL_Pos) | DMA_CTR2_DREQ;
            c->CSAR = (uint32_t)(uintptr_t)mem; c->CDAR = (uint32_t)(uintptr_t)reg;
        } else {                                                     // the timer -> memory, paced by the timer (source request)
            c->CTR1 = (2u << DMA_CTR1_SDW_LOG2_Pos) | (2u << DMA_CTR1_DDW_LOG2_Pos) | DMA_CTR1_DINC;
            c->CTR2 = (req << DMA_CTR2_REQSEL_Pos);
            c->CSAR = (uint32_t)(uintptr_t)reg; c->CDAR = (uint32_t)(uintptr_t)mem;
        }
        c->CBR1 = (uint32_t)n * 4u; c->CLLR = 0;
        c->CCR = DMA_CCR_EN;
    }
    WB_FAST bool busy() const { return (c->CCR & DMA_CCR_EN) && !(c->CSR & DMA_CSR_IDLEF); }
    WB_FAST uint16_t done() const { return (uint16_t)(n_ - (c->CBR1 & DMA_CBR1_BNDT) / 4u); }
};
#endif
#else
struct Dma {                                                         // (another family: no DShot)
    bool claim(TIM_TypeDef*, int) { return false; }
    void stop() {}
    void go(volatile uint32_t*, uint32_t*, uint16_t, bool) {}
    bool busy() const { return false; }
    uint16_t done() const { return 0; }
};
#endif

// the timer of a pin's DShot output, and its channel (1 .. 4); nullptr when the pin has none
inline TIM_TypeDef* timer_of(uint8_t pin, uint32_t& channel) {
    const PinName pn = (PinName)pin;
    TIM_TypeDef* tim = (TIM_TypeDef*)pinmap_peripheral(pn, PinMap_TIM);
    if (!tim) return nullptr;
    const int f = pinmap_function(pn, PinMap_TIM);
    channel = STM_PIN_CHANNEL(f);
    if (STM_PIN_INVERTED(f)) return nullptr;                         // a complementary output (CHxN): not with this scheme
    return channel >= 1 && channel <= 4 ? tim : nullptr;
}

// channel c (1 .. 4) of a timer as a PWM output (mode 1, preload; inverted: active low) or as an input capture on both edges
inline void channel_output(TIM_TypeDef* t, int c, bool inverted) {
    volatile uint32_t* ccmr = c <= 2 ? &t->CCMR1 : &t->CCMR2;
    const uint32_t sh = (c == 2 || c == 4) ? 8u : 0u, e = 4u * (uint32_t)(c - 1);
    t->CCER &= ~(0xFu << e);                                         // (CCxS can only change with the channel off)
    *ccmr = (*ccmr & ~(0xFFu << sh)) | (((6u << 4) | (1u << 3)) << sh);       // CCxS = output, OCxM = PWM mode 1, OCxPE
    t->CCER |= (1u | (inverted ? 2u : 0u)) << e;                     // CCxE (+ CCxP: active low)
}
inline void channel_capture(TIM_TypeDef* t, int c) {
    volatile uint32_t* ccmr = c <= 2 ? &t->CCMR1 : &t->CCMR2;
    const uint32_t sh = (c == 2 || c == 4) ? 8u : 0u, e = 4u * (uint32_t)(c - 1);
    t->CCER &= ~(0xFu << e);
    *ccmr = (*ccmr & ~(0xFFu << sh)) | ((1u | (2u << 4)) << sh);     // CCxS = TIx, ICxF = 2 (4 samples: a glitch filter), no prescaler
    t->CCER |= (1u | 2u | 8u) << e;                                  // CCxE, CCxP + CCxNP: both edges
}

// ---------------------------------------------------------------------------------------------------------------------
// The port of motor::DshotDirect (hal/motor/dshot_direct.h says what each call has to do) on STM32 timers, bidirectional only.
class TimDmaPort {
public:
    static constexpr int kMaxCh = 4, kCapWords = 32;
    typedef void (*AlarmFn)(void*);
    bool begin(const uint8_t* pins, int count, AlarmFn fn, void* arg, bool pullup, const char*& why) {
        fn_ = fn; arg_ = arg; count_ = 0;
        for (int m = 0; m < count; m++) {
            uint32_t ch = 0;
            TIM_TypeDef* tim = pins[m] == 255 ? nullptr : timer_of(pins[m], ch);
            if (!tim) { why = "a motor pin has no timer channel"; return false; }
            if (!port::pwm_reserve_timer(tim)) { why = "a motor's timer also runs a servo / the buzzer"; return false; }
            int g = 0; while (g < n_tim_ && tims_[g] != tim) g++;
            if (g == n_tim_) { tims_[n_tim_] = tim; ht_[n_tim_] = new HardwareTimer(tim); out_[n_tim_] = false; n_tim_++; }
            if (!dma_[m].claim(tim, (int)ch)) { why = "no DMA for a motor channel (F4: TIM4 CH4 has none)"; return false; }
            tim_of_[m] = (int8_t)g; ch_[m] = (int8_t)ch; pin_[m] = (PinName)pins[m];
            const PinName pn = pin_[m];
            pin_function(pn, pinmap_function(pn, PinMap_TIM));
            LL_GPIO_SetPinPull(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), pullup ? LL_GPIO_PULL_UP : LL_GPIO_PULL_NO);   // the line idles high while the ESC has it
        }
        tick_hz_ = ht_[0]->getTimerClkFreq();
        for (int g = 0; g < n_tim_; g++) {
            if (ht_[g]->getTimerClkFreq() != tick_hz_) { why = "the motor timers run on different clocks"; return false; }
            TIM_TypeDef* t = tims_[g];
            t->CR1 = 0; t->DIER = 0; t->PSC = 0; t->ARR = 0xFFFF; t->DCR = 0;
#if defined(TIM_BDTR_MOE)
            if (IS_TIM_BREAK_INSTANCE(t)) t->BDTR |= TIM_BDTR_MOE;
#endif
            t->CR1 = TIM_CR1_ARPE; t->EGR = TIM_EGR_UG; t->CR1 |= TIM_CR1_CEN;
        }
        if (!alarm_) {                                                // TIM6, one-shot: on_alarm() after the frame
            alarm_ = new HardwareTimer(TIM6);
            alarm_->setPrescaleFactor(alarm_->getTimerClkFreq() / 1000000u);       // 1 us ticks
            alarm_->setOverflow(1000, TICK_FORMAT);
            alarm_->attachInterrupt([]() { if (self_ && self_->fn_) self_->fn_(self_->arg_); });
            alarm_->setInterruptPriority(1, 0);                       // above FreeRTOS (the handler never calls it)
            TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_URS;                   // one pulse; UG does not raise the interrupt
        }
        self_ = this; count_ = count;
        for (int m = 0; m < count_; m++) { channel_output(tims_[tim_of_[m]], ch_[m], true); ccr(m) = 0; }
        for (int g = 0; g < n_tim_; g++) out_[g] = true;
        return true;
    }
    // ---- DshotDirect's port
    WB_FAST uint32_t micros() { return ::micros(); }
    uint32_t tick_hz() const { return tick_hz_; }
    WB_FAST void alarm_in(uint32_t us) { TIM6->CR1 &= ~TIM_CR1_CEN; TIM6->ARR = us ? us - 1u : 0u; TIM6->CNT = 0; TIM6->EGR = TIM_EGR_UG; TIM6->SR = 0; TIM6->CR1 |= TIM_CR1_CEN; }
    void delay_us(uint32_t us) { delayMicroseconds(us); }
    void listen(int) {}                                               // (a channel captures only as an input)
    WB_FAST void drive(int c) {                                       // the timer back to the bit rate, the channel an output again
        const int g = tim_of_[c]; TIM_TypeDef* t = tims_[g];
        dma_[c].stop();
        if (!out_[g]) { out_[g] = true; t->ARR = period_ ? period_ - 1u : 0xFFFFu; t->EGR = TIM_EGR_UG; }
        channel_output(t, ch_[c], true);
    }
    WB_FAST void release(int c) {                                     // the ESC's turn: input capture, DMA copying every edge
        const int g = tim_of_[c]; TIM_TypeDef* t = tims_[g];
        if (out_[g]) { out_[g] = false; t->ARR = 0xFFFF; t->EGR = TIM_EGR_UG; }
        channel_capture(t, ch_[c]);
        t->SR = ~(1u << ch_[c]);                                      // an old capture flag would be the first "edge"
        if (rx_armed_[c]) { dma_[c].go(&ccr(c), cap_[c], kCapWords, false); t->DIER |= 1u << (8 + ch_[c]); }       // CCyDE
    }
    WB_FAST void tx_load(int c, const uint32_t* w, int n) {          // RMT-style words -> compare values (the low part of each bit)
        int k = 0;
        for (int i = 0; i < n && k < motor::kDshotTxWords; i++) {
            const uint32_t d0 = w[i] & 0x7FFF, d1 = (w[i] >> 16) & 0x7FFF;
            if (!d0) break;
            if (!period_) period_ = d0 + d1;                          // the bit length the sequencer used (the same for every bit)
            tx_[c][k++] = d0;                                         // inverted line: the compare value is the low time
        }
        tx_[c][k++] = 0; tx_[c][k++] = 0;                             // the line high again (idle) after the frame
        tx_n_[c] = (uint16_t)k;
    }
    WB_FAST void tx_start(int c) {
        TIM_TypeDef* t = tims_[tim_of_[c]];
        if (t->ARR != period_ - 1u) { t->ARR = period_ - 1u; t->EGR = TIM_EGR_UG; }
        dma_[c].go(&ccr(c), tx_[c], tx_n_[c], true);
        t->DIER |= 1u << (8 + ch_[c]);                                // CCyDE: a request at each compare event = once a bit
    }
    WB_FAST void rx_start(int c) { rx_armed_[c] = true; }             // (the DMA starts in release(): an output channel's compare events are no edges)
    WB_FAST int rx_stop(int c, uint32_t* words, int max) {
        TIM_TypeDef* t = tims_[tim_of_[c]];
        t->DIER &= ~(1u << (8 + ch_[c]));
        const int n = rx_armed_[c] ? dma_[c].done() : 0;
        dma_[c].stop(); rx_armed_[c] = false;
        return motor::capture_to_words(cap_[c], n, words, max);      // (hal/motor/dshot_burst.h)
    }
private:
    volatile uint32_t& ccr(int c) { return (&tims_[tim_of_[c]]->CCR1)[ch_[c] - 1]; }
    static inline TimDmaPort* self_ = nullptr;
    static inline HardwareTimer* alarm_ = nullptr;
    AlarmFn fn_ = nullptr; void* arg_ = nullptr;
    TIM_TypeDef* tims_[kMaxCh] = {}; HardwareTimer* ht_[kMaxCh] = {}; bool out_[kMaxCh] = {}; int n_tim_ = 0;
    Dma dma_[kMaxCh]; int8_t tim_of_[kMaxCh] = {}, ch_[kMaxCh] = {}; PinName pin_[kMaxCh] = {};
    uint32_t tx_[kMaxCh][motor::kDshotTxWords + 2] = {}; uint16_t tx_n_[kMaxCh] = {};
    uint32_t cap_[kMaxCh][kCapWords] = {}; volatile bool rx_armed_[kMaxCh] = {};
    uint32_t tick_hz_ = 0, period_ = 0; int count_ = 0;
};

} // namespace stm32dshot

class DshotTimDmaHw : public MotorHw {
public:
    static constexpr int kMaxGroups = 4, kMaxCh = 4;
    const char* name() const override { return bidir_ ? "DShot/TIM+DMA, bidirectional" : wanted_bidir_ ? "DShot/TIM+DMA (bidirectional OFF)" : "DShot/TIM+DMA"; }
    void set_pullup(bool internal) override { pullup_ = internal; }
    WB_FAST bool bidirectional() const override { return bidir_; }
    bool rx_responsive(int slot) const override { return bidir_ && direct_.responsive(slot); }
    WB_FAST bool poll_reply(int slot, uint32_t& gcr20) override { return bidir_ && direct_.take_reply(slot, gcr20); }

    bool begin(const uint8_t* pins, int count, motor::Protocol proto, uint16_t, bool bidirectional) override {
        n_groups_ = 0; count_ = 0; why_ = nullptr; bidir_ = false; wanted_bidir_ = bidirectional;
        if (!motor::is_dshot_wire(proto)) { why_ = "ProShot: not on the STM32"; return false; }
        if (count < 1 || count > kMaxCh) return false;
        if (bidirectional) {                                          // each channel its own DMA, the line handed to the ESC after the frame
            if (!port_.begin(pins, count, &DshotTimDmaHw::on_alarm, this, pullup_, why_) || !direct_.begin(port_, count, proto, true)) return false;
            bidir_ = true; count_ = count;
            return true;
        }
        for (int m = 0; m < count; m++) {
            uint32_t ch = 0;
            TIM_TypeDef* tim = pins[m] == 255 ? nullptr : stm32dshot::timer_of(pins[m], ch);
            if (!tim) { why_ = "a motor pin has no timer channel"; return false; }
            if (!port::pwm_reserve_timer(tim)) { why_ = "a motor's timer also runs a servo / the buzzer"; return false; }
            int g = 0;
            while (g < n_groups_ && groups_[g].tim != tim) g++;
            if (g == n_groups_) { if (n_groups_ == kMaxGroups) return false; groups_[n_groups_++] = Group(); groups_[g].tim = tim; }
            Group& G = groups_[g];
            if (G.used[ch - 1]) { why_ = "two motors on the same timer channel"; return false; }
            G.used[ch - 1] = true; slot_group_[m] = (int8_t)g; slot_ch_[m] = (int8_t)ch;
        }
        for (int g = 0; g < n_groups_; g++) if (!setup(groups_[g], pins, count, g, proto)) return false;
        count_ = count;
        return true;
    }
    WB_FAST void write(const motor::Frame& f) override {
        if (!count_ || !f.send) return;
        if (bidir_) { direct_.send(f.value, f.telem_request); return; }
        for (int g = 0; g < n_groups_; g++) if (groups_[g].dma.busy()) { stats_.skipped++; return; }       // the previous frame is still going out
        for (int g = 0; g < n_groups_; g++) {
            Group& G = groups_[g];
            uint16_t pk[motor::kBurstMaxCh] = {};
            for (int m = 0; m < count_; m++) if (slot_group_[m] == g) pk[slot_ch_[m] - 1] = motor::dshot_packet(f.value[m], f.telem_request[m], false);
            const int n = motor::burst_fill(G.rows, G.first, G.n, pk, G.used, G.period);
            G.dma.go(&G.tim->DMAR, G.rows, (uint16_t)n, true);
        }
        stats_.frames++;
    }
    motor::HwStats stats() const override {
        motor::HwStats s;
        if (bidir_) { const auto& d = direct_.stats(); s.frames = d.frames; s.skipped = d.skipped; s.late_switch = d.late_switch; s.replies = d.replies; s.garbled = d.garbled; s.no_start = d.no_start; s.bad_bits = d.bad_bits; }
        else { s.frames = stats_.frames; s.skipped = stats_.skipped; }
        return s;
    }
    const char* why_not() const override { return why_; }

private:
    static void on_alarm(void* self) { ((DshotTimDmaHw*)self)->direct_.on_alarm(); }
    struct Group {
        TIM_TypeDef* tim = nullptr; HardwareTimer* ht = nullptr; stm32dshot::Dma dma;
        bool used[motor::kBurstMaxCh] = {}; int first = 0, n = 0; uint32_t period = 0;
        uint32_t rows[motor::kBurstRows * motor::kBurstMaxCh] = {};
    };
    bool setup(Group& G, const uint8_t* pins, int count, int g, motor::Protocol proto) {
        G.first = 0; int last = 0;
        for (int c = 1; c <= 4; c++) if (G.used[c - 1]) { if (!G.first) G.first = c; last = c; }
        G.n = last - G.first + 1;
        if (!G.ht) G.ht = new HardwareTimer(G.tim);                  // (clock on; the frequency of this timer's bus)
        G.period = motor::burst_period(G.ht->getTimerClkFreq(), proto);
        if (!G.period) { why_ = "the timer's clock cannot make that DShot rate"; return false; }
        if (!G.dma.claim(G.tim, 0)) { why_ = "no DMA for the motor timer's update request"; return false; }
        TIM_TypeDef* t = G.tim;
        t->CR1 = 0; t->DIER = 0; t->PSC = 0; t->ARR = G.period - 1;
        for (int c = 1; c <= 4; c++) if (G.used[c - 1]) { stm32dshot::channel_output(t, c, false); (&t->CCR1)[c - 1] = 0; }
        for (int m = 0; m < count; m++) if (slot_group_[m] == g) {
            const PinName pn = (PinName)pins[m];
            pin_function(pn, pinmap_function(pn, PinMap_TIM));          // the pin's alternate function: this timer channel
        }
#if defined(TIM_BDTR_MOE)
        if (IS_TIM_BREAK_INSTANCE(t)) t->BDTR |= TIM_BDTR_MOE;         // advanced timers (TIM1, TIM8): outputs on
#endif
        const uint32_t dba = 13u + (uint32_t)(G.first - 1);           // CCR1 is the 13th register (offset 0x34 / 4)
        t->DCR = dba | ((uint32_t)(G.n - 1) << TIM_DCR_DBL_Pos);
        t->CR1 = TIM_CR1_ARPE;
        t->EGR = TIM_EGR_UG;                                           // load PSC / ARR / the zero compares
        t->DIER = TIM_DIER_UDE;                                        // an update request per bit (the DMA counts the rows)
        t->CR1 |= TIM_CR1_CEN;
        return true;
    }
    Group groups_[kMaxGroups]; int n_groups_ = 0, count_ = 0;
    int8_t slot_group_[kMaxCh] = {}, slot_ch_[kMaxCh] = {};
    struct { uint32_t frames = 0, skipped = 0; } stats_;
    stm32dshot::TimDmaPort port_; motor::DshotDirect<stm32dshot::TimDmaPort> direct_;
    bool bidir_ = false, wanted_bidir_ = false, pullup_ = true;
    const char* why_ = nullptr;
};
