/*
 * This file is part of Windburst Project
 */

#pragma once
#include "platform.h"
#include <Arduino.h>
#if WB_PLATFORM_STM32
#include "pinmap.h"
#include "HardwareTimer.h"
#elif WB_PLATFORM_ESP32
#include "hal/ledc_ll.h"
#include "hal/gpio_ll.h"
#include "soc/ledc_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc_caps.h"
#include "esp_rom_gpio.h"
#endif

// ============================================================================
// A PWM output on a pin (motors in PWM / Oneshot / Multishot / brushed, servos, a passive buzzer):
//   pwm_attach(pin, hz, bits)   false: the pin cannot make PWM, or (STM32) its timer already runs at another rate
//   pwm_write(pin, duty)        duty 0 .. 2^bits - 1
//   pwm_detach(pin)             the pin becomes an input again
//
// ESP32: LEDC (low-speed group: 8 channels, 4 timers), any pin through the GPIO matrix. The registers are written directly
//        (hal/ledc_ll.h, as Betaflight's ESP32 motor driver does): the core's ledcWrite() is a library call in flash that takes a
//        lock and looks the pin up for every write - the motors write every control cycle. Outputs with the same rate and
//        resolution share a timer; a fifth combination is refused.
// PC (tests): the Arduino stub's ledcAttach / ledcWrite, which records what was written.
// STM32: the timer channel wired to the pin (STM32duino's PinMap_TIM, HardwareTimer). All channels of one timer share one
//        frequency, so two outputs on the same timer must want the same rate: a servo at 50 Hz cannot share TIM3 with motors
//        at 480 Hz. pwm_attach() refuses that instead of changing the rate under the other output, and pwm_timer_name() tells
//        which timer a pin is on (the boot report names it).
// ============================================================================
namespace port {

// the finest duty resolution an output can have (bits): the LEDC timer's width (S3: 14, ESP32: 20), or an STM32 timer's 16-bit
// counter (the PC's stub: the same)
#if WB_PLATFORM_ESP32
constexpr uint8_t kPwmMaxBits = SOC_LEDC_TIMER_BIT_WIDTH;
#else
constexpr uint8_t kPwmMaxBits = 16;
#endif

#if WB_PLATFORM_ESP32
namespace pwm_detail {
constexpr int kTimers = 4, kChannels = 8;
constexpr uint32_t kApbHz = 80000000u;                        // the low-speed timers count the APB clock (ledc_init selects it)
struct Timer { uint32_t hz = 0; uint8_t bits = 0, users = 0; };
inline Timer* timers() { static Timer t[kTimers]; return t; }
inline uint8_t* chan_pin() { static uint8_t p[kChannels] = {255, 255, 255, 255, 255, 255, 255, 255}; return p; }
inline uint8_t* chan_timer() { static uint8_t t[kChannels] = {}; return t; }
inline int find(uint8_t pin) { for (int c = 0; c < kChannels; c++) if (chan_pin()[c] == pin) return c; return -1; }
inline void ledc_init() {
    static bool done = false;
    if (done) return;
    int __DECLARE_RCC_ATOMIC_ENV __attribute__((unused));     // (the ll macros expect it: one-time start-up, nothing else uses the LEDC)
    ledc_ll_enable_bus_clock(true);
    ledc_ll_enable_reset_reg(false);
    ledc_ll_set_slow_clk_sel(&LEDC, LEDC_SLOW_CLK_APB);       // without a source the low-speed timers do not count at all
    done = true;
}
inline void channel_duty(int c, uint32_t duty) {
    const ledc_channel_t ch = (ledc_channel_t)c;
    ledc_ll_set_duty_int_part(&LEDC, LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_ll_set_duty_start(&LEDC, LEDC_LOW_SPEED_MODE, ch);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch);   // (taken at the start of the next period: never a cut pulse)
}
}

inline bool pwm_attach(uint8_t pin, uint32_t hz, uint8_t bits) {
    using namespace pwm_detail;
    if (pin == 255 || pin >= SOC_GPIO_PIN_COUNT || hz == 0 || bits < 1 || bits > SOC_LEDC_TIMER_BIT_WIDTH || find(pin) >= 0) return false;
    // the timer's divider: fixed point with 8 fraction bits, 1.0 .. 1023.99
    const uint64_t div_q8 = ((uint64_t)kApbHz << 8) / ((uint64_t)hz << bits);
    if (div_q8 < 256u || div_q8 >= (1u << 18)) return false;
    ledc_init();
    int t = -1, c = -1;
    for (int i = 0; i < kTimers; i++) if (timers()[i].users && timers()[i].hz == hz && timers()[i].bits == bits) { t = i; break; }
    if (t < 0) for (int i = 0; i < kTimers; i++) if (!timers()[i].users) { t = i; break; }
    for (int i = 0; i < kChannels; i++) if (chan_pin()[i] == 255) { c = i; break; }
    if (t < 0 || c < 0) return false;
    Timer& T = timers()[t]; const ledc_timer_t tm = (ledc_timer_t)t; const ledc_channel_t ch = (ledc_channel_t)c;
    if (!T.users) {
        ledc_ll_set_clock_divider(&LEDC, LEDC_LOW_SPEED_MODE, tm, (uint32_t)div_q8);
        ledc_ll_set_duty_resolution(&LEDC, LEDC_LOW_SPEED_MODE, tm, bits);
        ledc_ll_timer_rst(&LEDC, LEDC_LOW_SPEED_MODE, tm);
        ledc_ll_ls_timer_update(&LEDC, LEDC_LOW_SPEED_MODE, tm);
        ledc_ll_timer_resume(&LEDC, LEDC_LOW_SPEED_MODE, tm);
        T.hz = hz; T.bits = bits;
    }
    T.users++;
    // the channel: on that timer, output on, a static duty (no fade), low while idle; it starts at duty 0
    ledc_ll_bind_channel_timer(&LEDC, LEDC_LOW_SPEED_MODE, ch, tm);
    ledc_ll_set_hpoint(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_ll_set_duty_direction(&LEDC, LEDC_LOW_SPEED_MODE, ch, LEDC_DUTY_DIR_INCREASE);
    ledc_ll_set_duty_num(&LEDC, LEDC_LOW_SPEED_MODE, ch, 1);
    ledc_ll_set_duty_cycle(&LEDC, LEDC_LOW_SPEED_MODE, ch, 1);
    ledc_ll_set_duty_scale(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_ll_set_idle_level(&LEDC, LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_ll_set_sig_out_en(&LEDC, LEDC_LOW_SPEED_MODE, ch, true);
    chan_pin()[c] = pin; chan_timer()[c] = (uint8_t)t;
    channel_duty(c, 0);
    esp_rom_gpio_pad_select_gpio(pin);
    gpio_ll_output_enable(&GPIO, pin);
    esp_rom_gpio_connect_out_signal(pin, LEDC_LS_SIG_OUT0_IDX + c, false, false);
    return true;
}
inline void pwm_write(uint8_t pin, uint32_t duty) {
    const int c = pwm_detail::find(pin);
    if (c >= 0) pwm_detail::channel_duty(c, duty);
}
inline void pwm_detach(uint8_t pin) {
    using namespace pwm_detail;
    const int c = find(pin); if (c < 0) return;
    const ledc_channel_t ch = (ledc_channel_t)c;
    ledc_ll_set_sig_out_en(&LEDC, LEDC_LOW_SPEED_MODE, ch, false);
    ledc_ll_ls_channel_update(&LEDC, LEDC_LOW_SPEED_MODE, ch);
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);     // the pin back from the matrix, then an input
    gpio_ll_output_disable(&GPIO, pin);
    Timer& T = timers()[chan_timer()[c]];
    if (T.users && --T.users == 0) ledc_ll_timer_pause(&LEDC, LEDC_LOW_SPEED_MODE, (ledc_timer_t)chan_timer()[c]);
    chan_pin()[c] = 255;
}
inline const char* pwm_timer_name(uint8_t) { return "LEDC"; }

#elif !WB_PLATFORM_STM32
inline bool pwm_attach(uint8_t pin, uint32_t hz, uint8_t bits) { return ledcAttach(pin, hz, bits); }
inline void pwm_write(uint8_t pin, uint32_t duty) { ledcWrite(pin, duty); }
inline void pwm_detach(uint8_t pin) { ledcDetach(pin); }
inline const char* pwm_timer_name(uint8_t) { return "LEDC"; }

#else

namespace pwm_detail {
struct Timer { TIM_TypeDef* tim = nullptr; HardwareTimer* ht = nullptr; uint32_t hz = 0; uint8_t users = 0; bool reserved = false; };     // reserved: DShot has it (port/stm32/dshot_stm32.h)
struct Out { uint8_t pin = 255, timer = 255, channel = 0, bits = 0; };
constexpr int kTimers = 16, kOuts = 16;
inline Timer* timers() { static Timer t[kTimers]; return t; }
inline Out* outs() { static Out o[kOuts]; return o; }
inline Out* find(uint8_t pin) { for (int i = 0; i < kOuts; i++) if (outs()[i].pin == pin) return &outs()[i]; return nullptr; }
}

inline const char* pwm_timer_name(uint8_t pin) {
    const TIM_TypeDef* t = (const TIM_TypeDef*)pinmap_peripheral((PinName)pin, PinMap_TIM);
    if (!t) return "no timer";
#define WB_TIM_NAME(X) if (t == X) return #X;
#if defined(TIM1)
    WB_TIM_NAME(TIM1)
#endif
#if defined(TIM2)
    WB_TIM_NAME(TIM2)
#endif
#if defined(TIM3)
    WB_TIM_NAME(TIM3)
#endif
#if defined(TIM4)
    WB_TIM_NAME(TIM4)
#endif
#if defined(TIM5)
    WB_TIM_NAME(TIM5)
#endif
#if defined(TIM8)
    WB_TIM_NAME(TIM8)
#endif
#if defined(TIM12)
    WB_TIM_NAME(TIM12)
#endif
#if defined(TIM13)
    WB_TIM_NAME(TIM13)
#endif
#if defined(TIM14)
    WB_TIM_NAME(TIM14)
#endif
#if defined(TIM15)
    WB_TIM_NAME(TIM15)
#endif
#if defined(TIM16)
    WB_TIM_NAME(TIM16)
#endif
#if defined(TIM17)
    WB_TIM_NAME(TIM17)
#endif
#undef WB_TIM_NAME
    return "TIM";
}

// A timer the PWM outputs here already use (a servo, the buzzer), or that DShot took (pwm_reserve_timer): one owner per timer.
inline bool pwm_timer_in_use(TIM_TypeDef* tim) {
    for (int i = 0; i < pwm_detail::kTimers; i++) if (pwm_detail::timers()[i].tim == tim && (pwm_detail::timers()[i].users || pwm_detail::timers()[i].reserved)) return true;
    return false;
}
inline bool pwm_reserve_timer(TIM_TypeDef* tim) {
    using namespace pwm_detail;
    for (int i = 0; i < kTimers; i++) if (timers()[i].tim == tim) { if (timers()[i].users) return false; timers()[i].reserved = true; return true; }
    for (int i = 0; i < kTimers; i++) if (!timers()[i].tim) { timers()[i].tim = tim; timers()[i].reserved = true; return true; }
    return false;
}
inline bool pwm_attach(uint8_t pin, uint32_t hz, uint8_t bits) {
    using namespace pwm_detail;
    if (pin == 255 || hz == 0 || bits < 1 || bits > 16 || find(pin)) return false;
    const PinName pn = (PinName)pin;
    TIM_TypeDef* tim = (TIM_TypeDef*)pinmap_peripheral(pn, PinMap_TIM);
    if (!tim) return false;
    const uint32_t ch = STM_PIN_CHANNEL(pinmap_function(pn, PinMap_TIM));
    if (ch < 1 || ch > 4) return false;
    int t = -1;
    for (int i = 0; i < kTimers; i++) if (timers()[i].tim == tim) { t = i; break; }
    if (t < 0) for (int i = 0; i < kTimers; i++) if (!timers()[i].tim) { t = i; timers()[i].tim = tim; break; }
    if (t < 0) return false;
    Timer& T = timers()[t];
    if (T.reserved) return false;                             // the motors' DShot runs this timer
    if (T.users && T.hz != hz) return false;                  // another output runs this timer at another rate
    Out* o = find(255); if (!o) return false;
    if (!T.ht) T.ht = new HardwareTimer(tim);
    if (!T.users) { T.ht->pause(); T.ht->setOverflow(hz, HERTZ_FORMAT); T.hz = hz; }
    T.ht->setMode(ch, TIMER_OUTPUT_COMPARE_PWM1, pn);
    T.ht->setCaptureCompare(ch, 0, (TimerCompareFormat_t)bits);
    if (!T.users) T.ht->resume(); else T.ht->resumeChannel(ch);
    T.users++;
    o->pin = pin; o->timer = (uint8_t)t; o->channel = (uint8_t)ch; o->bits = bits;
    return true;
}
inline void pwm_write(uint8_t pin, uint32_t duty) {
    pwm_detail::Out* o = pwm_detail::find(pin); if (!o) return;
    pwm_detail::timers()[o->timer].ht->setCaptureCompare(o->channel, duty, (TimerCompareFormat_t)o->bits);
}
inline void pwm_detach(uint8_t pin) {
    using namespace pwm_detail;
    Out* o = find(pin); if (!o) return;
    Timer& T = timers()[o->timer];
    T.ht->setMode(o->channel, TIMER_DISABLED);
    pinMode(pinNametoDigitalPin((PinName)pin), INPUT);
    if (T.users && --T.users == 0) T.ht->pause();
    *o = Out();
}
#endif

} // namespace port
