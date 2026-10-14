/*
 * This file is part of Windburst Project
 */

#pragma once
#include "platform.h"
#include <Arduino.h>
#if WB_PLATFORM_STM32
#include "pinmap.h"
#include "HardwareTimer.h"
#endif

// ============================================================================
// A PWM output on a pin (motors in PWM / Oneshot / Multishot / brushed, servos, a passive buzzer):
//   pwm_attach(pin, hz, bits)   false: the pin cannot make PWM, or (STM32) its timer already runs at another rate
//   pwm_write(pin, duty)        duty 0 .. 2^bits - 1
//   pwm_detach(pin)             the pin becomes an input again
//
// ESP32: LEDC, any pin, 8 channels (the core's ledcAttach / ledcWrite / ledcDetach).
// STM32: the timer channel wired to the pin (STM32duino's PinMap_TIM, HardwareTimer). All channels of one timer share one
//        frequency, so two outputs on the same timer must want the same rate: a servo at 50 Hz cannot share TIM3 with motors
//        at 480 Hz. pwm_attach() refuses that instead of changing the rate under the other output, and pwm_timer_name() tells
//        which timer a pin is on (the boot report names it).
// ============================================================================
namespace port {

#if !WB_PLATFORM_STM32
inline bool pwm_attach(uint8_t pin, uint32_t hz, uint8_t bits) { return ledcAttach(pin, hz, bits); }
inline void pwm_write(uint8_t pin, uint32_t duty) { ledcWrite(pin, duty); }
inline void pwm_detach(uint8_t pin) { ledcDetach(pin); }
inline const char* pwm_timer_name(uint8_t) { return "LEDC"; }

#else

namespace pwm_detail {
struct Timer { TIM_TypeDef* tim = nullptr; HardwareTimer* ht = nullptr; uint32_t hz = 0; uint8_t users = 0; };
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
