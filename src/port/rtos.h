/*
 * This file is part of Windburst Project
 */

#pragma once
#include "platform.h"

// ============================================================================
// The few FreeRTOS calls that are spelt differently on the two chips.
//   ESP32: ESP-IDF's FreeRTOS (two cores, stacks in BYTES, spinlock critical sections).
//   STM32: the "STM32duino FreeRTOS" library (one core, stacks in WORDS, the scheduler started by us at the end of setup()).
// Everything else (vTaskDelay, mutexes, task notifications) is the same API on both.
// ============================================================================
#if WB_PLATFORM_ESP32
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#elif WB_PLATFORM_STM32
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#endif

#if !WB_PLATFORM_HOST
namespace port {

// Task priorities. On the ESP32 the flight task has a core to itself (core 1, with the sensor thread under it) and the link
// task the other one; on one core the order matters more: flight > sensors (receiver!) > link (bulk transfers) > idle.
#if WB_PLATFORM_ESP32
constexpr UBaseType_t kPrioFlight = 20, kPrioSensor = 1, kPrioLink = 2, kPrioSd = 1;
#else
constexpr UBaseType_t kPrioFlight = configMAX_PRIORITIES - 1, kPrioSensor = 3, kPrioLink = 2, kPrioSd = 1;
#endif

// stack_bytes: bytes on both chips. core: where the ESP32 pins it (ignored on one core).
inline bool task_create(TaskFunction_t fn, const char* name, uint32_t stack_bytes, void* arg, UBaseType_t prio, TaskHandle_t* handle, int core) {
#if WB_PLATFORM_ESP32
    return xTaskCreatePinnedToCore(fn, name, stack_bytes, arg, prio, handle, core) == pdPASS;
#else
    (void)core;
    return xTaskCreate(fn, name, (configSTACK_DEPTH_TYPE)((stack_bytes + sizeof(StackType_t) - 1) / sizeof(StackType_t)), arg, prio, handle) == pdPASS;
#endif
}
inline int core_id() {
#if WB_PLATFORM_ESP32
    return (int)xPortGetCoreID();
#else
    return 0;
#endif
}

[[noreturn]] inline void restart() {
#if WB_PLATFORM_ESP32
    ESP.restart();
#else
    NVIC_SystemReset();
#endif
    for (;;) {}
}

// A short critical section (microseconds): no task switch and no interrupt that could call FreeRTOS. On the STM32 the interrupts
// above FreeRTOS (UART, USB) still run; hard_lock() stops them too, for bit timing that must not be disturbed at all.
class Critical {
public:
#if WB_PLATFORM_ESP32
    void lock() { portENTER_CRITICAL(&mux_); }
    void unlock() { portEXIT_CRITICAL(&mux_); }
    void hard_lock() { lock(); }
    void hard_unlock() { unlock(); }
private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
#else
    void lock() { taskENTER_CRITICAL(); }
    void unlock() { taskEXIT_CRITICAL(); }
    void hard_lock() { primask_ = __get_PRIMASK(); __disable_irq(); }
    void hard_unlock() { __set_PRIMASK(primask_); }
private:
    uint32_t primask_ = 0;
#endif
};

} // namespace port
#endif
