/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../config/fast_code.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"
#include "flight_app.h"

// ============================================================================
// The ESP32 side of sync.h: the mutex, and the task that runs the control cycle.
//
//   hardware timer (period = 1 / loop rate)  --interrupt-->  wakes "wb_flight" (core 1, priority 20)
//   wb_flight:  FlightApp::flight_cycle()    takes the lock, IMU -> core -> motors -> log, releases it, sleeps until the next tick
//   loop():     FlightApp::slow_update()     the sensor thread (Arduino's loopTask: core 1, priority 1), once per millisecond
//
// The flight task preempts everything else on its core the moment the timer fires, so an I2C read of the barometer or a burst
// of GPS bytes no longer moves a control cycle. A tick that arrives while the previous cycle is still running is not made up
// later (one at most: the cycle after a long one starts at once), and the task stays away from the core for a quarter of the
// time its cycles take: the sensor thread always gets a fifth of the core (app::TickLoop in sync.h). The cycle count shows it
// (status: overruns, loop_hz).
//
// NOT yet confirmed on hardware.
// ============================================================================
class RtosSync : public app::Sync {
public:
    RtosSync() : m_(xSemaphoreCreateMutex()) {}                 // a mutex, not a binary semaphore: the low-priority holder inherits the flight task's priority
    WB_FAST void lock() override { xSemaphoreTake(m_, portMAX_DELAY); }
    WB_FAST void unlock() override { xSemaphoreGive(m_); }
private:
    SemaphoreHandle_t m_;
};

class FlightTask {
public:
    // Call at the end of setup(), from the core the flight loop shall run on (the timer interrupt is installed on the caller's core).
    bool start(FlightApp& app, int priority = 20) {
        app_ = &app;
        app.set_sync(&sync_);
        if (xTaskCreatePinnedToCore(&FlightTask::entry, "wb_flight", 12288, this, priority, &task_, xPortGetCoreID()) != pdPASS) return false;
        gptimer_config_t gc = {};
        gc.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        gc.direction = GPTIMER_COUNT_UP;
        gc.resolution_hz = 1000000;
        gc.intr_priority = 3;
        if (gptimer_new_timer(&gc, &timer_) != ESP_OK) return false;
        gptimer_event_callbacks_t cb = {};
        cb.on_alarm = on_tick;
        if (gptimer_register_event_callbacks(timer_, &cb, this) != ESP_OK) return false;
        gptimer_alarm_config_t a = {};
        a.alarm_count = app.period_us(); a.reload_count = 0; a.flags.auto_reload_on_alarm = true;
        if (gptimer_set_alarm_action(timer_, &a) != ESP_OK) return false;
        return gptimer_enable(timer_) == ESP_OK && gptimer_start(timer_) == ESP_OK;
    }
    bool running() const { return timer_ != nullptr && task_ != nullptr; }

private:
    static bool IRAM_ATTR on_tick(gptimer_handle_t, const gptimer_alarm_event_data_t*, void* self) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(((FlightTask*)self)->task_, &woken);
        return woken == pdTRUE;
    }
    static void entry(void* self) { static_cast<FlightTask*>(self)->run(); }
    struct Port {
        FlightApp* app;
        WB_FAST void wait_tick() { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }
        WB_FAST uint32_t now() { return micros(); }
        WB_FAST void cycle(uint32_t now_us) { app->flight_cycle(now_us); }
    };
    WB_FAST void run() {
        Port port{app_};
        app::TickLoop<Port> loop;                        // what is left of the core belongs to the sensor thread (receiver!): see sync.h
        for (;;) loop.turn(port, app_->period_us());
    }
    FlightApp* app_ = nullptr;
    RtosSync sync_;
    TaskHandle_t task_ = nullptr;
    gptimer_handle_t timer_ = nullptr;
};
