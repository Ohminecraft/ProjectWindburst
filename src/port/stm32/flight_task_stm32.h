/*
 * This file is part of Windburst Project
 */

#pragma once
#include "../rtos.h"
#include "../../config/ram_profile.h"
#include "HardwareTimer.h"
#include "../../app/flight_app.h"

// ============================================================================
// The STM32 side of sync.h (app/flight_task.h is the ESP32 one): the mutex, and the task that runs the control cycle.
//
//   TIM7 (basic timer, period = 1 / loop rate) --update interrupt--> wakes "wb_flight" (the highest FreeRTOS priority)
//   wb_flight:  FlightApp::flight_cycle()    lock, IMU -> core -> motors -> log, unlock, wait for the next tick
//   wb_sensor:  FlightApp::slow_update()     once per millisecond tick, below the flight task
//   wb_link:    the app link                 below both
//
// One core does all three, so the budget of app::TickLoop (sync.h) matters more than on the ESP32: the sensor thread (the
// receiver!) is always left a fifth of the core, whatever the loop rate setting asks for.
// The timer interrupt calls FreeRTOS, so its priority must be numerically at or above configMAX_SYSCALL_INTERRUPT_PRIORITY
// (5 in STM32duino FreeRTOS): it gets 6. The UART and USB interrupts of the core stay above FreeRTOS (they never call it).
// TIM7 is used by nothing else here: the motor, servo and buzzer timers are TIM1-5, 8, 12-17.
//
// NOT confirmed on hardware.
// ============================================================================
// (Nothing of FreeRTOS may be created before the end of setup(): the first FreeRTOS call masks the interrupts below
//  configMAX_SYSCALL_INTERRUPT_PRIORITY - SysTick, so millis() and delay() - until the scheduler starts. So the mutex is made
//  in start(), the last thing setup() does before vTaskStartScheduler().)
class RtosSync : public app::Sync {
public:
    bool create() { if (!m_) m_ = xSemaphoreCreateMutex(); return m_ != nullptr; }   // a mutex (priority inheritance)
    void lock() override { xSemaphoreTake(m_, portMAX_DELAY); }
    void unlock() override { xSemaphoreGive(m_); }
private:
    SemaphoreHandle_t m_ = nullptr;
};

class FlightTask {
public:
    static constexpr uint32_t kIrqPriority = 6;
    // Call at the end of setup(), before the scheduler starts. The timer is started by the task itself, once it runs.
    bool start(FlightApp& app) {
        app_ = &app;
        if (!sync_.create()) return false;
        app.set_sync(&sync_);
        if (!port::task_create(&FlightTask::entry, "wb_flight", WB_LOW_RAM ? 8192 : 24000, this, port::kPrioFlight, &task_, 0)) return false;
        self_ = this;
        return true;
    }
    bool running() const { return task_ != nullptr; }

private:
    static void on_tick() {
        BaseType_t woken = pdFALSE;
        if (self_ && self_->task_) vTaskNotifyGiveFromISR(self_->task_, &woken);
        portYIELD_FROM_ISR(woken);
    }
    static void entry(void* self) { static_cast<FlightTask*>(self)->run(); }
    struct Port {
        FlightApp* app;
        void wait_tick() { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }
        uint32_t now() { return micros(); }
        void cycle(uint32_t now_us) { app->flight_cycle(now_us); }
    };
    void run() {
        timer_ = new HardwareTimer(TIM7);
        timer_->setOverflow(app_->period_us(), MICROSEC_FORMAT);
        timer_->setInterruptPriority(kIrqPriority, 0);
        timer_->attachInterrupt(&FlightTask::on_tick);
        timer_->resume();
        Port port{app_};
        app::TickLoop<Port> loop;
        for (;;) loop.turn(port, app_->period_us());
    }
    static inline FlightTask* self_ = nullptr;
    FlightApp* app_ = nullptr;
    RtosSync sync_;
    TaskHandle_t task_ = nullptr;
    HardwareTimer* timer_ = nullptr;
};
