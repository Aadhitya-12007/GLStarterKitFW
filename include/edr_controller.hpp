#pragma once

#include <array>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "ring_buffer.hpp"
#include "stm32f407/imu_driver.hpp"

class EdrController {
public:
    static constexpr std::size_t kBufferSize = 150; // 3.0 s @ 50 Hz

    enum class State : uint8_t { DISARMED, ARMED, POST_CAPTURE };
    enum class Trigger : uint8_t { NONE = 0, SHOCK = 1, TILT = 2 };

    // Frozen 3-second snapshot in RAM (DAY3)
    struct Snapshot {
        uint32_t event_id     = 0;
        uint32_t timestamp_ms = 0;
        Trigger  trigger      = Trigger::NONE;
        float    peak_g       = 0.0f;
        std::array<stm32f407::ImuSample, kBufferSize> samples{};
    };

    explicit EdrController(stm32f407::ImuDriver& imu);

    void start();                  // Launch 50 Hz FreeRTOS task
    void toggle_arm();             // Called by Ui on SW2 press
    void set_armed(bool armed);    // Ready for Day 4 MQTT commands

    // Read by Ui task to refresh the 16x2 LCD
    State           state()           const { return state_; }
    Trigger         last_trigger()    const { return last_trigger_; }
    uint32_t        event_count()     const { return event_count_; }
    float           live_g()          const { return live_g_; }
    const Snapshot& latest_snapshot() const { return snapshot_; }

private:
    static void task_entry(void* arg); // FreeRTOS C-to-C++ bridge
    void run();                        // Main 50 Hz loop
    void save_snapshot();              // Unroll ring buffer + start LED blink

    stm32f407::ImuDriver& imu_;
    RingBuffer<stm32f407::ImuSample, kBufferSize> ring_{};
    Snapshot snapshot_{};
    TaskHandle_t task_handle_ = nullptr;

    // Shared with Ui task (single-writer/single-reader on single-core M4)
    volatile bool     armed_cmd_    = false;
    volatile State    state_        = State::DISARMED;
    volatile Trigger  last_trigger_ = Trigger::NONE;
    volatile uint32_t event_count_  = 0;
    volatile float    live_g_       = 0.0f;

    // Internal FSM state (only touched by the 50 Hz task)
    Trigger  active_trigger_  = Trigger::NONE;
    float    peak_g_          = 0.0f;
    uint32_t trigger_time_ms_ = 0;
    int      post_ticks_      = 0;
    int      tilt_ticks_      = 0;
    int      cooldown_ticks_  = 0;
    int      led_ticks_       = 0;
};