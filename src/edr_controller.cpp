#include "edr_controller.hpp"

#include <cmath>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace {
constexpr uint32_t kPeriodMs      = 20;     // 50 Hz
constexpr float    kShockLimitSq  = 6.25f;  // 2.5g * 2.5g
constexpr int      kPostTicks     = 50;     // 1.0 s after trigger
constexpr int      kTiltDebounce  = 5;      // 100 ms sustained tilt
constexpr int      kCooldownTicks = 75;     // 1.5 s before re-trigger
constexpr int      kLedBlinkTicks = 30;     // 3 flashes (600 ms total)
} // namespace

EdrController::EdrController(stm32f407::ImuDriver& imu) : imu_(imu) {}

void EdrController::start() {
    xTaskCreate(&EdrController::task_entry, "edr", 512, this,
                tskIDLE_PRIORITY + 2, &task_handle_);
}

void EdrController::toggle_arm() { armed_cmd_ = !armed_cmd_; }
void EdrController::set_armed(bool armed) { armed_cmd_ = armed; }

void EdrController::task_entry(void* arg) {
    static_cast<EdrController*>(arg)->run();
}

void EdrController::run() {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // 1. Non-blocking Red LED (PD14) flasher: 100 ms ON / 100 ms OFF
        if (led_ticks_ > 0) {
            --led_ticks_;
            bool on = ((led_ticks_ / 5) % 2) == 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        }

        // 2. Read 6-axis IMU sample and push into 3-second rolling buffer
        stm32f407::ImuSample s{};
        if (imu_.read_sample(s)) {
            ring_.push(s);

            float ax = stm32f407::ImuDriver::raw_to_g(s.ax);
            float ay = stm32f407::ImuDriver::raw_to_g(s.ay);
            float az = stm32f407::ImuDriver::raw_to_g(s.az);

            float xy_sq = ax * ax + ay * ay;
            float z_sq  = az * az;
            float g_sq  = xy_sq + z_sq;
            float g_now = std::sqrt(g_sq);
            live_g_ = g_now;

            // 3. Handle Arm/Disarm command from SW2
            if (!armed_cmd_) {
                state_          = State::DISARMED;
                post_ticks_     = 0;
                tilt_ticks_     = 0;
                cooldown_ticks_ = 0;
            } else if (state_ == State::DISARMED) {
                state_          = State::ARMED;
                tilt_ticks_     = 0;
                cooldown_ticks_ = 0;
            }

            // 4. Run FSM
            if (state_ == State::ARMED) {
                if (cooldown_ticks_ > 0) {
                    --cooldown_ticks_;
                } else {
                    bool shock = (g_sq > kShockLimitSq);

                    if (g_sq > 0.25f && xy_sq > z_sq) {
                        ++tilt_ticks_;
                    } else {
                        tilt_ticks_ = 0;
                    }
                    bool tilt = (tilt_ticks_ >= kTiltDebounce);

                    if (shock || tilt) {
                        active_trigger_  = shock ? Trigger::SHOCK : Trigger::TILT;
                        peak_g_          = g_now;
                        trigger_time_ms_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        post_ticks_      = kPostTicks;
                        tilt_ticks_      = 0;
                        state_           = State::POST_CAPTURE;
                    }
                }
            } else if (state_ == State::POST_CAPTURE) {
                if (g_now > peak_g_) {
                    peak_g_ = g_now;
                }
                if (--post_ticks_ <= 0) {
                    save_snapshot();
                    cooldown_ticks_ = kCooldownTicks;
                    state_          = State::ARMED;
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));
    }
}

void EdrController::save_snapshot() {
    uint32_t next_id = event_count_ + 1;

    snapshot_.event_id     = next_id;
    snapshot_.timestamp_ms = trigger_time_ms_;
    snapshot_.trigger      = active_trigger_;
    snapshot_.peak_g       = peak_g_;
    ring_.copy_chronological(snapshot_.samples);

    last_trigger_ = active_trigger_;
    event_count_  = next_id;
    led_ticks_    = kLedBlinkTicks;
}