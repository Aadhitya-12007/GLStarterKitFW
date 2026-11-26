#include "edr_controller.hpp"

#include <cmath>
#include <cstring>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace {
constexpr uint32_t kPeriodMs      = 20U;   // 50 Hz sampling cadence
constexpr uint32_t kFlashSector   = 0U;    // Single-slot storage (Sector 0)
constexpr float    kShockLimitSq  = 6.25f; // 2.5 g * 2.5 g
constexpr int      kPostTicks     = 50;    // 1.0 s post-shock capture
constexpr int      kCooldownTicks = 50;    // 1.0 s lockout after saving
} // namespace

EdrController::EdrController(stm32f407::ImuDriver& imu,
                             stm32f407::BlackBoxStorage& storage)
    : imu_(imu), storage_(storage) {}

void EdrController::start() {
    xTaskCreate(&EdrController::task_entry, "edr", 512, this,
                tskIDLE_PRIORITY + 2, &task_handle_);
}

void EdrController::toggle_arm()          { armed_cmd_  = !armed_cmd_; }
void EdrController::set_armed(bool armed) { armed_cmd_  = armed; }
void EdrController::request_tamper()      { tamper_cmd_ = true; }

bool EdrController::take_crash_ready() {
    if (!crash_ready_) {
        return false;
    }
    crash_ready_ = false;
    return true;
}

void EdrController::task_entry(void* arg) {
    static_cast<EdrController*>(arg)->run();
}

// ---------------------------------------------------------------------------
// Hardware Helpers: STM32F407 CRC-32 Unit (RM0090 Section 4) & Red LED (PD14)
// ---------------------------------------------------------------------------
uint32_t EdrController::compute_crc32(const void* data, std::size_t len_bytes) {
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET; // Resets CRC_DR to 0xFFFFFFFF (poly 0x04C11DB7)

    const auto* bytes = static_cast<const uint8_t*>(data);
    for (std::size_t i = 0U; i + 4U <= len_bytes; i += 4U) {
        uint32_t word = 0U;
        std::memcpy(&word, bytes + i, sizeof(word));
        CRC->DR = word;
    }
    return CRC->DR;
}

void EdrController::set_red_led(bool on) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ---------------------------------------------------------------------------
// 1. Boot Recovery: Load Sector 0 & Verify Hardware CRC-32
// ---------------------------------------------------------------------------
void EdrController::load_from_flash() {
    std::memset(&record_, 0, sizeof(record_));
    has_crash_   = false;
    crc_ok_      = true;
    event_count_ = 0U;

    if (!storage_.is_ready()) {
        set_red_led(false);
        return;
    }

    if (!storage_.read_sector(kFlashSector,
                              reinterpret_cast<uint8_t*>(&record_),
                              sizeof(record_))) {
        std::memset(&record_, 0, sizeof(record_));
        set_red_led(false);
        return;
    }

    if (record_.magic == edr::kRecordMagic) {
        const uint32_t calc = compute_crc32(&record_, offsetof(edr::CrashRecord, crc32));
        has_crash_   = true;
        crc_ok_      = (calc == record_.crc32);
        event_count_ = record_.event_id;
        set_red_led(!crc_ok_); // Solid ON immediately if Flash was tampered before reboot
    } else {
        std::memset(&record_, 0, sizeof(record_));
        set_red_led(false);
    }
}

// ---------------------------------------------------------------------------
// 2. Main 50 Hz FreeRTOS Loop (Accelerometer-Only Shock FSM)
// ---------------------------------------------------------------------------
void EdrController::run() {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // Service SW4 tamper request safely when not capturing a crash
        if (tamper_cmd_ && state_ != State::POST_CAPTURE) {
            tamper_cmd_ = false;
            simulate_tamper();
            last_wake = xTaskGetTickCount(); // Re-sync cadence after SPI Flash write
        }

        // Read 3-axis accelerometer sample and keep the 3-second ring buffer warm
        stm32f407::ImuSample s{};
        if (imu_.read_sample(s)) {
            ring_.push(s);

            const float ax    = stm32f407::ImuDriver::raw_to_g(s.ax);
            const float ay    = stm32f407::ImuDriver::raw_to_g(s.ay);
            const float az    = stm32f407::ImuDriver::raw_to_g(s.az);
            const float g_sq  = (ax * ax) + (ay * ay) + (az * az);
            const float g_now = std::sqrt(g_sq);
            live_g_ = g_now;

            // Apply Arm / Disarm command from SW2 or MQTT
            if (!armed_cmd_) {
                state_          = State::DISARMED;
                post_ticks_     = 0;
                cooldown_ticks_ = 0;
            } else if (state_ == State::DISARMED) {
                state_          = State::ARMED;
                cooldown_ticks_ = 0;
            }

            // 3-state FSM: DISARMED <-> ARMED -> POST_CAPTURE -> ARMED
            if (state_ == State::ARMED) {
                if (cooldown_ticks_ > 0) {
                    --cooldown_ticks_;
                } else if (g_sq > kShockLimitSq) {
                    peak_g_     = g_now;
                    post_ticks_ = kPostTicks;
                    state_      = State::POST_CAPTURE;
                }
            } else if (state_ == State::POST_CAPTURE) {
                if (g_now > peak_g_) {
                    peak_g_ = g_now;
                }
                if (--post_ticks_ <= 0) {
                    save_crash();
                    cooldown_ticks_ = kCooldownTicks;
                    state_          = State::ARMED;
                    last_wake       = xTaskGetTickCount(); // Re-sync after Flash write
                }
            }
        }

        // Red LED (PD14): solid ON during 1 s POST_CAPTURE or when tampered, else OFF
        const bool led_on = (state_ == State::POST_CAPTURE) || (has_crash_ && !crc_ok_);
        set_red_led(led_on);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));
    }
}

// ---------------------------------------------------------------------------
// 3. Unroll RingBuffer -> Hardware CRC-32 -> Write Sector 0 -> Flag MQTT
// ---------------------------------------------------------------------------
void EdrController::save_crash() {
    const uint32_t next_id = event_count_ + 1U;

    record_.magic    = edr::kRecordMagic;
    record_.event_id = next_id;
    record_.peak_g   = peak_g_;
    ring_.copy_chronological(record_.samples);
    record_.crc32    = compute_crc32(&record_, offsetof(edr::CrashRecord, crc32));

    if (storage_.is_ready()) {
        storage_.write_sector(kFlashSector,
                              reinterpret_cast<const uint8_t*>(&record_),
                              sizeof(record_));
    }

    event_count_ = next_id;
    has_crash_   = true;
    crc_ok_      = true;
    set_red_led(false);
    crash_ready_ = true;
}

// ---------------------------------------------------------------------------
// 4. SW4 Live Tamper Test: Flip 1 Bit in Sector 0 Without Updating CRC-32
// ---------------------------------------------------------------------------
void EdrController::simulate_tamper() {
    if (!storage_.is_ready() || !has_crash_) {
        return;
    }

    if (!storage_.read_sector(kFlashSector,
                              reinterpret_cast<uint8_t*>(&record_),
                              sizeof(record_))) {
        return;
    }

    // Flip 1 LSB (0.000244 g) at sample 100 (t = 0 impact boundary) without recomputing CRC
    record_.samples[100].az = static_cast<int16_t>(record_.samples[100].az ^ 1);

    if (storage_.write_sector(kFlashSector,
                              reinterpret_cast<const uint8_t*>(&record_),
                              sizeof(record_))) {
        const uint32_t calc = compute_crc32(&record_, offsetof(edr::CrashRecord, crc32));
        crc_ok_ = (calc == record_.crc32); // Immediately flips to false
        set_red_led(!crc_ok_);             // Immediately lights Red LED solid ON
    }
}