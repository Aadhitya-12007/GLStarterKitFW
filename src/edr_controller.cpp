#include "edr_controller.hpp"

#include <cmath>
#include <cstddef>
#include <cstring>

extern "C" {
#include "stm32f4xx_hal.h"
}

#include "stm32f407/hw_security.hpp"

namespace {
constexpr uint32_t kPeriodMs      = 20;    // 50 Hz sampling cadence
constexpr float    kShockLimitSq  = 6.25f; // 2.5g * 2.5g
constexpr int      kPostTicks     = 50;    // 1.0 s post-impact capture
constexpr int      kTiltDebounce  = 5;     // 100 ms sustained tilt (>45 deg)
constexpr int      kCooldownTicks = 75;    // 1.5 s lockout after an event
constexpr int      kLedBlinkTicks = 30;    // 600 ms total -> 3 red flashes
} // namespace

EdrController::EdrController(stm32f407::ImuDriver& imu,
                             stm32f407::BlackBoxStorage& storage)
    : imu_(imu), storage_(storage) {}

void EdrController::start() {
    xTaskCreate(&EdrController::task_entry, "edr", 512, this,
                tskIDLE_PRIORITY + 2, &task_handle_);
}

void EdrController::toggle_arm() {
    armed_cmd_ = !armed_cmd_;
}

void EdrController::set_armed(bool armed) {
    armed_cmd_ = armed;
}

void EdrController::request_tamper() {
    tamper_cmd_ = true;
}

void EdrController::task_entry(void* arg) {
    static_cast<EdrController*>(arg)->run();
}

// ---------------------------------------------------------------------------
// 1. Cold-Boot Recovery: Scan Flash, Recover Seed/Count, Verify CRC-32 Chain
// ---------------------------------------------------------------------------
void EdrController::restore_from_flash() {
    if (!storage_.is_ready()) {
        stm32f407::hw_security::generate_random_u32(chain_seed_);
        last_crc32_ = chain_seed_;
        return;
    }

    const uint32_t count = storage_.scan_valid_records(edr::kRecordMagic, kMaxEvents);

    if (count == 0U) {
        // Fresh/empty Flash: seed the chain with a true 32-bit Hardware RNG nonce
        stm32f407::hw_security::generate_random_u32(chain_seed_);
        last_crc32_      = chain_seed_;
        event_count_     = 0U;
        chain_ok_        = true;
        broken_event_id_ = 0U;
        return;
    }

    // Verify all stored sectors 0..(count-1) and restore chain_seed_ + last_crc32_
    verify_chain(count);

    // record_buf_ now holds the latest sector (count - 1); restore UI snapshot too
    event_count_           = count;
    last_trigger_          = static_cast<Trigger>(record_buf_.trigger_type);
    snapshot_.event_id     = record_buf_.event_id;
    snapshot_.timestamp_ms = record_buf_.timestamp_ms;
    snapshot_.trigger      = last_trigger_;
    snapshot_.peak_g       = static_cast<float>(record_buf_.peak_g_x100) * 0.01f;
    std::memcpy(snapshot_.samples.data(), record_buf_.samples, sizeof(record_buf_.samples));
}

// ---------------------------------------------------------------------------
// 2. Main 50 Hz FreeRTOS Task Loop
// ---------------------------------------------------------------------------
void EdrController::run() {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        // 1. Service SW4 tamper simulation request safely outside active capture
        if (tamper_cmd_ && state_ != State::POST_CAPTURE) {
            tamper_cmd_ = false;
            simulate_tamper();
        }

        // 2. Non-blocking Red LED (PD14) flasher: 5 ticks ON / 5 ticks OFF
        if (led_ticks_ > 0) {
            --led_ticks_;
            bool on = ((led_ticks_ / 5) % 2) == 1;
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        }

        // 3. Read 6-axis IMU sample over I2C1 and push into 3-second ring buffer
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

            // 4. Apply SW2 Arm/Disarm button state
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

            // 5. Execute 3-state FSM
            if (state_ == State::ARMED) {
                if (cooldown_ticks_ > 0) {
                    --cooldown_ticks_;
                } else {
                    bool shock = (g_sq > kShockLimitSq);

                    // Tilt > 45 deg: tan(45) = 1 => ax^2 + ay^2 > az^2 (sustained 100 ms)
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

        // 6. Sleep until the exact next 20 ms boundary (50 Hz)
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kPeriodMs));
    }
}

// ---------------------------------------------------------------------------
// 3. Chain Verifier: Recomputes STM32 HW CRC-32 Across Every Stored Sector
// ---------------------------------------------------------------------------
void EdrController::verify_chain(uint32_t count) {
    bool     all_ok      = true;
    uint32_t first_bad   = 0U;
    uint32_t expect_prev = 0U;

    for (uint32_t slot = 0U; slot < count; ++slot) {
        if (!storage_.read_sector(slot,
                                  reinterpret_cast<uint8_t*>(&record_buf_),
                                  sizeof(record_buf_))) {
            all_ok    = false;
            first_bad = slot + 1U;
            break;
        }

        if (slot == 0U) {
            // Event #1 carries the persistent Hardware RNG genesis seed
            chain_seed_ = record_buf_.prev_crc32;
            expect_prev = chain_seed_;
        }

        const uint32_t calc_crc = stm32f407::hw_security::compute_crc32(
            &record_buf_, offsetof(edr::IncidentRecord, record_crc32));

        const bool id_ok   = (record_buf_.event_id == (slot + 1U));
        const bool link_ok = (record_buf_.prev_crc32 == expect_prev);
        const bool crc_ok  = (record_buf_.record_crc32 == calc_crc);

        if (!(id_ok && link_ok && crc_ok) && all_ok) {
            all_ok    = false;
            first_bad = slot + 1U;
        }

        // Advance chain pointer for the next record
        expect_prev = record_buf_.record_crc32;
    }

    last_crc32_      = expect_prev;
    chain_ok_        = all_ok;
    broken_event_id_ = first_bad;
}

// ---------------------------------------------------------------------------
// 4. Seal & Commit Snapshot to SST25VF016B Flash (end of POST_CAPTURE)
// ---------------------------------------------------------------------------
void EdrController::save_snapshot() {
    const uint32_t slot   = event_count_;
    const uint32_t new_id = slot + 1U;

    // Update RAM snapshot for Screen 1
    snapshot_.event_id     = new_id;
    snapshot_.timestamp_ms = trigger_time_ms_;
    snapshot_.trigger      = active_trigger_;
    snapshot_.peak_g       = peak_g_;
    ring_.copy_chronological(snapshot_.samples);

    // Build packed IncidentRecord in static .bss record_buf_
    std::memset(&record_buf_, 0, sizeof(record_buf_));
    record_buf_.magic        = edr::kRecordMagic;
    record_buf_.event_id     = new_id;
    record_buf_.timestamp_ms = trigger_time_ms_;
    record_buf_.peak_g_x100  = static_cast<uint16_t>(peak_g_ * 100.0f + 0.5f);
    record_buf_.trigger_type = static_cast<uint8_t>(active_trigger_);
    record_buf_.temp_c_x10   = 0; // Populated from DHT11 on Day 4
    record_buf_.humidity_x10 = 0;

    const auto& mac = storage_.cached_mac();
    std::memcpy(record_buf_.device_mac, mac.data(), mac.size());

    // Chain to previous record (or RNG genesis seed for Event #1)
    record_buf_.prev_crc32 = last_crc32_;
    std::memcpy(record_buf_.samples, snapshot_.samples.data(), sizeof(record_buf_.samples));

    // Hardware CRC-32 over all preceding 458 words (1,832 bytes)
    record_buf_.record_crc32 = stm32f407::hw_security::compute_crc32(
        &record_buf_, offsetof(edr::IncidentRecord, record_crc32));

    // Commit to 4 KB Flash sector (only if storage is ready and within slot budget)
    if (storage_.is_ready() && slot < kMaxEvents) {
        if (storage_.write_sector(slot,
                                  reinterpret_cast<const uint8_t*>(&record_buf_),
                                  sizeof(record_buf_))) {
            last_crc32_ = record_buf_.record_crc32;
        }
    }

    event_count_  = new_id;
    last_trigger_ = active_trigger_;
    led_ticks_    = kLedBlinkTicks; // 3 crisp non-blocking Red LED (PD14) flashes
}

// ---------------------------------------------------------------------------
// 5. Simulate Tamper Action (Triggered by SW4 inside 50 Hz Task)
// ---------------------------------------------------------------------------
void EdrController::simulate_tamper() {
    if (!storage_.is_ready() || event_count_ == 0U) {
        return;
    }

    const uint32_t latest_slot = event_count_ - 1U;

    // 1. Read the latest valid record from Flash
    if (!storage_.read_sector(latest_slot,
                              reinterpret_cast<uint8_t*>(&record_buf_),
                              sizeof(record_buf_))) {
        return;
    }

    // 2. Flip 1 single bit in the impact sample (index 100, Z-axis accel)
    //    WITHOUT updating record_crc32
    record_buf_.samples[100].az ^= 0x0001;

    // 3. Write the corrupted record back into the same 4 KB Flash sector
    if (storage_.write_sector(latest_slot,
                              reinterpret_cast<const uint8_t*>(&record_buf_),
                              sizeof(record_buf_))) {
        // 4. Re-verify chain immediately so LCD flags "TAMPER!" right away
        verify_chain(event_count_);
        led_ticks_ = kLedBlinkTicks;
    }
}