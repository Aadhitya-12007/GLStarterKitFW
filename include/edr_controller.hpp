#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "ring_buffer.hpp"
#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/imu_driver.hpp"

namespace edr {

constexpr uint32_t    kRecordMagic   = 0xED4B1AC0U;
constexpr std::size_t kRecordSamples = 150U; // 3.0 s @ 50 Hz (100 pre + 50 post)

// Single struct used in both RAM and Flash Sector 0 (916 bytes total).
struct __attribute__((packed)) CrashRecord {
    uint32_t             magic;
    uint32_t             event_id;
    float                peak_g;
    stm32f407::ImuSample samples[kRecordSamples]; // 150 * 6 B = 900 B (accel only)
    uint32_t             crc32;                   // STM32 HW CRC-32 over bytes 0..911
};

static_assert(sizeof(stm32f407::ImuSample) == 6U,
              "Expected accel-only 6-byte ImuSample (ax, ay, az)");
static_assert(offsetof(CrashRecord, crc32) == 912U,
              "CRC payload must be 912 bytes (228 exact 32-bit words)");
static_assert((offsetof(CrashRecord, crc32) % 4U) == 0U,
              "Hardware CRC-32 input must be 4-byte aligned");
static_assert(sizeof(CrashRecord) == 916U,
              "CrashRecord must be 916 bytes");
static_assert(sizeof(CrashRecord) <= 4096U,
              "CrashRecord must fit inside a single 4 KB Flash sector");

} // namespace edr

class EdrController {
public:
    enum class State : uint8_t {
        DISARMED = 0,
        ARMED,
        POST_CAPTURE,
    };

    EdrController(stm32f407::ImuDriver& imu,
                  stm32f407::BlackBoxStorage& storage);

    // Call once at boot after storage.init() and before vTaskStartScheduler().
    void load_from_flash();

    // Starts the 50 Hz FreeRTOS task.
    void start();

    // Thread-safe commands for Ui (SW2 / SW4) and MqttClient (ARM / DISARM).
    void toggle_arm();
    void set_armed(bool armed);
    void request_tamper();

    // Status getters for Ui (16x2 LCD) and MqttClient.
    State    state()       const { return state_; }
    float    live_g()      const { return live_g_; }
    bool     has_crash()   const { return has_crash_; }
    bool     crc_ok()      const { return crc_ok_; }
    uint32_t event_count() const { return event_count_; }
    float    peak_g()      const { return has_crash_ ? record_.peak_g : 0.0f; }

    // Returns true once after a new crash is committed so MQTT publishes it.
    bool take_crash_ready();

    // Read-only access to the single Sector 0 record (for MQTT publish & GDB).
    const edr::CrashRecord& record() const { return record_; }

private:
    static void     task_entry(void* arg);
    static uint32_t compute_crc32(const void* data, std::size_t len_bytes);
    static void     set_red_led(bool on);

    void run();
    void save_crash();
    void simulate_tamper();

    stm32f407::ImuDriver&       imu_;
    stm32f407::BlackBoxStorage& storage_;

    RingBuffer<stm32f407::ImuSample, edr::kRecordSamples> ring_{};
    edr::CrashRecord record_{}; // Static .bss storage (916 B, never on task stack)
    TaskHandle_t     task_handle_ = nullptr;

    // Shared with Ui and MqttClient tasks (single-core Cortex-M4)
    volatile bool     armed_cmd_   = false;
    volatile bool     tamper_cmd_  = false;
    volatile State    state_       = State::DISARMED;
    volatile float    live_g_      = 0.0f;
    volatile bool     has_crash_   = false;
    volatile bool     crc_ok_      = true;
    volatile uint32_t event_count_ = 0U;
    volatile bool     crash_ready_ = false;

    // Internal FSM variables (touched only by the 50 Hz task)
    float peak_g_         = 0.0f;
    int   post_ticks_     = 0;
    int   cooldown_ticks_ = 0;
};