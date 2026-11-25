#pragma once

#include <array>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "incident_record.hpp"
#include "ring_buffer.hpp"
#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/imu_driver.hpp"

class EdrController {
public:
    static constexpr std::size_t kBufferSize = edr::kRecordSamples; // 150 (3.0 s @ 50 Hz)
    static constexpr uint32_t    kMaxEvents  = 64U;                 // 64 sectors (expandable to 512)

    enum class State   : uint8_t { DISARMED, ARMED, POST_CAPTURE };
    enum class Trigger : uint8_t { NONE = 0, SHOCK = 1, TILT = 2 };

    struct Snapshot {
        uint32_t event_id     = 0;
        uint32_t timestamp_ms = 0;
        Trigger  trigger      = Trigger::NONE;
        float    peak_g       = 0.0f;
        std::array<stm32f407::ImuSample, kBufferSize> samples{};
    };

    EdrController(stm32f407::ImuDriver& imu, stm32f407::BlackBoxStorage& storage);

    // Call once at boot (after storage.init() & hw_security::init(), before scheduler start).
    void restore_from_flash();

    void start();                  // Launch 50 Hz FreeRTOS task
    void toggle_arm();             // Called by Ui on SW2 press
    void set_armed(bool armed);    // Ready for Day 4 MQTT commands
    void request_tamper();         // Called by Ui on SW4 press (handled safely inside EDR task)

    // Read by Ui task (and Day 4 ForensicsServer)
    State           state()           const { return state_; }
    Trigger         last_trigger()    const { return last_trigger_; }
    uint32_t        event_count()     const { return event_count_; }
    float           live_g()          const { return live_g_; }
    bool            chain_ok()        const { return chain_ok_; }
    uint32_t        broken_event_id() const { return broken_event_id_; }
    uint32_t        chain_seed()      const { return chain_seed_; }
    const Snapshot& latest_snapshot() const { return snapshot_; }
    const std::array<uint8_t, 6>& device_mac() const { return storage_.cached_mac(); }

private:
    static void task_entry(void* arg);
    void run();
    void save_snapshot();
    void simulate_tamper();
    void verify_chain(uint32_t count);

    stm32f407::ImuDriver&       imu_;
    stm32f407::BlackBoxStorage& storage_;

    RingBuffer<stm32f407::ImuSample, kBufferSize> ring_{};
    Snapshot            snapshot_{};
    edr::IncidentRecord record_buf_{}; // Static .bss buffer (keeps 1836 B off 2 KB task stack!)
    TaskHandle_t        task_handle_ = nullptr;

    // Shared with Ui task (single-writer/single-reader on single-core Cortex-M4)
    volatile bool     armed_cmd_       = false;
    volatile bool     tamper_cmd_      = false;
    volatile State    state_           = State::DISARMED;
    volatile Trigger  last_trigger_    = Trigger::NONE;
    volatile uint32_t event_count_     = 0;
    volatile float    live_g_          = 0.0f;
    volatile bool     chain_ok_        = true;
    volatile uint32_t broken_event_id_ = 0;

    // Root-of-Trust chain state (solely owned by EdrController)
    uint32_t chain_seed_ = 0U;
    uint32_t last_crc32_ = 0U;

    // Internal FSM state
    Trigger  active_trigger_  = Trigger::NONE;
    float    peak_g_          = 0.0f;
    uint32_t trigger_time_ms_ = 0;
    int      post_ticks_      = 0;
    int      tilt_ticks_      = 0;
    int      cooldown_ticks_  = 0;
    int      led_ticks_       = 0;
};