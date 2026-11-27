#pragma once

#include <atomic>
#include <cstddef>

#include "edr_controller.hpp"
#include "mqtt_client.hpp"
#include "stm32f407/device_id.hpp"

namespace network {

// Bridges EdrController <-> MqttClient using two topics:
//   Publish:   SBC/<DeviceID>/edr/data  (header + 75 downsampled G-force values)
//   Subscribe: SBC/<DeviceID>/edr/cmd   ("ARM", "DISARM", "READ")
class EdrMqtt final : public IBrokerConnectionListener,
                      public IMqttMessageListener {
public:
    EdrMqtt(MqttClient& mqtt,
            const stm32f407::DeviceId& device_id,
            EdrController& edr);

    bool start();

    // Called from the coreMQTT agent task (never block here).
    void on_broker_connection_changed(bool connected) override;
    void on_mqtt_message(const char* topic,
                         std::size_t topic_length,
                         const char* payload,
                         std::size_t payload_length) override;

private:
    static void task_entry(void* context);

    void run();
    bool publish_crash_record();

    MqttClient& mqtt_;
    EdrController& edr_;

    char data_topic_[64]{};
    char cmd_topic_[64]{};
    // Kept as a member (static .bss) instead of on the FreeRTOS task stack.
    char payload_[640]{};

    std::atomic<bool> need_subscribe_{false};
    std::atomic<bool> read_requested_{false};
    bool pending_publish_ = false;
};

} // namespace network