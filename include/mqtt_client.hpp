#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "ethernet.hpp"
#include "mqtt_transport.hpp"

extern "C" {
#include "core_mqtt_agent.h"
#include "event_groups.h"
}

namespace network {

struct MqttBroker {
    const char* host;
    std::uint16_t port;
};

class IBrokerConnectionListener {
public:
    virtual void on_broker_connection_changed(bool connected) = 0;

protected:
    ~IBrokerConnectionListener() = default;
};

class IMqttMessageListener {
public:
    virtual void on_mqtt_message(const char* topic,
                                 std::size_t topic_length,
                                 const char* payload,
                                 std::size_t payload_length) = 0;

protected:
    ~IMqttMessageListener() = default;
};

// Owns the broker connection and runs the coreMQTT agent task that serves it.
class MqttClient final : public ILinkStateClient {
public:
    MqttClient(MqttBroker broker, const char* client_id);

    MqttClient(const MqttClient&) = delete;
    MqttClient& operator=(const MqttClient&) = delete;
    MqttClient(MqttClient&&) = delete;
    MqttClient& operator=(MqttClient&&) = delete;

    // Listeners must be added before start() and run in the agent task.
    bool add_broker_connection_listener(IBrokerConnectionListener& listener);
    void set_message_listener(IMqttMessageListener& listener);

    bool start();
    void on_link_state_changed(LinkState state) override;

    bool is_connected() const { return connected_.load(); }

    // Sends a QoS 0 publish/subscribe and blocks until the agent has handled it.
    // Must NOT be called from inside the agent task's own callbacks.
    bool publish(const char* topic, const char* payload);
    bool subscribe(const char* topic);

private:
    static constexpr std::size_t max_listeners = 2U;
    // Sized for compact 75-sample EDR waveform payloads (~420 bytes) + topic/header.
    static constexpr std::size_t network_buffer_size = 1024U;

    static void task_entry(void* context);
    static void incoming_publish_callback(MQTTAgentContext_t* context,
                                          std::uint16_t packet_id,
                                          MQTTPublishInfo_t* publish_info);

    void run();
    bool connect_to_broker();
    void set_connected(bool connected);

    MqttBroker broker_;
    const char* client_id_;
    NetworkContext network_{};
    MQTTAgentContext_t agent_{};
    std::array<std::uint8_t, network_buffer_size> network_buffer_{};
    EventGroupHandle_t link_events_ = nullptr;
    std::atomic<bool> link_up_{false};
    std::atomic<bool> connected_{false};
    std::array<IBrokerConnectionListener*, max_listeners> listeners_{};
    std::size_t listener_count_ = 0U;
    IMqttMessageListener* message_listener_ = nullptr;
    bool started_ = false;
};

} // namespace network