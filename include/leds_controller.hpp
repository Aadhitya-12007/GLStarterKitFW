#pragma once

#include <array>

#include "mqtt_client.hpp"
#include "stm32f407/device_id.hpp"
#include "stm32f407/keyboard.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "queue.h"
}

// Toggles the green LED on each SW2 press and publishes its new state to the
// board's green LED topic, retrying while the broker is unreachable until a
// newer state replaces it. The controller's own task does all of this.
class LedsController final : public stm32f407::IButtonListener {
public:
    LedsController(network::MqttClient& client, const stm32f407::DeviceId& device_id);

    LedsController(const LedsController&) = delete;
    LedsController& operator=(const LedsController&) = delete;
    LedsController(LedsController&&) = delete;
    LedsController& operator=(LedsController&&) = delete;

    bool start();
    void on_button_event(const stm32f407::ButtonEvent& event) override;

private:
    static void task_entry(void* context);

    void run();

    network::MqttClient& client_;
    // "SBC/" + Device ID + "/leds/green" + NUL.
    std::array<char, 40> topic_{};
    // Not a task notification: MqttClient::publish waits on this task's notification.
    QueueHandle_t events_ = nullptr;
    bool green_on_ = false;
};
