#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "stm32f407/device_id.hpp"

// Prefix + the last device_id_chars Device ID characters + NUL, within MQTT 3.1.1's guaranteed 23.
constexpr char mqtt_client_id_prefix[] = "sbc-";
constexpr std::size_t mqtt_client_id_prefix_length = sizeof(mqtt_client_id_prefix) - 1;
constexpr std::size_t device_id_chars = 16;
using MqttClientId = std::array<char, mqtt_client_id_prefix_length + device_id_chars + 1>;

inline MqttClientId make_mqtt_client_id(const stm32f407::DeviceId& device_id)
{
    MqttClientId client_id{};
    std::copy_n(mqtt_client_id_prefix, mqtt_client_id_prefix_length, client_id.begin());
    // The Device ID's own NUL is not copied; client_id's is already zero.
    std::copy_n(device_id.end() - 1 - device_id_chars, device_id_chars,
                client_id.begin() + mqtt_client_id_prefix_length);
    return client_id;
}
