#pragma once

#include <cstdint>

#include "ethernet.hpp"
#include "mqtt_client.hpp"

class Settings {
public:
    Settings() = delete;

    static constexpr network::EthernetConfig ethernet_config{
        {192U, 168U, 137U, 42U},
        {255U, 255U, 255U, 0U},
        {192U, 168U, 137U, 1U},
        {192U, 168U, 137U, 1U},
    };

    static constexpr network::MqttBroker mqtt_broker{"test.mosquitto.org", 1883U};

    // Whole percent and tenths of a degree Celsius, matching the DHT11's 1 % and 1 °C resolution.
    static constexpr std::uint8_t humidity_reporting_threshold = 1U;
    static constexpr std::uint16_t temperature_reporting_threshold = 10U;

    // One step of each Screen's last shown digit: 0.1 °C and 1 %.
    static constexpr std::uint16_t ui_temperature_threshold = 1U;
    static constexpr std::uint8_t ui_humidity_threshold = 1U;
};
