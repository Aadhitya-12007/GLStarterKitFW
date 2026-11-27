#include "edr_mqtt.hpp"

#include <cmath>
#include <cstring>

#include <printf/printf.h>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "stm32f407/imu_driver.hpp"

namespace {

constexpr configSTACK_DEPTH_TYPE edr_mqtt_stack_depth = 512U;
constexpr UBaseType_t edr_mqtt_task_priority = tskIDLE_PRIORITY + 2U;
constexpr TickType_t poll_period_ticks = pdMS_TO_TICKS(100U);

// Matches plain-text commands while ignoring trailing CR/LF/spaces from CLI tools.
bool matches_command(const char* payload, std::size_t length, const char* expected)
{
    while (length > 0U &&
           (payload[length - 1U] == '\n' ||
            payload[length - 1U] == '\r' ||
            payload[length - 1U] == ' ')) {
        --length;
    }

    const std::size_t expected_len = std::strlen(expected);
    return length == expected_len && std::memcmp(payload, expected, expected_len) == 0;
}

} // namespace

namespace network {

EdrMqtt::EdrMqtt(MqttClient& mqtt,
                 const stm32f407::DeviceId& device_id,
                 EdrController& edr)
    : mqtt_(mqtt)
    , edr_(edr)
{
    snprintf(data_topic_, sizeof(data_topic_), "SBC/%s/edr/data", device_id.data());
    snprintf(cmd_topic_, sizeof(cmd_topic_), "SBC/%s/edr/cmd", device_id.data());
}

bool EdrMqtt::start()
{
    return xTaskCreate(task_entry,
                       "edr_mqtt",
                       edr_mqtt_stack_depth,
                       this,
                       edr_mqtt_task_priority,
                       nullptr) == pdPASS;
}

void EdrMqtt::on_broker_connection_changed(bool connected)
{
    // Re-subscribe on every connect/reconnect (cleanSession = true).
    if (connected) {
        need_subscribe_ = true;
    }
}

void EdrMqtt::on_mqtt_message(const char* topic,
                              std::size_t topic_length,
                              const char* payload,
                              std::size_t payload_length)
{
    const std::size_t cmd_len = std::strlen(cmd_topic_);
    if (topic_length != cmd_len || std::memcmp(topic, cmd_topic_, cmd_len) != 0) {
        return;
    }

    if (matches_command(payload, payload_length, "ARM")) {
        edr_.set_armed(true);
        printf("MQTT cmd: ARM\n");
    } else if (matches_command(payload, payload_length, "DISARM")) {
        edr_.set_armed(false);
        printf("MQTT cmd: DISARM\n");
    } else if (matches_command(payload, payload_length, "READ")) {
        read_requested_ = true;
        printf("MQTT cmd: READ\n");
    }
}

void EdrMqtt::task_entry(void* context)
{
    static_cast<EdrMqtt*>(context)->run();
}

void EdrMqtt::run()
{
    for (;;) {
        if (edr_.take_crash_ready()) {
            pending_publish_ = true;
        }
        if (read_requested_.exchange(false)) {
            pending_publish_ = true;
        }

        if (mqtt_.is_connected()) {
            if (need_subscribe_.load()) {
                if (mqtt_.subscribe(cmd_topic_)) {
                    need_subscribe_ = false;
                    printf("MQTT subscribed: %s\n", cmd_topic_);
                }
            }

            if (pending_publish_) {
                if (publish_crash_record()) {
                    pending_publish_ = false;
                    printf("MQTT published: %s\n", data_topic_);
                }
            }
        }

        vTaskDelay(poll_period_ticks);
    }
}

bool EdrMqtt::publish_crash_record()
{
    if (!edr_.has_crash()) {
        return mqtt_.publish(data_topic_, "EMPTY");
    }

    const auto& rec = edr_.record();
    int pos = snprintf(payload_,
                       sizeof(payload_),
                       "ID:%lu,PEAK:%.2f,CRC:%s,G:",
                       static_cast<unsigned long>(rec.event_id),
                       static_cast<double>(rec.peak_g),
                       edr_.crc_ok() ? "OK" : "FAIL");
    if (pos < 0 || static_cast<std::size_t>(pos) >= sizeof(payload_)) {
        return false;
    }

    // Downsample 150 samples (50 Hz) -> 75 values (25 Hz): 50 pre-shock + 25 post-shock.
    for (std::size_t i = 0U; i < edr::kRecordSamples; i += 2U) {
        const auto& s = rec.samples[i];
        const float ax = stm32f407::ImuDriver::raw_to_g(s.ax);
        const float ay = stm32f407::ImuDriver::raw_to_g(s.ay);
        const float az = stm32f407::ImuDriver::raw_to_g(s.az);
        const float g = std::sqrt(ax * ax + ay * ay + az * az);

        const int written = snprintf(payload_ + pos,
                                     sizeof(payload_) - static_cast<std::size_t>(pos),
                                     (i == 0U) ? "%.2f" : ",%.2f",
                                     static_cast<double>(g));
        if (written <= 0 || static_cast<std::size_t>(pos + written) >= sizeof(payload_)) {
            break;
        }
        pos += written;
    }

    return mqtt_.publish(data_topic_, payload_);
}

} // namespace network