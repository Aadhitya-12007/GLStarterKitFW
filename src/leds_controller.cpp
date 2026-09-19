#include "leds_controller.hpp"

#include <cstddef>
#include <cstring>
#include <initializer_list>

#include "printf/printf.h"

#include "stm32f407/board.hpp"

extern "C" {
#include "task.h"
}

namespace {

constexpr configSTACK_DEPTH_TYPE leds_stack_depth = 512U;
constexpr UBaseType_t leds_task_priority = tskIDLE_PRIORITY + 1U;
constexpr UBaseType_t event_queue_length = 4U;
constexpr TickType_t retry_period = pdMS_TO_TICKS(1000U);

} // namespace

LedsController::LedsController(network::MqttClient& client,
                               const stm32f407::DeviceId& device_id)
    : client_(client)
{
    std::size_t length = 0U;
    for (const char* part : {"SBC/", device_id.data(), "/leds/green"}) {
        const std::size_t part_length = std::strlen(part);
        std::memcpy(topic_.data() + length, part, part_length);
        length += part_length;
    }
}

bool LedsController::start()
{
    events_ = xQueueCreate(event_queue_length, sizeof(stm32f407::ButtonEvent));
    if (events_ == nullptr) {
        return false;
    }

    return xTaskCreate(task_entry,
                       "leds",
                       leds_stack_depth,
                       this,
                       leds_task_priority,
                       nullptr) == pdPASS;
}

void LedsController::on_button_event(const stm32f407::ButtonEvent& event)
{
    if (event.button == stm32f407::Button::SW2 &&
        event.type == stm32f407::ButtonEventType::PRESSED) {
        (void)xQueueSend(events_, &event, 0U);
    }
}

void LedsController::task_entry(void* context)
{
    static_cast<LedsController*>(context)->run();
}

void LedsController::run()
{
    stm32f407::ButtonEvent event{};
    bool pending = false;
    for (;;) {
        // Times out only while a state is still pending, to retry it.
        if (xQueueReceive(events_, &event, pending ? retry_period : portMAX_DELAY) == pdPASS) {
            stm32f407::toggle_green_led();
            green_on_ = !green_on_;
            printf("Green led state: %d\n", green_on_);
        }
        pending = !client_.publish(topic_.data(), green_on_ ? "1" : "0");
    }
}
