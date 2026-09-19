#include "ui.hpp"

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <string_view>

namespace {

constexpr configSTACK_DEPTH_TYPE ui_stack_depth = 512U;
constexpr UBaseType_t ui_task_priority = tskIDLE_PRIORITY + 1U;

using Line = std::array<char, stm32f407::Wh1602::columns>;

Line blank_line()
{
    Line line;
    line.fill(' ');
    return line;
}

Line text_line(std::string_view text)
{
    Line line = blank_line();
    std::copy_n(text.begin(), std::min(text.size(), line.size()), line.begin());
    return line;
}

// Tenths as "-5.0" or "23.0".
Line tenths_line(std::int16_t tenths)
{
    Line line = blank_line();
    char* position = line.data();
    if (tenths < 0) {
        *position++ = '-';
    }
    const auto magnitude = static_cast<std::uint32_t>(std::abs(tenths));
    position = std::to_chars(position, line.end(), magnitude / 10U).ptr;
    *position++ = '.';
    std::to_chars(position, line.end(), magnitude % 10U);
    return line;
}

Line whole_line(std::uint8_t value)
{
    Line line = blank_line();
    std::to_chars(line.data(), line.end(), value);
    return line;
}

template <typename Value, typename Format>
Line value_line(const std::optional<Value>& value, Format format)
{
    return value ? format(*value) : text_line("--");
}

} // namespace

namespace ui {

Ui::Ui(stm32f407::Wh1602& display, const std::array<std::uint8_t, 4>& ip)
    : display_(display)
    , ip_line_(blank_line())
{
    char* position = ip_line_.data();
    for (std::size_t index = 0U; index < ip.size(); ++index) {
        if (index != 0U) {
            *position++ = '.';
        }
        position = std::to_chars(position, ip_line_.end(), ip[index]).ptr;
    }
}

bool Ui::start()
{
    configASSERT(task_ == nullptr);

    return xTaskCreate(task_entry, "ui", ui_stack_depth, this, ui_task_priority, &task_) ==
           pdPASS;
}

void Ui::on_humidity_changed(std::uint8_t humidity)
{
    change([humidity](State& state) { state.humidity = humidity; });
}

void Ui::on_temperature_changed(std::int16_t temperature)
{
    change([temperature](State& state) { state.temperature = temperature; });
}

void Ui::on_button_event(const stm32f407::ButtonEvent& event)
{
    if (event.type != stm32f407::ButtonEventType::PRESSED) {
        return;
    }

    std::size_t step;
    if (event.button == stm32f407::Button::SW1) {
        step = 1U;
    } else if (event.button == stm32f407::Button::SW3) {
        step = screen_count - 1U;
    } else {
        return;
    }

    change([step](State& state) {
        const std::size_t screen = static_cast<std::size_t>(state.screen);
        state.screen = static_cast<Screen>((screen + step) % screen_count);
    });
}

void Ui::on_link_state_changed(network::LinkState state)
{
    const bool link_up = state == network::LinkState::UP;
    change([link_up](State& state) { state.link_up = link_up; });
}

void Ui::on_broker_connection_changed(bool connected)
{
    change([connected](State& state) { state.broker_connected = connected; });
}

template <typename Update>
void Ui::change(Update update)
{
    configASSERT(task_ != nullptr);

    taskENTER_CRITICAL();
    update(state_);
    taskEXIT_CRITICAL();

    xTaskNotifyGive(task_);
}

void Ui::task_entry(void* context)
{
    static_cast<Ui*>(context)->run();
}

void Ui::run()
{
    display_.initialize();
    display_.set_backlight(true);

    for (;;) {
        taskENTER_CRITICAL();
        const State state = state_;
        taskEXIT_CRITICAL();

        draw(state);
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void Ui::draw(const State& state)
{
    Line title = blank_line();
    Line value = blank_line();

    switch (state.screen) {
    case Screen::NETWORK:
    {
        // "ETH off MQTT off" at its widest.
        constexpr std::size_t mqtt_column = 8U;
        const std::string_view mqtt = state.broker_connected ? "MQTT on" : "MQTT off";
        title = ip_line_;
        value = text_line(state.link_up ? "ETH on" : "ETH off");
        std::copy(mqtt.begin(), mqtt.end(), value.begin() + mqtt_column);
        break;
    }
    case Screen::TEMPERATURE:
        title = text_line("Temperature");
        title[14] = stm32f407::Wh1602::degree;
        title[15] = 'C';
        value = value_line(state.temperature, tenths_line);
        break;
    case Screen::HUMIDITY:
        title = text_line("Humidity       %");
        value = value_line(state.humidity, whole_line);
        break;
    }

    display_.write(0U, 0U, {title.data(), title.size()});
    display_.write(1U, 0U, {value.data(), value.size()});
}

} // namespace ui
