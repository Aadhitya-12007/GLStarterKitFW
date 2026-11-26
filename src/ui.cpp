#include "ui.hpp"

#include <cstddef>
#include <cstring>
#include <string_view>

#include <printf/printf.h>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace {

constexpr std::uint32_t kRefreshPeriodMs = 100U;        // 10 Hz HMI refresh
constexpr std::uint16_t kGreenLedPin     = GPIO_PIN_12; // PD12 (LD4)
constexpr std::uint16_t kRedLedPin       = GPIO_PIN_14; // PD14 (LD5)

void configure_leds() {
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin   = kGreenLedPin | kRedLedPin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);

    HAL_GPIO_WritePin(GPIOD, kGreenLedPin | kRedLedPin, GPIO_PIN_RESET);
}

void split_g(float g, unsigned& whole, unsigned& frac) {
    if (g < 0.0f) {
        g = 0.0f;
    }
    unsigned g100 = static_cast<unsigned>(g * 100.0f + 0.5f);
    if (g100 > 999U) {
        g100 = 999U; // Keep "G:X.XX" fixed-width on the 16-char LCD row
    }
    whole = g100 / 100U;
    frac  = g100 % 100U;
}

} // namespace

Ui::Ui(stm32f407::Wh1602& display, EdrController& edr)
    : display_(display), edr_(edr) {}

bool Ui::start() {
    const BaseType_t ok = xTaskCreate(
        &Ui::task_entry, "ui", 384, this, tskIDLE_PRIORITY + 1, &task_handle_);
    return ok == pdPASS;
}

void Ui::task_entry(void* arg) {
    static_cast<Ui*>(arg)->run();
}

void Ui::on_button_event(const stm32f407::ButtonEvent& event) {
    if (event.type != stm32f407::ButtonEventType::PRESSED) {
        return;
    }

    switch (event.button) {
        case stm32f407::Button::SW1:
            screen_ = (screen_ == 0U)
                          ? static_cast<std::uint8_t>(kScreenCount - 1U)
                          : static_cast<std::uint8_t>(screen_ - 1U);
            break;

        case stm32f407::Button::SW2:
            edr_.toggle_arm();
            break;

        case stm32f407::Button::SW3:
            screen_ = static_cast<std::uint8_t>((screen_ + 1U) % kScreenCount);
            break;

        case stm32f407::Button::SW4:
            edr_.request_tamper();
            break;

        default:
            break;
    }
}

void Ui::run() {
    // Configure LEDs (since LedsController is omitted) and init WH1602 inside a task.
    configure_leds();
    display_.initialize();
    display_.set_backlight(true);

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        update_leds();
        render_screen();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kRefreshPeriodMs));
    }
}

void Ui::update_leds() {
    const auto state    = edr_.state();
    const bool armed    = (state == EdrController::State::ARMED) ||
                          (state == EdrController::State::POST_CAPTURE);
    const bool tampered = edr_.has_crash() && !edr_.crc_ok();
    const bool red_on   = (state == EdrController::State::POST_CAPTURE) || tampered;

    HAL_GPIO_WritePin(GPIOD, kGreenLedPin, armed  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, kRedLedPin,   red_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Ui::render_screen() {
    char row0[17]{};
    char row1[17]{};

    const std::uint8_t page      = screen_;
    const auto         state     = edr_.state();
    const bool         has_crash = edr_.has_crash();
    const bool         crc_ok    = edr_.crc_ok();

    if (page == 0U) {
        const char* state_str = "DISARM";
        if (state == EdrController::State::ARMED) {
            state_str = "ARMED ";
        } else if (state == EdrController::State::POST_CAPTURE) {
            state_str = "CAPTUR";
        }

        const char* integrity_str = "NO LOG ";
        if (has_crash) {
            integrity_str = crc_ok ? "CRC:OK " : "TAMPER!";
        }

        snprintf(row0, sizeof(row0), "%-6s   %-7s", state_str, integrity_str);

        unsigned w = 0U;
        unsigned f = 0U;
        split_g(edr_.live_g(), w, f);
        if (has_crash) {
            const unsigned id = static_cast<unsigned>(edr_.record().event_id % 100U);
            snprintf(row1, sizeof(row1), "G:%u.%02u   EVT:#%02u", w, f, id);
        } else {
            snprintf(row1, sizeof(row1), "G:%u.%02u   EVT:---", w, f);
        }
    } else if (page == 1U) {
        if (has_crash) {
            const auto&    rec     = edr_.record();
            const unsigned id      = static_cast<unsigned>(rec.event_id % 100U);
            const char*    crc_str = crc_ok ? "CRC:OK " : "TAMPER!";

            unsigned pw = 0U;
            unsigned pf = 0U;
            split_g(rec.peak_g, pw, pf);

            snprintf(row0, sizeof(row0), "EVT #%02u  %-7s", id, crc_str);
            snprintf(row1, sizeof(row1), "PEAK: %u.%02ug", pw, pf);
        } else {
            snprintf(row0, sizeof(row0), "FLASH SECTOR 0");
            snprintf(row1, sizeof(row1), "NO CRASH SAVED");
        }
    } else {
        snprintf(row0, sizeof(row0), "192.168.137.42");
        snprintf(row1, sizeof(row1), "ETH:%-3s MQTT:%-3s",
                 eth_up_  ? "ON" : "OFF",
                 mqtt_up_ ? "ON" : "OFF");
    }

    write_line(0U, row0);
    write_line(1U, row1);
}

void Ui::write_line(std::uint8_t row, const char* text) {
    char padded[stm32f407::Wh1602::columns];
    std::size_t i = 0U;

    while (i < sizeof(padded) && text[i] != '\0') {
        padded[i] = text[i];
        ++i;
    }
    while (i < sizeof(padded)) {
        padded[i++] = ' ';
    }

    if (std::memcmp(last_lines_[row], padded, sizeof(padded)) == 0) {
        return;
    }

    std::memcpy(last_lines_[row], padded, sizeof(padded));
    display_.write(row, 0U, std::string_view{padded, sizeof(padded)});
}