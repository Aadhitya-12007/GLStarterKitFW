#include "ui.hpp"

#include <cstdio>
#include <cstring>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace {
constexpr uint32_t kRefreshPeriodMs = 100U; // 10 Hz HMI refresh (display + LEDs)
constexpr uint16_t kGreenLedPin     = GPIO_PIN_12; // PD12 (LD4)
constexpr uint16_t kRedLedPin       = GPIO_PIN_14; // PD14 (LD5)

// Converts float G-force into whole + hundredths without requiring float printf
void split_g(float g, unsigned& whole, unsigned& frac) {
    if (g < 0.0f) {
        g = 0.0f;
    }
    const unsigned g100 = static_cast<unsigned>(g * 100.0f + 0.5f);
    whole = g100 / 100U;
    frac  = g100 % 100U;
}
} // namespace

Ui::Ui(stm32f407::Wh1602& display,
       stm32f407::Keyboard& keyboard,
       EdrController& edr)
    : display_(display), keyboard_(keyboard), edr_(edr) {}

void Ui::start() {
    xTaskCreate(&Ui::task_entry, "ui", 384, this,
                tskIDLE_PRIORITY + 1, &task_handle_);
}

void Ui::task_entry(void* arg) {
    static_cast<Ui*>(arg)->run();
}

// ---------------------------------------------------------------------------
// 1. Switch Handler (Runs inside Keyboard task — zero hardware/SPI/LCD access)
// ---------------------------------------------------------------------------
void Ui::on_button_event(stm32f407::Keyboard::Button button) {
    using Btn = stm32f407::Keyboard::Button;

    switch (button) {
        case Btn::SW1:
            screen_ = (screen_ == 0U) ? (kScreenCount - 1U) : static_cast<uint8_t>(screen_ - 1U);
            break;

        case Btn::SW2:
            edr_.toggle_arm();
            break;

        case Btn::SW3:
            screen_ = static_cast<uint8_t>((screen_ + 1U) % kScreenCount);
            break;

        case Btn::SW4:
            edr_.request_tamper();
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// 2. Main 10 Hz HMI Loop (Sole Owner of Display + PD12/PD14 LEDs)
// ---------------------------------------------------------------------------
void Ui::run() {
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        update_leds();
        render_screen();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kRefreshPeriodMs));
    }
}

// ---------------------------------------------------------------------------
// 3. LED Control (Zero Blink Math — Clear Solid Status)
// ---------------------------------------------------------------------------
void Ui::update_leds() {
    const auto state    = edr_.state();
    const bool armed    = (state == EdrController::State::ARMED) ||
                          (state == EdrController::State::POST_CAPTURE);
    const bool tampered = edr_.has_crash() && !edr_.crc_ok();
    const bool red_on   = (state == EdrController::State::POST_CAPTURE) || tampered;

    HAL_GPIO_WritePin(GPIOD, kGreenLedPin, armed  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, kRedLedPin,   red_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// ---------------------------------------------------------------------------
// 4. Three Minimal 16x2 Screens
// ---------------------------------------------------------------------------
void Ui::render_screen() {
    char row0[17]{};
    char row1[17]{};

    const uint8_t page      = screen_;
    const auto    state     = edr_.state();
    const bool    has_crash = edr_.has_crash();
    const bool    crc_ok    = edr_.crc_ok();

    if (page == 0U) {
        // Screen 0: Live Status (State + Integrity / Live G + Saved Event ID)
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

        std::snprintf(row0, sizeof(row0), "%-6s   %-7s", state_str, integrity_str);

        unsigned w = 0, f = 0;
        split_g(edr_.live_g(), w, f);
        if (has_crash) {
            const unsigned id = static_cast<unsigned>(edr_.latest_record().event_id % 100U);
            std::snprintf(row1, sizeof(row1), "G:%u.%02u   EVT:#%02u", w, f, id);
        } else {
            std::snprintf(row1, sizeof(row1), "G:%u.%02u   EVT:---", w, f);
        }
    } else if (page == 1U) {
        // Screen 1: Stored CrashRecord Summary (Sector 0)
        if (has_crash) {
            const auto& rec = edr_.latest_record();
            const unsigned id = static_cast<unsigned>(rec.event_id % 100U);
            const char* crc_str = crc_ok ? "CRC:OK " : "TAMPER!";

            unsigned pw = 0, pf = 0;
            split_g(rec.peak_g, pw, pf);

            std::snprintf(row0, sizeof(row0), "EVT #%02u  %-7s", id, crc_str);
            std::snprintf(row1, sizeof(row1), "PEAK: %u.%02ug", pw, pf);
        } else {
            std::snprintf(row0, sizeof(row0), "FLASH SECTOR 0");
            std::snprintf(row1, sizeof(row1), "NO CRASH SAVED");
        }
    } else {
        // Screen 2: Network & MQTT Status
        std::snprintf(row0, sizeof(row0), "192.168.137.42");
        std::snprintf(row1, sizeof(row1), "ETH:%-3s MQTT:%-3s",
                      eth_up_  ? "ON" : "OFF",
                      mqtt_up_ ? "ON" : "OFF");
    }

    write_line(0U, row0);
    write_line(1U, row1);
}

// Pads to exactly 16 characters and skips redundant display writes (prevents flicker)
void Ui::write_line(uint8_t row, const char* text) {
    char padded[17];
    std::snprintf(padded, sizeof(padded), "%-16.16s", text);

    if (std::memcmp(last_lines_[row], padded, 16U) == 0) {
        return;
    }

    std::memcpy(last_lines_[row], padded, 17U);
    display_.set_cursor(0U, row); // Match your Wh1602 API (e.g., set_cursor / write)
    display_.print(padded);
}