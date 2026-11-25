#include "ui.hpp"

#include <cstddef>
#include <string_view>

#include <printf/printf.h>

namespace ui {

namespace {

constexpr TickType_t kRefreshPeriod = pdMS_TO_TICKS(150); // ~6.7 Hz LCD refresh

const char* state_label(EdrController::State s) {
    switch (s) {
        case EdrController::State::DISARMED:     return "DISARM";
        case EdrController::State::ARMED:        return "ARMED ";
        case EdrController::State::POST_CAPTURE: return "CAPTUR";
    }
    return "------";
}

const char* trigger_label(EdrController::Trigger t) {
    switch (t) {
        case EdrController::Trigger::SHOCK: return "SHK";
        case EdrController::Trigger::TILT:  return "TLT";
        default:                            return "---";
    }
}

} // namespace

Ui::Ui(stm32f407::Wh1602& display, EdrController& edr)
    : display_(display), edr_(edr) {}

bool Ui::start() {
    const BaseType_t ok = xTaskCreate(
        &Ui::task_entry,
        "ui",
        512,
        this,
        tskIDLE_PRIORITY + 1,
        &task_handle_
    );
    return ok == pdPASS;
}

void Ui::on_button_event(const stm32f407::ButtonEvent& event) {
    if (event.type != stm32f407::ButtonEventType::PRESSED) {
        return;
    }

    switch (event.button) {
        case stm32f407::Button::SW1:
            screen_ = (screen_ + kScreenCount - 1) % kScreenCount; // Previous view
            break;

        case stm32f407::Button::SW2:
            edr_.toggle_arm(); // Arm / Disarm EDR
            break;

        case stm32f407::Button::SW3:
            screen_ = (screen_ + 1) % kScreenCount; // Next view
            break;

        default:
            break;
    }
}

void Ui::task_entry(void* arg) {
    static_cast<Ui*>(arg)->run();
}

void Ui::run() {
    // Wh1602::initialize() uses vTaskDelay(), so it must run inside a FreeRTOS task.
    display_.initialize();
    display_.set_backlight(true);

    while (true) {
        switch (screen_) {
            case 0:  render_edr_status(); break;
            case 1:  render_last_event(); break;
            default: render_controls();   break;
        }
        vTaskDelay(kRefreshPeriod);
    }
}

void Ui::render_edr_status() {
    char line0[17];
    char line1[17];

    int g100 = static_cast<int>(edr_.live_g() * 100.0f + 0.5f);
    if (g100 < 0)   g100 = 0;
    if (g100 > 999) g100 = 999; // Keep "G:X.XX" within 6 characters

    snprintf(line0, sizeof(line0), "EDR:%-6s  E:%02lu",
             state_label(edr_.state()),
             static_cast<unsigned long>(edr_.event_count()));

    snprintf(line1, sizeof(line1), "G:%d.%02d  LAST:%3s",
             g100 / 100, g100 % 100,
             trigger_label(edr_.last_trigger()));

    write_line(0, line0);
    write_line(1, line1);
}

void Ui::render_last_event() {
    const auto& snap = edr_.latest_snapshot();
    if (snap.event_id == 0) {
        write_line(0, "LAST SNAPSHOT");
        write_line(1, "NO EVENTS YET");
        return;
    }

    char line0[17];
    char line1[17];

    int pk100 = static_cast<int>(snap.peak_g * 100.0f + 0.5f);
    if (pk100 < 0)   pk100 = 0;
    if (pk100 > 999) pk100 = 999;

    const unsigned long t_sec_x10 = snap.timestamp_ms / 100U;

    snprintf(line0, sizeof(line0), "EVT #%02lu   %3s",
             static_cast<unsigned long>(snap.event_id),
             trigger_label(snap.trigger));

    snprintf(line1, sizeof(line1), "PK:%d.%02dg %lu.%lus",
             pk100 / 100, pk100 % 100,
             t_sec_x10 / 10U, t_sec_x10 % 10U);

    write_line(0, line0);
    write_line(1, line1);
}

void Ui::render_controls() {
    write_line(0, "192.168.137.42");
    write_line(1, "SW2:ARM SW1/3:PG");
}

void Ui::write_line(std::uint8_t row, const char* text) {
    char buf[stm32f407::Wh1602::columns];
    std::size_t i = 0;

    while (i < sizeof(buf) && text[i] != '\0') {
        buf[i] = text[i];
        ++i;
    }
    while (i < sizeof(buf)) {
        buf[i++] = ' ';
    }

    display_.write(row, 0, std::string_view{buf, sizeof(buf)});
}

} // namespace ui