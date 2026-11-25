#pragma once

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "edr_controller.hpp"
#include "stm32f407/keyboard.hpp"
#include "stm32f407/wh1602.hpp"

namespace ui {

class Ui : public stm32f407::IButtonListener {
public:
    Ui(stm32f407::Wh1602& display, EdrController& edr);

    // Creates the LCD refresh task. Call before vTaskStartScheduler().
    bool start();

    // Called from Keyboard task when SW1..SW5 are pressed/released.
    void on_button_event(const stm32f407::ButtonEvent& event) override;

private:
    static constexpr int kScreenCount = 3;

    static void task_entry(void* arg);
    void run();

    void render_edr_status();
    void render_last_event();
    void render_controls();
    void write_line(std::uint8_t row, const char* text);

    stm32f407::Wh1602& display_;
    EdrController&     edr_;
    TaskHandle_t       task_handle_ = nullptr;

    // Written by Keyboard task (SW1/SW3), read by Ui task.
    volatile int screen_ = 0;
};

} // namespace ui