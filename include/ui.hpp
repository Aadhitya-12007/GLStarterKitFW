#pragma once

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "edr_controller.hpp"
#include "stm32f407/keyboard.hpp"
#include "stm32f407/wh1602.hpp"

class Ui {
public:
    Ui(stm32f407::Wh1602& display,
       stm32f407::Keyboard& keyboard,
       EdrController& edr);

    // Starts the 10 Hz display + LED refresh task. Call before vTaskStartScheduler().
    void start();

    // Called from Keyboard task when a D-pad button is pressed.
    // Match your starter repo's Keyboard button enum type if named differently.
    void on_button_event(stm32f407::Keyboard::Button button);

    // Called from Ethernet / MqttClient callbacks to update Screen 2.
    void set_eth_connected(bool connected)  { eth_up_  = connected; }
    void set_mqtt_connected(bool connected) { mqtt_up_ = connected; }

private:
    static constexpr uint8_t kScreenCount = 3U;

    static void task_entry(void* arg);
    void run();

    void update_leds();
    void render_screen();
    void write_line(uint8_t row, const char* text);

    stm32f407::Wh1602&   display_;
    stm32f407::Keyboard& keyboard_;
    EdrController&       edr_;

    TaskHandle_t task_handle_ = nullptr;

    // Shared across tasks (single-byte aligned reads/writes on single-core Cortex-M4)
    volatile uint8_t screen_  = 0U;
    volatile bool    eth_up_  = false;
    volatile bool    mqtt_up_ = false;

    // Shadow buffer so we only write to the 16x2 display when text actually changes
    char last_lines_[2][17]{};
};