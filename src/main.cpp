extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include <printf/printf.h>

#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/board.hpp"
#include "stm32f407/console.hpp"
#include "stm32f407/i2c1_bus.hpp"
#include "stm32f407/imu_driver.hpp"
#include "stm32f407/keyboard.hpp"
#include "stm32f407/wh1602.hpp"

#include "edr_controller.hpp"
#include "ui.hpp"

namespace {

stm32f407::I2c1Bus i2c1_bus;
stm32f407::ImuDriver imu{i2c1_bus};
stm32f407::BlackBoxStorage storage;

EdrController edr_controller{imu, storage};

stm32f407::Keyboard keyboard;
stm32f407::Wh1602 display;
Ui user_interface{display, edr_controller};

} // namespace

int main() {
    if (!stm32f407::initialize_system_clock()) {
        stm32f407::halt();
    }

    stm32f407::initialize_board();

    if (!stm32f407::start_console()) {
        stm32f407::halt();
    }

    printf("BlackBox-EDR bare-hardware boot\n");

    if (!i2c1_bus.init()) {
        printf("I2C1 init failed!\n");
    } else if (!imu.init()) {
        printf("IMU init failed! (WHO_AM_I=0x%02X)\n", imu.who_am_i());
    } else {
        printf("IMU ready (WHO_AM_I=0x%02X)\n", imu.who_am_i());
    }

    if (!storage.init()) {
        const auto id = storage.jedec_id();
        printf("Flash init failed! (JEDEC=%02X %02X %02X)\n", id[0], id[1], id[2]);
    } else {
        const auto id = storage.jedec_id();
        printf("Flash ready (JEDEC=%02X %02X %02X)\n", id[0], id[1], id[2]);
        edr_controller.load_from_flash();
    }

    if (!user_interface.start() || !keyboard.add_listener(user_interface)) {
        stm32f407::halt();
    }

    keyboard.start();
    edr_controller.start();

    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();

    stm32f407::halt();
}