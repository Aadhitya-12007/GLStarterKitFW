extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include <printf/printf.h>

#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/board.hpp"
#include "stm32f407/console.hpp"
#include "stm32f407/device_id.hpp"
#include "stm32f407/i2c1_bus.hpp"
#include "stm32f407/imu_driver.hpp"
#include "stm32f407/keyboard.hpp"
#include "stm32f407/wh1602.hpp"

#include "edr_controller.hpp"
#include "edr_mqtt.hpp"
#include "ethernet.hpp"
#include "mqtt_client.hpp"
#include "mqtt_client_id.hpp"
#include "settings.hpp"
#include "ui.hpp"

namespace {

// Bare-hardware drivers and EDR core (allocated in static .bss, never on MSP stack).
stm32f407::I2c1Bus i2c1_bus;
stm32f407::ImuDriver imu{i2c1_bus};
stm32f407::BlackBoxStorage storage;
EdrController edr_controller{imu, storage};

// Local UI and input peripherals.
stm32f407::Keyboard keyboard;
stm32f407::Wh1602 display;
Ui user_interface{display, edr_controller};

// Ethernet + MQTT telemetry/command stack.
network::Ethernet ethernet{Settings::ethernet_config};
const stm32f407::DeviceId device_id = stm32f407::read_device_id();
const MqttClientId mqtt_client_id = make_mqtt_client_id(device_id);
network::MqttClient mqtt_client{Settings::mqtt_broker, mqtt_client_id.data()};
network::EdrMqtt edr_mqtt{mqtt_client, device_id, edr_controller};

} // namespace

int main() {
    if (!stm32f407::initialize_system_clock()) {
        stm32f407::halt();
    }

    // Configures base board pins and isolates unused Discovery chips (PD4 LOW, PE3 HIGH).
    stm32f407::initialize_board();

    if (!stm32f407::start_console()) {
        stm32f407::halt();
    }

    printf("BlackBox-EDR booting\n");
    printf("Device ID: %s\n", device_id.data());

    // 1. Initialize I2C1 (PB6/PB9) and LSM9DS1 IMU (0x6B) before starting tasks.
    if (!i2c1_bus.init()) {
        printf("I2C1 init failed!\n");
    } else if (!imu.init()) {
        printf("IMU init failed! (WHO_AM_I=0x%02X)\n", imu.who_am_i());
    } else {
        printf("IMU ready (WHO_AM_I=0x%02X)\n", imu.who_am_i());
    }

    // 2. Initialize SPI1 SST25VF016B Flash (PD7 CS) and restore any persisted event.
    if (!storage.init()) {
        const auto id = storage.jedec_id();
        printf("Flash init failed! (JEDEC=%02X %02X %02X)\n", id[0], id[1], id[2]);
    } else {
        const auto id = storage.jedec_id();
        printf("Flash ready (JEDEC=%02X %02X %02X)\n", id[0], id[1], id[2]);
        edr_controller.load_from_flash();
    }

    // 3. Start UI and register it with Keyboard BEFORE keyboard.start().
    if (!user_interface.start() || !keyboard.add_listener(user_interface)) {
        stm32f407::halt();
    }
    keyboard.start();

    // 4. Start the 50 Hz EDR acquisition/FSM task.
    edr_controller.start();

    // 5. Wire MQTT listeners BEFORE mqtt_client.start() (configASSERT(!started_)),
    //    and attach Ethernet link-state notifications AFTER mqtt_client.start().
    mqtt_client.set_message_listener(edr_mqtt);
    if (!mqtt_client.add_broker_connection_listener(edr_mqtt)
        || !ethernet.start()
        || !mqtt_client.start()
        || !ethernet.add_link_state_client(mqtt_client)
        || !edr_mqtt.start()) {
        stm32f407::halt();
    }

    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();

    // Unreachable unless FreeRTOS heap_4 lacks RAM for Idle/Timer tasks.
    stm32f407::halt();
}