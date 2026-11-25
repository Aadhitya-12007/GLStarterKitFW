extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include <printf/printf.h>

#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/board.hpp"
#include "stm32f407/console.hpp"
#include "stm32f407/device_id.hpp"
#include "stm32f407/dht11.hpp"
#include "stm32f407/hw_security.hpp"
#include "stm32f407/i2c1_bus.hpp"
#include "stm32f407/imu_driver.hpp"
#include "stm32f407/keyboard.hpp"
#include "stm32f407/wh1602.hpp"

#include "edr_controller.hpp"
#include "ethernet.hpp"
#include "humidity_publisher.hpp"
#include "leds_controller.hpp"
#include "mqtt_client.hpp"
#include "mqtt_client_id.hpp"
#include "settings.hpp"
#include "tcp_echo_server.hpp"
#include "temperature_publisher.hpp"
#include "ui.hpp"

namespace {

network::Ethernet ethernet{Settings::ethernet_config};
network::TcpEchoServer echo_server;
stm32f407::Keyboard keyboard;

const stm32f407::DeviceId device_id = stm32f407::read_device_id();
const MqttClientId mqtt_client_id = make_mqtt_client_id(device_id);
network::MqttClient mqtt_client{Settings::mqtt_broker, mqtt_client_id.data()};
network::TemperaturePublisher temperature_publisher{mqtt_client, device_id};
network::HumidityPublisher humidity_publisher{mqtt_client, device_id};
LedsController leds_controller{mqtt_client, device_id};

stm32f407::Dht11 climate_sensor;

stm32f407::I2c1Bus i2c1_bus;
stm32f407::ImuDriver imu{i2c1_bus};
stm32f407::BlackBoxStorage storage{i2c1_bus};
EdrController edr_controller{imu, storage};

stm32f407::Wh1602 display;
ui::Ui user_interface{display, edr_controller};

} // namespace

int main() {
    if (!stm32f407::initialize_system_clock()) {
        stm32f407::halt();
    }

    stm32f407::initialize_board();

    if (!stm32f407::start_console()) {
        stm32f407::halt();
    }

    printf("BlackBox-EDR booting...\n");

    // 1. Initialize shared I2C1 bus and LSM9DS1 IMU
    if (!i2c1_bus.init() || !imu.init()) {
        printf("IMU init failed!\n");
    }

    // 2. Initialize SPI1 SST25VF016B Flash (JEDEC ID + unlock) and read EUI-48 MAC
    if (storage.init()) {
        const auto& mac = storage.cached_mac();
        printf("Storage OK | MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        printf("BlackBoxStorage init failed!\n");
    }

    // 3. Enable STM32 Hardware CRC-32 & Hardware RNG, then recover Flash chain
    stm32f407::hw_security::init();
    edr_controller.restore_from_flash();
    printf("Flash recovery: EVENTS=%lu CHAIN=%s SEED=0x%08lX\n",
           static_cast<unsigned long>(edr_controller.event_count()),
           edr_controller.chain_ok() ? "OK" : "TAMPER",
           static_cast<unsigned long>(edr_controller        .chain_seed()));

    // 4. Start UI before sources/listeners report to it
    if (!user_interface.start()
        || !keyboard.add_listener(user_interface)) {
        stm32f407::halt();
    }

    // Keep leds_controller.start() for PD12-PD15 GPIO init,
    // but detach SW2 so SW2 exclusively controls EDR Arm/Disarm.
    if (!leds_controller.start()) {
        stm32f407::halt();
    }
    keyboard.start();

    if (!ethernet.start()
        || !echo_server.start()
        || !ethernet.add_link_state_client(echo_server)) {
        stm32f407::halt();
    }

    if (!mqtt_client.start()
        || !ethernet.add_link_state_client(mqtt_client)
        || !temperature_publisher.start()
        || !humidity_publisher.start()) {
        stm32f407::halt();
    }

    if (!climate_sensor.add_humidity_listener(humidity_publisher, Settings::humidity_reporting_threshold)
        || !climate_sensor.add_temperature_listener(temperature_publisher, Settings::temperature_reporting_threshold)) {
        stm32f407::halt();
    }

    climate_sensor.start();
    edr_controller.start();

    printf("Starting FreeRTOS scheduler\n");

    vTaskStartScheduler();
    stm32f407::halt();
}