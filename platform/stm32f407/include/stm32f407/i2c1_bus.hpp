#pragma once

#include <cstdint>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace stm32f407 {

// Single-owner wrapper for STM32F407 I2C1 (PB6 SCL / PB9 SDA).
// Rule: Only call after init() from one owner task (EdrController).
class I2c1Bus {
public:
    I2c1Bus() = default;
    I2c1Bus(const I2c1Bus&) = delete;
    I2c1Bus& operator=(const I2c1Bus&) = delete;

    // Configures PB6/PB9 (AF4 Open-Drain) and I2C1 at 400 kHz Fast Mode.
    bool init();

    // addr_7bit: 7-bit I2C slave address (0x6B for LSM9DS1, 0x50 for EEPROM).
    bool write_reg(uint8_t addr_7bit, uint8_t reg, const uint8_t* data, uint16_t len);
    bool read_reg(uint8_t addr_7bit, uint8_t reg, uint8_t* data, uint16_t len);

private:
    I2C_HandleTypeDef hi2c1_{};
    bool initialized_{false};

    static constexpr uint32_t kI2cClockHz   = 400000U; // 400 kHz Fast Mode
    static constexpr uint32_t kHalTimeoutMs = 50U;
};

} // namespace stm32f407