#include "stm32f407/i2c1_bus.hpp"

namespace stm32f407 {

bool I2c1Bus::init() {
    if (initialized_) {
        return true;
    }

    // 1. Turn on clocks for GPIO Port B and the I2C1 peripheral.
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    // 2. Wire PB6 (SCL) and PB9 (SDA) to I2C1 (AF4) in Open-Drain mode.
    //    External 4.7 kOhm pull-ups are already on the GL board -> GPIO_NOPULL.
    GPIO_InitTypeDef i2c_pins{};
    i2c_pins.Pin       = GPIO_PIN_6 | GPIO_PIN_9;
    i2c_pins.Mode      = GPIO_MODE_AF_OD;
    i2c_pins.Pull      = GPIO_NOPULL;
    i2c_pins.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    i2c_pins.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &i2c_pins);

    // 3. Reset I2C1 silicon once so a stale BUSY flag after debugging cannot lock the bus.
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    // 4. Configure I2C1 as 7-bit Master at 400 kHz Fast Mode.
    hi2c1_.Instance             = I2C1;
    hi2c1_.Init.ClockSpeed      = kI2cClockHz;
    hi2c1_.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1_.Init.OwnAddress1     = 0;
    hi2c1_.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1_.Init.OwnAddress2     = 0;
    hi2c1_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1_.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1_) != HAL_OK) {
        return false;
    }

    initialized_ = true;
    return true;
}

bool I2c1Bus::write_reg(uint8_t addr_7bit, uint8_t reg, const uint8_t* data, uint16_t len) {
    if (!initialized_) {
        return false;
    }

    const uint16_t dev_addr_8bit = static_cast<uint16_t>(addr_7bit << 1);
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &hi2c1_,
        dev_addr_8bit,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        const_cast<uint8_t*>(data),
        len,
        kHalTimeoutMs
    );
    return status == HAL_OK;
}

bool I2c1Bus::read_reg(uint8_t addr_7bit, uint8_t reg, uint8_t* data, uint16_t len) {
    if (!initialized_) {
        return false;
    }

    const uint16_t dev_addr_8bit = static_cast<uint16_t>(addr_7bit << 1);
    const HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1_,
        dev_addr_8bit,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len,
        kHalTimeoutMs
    );
    return status == HAL_OK;
}

} // namespace stm32f407