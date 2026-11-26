#include "stm32f407/imu_driver.hpp"

namespace stm32f407 {

ImuDriver::ImuDriver(I2c1Bus& bus) noexcept : bus_(bus) {
    ctx_.write_reg = &ImuDriver::platform_write;
    ctx_.read_reg  = &ImuDriver::platform_read;
    ctx_.handle    = &bus_;
}

int32_t ImuDriver::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto* bus = static_cast<I2c1Bus*>(handle);
    // ST's lsm9ds1-pid convention: 0 = success, non-zero = error.
    return bus->write_reg(kI2cAddr7Bit, reg, bufp, len) ? 0 : -1;
}

int32_t ImuDriver::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto* bus = static_cast<I2c1Bus*>(handle);
    return bus->read_reg(kI2cAddr7Bit, reg, bufp, len) ? 0 : -1;
}

bool ImuDriver::init() {
    // 1. Verify LSM9DS1 identity on I2C1 (0x68).
    who_am_i_ = 0;
    if (lsm9ds1_read_reg(&ctx_, LSM9DS1_WHO_AM_I, &who_am_i_, 1) != 0) {
        return false;
    }
    if (who_am_i_ != LSM9DS1_IMU_ID) {
        return false;
    }

    // 2. Read-Modify-Write CTRL_REG8 (0x22):
    //    - BDU = 1        -> prevents high/low byte tearing during 16-bit reads.
    //    - IF_ADD_INC = 1 -> auto-increments register pointer across 0x28..0x2D.
    lsm9ds1_ctrl_reg8_t ctrl8{};
    if (lsm9ds1_read_reg(&ctx_, LSM9DS1_CTRL_REG8, reinterpret_cast<uint8_t*>(&ctrl8), 1) != 0) {
        return false;
    }
    ctrl8.bdu        = PROPERTY_ENABLE;
    ctrl8.if_add_inc = PROPERTY_ENABLE;
    if (lsm9ds1_write_reg(&ctx_, LSM9DS1_CTRL_REG8, reinterpret_cast<uint8_t*>(&ctrl8), 1) != 0) {
        return false;
    }

    // 3. Set accelerometer full scale to +/-8g (0.244 mg/LSB) in CTRL_REG6_XL (0x20).
    //    Default +/-2g clips below our 2.5g shock trigger.
    if (lsm9ds1_xl_full_scale_set(&ctx_, LSM9DS1_8g) != 0) {
        return false;
    }

    //    Writes ODR_G = 000 (Gyro POWER-DOWN) in CTRL_REG1_G (0x10)
    //    ODR_XL = 011 (119 Hz Accel in CTRL_REG6_XL 0x20)
    if (lsm9ds1_imu_data_rate_set(&ctx_, LSM9DS1_GY_OFF_XL_119Hz) != 0) {
        return false;
    }

    return true;
}

bool ImuDriver::read_sample(ImuSample& out) {
    int16_t accel_raw[3]{0, 0, 0};

    // Single 6-byte I2C read from (0x28 to 0x2D).
    if (lsm9ds1_acceleration_raw_get(&ctx_, accel_raw) != 0) {
        return false;
    }

    out.ax = accel_raw[0];
    out.ay = accel_raw[1];
    out.az = accel_raw[2];
    return true;
}

} // namespace stm32f407