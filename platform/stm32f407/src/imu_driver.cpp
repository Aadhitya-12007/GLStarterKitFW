#include "stm32f407/imu_driver.hpp"

namespace stm32f407 {

ImuDriver::ImuDriver(I2c1Bus& bus) noexcept : bus_(bus) {
    ctx_.write_reg = &ImuDriver::platform_write;
    ctx_.read_reg  = &ImuDriver::platform_read;
    ctx_.handle    = &bus_;
}

int32_t ImuDriver::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len) {
    auto* bus = static_cast<I2c1Bus*>(handle);
    // ST's lsm9ds1-pid convention: return 0 on success, non-zero on error.
    return bus->write_reg(kI2cAddr7Bit, reg, bufp, len) ? 0 : -1;
}

int32_t ImuDriver::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len) {
    auto* bus = static_cast<I2c1Bus*>(handle);
    return bus->read_reg(kI2cAddr7Bit, reg, bufp, len) ? 0 : -1;
}

bool ImuDriver::init() {
    // 1. Verify LSM9DS1 identity on I2C1 (WHO_AM_I @ 0x0F must return 0x68).
    who_am_i_ = 0;
    if (lsm9ds1_read_reg(&ctx_, LSM9DS1_WHO_AM_I, &who_am_i_, 1) != 0) {
        return false;
    }
    if (who_am_i_ != LSM9DS1_IMU_ID) {
        return false;
    }

    // 2. Enable Block Data Update (BDU = 1) and Register Auto-Increment (IF_ADD_INC = 1)
    //    in CTRL_REG8 (0x22) so 16-bit samples never tear and 6-byte bursts advance automatically.
    lsm9ds1_ctrl_reg8_t ctrl8{};
    if (lsm9ds1_read_reg(&ctx_, LSM9DS1_CTRL_REG8, reinterpret_cast<uint8_t*>(&ctrl8), 1) != 0) {
        return false;
    }
    ctrl8.bdu        = PROPERTY_ENABLE;
    ctrl8.if_add_inc = PROPERTY_ENABLE;
    if (lsm9ds1_write_reg(&ctx_, LSM9DS1_CTRL_REG8, reinterpret_cast<uint8_t*>(&ctrl8), 1) != 0) {
        return false;
    }

    // 3. Set Full-Scale Ranges:
    //    - Accel +/-8g (prevents clipping above your 2.5g shock trigger)
    //    - Gyro  +/-500 dps (captures rapid vehicle rollover / spin)
    if (lsm9ds1_xl_full_scale_set(&ctx_, LSM9DS1_8g) != 0) {
        return false;
    }
    if (lsm9ds1_gy_full_scale_set(&ctx_, LSM9DS1_500dps) != 0) {
        return false;
    }

    // 4. Wake up BOTH Accelerometer + Gyroscope at 59.5 Hz ODR (writes CTRL_REG1_G 0x10).
    if (lsm9ds1_imu_data_rate_set(&ctx_, LSM9DS1_IMU_59Hz5) != 0) {
        return false;
    }

    return true;
}

bool ImuDriver::read_sample(ImuSample& out) {
    int16_t accel_raw[3]{0, 0, 0};
    int16_t gyro_raw[3]{0, 0, 0};

    // Burst-read 6 bytes from OUT_X_L_G..OUT_Z_H_G (0x18..0x1D)
    if (lsm9ds1_angular_rate_raw_get(&ctx_, gyro_raw) != 0) {
        return false;
    }

    // Burst-read 6 bytes from OUT_X_L_XL..OUT_Z_H_XL (0x28..0x2D)
    if (lsm9ds1_acceleration_raw_get(&ctx_, accel_raw) != 0) {
        return false;
    }

    out.ax = accel_raw[0];
    out.ay = accel_raw[1];
    out.az = accel_raw[2];
    out.gx = gyro_raw[0];
    out.gy = gyro_raw[1];
    out.gz = gyro_raw[2];
    return true;
}

} // namespace stm32f407