#pragma once

#include <cstdint>

#include "stm32f407/i2c1_bus.hpp"

extern "C" {
#include "lsm9ds1_reg.h"
}

namespace stm32f407 {

// Compact 6-byte packed 3-axis accelerometer sample.
// Stored directly in RingBuffer (150 * 6 = 900 B) and Flash Sector 0 CrashRecord.
struct __attribute__((packed)) ImuSample {
    int16_t ax; // Raw X acceleration (0.244 mg/LSB at +/-8g)
    int16_t ay; // Raw Y acceleration (0.244 mg/LSB at +/-8g)
    int16_t az; // Raw Z acceleration (0.244 mg/LSB at +/-8g)
};
static_assert(sizeof(ImuSample) == 6U, "ImuSample must remain exactly 6 bytes");

class ImuDriver {
public:
    static constexpr uint8_t kI2cAddr7Bit   = 0x6B;   // SDO_A/G tied HIGH -> 7-bit 0x6B (HAL 0xD6)
    static constexpr float   kAccelMgPerLsb = 0.244f; // +/-8g full scale (1.0g ~= 4098 LSB)

    explicit ImuDriver(I2c1Bus& bus) noexcept;

    // Verifies WHO_AM_I == 0x68, enables BDU + auto-increment, sets +/-8g,
    // powers down the gyroscope, and starts the accelerometer at 119 Hz ODR.
    bool init();

    // Single 6-byte I2C1 burst read from OUT_X_L_XL..OUT_Z_H_XL (0x28..0x2D).
    bool read_sample(ImuSample& out);

    uint8_t who_am_i() const noexcept { return who_am_i_; }

    // Converts raw 16-bit signed LSB count to physical g-force.
    static constexpr float raw_to_g(int16_t raw) noexcept {
        return (static_cast<float>(raw) * kAccelMgPerLsb) * 0.001f;
    }

private:
    // Static C-compatible trampolines required by ST's stmdev_ctx_t function pointers.
    static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
    static int32_t platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len);

    I2c1Bus&     bus_;
    stmdev_ctx_t ctx_{};
    uint8_t      who_am_i_{0};
};

} // namespace stm32f407