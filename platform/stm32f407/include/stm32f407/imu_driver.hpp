#pragma once

#include <cstdint>

#include "stm32f407/i2c1_bus.hpp"

extern "C" {
#include "lsm9ds1_reg.h"
}

namespace stm32f407 {

// Exact 12-byte packed sample stored directly in Day 2's RingBuffer & Day 3's Flash IncidentRecord.
struct __attribute__((packed)) ImuSample {
    int16_t ax; // Raw X acceleration (0.244 mg/LSB at +/-8g)
    int16_t ay; // Raw Y acceleration (0.244 mg/LSB at +/-8g)
    int16_t az; // Raw Z acceleration (0.244 mg/LSB at +/-8g)
    int16_t gx; // Raw X angular rate (17.50 mdps/LSB at +/-500 dps)
    int16_t gy; // Raw Y angular rate (17.50 mdps/LSB at +/-500 dps)
    int16_t gz; // Raw Z angular rate (17.50 mdps/LSB at +/-500 dps)
};
static_assert(sizeof(ImuSample) == 12, "ImuSample must remain exactly 12 bytes");

class ImuDriver {
public:
    static constexpr uint8_t kI2cAddr7Bit    = 0x6B;    // SDO_A/G tied HIGH -> 7-bit 0x6B (8-bit 0xD6/0xD7)
    static constexpr float   kAccelMgPerLsb  = 0.244f;  // +/-8g full scale (1.0g ~= 4098 LSB)
    static constexpr float   kGyroMdpsPerLsb = 17.50f;  // +/-500 dps full scale

    explicit ImuDriver(I2c1Bus& bus) noexcept;

    // Verifies WHO_AM_I == 0x68, enables BDU + auto-increment, sets +/-8g, +/-500 dps, 59.5 Hz ODR.
    bool init();

    // Performs two 6-byte burst reads (Gyro 0x18..0x1D and Accel 0x28..0x2D).
    bool read_sample(ImuSample& out);

    uint8_t who_am_i() const noexcept { return who_am_i_; }

    // Convenience unit conversion helpers for UI / debugging (keep Flash records raw int16_t).
    static constexpr float raw_to_g(int16_t raw) noexcept {
        return (static_cast<float>(raw) * kAccelMgPerLsb) * 0.001f;
    }
    static constexpr float raw_to_dps(int16_t raw) noexcept {
        return (static_cast<float>(raw) * kGyroMdpsPerLsb) * 0.001f;
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