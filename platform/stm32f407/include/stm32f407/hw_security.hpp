#pragma once

#include <cstddef>
#include <cstdint>

namespace stm32f407::hw_security {

// Enables AHB1 CRC clock and AHB2 RNG clock (+ RNGEN analog noise generator).
void init() noexcept;

// Generates a true 32-bit random word from STM32F407 analog RNG (FIPS 140-2 checked).
// Returns true on hardware RNG success; writes a non-zero fallback seed if RNG clock fails.
bool generate_random_u32(uint32_t& out) noexcept;

// Computes STM32 Hardware CRC-32 (poly 0x04C11DB7, init 0xFFFFFFFF) over 32-bit words.
// byte_len must be a multiple of 4.
uint32_t compute_crc32(const void* data, std::size_t byte_len) noexcept;

} // namespace stm32f407::hw_security