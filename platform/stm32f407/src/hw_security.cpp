#include "stm32f407/hw_security.hpp"

#include <cstring>
#include "stm32f4xx_hal.h"

namespace stm32f407::hw_security {

namespace {

// Reads one raw 32-bit sample from RNG_DR with clock/seed error recovery (RM0090 24.3.2).
bool read_rng_word(uint32_t& out) noexcept {
    const uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 20U) {
        const uint32_t sr = RNG->SR;

        // Clock error: PLL48CLK missing or too slow
        if ((sr & (RNG_SR_CEIS | RNG_SR_CECS)) != 0U) {
            RNG->SR &= ~RNG_SR_CEIS;
            return false;
        }

        // Seed error: restart analog entropy generator per RM0090 24.3.2
        if ((sr & (RNG_SR_SEIS | RNG_SR_SECS)) != 0U) {
            RNG->SR &= ~RNG_SR_SEIS;
            RNG->CR &= ~RNG_CR_RNGEN;
            RNG->CR |=  RNG_CR_RNGEN;
            continue;
        }

        if ((sr & RNG_SR_DRDY) != 0U) {
            out = RNG->DR;
            return true;
        }
    }
    return false;
}

} // namespace

void init() noexcept {
    __HAL_RCC_CRC_CLK_ENABLE();
    __HAL_RCC_RNG_CLK_ENABLE();

    // Clear any stale error flags and turn on the analog noise generator
    RNG->SR &= ~(RNG_SR_SEIS | RNG_SR_CEIS);
    RNG->CR |= RNG_CR_RNGEN;
}

bool generate_random_u32(uint32_t& out) noexcept {
    // RM0090 24.3.1 (FIPS PUB 140-2): discard the first word after enabling RNG
    // and ensure two consecutive readings are never identical.
    uint32_t first  = 0U;
    uint32_t second = 0U;

    if (read_rng_word(first) && read_rng_word(second) && second != first) {
        out = (second != 0U) ? second : 0xA55A1234U;
        return true;
    }

    // Safe fallback if PLL48CLK wasn't configured in starter clock init:
    // mix STM32 96-bit factory UID + tick counter through Hardware CRC-32.
    const uint32_t fallback[4] = {
        HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2(), HAL_GetTick() ^ 0x5A3C960FU
    };
    out = compute_crc32(fallback, sizeof(fallback));
    return false;
}

uint32_t compute_crc32(const void* data, std::size_t byte_len) noexcept {
    CRC->CR = CRC_CR_RESET; // Resets CRC_DR to 0xFFFFFFFF

    const auto* bytes = static_cast<const uint8_t*>(data);
    const std::size_t words = byte_len / 4U;

    for (std::size_t i = 0U; i < words; ++i) {
        uint32_t word = 0U;
        std::memcpy(&word, bytes + (i * 4U), sizeof(word));
        CRC->DR = word; // Hardware computes 32-bit CRC step in 4 HCLK cycles
    }

    return CRC->DR;
}

} // namespace stm32f407::hw_security