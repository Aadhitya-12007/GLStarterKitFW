#include "stm32f407/blackbox_storage.hpp"

#include <cstring>

namespace stm32f407 {

// ---------------------------------------------------------------------------
// RAII Chip-Select Guard (PD7 = active-low SPI1_CS0#)
// ---------------------------------------------------------------------------
BlackBoxStorage::CsGuard::CsGuard() noexcept {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

BlackBoxStorage::CsGuard::~CsGuard() noexcept {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

// ---------------------------------------------------------------------------
// Constructor & Two-Phase Initialization
// ---------------------------------------------------------------------------
BlackBoxStorage::BlackBoxStorage(I2c1Bus& i2c1) noexcept
    : i2c1_(i2c1) {}

bool BlackBoxStorage::init() noexcept {
    ready_ = false;

    if (!init_spi1_hardware()) {
        return false;
    }

    // 1. Verify SST25VF016B identity over SPI1 (expected: 0xBF, 0x25, 0x41)
    std::array<uint8_t, 3> id{};
    if (!read_jedec_id(id)) {
        return false;
    }
    if (id[0] != kJedecMfrId || id[1] != kJedecMemType || id[2] != kJedecCapacity) {
        return false;
    }

    // 2. CRITICAL: SST25VF016B powers up with BP0..BP3 = 1 (all blocks write-protected).
    if (!unlock_write_protection()) {
        return false;
    }

    // 3. Read globally unique factory EUI-48 MAC from 24AA02E48T (read-only)
    if (!read_mac(mac_)) {
        return false;
    }

    ready_ = true;
    return true;
}

bool BlackBoxStorage::init_spi1_hardware() noexcept {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Drive CS (PD7) HIGH *before* enabling output mode so Flash never sees a glitch LOW.
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio{};
    gpio.Pin       = GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);

    // PA5 -> SPI1_SCK (AF5)
    gpio.Pin       = GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PB4 -> SPI1_MISO, PB5 -> SPI1_MOSI (AF5)
    gpio.Pin       = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);

    // SPI1 sits on APB2 (84 MHz at 168 MHz SYSCLK).
    // Prescaler /8 => 10.5 MHz SCK: fast, clean edges, safely below 25 MHz limit for 0x03 Read.
    hspi1_.Instance               = SPI1;
    hspi1_.Init.Mode              = SPI_MODE_MASTER;
    hspi1_.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1_.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1_.Init.CLKPolarity       = SPI_POLARITY_LOW;       // SPI Mode 0 (CPOL=0)
    hspi1_.Init.CLKPhase          = SPI_PHASE_1EDGE;        // SPI Mode 0 (CPHA=0)
    hspi1_.Init.NSS               = SPI_NSS_SOFT;           // Software CS on PD7
    hspi1_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1_.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1_.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1_.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;

    return HAL_SPI_Init(&hspi1_) == HAL_OK;
}

// ---------------------------------------------------------------------------
// Hardware Identity: SST25VF016B JEDEC ID & 24AA02E48T Factory EUI-48 MAC
// ---------------------------------------------------------------------------
bool BlackBoxStorage::read_jedec_id(std::array<uint8_t, 3>& id_out) noexcept {
    const uint8_t cmd = kCmdJedecId;
    CsGuard cs;

    if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(&cmd), 1U, kSpiTimeoutMs) != HAL_OK) {
        return false;
    }
    return HAL_SPI_Receive(&hspi1_, id_out.data(), static_cast<uint16_t>(id_out.size()), kSpiTimeoutMs) == HAL_OK;
}

bool BlackBoxStorage::read_mac(std::array<uint8_t, 6>& mac_out) noexcept {
    if (!i2c1_.read_reg(kEepromAddr7Bit, kEepromMacReg, mac_out.data(), mac_out.size())) {
        return false;
    }

    // Sanity check: an unpowered/floating I2C line returns all 0x00 or all 0xFF.
    bool all_zero = true;
    bool all_ff   = true;
    for (uint8_t b : mac_out) {
        if (b != 0x00U) all_zero = false;
        if (b != 0xFFU) all_ff   = false;
    }
    return !(all_zero || all_ff);
}

// ---------------------------------------------------------------------------
// Public 4 KB Sector API
// ---------------------------------------------------------------------------
bool BlackBoxStorage::erase_sector(uint32_t sector_index) noexcept {
    if (sector_index >= kTotalSectors) {
        return false;
    }

    const uint32_t addr = sector_index * kSectorSize;
    const uint8_t cmd[4] = {
        kCmdSectorErase,
        static_cast<uint8_t>((addr >> 16) & 0xFFU),
        static_cast<uint8_t>((addr >>  8) & 0xFFU),
        static_cast<uint8_t>( addr        & 0xFFU)
    };

    if (!write_enable()) {
        return false;
    }

    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(cmd), sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    } // CS rising edge triggers the internal 4 KB sector erase (~18 ms typ, 25 ms max)

    return wait_busy(100U);
}

bool BlackBoxStorage::write_sector(uint32_t sector_index, const uint8_t* data, std::size_t len) noexcept {
    if (data == nullptr || len == 0U || len > kSectorSize || sector_index >= kTotalSectors) {
        return false;
    }

    // NOR Flash rule: bits can only transition 1 -> 0; always erase sector to 0xFF first.
    if (!erase_sector(sector_index)) {
        return false;
    }

    const uint32_t base_addr = sector_index * kSectorSize;
    for (std::size_t i = 0U; i < len; ++i) {
        // Erased Flash is already 0xFF; skip programming 0xFF bytes to save time & wear.
        if (data[i] == 0xFFU) {
            continue;
        }
        if (!program_byte(base_addr + static_cast<uint32_t>(i), data[i])) {
            return false;
        }
    }
    return true;
}

bool BlackBoxStorage::read_sector(uint32_t sector_index, uint8_t* out, std::size_t len) noexcept {
    if (out == nullptr || len == 0U || len > kSectorSize || sector_index >= kTotalSectors) {
        return false;
    }
    return read_bytes(sector_index * kSectorSize, out, len);
}

uint32_t BlackBoxStorage::scan_valid_records(uint32_t expected_magic, uint32_t max_slots) noexcept {
    if (max_slots > kTotalSectors) {
        max_slots = kTotalSectors;
    }

    uint32_t count = 0U;
    for (uint32_t slot = 0U; slot < max_slots; ++slot) {
        uint8_t raw[4]{};
        if (!read_bytes(slot * kSectorSize, raw, sizeof(raw))) {
            break;
        }
        uint32_t magic = 0U;
        std::memcpy(&magic, raw, sizeof(magic));
        if (magic != expected_magic) {
            break; // First erased (0xFFFFFFFF) or invalid slot ends the contiguous log
        }
        ++count;
    }
    return count;
}

// ---------------------------------------------------------------------------
// Low-Level SST25VF016B Primitives
// ---------------------------------------------------------------------------
bool BlackBoxStorage::unlock_write_protection() noexcept {
    // Sequence from SST25VF016B datasheet: EWSR (0x50), then WRSR (0x01, 0x00)
    const uint8_t ewsr = kCmdEnableWrsr;
    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(&ewsr), 1U, kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }

    const uint8_t wrsr[2] = { kCmdWriteStatus, 0x00U };
    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(wrsr), sizeof(wrsr), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }

    return wait_busy(10U);
}

bool BlackBoxStorage::write_enable() noexcept {
    const uint8_t cmd = kCmdWriteEnable;
    CsGuard cs;
    return HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(&cmd), 1U, kSpiTimeoutMs) == HAL_OK;
}

bool BlackBoxStorage::wait_busy(uint32_t timeout_ms) noexcept {
    const uint32_t start = HAL_GetTick();
    const uint8_t cmd    = kCmdReadStatus;

    while (true) {
        uint8_t status = 0xFFU;
        {
            CsGuard cs;
            if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(&cmd), 1U, kSpiTimeoutMs) != HAL_OK) {
                return false;
            }
            if (HAL_SPI_Receive(&hspi1_, &status, 1U, kSpiTimeoutMs) != HAL_OK) {
                return false;
            }
        }

        if ((status & kStatusBusyBit) == 0U) {
            return true;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
    }
}

bool BlackBoxStorage::program_byte(uint32_t addr, uint8_t value) noexcept {
    if (!write_enable()) {
        return false;
    }

    const uint8_t cmd[5] = {
        kCmdByteProgram,
        static_cast<uint8_t>((addr >> 16) & 0xFFU),
        static_cast<uint8_t>((addr >>  8) & 0xFFU),
        static_cast<uint8_t>( addr        & 0xFFU),
        value
    };

    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(cmd), sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    } // CS rising edge commits the single byte (~7 us typ, 10 us max)

    return wait_busy(5U);
}

bool BlackBoxStorage::read_bytes(uint32_t addr, uint8_t* out, std::size_t len) noexcept {
    const uint8_t cmd[4] = {
        kCmdRead,
        static_cast<uint8_t>((addr >> 16) & 0xFFU),
        static_cast<uint8_t>((addr >>  8) & 0xFFU),
        static_cast<uint8_t>( addr        & 0xFFU)
    };

    CsGuard cs;
    if (HAL_SPI_Transmit(&hspi1_, const_cast<uint8_t*>(cmd), sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
        return false;
    }
    return HAL_SPI_Receive(&hspi1_, out, static_cast<uint16_t>(len), kSpiTimeoutMs) == HAL_OK;
}

} // namespace stm32f407