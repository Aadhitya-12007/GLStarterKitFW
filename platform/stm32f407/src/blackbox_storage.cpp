#include "stm32f407/blackbox_storage.hpp"

namespace stm32f407 {

namespace {

constexpr uint32_t kSpiTimeoutMs      = 100U;
constexpr uint32_t kEraseTimeoutMs    = 100U; // 4 KB sector erase: 18 ms typ, 25 ms max
constexpr uint32_t kByteProgTimeoutMs = 10U;  // Byte program: 7 us typ, 10 us max

// SST25VF016B SPI opcodes
constexpr uint8_t kCmdWrsr        = 0x01U; // Write Status Register
constexpr uint8_t kCmdByteProgram = 0x02U; // Program 1 byte
constexpr uint8_t kCmdRead        = 0x03U; // Standard Read (up to 25 MHz)
constexpr uint8_t kCmdRdsr        = 0x05U; // Read Status Register
constexpr uint8_t kCmdWren        = 0x06U; // Write Enable
constexpr uint8_t kCmdSectorErase = 0x20U; // 4 KB Sector Erase
constexpr uint8_t kCmdEwsr        = 0x50U; // Enable Write Status Register
constexpr uint8_t kCmdJedecId     = 0x9FU; // Read JEDEC ID (expect BF 25 41)

constexpr uint8_t kStatusBusyMask = 0x01U; // RDSR bit 0: 1 = internal write/erase active

// RAII Chip-Select guard for PD7 (SPI1_CS0#): guarantees CS always returns HIGH.
struct CsGuard {
    CsGuard()  { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); }
    ~CsGuard() { HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);   }
};

} // namespace

bool BlackBoxStorage::init() {
    ready_ = false;
    jedec_id_.fill(0U);

    if (!init_spi1()) {
        return false;
    }

    if (!read_jedec_id(jedec_id_)) {
        return false;
    }

    // SST25VF016B JEDEC ID: 0xBF (Microchip/SST), 0x25 (SST25VF), 0x41 (16 Mbit)
    if (jedec_id_[0] != 0xBFU || jedec_id_[1] != 0x25U || jedec_id_[2] != 0x41U) {
        return false;
    }

    if (!unlock_write_protection()) {
        return false;
    }

    ready_ = true;
    return true;
}

bool BlackBoxStorage::read_sector(uint32_t sector, uint8_t* dst, std::size_t len) {
    if (!ready_ || sector >= kNumSectors || dst == nullptr || len == 0U || len > kSectorSize) {
        return false;
    }

    const uint32_t addr = sector * static_cast<uint32_t>(kSectorSize);
    uint8_t cmd[4] = {
        kCmdRead,
        static_cast<uint8_t>((addr >> 16U) & 0xFFU),
        static_cast<uint8_t>((addr >> 8U)  & 0xFFU),
        static_cast<uint8_t>(addr          & 0xFFU),
    };

    CsGuard cs;
    if (HAL_SPI_Transmit(&hspi1_, cmd, sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
        return false;
    }
    return HAL_SPI_Receive(&hspi1_, dst, static_cast<uint16_t>(len), kSpiTimeoutMs) == HAL_OK;
}

bool BlackBoxStorage::write_sector(uint32_t sector, const uint8_t* src, std::size_t len) {
    if (!ready_ || sector >= kNumSectors || src == nullptr || len == 0U || len > kSectorSize) {
        return false;
    }

    const uint32_t base_addr = sector * static_cast<uint32_t>(kSectorSize);
    if (!erase_sector(base_addr)) {
        return false;
    }

    for (std::size_t i = 0U; i < len; ++i) {
        if (src[i] == 0xFFU) {
            continue; // Freshly erased NOR Flash bytes are already 0xFF
        }
        if (!program_byte(base_addr + static_cast<uint32_t>(i), src[i])) {
            return false;
        }
    }

    return true;
}

bool BlackBoxStorage::init_spi1() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    // Keep Discovery onboard accelerometer CS (PE3) HIGH so it stays tri-stated on PA5
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    GPIO_InitTypeDef gpio{};
    gpio.Pin   = GPIO_PIN_3;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    // PD7: SPI1_CS0# (idle HIGH before enabling output)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    gpio.Pin   = GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &gpio);

    // PA5: SPI1_SCK (AF5)
    gpio.Pin       = GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio);

    // PB4: SPI1_MISO, PB5: SPI1_MOSI (AF5)
    gpio.Pin       = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);

    // SPI1 sits on APB2 (84 MHz). Prescaler 16 -> 5.25 MHz SCK:
    // fast (~33 ms full sector write) and cleanly decodable on a 24 MHz Logic Analyzer.
    hspi1_.Instance               = SPI1;
    hspi1_.Init.Mode              = SPI_MODE_MASTER;
    hspi1_.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1_.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1_.Init.CLKPolarity       = SPI_POLARITY_LOW;       // SPI Mode 0 (CPOL=0)
    hspi1_.Init.CLKPhase          = SPI_PHASE_1EDGE;        // SPI Mode 0 (CPHA=0)
    hspi1_.Init.NSS               = SPI_NSS_SOFT;           // Software CS on PD7
    hspi1_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1_.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1_.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1_.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1_.Init.CRCPolynomial     = 7U;

    return HAL_SPI_Init(&hspi1_) == HAL_OK;
}

bool BlackBoxStorage::read_jedec_id(std::array<uint8_t, 3>& out) {
    uint8_t cmd = kCmdJedecId;
    CsGuard cs;
    if (HAL_SPI_Transmit(&hspi1_, &cmd, 1U, kSpiTimeoutMs) != HAL_OK) {
        return false;
    }
    return HAL_SPI_Receive(&hspi1_, out.data(), static_cast<uint16_t>(out.size()), kSpiTimeoutMs) == HAL_OK;
}

bool BlackBoxStorage::unlock_write_protection() {
    // SST25VF016B powers up with BP0..BP3 write-protected; EWSR (0x50) + WRSR (0x01 0x00) unlocks it.
    {
        uint8_t ewsr = kCmdEwsr;
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, &ewsr, 1U, kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }
    {
        uint8_t wrsr[2] = {kCmdWrsr, 0x00U};
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, wrsr, sizeof(wrsr), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }
    return wait_busy(kByteProgTimeoutMs);
}

bool BlackBoxStorage::write_enable() {
    uint8_t cmd = kCmdWren;
    CsGuard cs;
    return HAL_SPI_Transmit(&hspi1_, &cmd, 1U, kSpiTimeoutMs) == HAL_OK;
}

bool BlackBoxStorage::wait_busy(uint32_t timeout_ms) {
    const uint32_t start = HAL_GetTick();
    uint8_t cmd    = kCmdRdsr;
    uint8_t status = 0U;

    while (true) {
        {
            CsGuard cs;
            if (HAL_SPI_Transmit(&hspi1_, &cmd, 1U, kSpiTimeoutMs) != HAL_OK) {
                return false;
            }
            if (HAL_SPI_Receive(&hspi1_, &status, 1U, kSpiTimeoutMs) != HAL_OK) {
                return false;
            }
        }

        if ((status & kStatusBusyMask) == 0U) {
            return true;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
    }
}

bool BlackBoxStorage::erase_sector(uint32_t addr) {
    if (!write_enable()) {
        return false;
    }

    uint8_t cmd[4] = {
        kCmdSectorErase,
        static_cast<uint8_t>((addr >> 16U) & 0xFFU),
        static_cast<uint8_t>((addr >> 8U)  & 0xFFU),
        static_cast<uint8_t>(addr          & 0xFFU),
    };
    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, cmd, sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }
    return wait_busy(kEraseTimeoutMs);
}

bool BlackBoxStorage::program_byte(uint32_t addr, uint8_t byte) {
    if (!write_enable()) {
        return false;
    }

    uint8_t cmd[5] = {
        kCmdByteProgram,
        static_cast<uint8_t>((addr >> 16U) & 0xFFU),
        static_cast<uint8_t>((addr >> 8U)  & 0xFFU),
        static_cast<uint8_t>(addr          & 0xFFU),
        byte,
    };
    {
        CsGuard cs;
        if (HAL_SPI_Transmit(&hspi1_, cmd, sizeof(cmd), kSpiTimeoutMs) != HAL_OK) {
            return false;
        }
    }
    return wait_busy(kByteProgTimeoutMs);
}

} // namespace stm32f407