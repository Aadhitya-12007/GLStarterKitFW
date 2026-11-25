#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "stm32f4xx_hal.h"
#include "i2c1_bus.hpp"

namespace stm32f407 {

class BlackBoxStorage {
public:
    static constexpr uint32_t kSectorSize      = 4096U;            // 4 KB erase sector
    static constexpr uint32_t kTotalSectors    = 512U;             // 512 * 4 KB = 2 MB (16 Mbit)
    static constexpr uint32_t kFlashSizeBytes  = kSectorSize * kTotalSectors;

    static constexpr uint8_t  kJedecMfrId      = 0xBFU;            // SST / Microchip
    static constexpr uint8_t  kJedecMemType    = 0x25U;            // SPI Flash
    static constexpr uint8_t  kJedecCapacity   = 0x41U;            // 16 Mbit (SST25VF016B)

    static constexpr uint8_t  kEepromAddr7Bit  = 0x50U;            // 24AA02E48T on I2C1
    static constexpr uint8_t  kEepromMacReg    = 0xFAU;            // Factory EUI-48: 0xFA..0xFF

    explicit BlackBoxStorage(I2c1Bus& i2c1) noexcept;

    BlackBoxStorage(const BlackBoxStorage&)            = delete;
    BlackBoxStorage& operator=(const BlackBoxStorage&) = delete;

    // Configures SPI1 + PD7 CS, checks JEDEC ID, unlocks Flash write-protect, reads MAC.
    bool init() noexcept;

    // Reads 3-byte JEDEC ID (expected: 0xBF, 0x25, 0x41).
    bool read_jedec_id(std::array<uint8_t, 3>& id_out) noexcept;

    // Reads 6-byte factory EUI-48 MAC directly from 24AA02E48T (0xFA..0xFF).
    bool read_mac(std::array<uint8_t, 6>& mac_out) noexcept;

    // Returns the cached 6-byte MAC read during init() (zero I2C traffic).
    const std::array<uint8_t, 6>& cached_mac() const noexcept { return mac_; }
    bool is_ready() const noexcept { return ready_; }

    // 4 KB sector operations (1 IncidentRecord per sector: addr = sector_index * 4096).
    bool erase_sector(uint32_t sector_index) noexcept;
    bool write_sector(uint32_t sector_index, const uint8_t* data, std::size_t len) noexcept;
    bool read_sector(uint32_t sector_index, uint8_t* out, std::size_t len) noexcept;

    // Boot-time recovery: counts sequential sectors starting with expected_magic.
    uint32_t scan_valid_records(uint32_t expected_magic, uint32_t max_slots = 64U) noexcept;

private:
    // SST25VF016B SPI opcodes
    static constexpr uint8_t kCmdRead         = 0x03U;
    static constexpr uint8_t kCmdByteProgram  = 0x02U;
    static constexpr uint8_t kCmdReadStatus   = 0x05U;
    static constexpr uint8_t kCmdWriteEnable  = 0x06U;
    static constexpr uint8_t kCmdSectorErase  = 0x20U;
    static constexpr uint8_t kCmdEnableWrsr   = 0x50U;
    static constexpr uint8_t kCmdWriteStatus  = 0x01U;
    static constexpr uint8_t kCmdJedecId      = 0x9FU;

    static constexpr uint8_t  kStatusBusyBit  = 0x01U;
    static constexpr uint32_t kSpiTimeoutMs   = 50U;

    // RAII chip-select helper: pulls PD7 LOW on entry, guarantees HIGH on scope exit.
    struct CsGuard {
        CsGuard() noexcept;
        ~CsGuard() noexcept;
    };

    bool init_spi1_hardware() noexcept;
    bool unlock_write_protection() noexcept;
    bool write_enable() noexcept;
    bool wait_busy(uint32_t timeout_ms) noexcept;
    bool read_bytes(uint32_t addr, uint8_t* out, std::size_t len) noexcept;
    bool program_byte(uint32_t addr, uint8_t value) noexcept;

    I2c1Bus&                 i2c1_;
    SPI_HandleTypeDef        hspi1_{};
    std::array<uint8_t, 6>   mac_{};
    bool                     ready_{false};
};

} // namespace stm32f407