#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace stm32f407 {

// Minimal SPI1 driver for the onboard SST25VF016B 2 MB NOR Flash (U11).
// Pins: PA5 (SCK), PB4 (MISO), PB5 (MOSI), PD7 (CS0#, active-low).
class BlackBoxStorage {
public:
    static constexpr std::size_t kSectorSize = 4096U;
    static constexpr uint32_t    kNumSectors = 512U; // 512 * 4 KB = 2 MB

    BlackBoxStorage() = default;

    // Initializes SPI1 + PD7 CS, verifies JEDEC ID (BF 25 41), and clears
    // SST25VF016B's power-on write protection. Call once before RTOS starts.
    bool init();

    bool is_ready() const { return ready_; }
    std::array<uint8_t, 3> jedec_id() const { return jedec_id_; }

    // Reads `len` bytes (1..4096) starting at `sector * 4096`.
    bool read_sector(uint32_t sector, uint8_t* dst, std::size_t len);

    // Erases `sector` (4 KB -> 0xFF) and programs `len` bytes (1..4096).
    bool write_sector(uint32_t sector, const uint8_t* src, std::size_t len);

private:
    bool init_spi1();
    bool read_jedec_id(std::array<uint8_t, 3>& out);
    bool unlock_write_protection();
    bool write_enable();
    bool wait_busy(uint32_t timeout_ms);
    bool erase_sector(uint32_t addr);
    bool program_byte(uint32_t addr, uint8_t byte);

    SPI_HandleTypeDef      hspi1_{};
    std::array<uint8_t, 3> jedec_id_{};
    bool                   ready_ = false;
};

} // namespace stm32f407