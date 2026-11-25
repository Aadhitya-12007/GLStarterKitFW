#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32f407/blackbox_storage.hpp"
#include "stm32f407/imu_driver.hpp"

namespace edr {

constexpr uint32_t kRecordMagic   = 0xED4B1AC0U; // Marks a valid programmed Flash sector
constexpr uint32_t kRecordSamples = 150U;        // 100 pre-trigger + 50 post-trigger @ 50 Hz

struct __attribute__((packed)) IncidentRecord {
    uint32_t magic;                               // Offset    0 (4 B): 0xED4B1AC0
    uint32_t event_id;                            // Offset    4 (4 B): 1, 2, 3...
    uint32_t timestamp_ms;                        // Offset    8 (4 B): FreeRTOS tick at trigger
    uint16_t peak_g_x100;                         // Offset   12 (2 B): e.g. 318 = 3.18 g
    uint8_t  trigger_type;                        // Offset   14 (1 B): 1=SHOCK, 2=TILT, 3=THERMAL
    uint8_t  reserved;                            // Offset   15 (1 B): 0x00
    int16_t  temp_c_x10;                          // Offset   16 (2 B): DHT11 temp (Day 4 hook)
    uint16_t humidity_x10;                        // Offset   18 (2 B): DHT11 humidity (Day 4 hook)
    uint8_t  device_mac[6];                       // Offset   20 (6 B): Factory EUI-48 from 24AA02E48T
    uint8_t  pad[2];                              // Offset   26 (2 B): 0x00 (keeps 4-byte alignment)
    uint32_t prev_crc32;                          // Offset   28 (4 B): Event #1 = RNG seed; #N = #(N-1) CRC
    stm32f407::ImuSample samples[kRecordSamples]; // Offset   32 (1800 B): 150 * 12 bytes
    uint32_t record_crc32;                        // Offset 1832 (4 B): STM32 HW CRC-32 of bytes 0..1831
};

static_assert(offsetof(IncidentRecord, record_crc32) == 1832U, "Header + samples must be 1832 bytes");
static_assert(offsetof(IncidentRecord, record_crc32) % 4U == 0U, "CRC input must be 32-bit word aligned");
static_assert(sizeof(IncidentRecord) == 1836U, "IncidentRecord must be exactly 1836 bytes");
static_assert(sizeof(IncidentRecord) <= stm32f407::BlackBoxStorage::kSectorSize, "Must fit in 4 KB sector");

} // namespace edr