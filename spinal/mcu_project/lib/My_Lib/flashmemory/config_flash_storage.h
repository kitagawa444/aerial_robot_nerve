#pragma once

#ifndef SIMULATION

#include "flashmemory/config_flash_database.h"

namespace ConfigFlashStorage
{
// STM32H743 Bank2 sectors 5 and 6. Sector 7 remains the legacy runtime
// parameter/calibration store.
constexpr uint32_t SLOT_A_ADDRESS = 0x081A0000UL;
constexpr uint32_t SLOT_B_ADDRESS = 0x081C0000UL;

bool load(ConfigFlashDatabase &database);
bool reload(ConfigFlashDatabase &database);
bool commit(ConfigFlashDatabase &database);
}  // namespace ConfigFlashStorage

#endif  // !SIMULATION
