#ifndef SIMULATION

#include "flashmemory/config_flash_storage.h"

#include <cstring>

#include "stm32h7xx_hal.h"

namespace
{
constexpr uint32_t FLASH_TIMEOUT_MS = 50000U;

uint32_t slotAddress(int8_t slot)
{
  return slot == ConfigFlashDatabase::SLOT_A ? ConfigFlashStorage::SLOT_A_ADDRESS : ConfigFlashStorage::SLOT_B_ADDRESS;
}

uint32_t slotSector(int8_t slot) { return slot == ConfigFlashDatabase::SLOT_A ? FLASH_SECTOR_5 : FLASH_SECTOR_6; }

bool eraseSlot(int8_t slot)
{
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  FLASH_EraseInitTypeDef erase{};
  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_2;
  erase.Sector = slotSector(slot);
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t sector_error = 0U;
  const HAL_StatusTypeDef erase_status = HAL_FLASHEx_Erase(&erase, &sector_error);
  const HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();
  return erase_status == HAL_OK && lock_status == HAL_OK;
}

bool writeSlot(int8_t slot, const ConfigFlashImage &image)
{
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  alignas(32) ConfigFlashImage aligned_image = image;
  const HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, slotAddress(slot),
                                                     reinterpret_cast<uint32_t>(&aligned_image));
  const HAL_StatusTypeDef wait_status = status == HAL_OK ? FLASH_WaitForLastOperation(FLASH_TIMEOUT_MS, FLASH_BANK_2) :
                                                           status;
  const HAL_StatusTypeDef lock_status = HAL_FLASH_Lock();
  return status == HAL_OK && wait_status == HAL_OK && lock_status == HAL_OK;
}
}  // namespace

namespace ConfigFlashStorage
{
bool load(ConfigFlashDatabase &database)
{
  ConfigFlashImage slot_a;
  ConfigFlashImage slot_b;
  std::memcpy(&slot_a, reinterpret_cast<const void *>(SLOT_A_ADDRESS), sizeof(slot_a));
  std::memcpy(&slot_b, reinterpret_cast<const void *>(SLOT_B_ADDRESS), sizeof(slot_b));
  database.load(slot_a, slot_b);
  return database.valid();
}

bool reload(ConfigFlashDatabase &database)
{
  ConfigFlashImage slot_a;
  ConfigFlashImage slot_b;
  std::memcpy(&slot_a, reinterpret_cast<const void *>(SLOT_A_ADDRESS), sizeof(slot_a));
  std::memcpy(&slot_b, reinterpret_cast<const void *>(SLOT_B_ADDRESS), sizeof(slot_b));
  database.reload(slot_a, slot_b);
  return database.valid();
}

bool commit(ConfigFlashDatabase &database)
{
  if (database.valid() && !database.dirty()) return true;
  ConfigFlashImage pending;
  int8_t target_slot = ConfigFlashDatabase::SLOT_NONE;
  if (!database.prepareCommit(pending, target_slot)) return false;
  if (!eraseSlot(target_slot) || !writeSlot(target_slot, pending)) return false;

  SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(slotAddress(target_slot)), sizeof(ConfigFlashImage));
  ConfigFlashImage verified;
  std::memcpy(&verified, reinterpret_cast<const void *>(slotAddress(target_slot)), sizeof(verified));
  return std::memcmp(&verified, &pending, sizeof(verified)) == 0 && database.acceptCommit(verified, target_slot);
}
}  // namespace ConfigFlashStorage

#endif  // !SIMULATION
