#ifdef SIMULATION

#include "flashmemory/config_flash_storage_sim.h"

#include <cstdio>
#include <fstream>

bool ConfigFlashStorageSim::init(const std::string &path, ConfigFlashDatabase &database)
{
  path_ = path;
  if (!read_())
  {
    slot_a_ = ConfigFlashImage{};
    slot_b_ = ConfigFlashImage{};
  }
  database.load(slot_a_, slot_b_);
  return database.valid();
}

bool ConfigFlashStorageSim::reload(ConfigFlashDatabase &database)
{
  if (!read_()) return false;
  database.reload(slot_a_, slot_b_);
  return database.valid();
}

bool ConfigFlashStorageSim::commit(ConfigFlashDatabase &database)
{
  if (database.valid() && !database.dirty()) return true;
  ConfigFlashImage pending;
  int8_t target_slot = ConfigFlashDatabase::SLOT_NONE;
  if (!database.prepareCommit(pending, target_slot)) return false;

  ConfigFlashImage old_a = slot_a_;
  ConfigFlashImage old_b = slot_b_;
  if (target_slot == ConfigFlashDatabase::SLOT_A)
    slot_a_ = pending;
  else if (target_slot == ConfigFlashDatabase::SLOT_B)
    slot_b_ = pending;
  else
    return false;

  if (!write_())
  {
    slot_a_ = old_a;
    slot_b_ = old_b;
    return false;
  }
  return database.acceptCommit(pending, target_slot);
}

bool ConfigFlashStorageSim::read_()
{
  if (path_.empty()) return false;
  std::ifstream stream(path_, std::ios::binary);
  if (!stream) return false;
  stream.read(reinterpret_cast<char *>(&slot_a_), sizeof(slot_a_));
  stream.read(reinterpret_cast<char *>(&slot_b_), sizeof(slot_b_));
  return stream.good() || stream.gcount() == static_cast<std::streamsize>(sizeof(slot_b_));
}

bool ConfigFlashStorageSim::write_() const
{
  if (path_.empty()) return false;
  const std::string temporary = path_ + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) return false;
    stream.write(reinterpret_cast<const char *>(&slot_a_), sizeof(slot_a_));
    stream.write(reinterpret_cast<const char *>(&slot_b_), sizeof(slot_b_));
    stream.flush();
    if (!stream.good()) return false;
  }
  if (std::rename(temporary.c_str(), path_.c_str()) != 0)
  {
    (void)std::remove(temporary.c_str());
    return false;
  }
  return true;
}

#endif  // SIMULATION
