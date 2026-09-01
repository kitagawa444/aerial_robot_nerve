#pragma once

#ifdef SIMULATION

#include <string>

#include "flashmemory/config_flash_database.h"

class ConfigFlashStorageSim
{
public:
  bool init(const std::string &path, ConfigFlashDatabase &database);
  bool reload(ConfigFlashDatabase &database);
  bool commit(ConfigFlashDatabase &database);

  const std::string &path() const { return path_; }

private:
  std::string path_;
  ConfigFlashImage slot_a_{};
  ConfigFlashImage slot_b_{};

  bool read_();
  bool write_() const;
};

#endif  // SIMULATION
