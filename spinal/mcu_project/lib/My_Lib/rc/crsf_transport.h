#pragma once

#include <cstddef>
#include <cstdint>

class CrsfTransport
{
public:
  virtual ~CrsfTransport() = default;

  virtual void setEnabled(bool enabled) = 0;
  virtual void update(uint32_t now_ms) = 0;
  virtual std::size_t read(uint8_t *data, std::size_t capacity) = 0;
};
