// Реальна шина через ядровий інтерфейс i2c-dev.
#pragma once

#include <string>

#include "I2cBus.hpp"

namespace i2c {

/// Володіє файловим дескриптором /dev/i2c-N. Копіювання заборонене (RAII).
class LinuxBus final : public Bus {
 public:
  LinuxBus() = default;
  ~LinuxBus() override;

  /// open("/dev/i2c-1", O_RDWR)
  Result open(const std::string& path);
  void close() noexcept;
  [[nodiscard]] bool isOpen() const noexcept { return fd_ >= 0; }

  Result selectDevice(std::uint8_t addr) override;
  Result writeReg(std::uint8_t reg, std::uint8_t value) override;
  Result readBlock(std::uint8_t reg, std::uint8_t* out,
                   std::size_t len) override;

 private:
  int fd_ = -1;
};

}  // namespace i2c
