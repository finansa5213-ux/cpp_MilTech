// Абстракція шини I²C.
//
// Навіщо інтерфейс, якщо реалізація одна? Щоб у тестах підставити FakeBus і
// відтворити ситуації, які на живій шині відтворити неможливо: пристрою немає,
// читання обірвалося на середині, ID не той. Без цього вимогу ТЗ «обробляє
// помилки» неможливо перевірити автоматично.
#pragma once

#include <cstddef>
#include <cstdint>

#include "I2cError.hpp"

namespace i2c {

class Bus {
 public:
  Bus() = default;
  virtual ~Bus() = default;

  Bus(const Bus&) = delete;
  Bus& operator=(const Bus&) = delete;
  Bus(Bus&&) = delete;
  Bus& operator=(Bus&&) = delete;

  /// Обрати адресу пристрою на шині (для i2c-dev — ioctl(I2C_SLAVE)).
  virtual Result selectDevice(std::uint8_t addr) = 0;

  /// Записати один байт у регістр: [addr+W][reg][value].
  virtual Result writeReg(std::uint8_t reg, std::uint8_t value) = 0;

  /// Двофазне читання: спершу пишемо номер регістра, потім читаємо len байтів.
  virtual Result readBlock(std::uint8_t reg, std::uint8_t* out,
                           std::size_t len) = 0;

  /// Окремий випадок readBlock на один байт.
  Result readReg(std::uint8_t reg, std::uint8_t& out) {
    return readBlock(reg, &out, 1);
  }
};

}  // namespace i2c
