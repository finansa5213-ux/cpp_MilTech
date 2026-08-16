#pragma once

#include <cstddef>
#include <cstdint>

namespace mavtel {

/// Абстракція каналу передавання байтів.
/// Потрібна, щоб логіку MAVLink (повтори команди до ACK) можна було покрити
/// юніт-тестами без реального UDP-сокета.
class ITransport {
public:
  virtual ~ITransport() = default;

  ITransport() = default;
  ITransport(const ITransport&) = delete;
  ITransport& operator=(const ITransport&) = delete;
  ITransport(ITransport&&) = delete;
  ITransport& operator=(ITransport&&) = delete;

  /// Надіслати одну датаграму. true — успіх.
  virtual bool send(const std::uint8_t* data, std::size_t len) = 0;

  /// Прочитати одну датаграму без блокування.
  /// Повертає кількість прочитаних байтів або 0, якщо даних немає.
  virtual std::size_t receive(std::uint8_t* buffer, std::size_t capacity) = 0;
};

}  // namespace mavtel
