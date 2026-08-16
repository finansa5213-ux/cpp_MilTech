// Коди помилок обміну по шині I²C та їх людиночитний опис.
#pragma once

#include <string>

namespace i2c {

/// Причина, чому транзакція не відбулася.
enum class Error {
  Ok = 0,
  BadArgs,              ///< некоректні аргументи виклику
  OpenFailed,           ///< не вдалося відкрити /dev/i2c-N
  SelectAddressFailed,  ///< ioctl(I2C_SLAVE) не пройшов
  NoAck,                ///< пристрій не відповів на шині (немає ACK)
  WriteFailed,          ///< обрив запису
  ReadFailed,           ///< обрив читання
  ShortRead,            ///< прочитано менше байтів, ніж запитано
  BadId,                ///< регістр ідентифікації повернув не те значення
  NotApplied,           ///< запис пройшов, але перечитування дало інше значення
};

/// Результат транзакції: код помилки + системний errno (якщо був).
struct Result {
  Error code = Error::Ok;
  int sys_errno = 0;

  [[nodiscard]] constexpr bool ok() const noexcept { return code == Error::Ok; }
  explicit constexpr operator bool() const noexcept { return ok(); }

  [[nodiscard]] static constexpr Result success() noexcept { return {}; }
  [[nodiscard]] static constexpr Result fail(Error c, int e = 0) noexcept {
    return {c, e};
  }
};

/// Формує повідомлення на кшталт
/// "пристрій не відповідає на шині (немає ACK) [errno 6: No such device or address]".
[[nodiscard]] std::string describe(const Result& r);

}  // namespace i2c
