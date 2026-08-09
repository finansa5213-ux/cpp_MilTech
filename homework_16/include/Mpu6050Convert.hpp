// Перерахунок сирих відліків у фізичні величини.
//
// Тут навмисне немає жодного звертання до шини: усе — чисті функції від чисел.
// Саме тому їх можна повністю покрити тестами без датчика й без симулятора.
#pragma once

#include "Mpu6050Types.hpp"

namespace mpu {

/// Збирає 16-бітне знакове число зі старшого й молодшого байтів.
/// У MPU-6050 порядок байтів — старший перший (big-endian).
constexpr std::int16_t be16(std::uint8_t hi, std::uint8_t lo) noexcept {
  const auto raw =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
  return static_cast<std::int16_t>(raw);
}

/// Чутливість акселерометра, відліків на 1 g (даташит, п. 6.2).
constexpr double accelLsbPerG(AccelRange r) noexcept {
  switch (r) {
    case AccelRange::G2:
      return 16384.0;
    case AccelRange::G4:
      return 8192.0;
    case AccelRange::G8:
      return 4096.0;
    case AccelRange::G16:
      return 2048.0;
  }
  return 16384.0;
}

/// Чутливість гіроскопа, відліків на 1 °/с (даташит, п. 6.1).
constexpr double gyroLsbPerDps(GyroRange r) noexcept {
  switch (r) {
    case GyroRange::Dps250:
      return 131.0;
    case GyroRange::Dps500:
      return 65.5;
    case GyroRange::Dps1000:
      return 32.8;
    case GyroRange::Dps2000:
      return 16.4;
  }
  return 131.0;
}

/// Зворотне перетворення: біти 4:3 регістра конфігурації -> діапазон.
/// Потрібне, щоб перечитати регістр і дізнатися, який діапазон пристрій
/// насправді прийняв, а не який ми просили.
constexpr AccelRange accelRangeFromBits(std::uint8_t reg_value) noexcept {
  switch ((reg_value >> 3U) & 0x03U) {
    case 1:
      return AccelRange::G4;
    case 2:
      return AccelRange::G8;
    case 3:
      return AccelRange::G16;
    default:
      return AccelRange::G2;
  }
}

constexpr GyroRange gyroRangeFromBits(std::uint8_t reg_value) noexcept {
  switch ((reg_value >> 3U) & 0x03U) {
    case 1:
      return GyroRange::Dps500;
    case 2:
      return GyroRange::Dps1000;
    case 3:
      return GyroRange::Dps2000;
    default:
      return GyroRange::Dps250;
  }
}

/// Числовий підпис діапазону акселерометра, ±g.
constexpr int accelRangeG(AccelRange r) noexcept {
  switch (r) {
    case AccelRange::G2:
      return 2;
    case AccelRange::G4:
      return 4;
    case AccelRange::G8:
      return 8;
    case AccelRange::G16:
      return 16;
  }
  return 2;
}

/// Числовий підпис діапазону гіроскопа, ±°/с.
constexpr int gyroRangeDps(GyroRange r) noexcept {
  switch (r) {
    case GyroRange::Dps250:
      return 250;
    case GyroRange::Dps500:
      return 500;
    case GyroRange::Dps1000:
      return 1000;
    case GyroRange::Dps2000:
      return 2000;
  }
  return 250;
}

constexpr double toG(std::int16_t raw, AccelRange r) noexcept {
  return static_cast<double>(raw) / accelLsbPerG(r);
}

constexpr double toDps(std::int16_t raw, GyroRange r) noexcept {
  return static_cast<double>(raw) / gyroLsbPerDps(r);
}

/// Температура кристала: T[°C] = raw/340 + 36.53 (даташит, п. 4.18).
constexpr double toCelsius(std::int16_t raw) noexcept {
  return static_cast<double>(raw) / 340.0 + 36.53;
}

/// Розбирає 14-байтовий блок 0x3B..0x48 у фізичні величини.
inline Sample decode(const std::array<std::uint8_t, kBurstLen>& raw,
                     AccelRange a, GyroRange g) noexcept {
  Sample s;
  s.raw = raw;
  s.ax_g = toG(be16(raw[0], raw[1]), a);
  s.ay_g = toG(be16(raw[2], raw[3]), a);
  s.az_g = toG(be16(raw[4], raw[5]), a);
  s.temp_c = toCelsius(be16(raw[6], raw[7]));
  s.gx_dps = toDps(be16(raw[8], raw[9]), g);
  s.gy_dps = toDps(be16(raw[10], raw[11]), g);
  s.gz_dps = toDps(be16(raw[12], raw[13]), g);
  return s;
}

}  // namespace mpu
