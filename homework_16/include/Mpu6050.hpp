// Драйвер MPU-6050 поверх абстрактної шини.
#pragma once

#include "I2cBus.hpp"
#include "Mpu6050Types.hpp"

namespace mpu {

class Mpu6050 {
 public:
  explicit Mpu6050(i2c::Bus& bus, std::uint8_t addr = kDefaultAddress)
      : bus_(bus), addr_(addr) {}

  /// Обрати адресу цього пристрою на шині.
  i2c::Result select() { return bus_.selectDevice(addr_); }

  /// Прочитати WHO_AM_I (0x75) і звірити з 0x68.
  /// Повертає BadId, якщо обмін пройшов, але значення не те.
  i2c::Result checkId(std::uint8_t& id_out);

  /// Вивести з режиму сну: PWR_MGMT_1 = 0x00, потім перечитати й
  /// переконатися, що біт SLEEP справді знявся.
  /// Після подачі живлення датчик спить — без цього кроку покази «мертві».
  i2c::Result wakeUp();

  /// Записати діапазони у GYRO_CONFIG (0x1B) і ACCEL_CONFIG (0x1C),
  /// потім ПЕРЕЧИТАТИ їх і взяти за основу перерахунку саме те, що пристрій
  /// підтвердив. Якщо він проігнорував запис, повертає Ok, але effective-
  /// діапазон відрізнятиметься від requested — це має перевірити викликач.
  i2c::Result configure(AccelRange a, GyroRange g);

  /// Прочитати блок 14 байтів з 0x3B і перерахувати у фізичні величини.
  i2c::Result read(Sample& out) const;

  [[nodiscard]] std::uint8_t address() const noexcept { return addr_; }
  /// Діапазон, за яким реально ведеться перерахунок (перечитаний з пристрою).
  [[nodiscard]] AccelRange accelRange() const noexcept { return accel_; }
  [[nodiscard]] GyroRange gyroRange() const noexcept { return gyro_; }

  /// Діапазон, який ми просили встановити.
  [[nodiscard]] AccelRange requestedAccelRange() const noexcept {
    return requested_accel_;
  }
  [[nodiscard]] GyroRange requestedGyroRange() const noexcept {
    return requested_gyro_;
  }

  /// true, якщо пристрій прийняв обидва запитані діапазони.
  [[nodiscard]] bool rangesApplied() const noexcept {
    return accel_ == requested_accel_ && gyro_ == requested_gyro_;
  }

 private:
  i2c::Bus& bus_;
  std::uint8_t addr_;
  AccelRange accel_ = AccelRange::G2;
  GyroRange gyro_ = GyroRange::Dps250;
  AccelRange requested_accel_ = AccelRange::G2;
  GyroRange requested_gyro_ = GyroRange::Dps250;
};

}  // namespace mpu
