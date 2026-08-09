#include "Mpu6050.hpp"

#include "Mpu6050Convert.hpp"

namespace mpu {

i2c::Result Mpu6050::checkId(std::uint8_t& id_out) {
  id_out = 0;
  const i2c::Result r = bus_.readReg(reg::kWhoAmI, id_out);
  if (!r) {
    return r;
  }
  if (id_out != kWhoAmIExpected) {
    return i2c::Result::fail(i2c::Error::BadId);
  }
  return i2c::Result::success();
}

i2c::Result Mpu6050::wakeUp() {
  // Біт SLEEP (6) у нуль, джерело тактування — внутрішній генератор.
  const i2c::Result w = bus_.writeReg(reg::kPwrMgmt1, 0x00);
  if (!w) {
    return w;
  }
  // Не віримо запису на слово: перечитуємо й перевіряємо, що SLEEP знявся.
  std::uint8_t pwr = 0;
  const i2c::Result r = bus_.readReg(reg::kPwrMgmt1, pwr);
  if (!r) {
    return r;
  }
  if ((pwr & 0x40U) != 0U) {
    return i2c::Result::fail(i2c::Error::NotApplied);
  }
  return i2c::Result::success();
}

i2c::Result Mpu6050::configure(AccelRange a, GyroRange g) {
  requested_accel_ = a;
  requested_gyro_ = g;

  // FS_SEL і AFS_SEL живуть у бітах 4:3, тому зсув на 3.
  const auto gyro_bits = static_cast<std::uint8_t>(static_cast<std::uint8_t>(g)
                                                   << 3U);
  const auto accel_bits = static_cast<std::uint8_t>(static_cast<std::uint8_t>(a)
                                                    << 3U);

  i2c::Result r = bus_.writeReg(reg::kGyroConfig, gyro_bits);
  if (!r) {
    return r;
  }
  r = bus_.writeReg(reg::kAccelConfig, accel_bits);
  if (!r) {
    return r;
  }

  // Ключовий момент: перерахунок ведемо за тим, що пристрій ПІДТВЕРДИВ
  // при читанні, а не за тим, що ми йому надіслали. Якщо він проігнорував
  // запис (так робить курсовий симулятор), відліки лишаються в старій
  // шкалі — і ділення на «замовлену» чутливість дало б хибні величини.
  std::uint8_t gyro_read = 0;
  std::uint8_t accel_read = 0;
  r = bus_.readReg(reg::kGyroConfig, gyro_read);
  if (!r) {
    return r;
  }
  r = bus_.readReg(reg::kAccelConfig, accel_read);
  if (!r) {
    return r;
  }

  gyro_ = gyroRangeFromBits(gyro_read);
  accel_ = accelRangeFromBits(accel_read);
  return i2c::Result::success();
}

i2c::Result Mpu6050::read(Sample& out) const {
  std::array<std::uint8_t, kBurstLen> raw{};
  const i2c::Result r = bus_.readBlock(reg::kAccelXoutH, raw.data(), raw.size());
  if (!r) {
    return r;
  }
  out = decode(raw, accel_, gyro_);
  return i2c::Result::success();
}

}  // namespace mpu
