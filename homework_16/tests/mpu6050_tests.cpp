// Тести працюють без датчика й без симулятора: шину підмінює FakeBus.
#include <gtest/gtest.h>

#include <map>
#include <utility>
#include <vector>

#include "Mpu6050.hpp"
#include "Mpu6050Convert.hpp"

namespace {

/// Керована «шина»: віддає задані значення регістрів і вміє ламатися на вимогу.
class FakeBus final : public i2c::Bus {
 public:
  std::map<std::uint8_t, std::uint8_t> regs;
  std::vector<std::pair<std::uint8_t, std::uint8_t>> writes;
  std::uint8_t selected = 0;

  bool device_present = true;   ///< false => жодного ACK
  bool truncate_reads = false;  ///< читання віддає на 1 байт менше
  bool swallow_config_writes = false;  ///< приймає запис у 0x1B/0x1C, але
                                       ///< не зберігає (так робить симулятор)
  bool stuck_asleep = false;           ///< біт SLEEP не знімається

  i2c::Result selectDevice(std::uint8_t addr) override {
    selected = addr;
    return i2c::Result::success();
  }

  i2c::Result writeReg(std::uint8_t reg, std::uint8_t value) override {
    if (!device_present) {
      return i2c::Result::fail(i2c::Error::NoAck, ENXIO);
    }
    writes.emplace_back(reg, value);
    const bool config_reg =
        (reg == mpu::reg::kGyroConfig || reg == mpu::reg::kAccelConfig);
    if (swallow_config_writes && config_reg) {
      return i2c::Result::success();  // ACK є, ефекту немає
    }
    if (stuck_asleep && reg == mpu::reg::kPwrMgmt1) {
      regs[reg] = 0x40;
      return i2c::Result::success();
    }
    regs[reg] = value;
    return i2c::Result::success();
  }

  i2c::Result readBlock(std::uint8_t reg, std::uint8_t* out,
                        std::size_t len) override {
    if (!device_present) {
      return i2c::Result::fail(i2c::Error::NoAck, ENXIO);
    }
    if (truncate_reads) {
      return i2c::Result::fail(i2c::Error::ShortRead);
    }
    for (std::size_t i = 0; i < len; ++i) {
      const auto key = static_cast<std::uint8_t>(reg + i);
      const auto it = regs.find(key);
      out[i] = (it == regs.end()) ? 0 : it->second;
    }
    return i2c::Result::success();
  }

  /// Розкласти 16-бітне значення у пару регістрів (старший байт першим).
  void setWord(std::uint8_t reg, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    regs[reg] = static_cast<std::uint8_t>(raw >> 8U);
    regs[static_cast<std::uint8_t>(reg + 1)] = static_cast<std::uint8_t>(raw);
  }
};

/// Шина заборонена до копіювання/переміщення (RAII на дескрипторі),
/// тому налаштовуємо вже створений об'єкт, а не повертаємо новий.
void makeHealthy(FakeBus& bus) {
  bus.regs[mpu::reg::kWhoAmI] = mpu::kWhoAmIExpected;
}

// ---------------------------------------------------------------------------
// Збирання 16-бітних значень
// ---------------------------------------------------------------------------

TEST(Be16, CombinesBytesMsbFirst) {
  EXPECT_EQ(mpu::be16(0x01, 0x02), 258);
  EXPECT_EQ(mpu::be16(0x00, 0x00), 0);
  EXPECT_EQ(mpu::be16(0x7F, 0xFF), 32767);
}

TEST(Be16, HandlesNegativeValues) {
  EXPECT_EQ(mpu::be16(0xFF, 0xFF), -1);
  EXPECT_EQ(mpu::be16(0x80, 0x00), -32768);
  EXPECT_EQ(mpu::be16(0xC0, 0x00), -16384);
}

// ---------------------------------------------------------------------------
// Перерахунок у фізичні величини
// ---------------------------------------------------------------------------

TEST(Convert, AccelerationMatchesDatasheetSensitivity) {
  EXPECT_DOUBLE_EQ(mpu::toG(16384, mpu::AccelRange::G2), 1.0);
  EXPECT_DOUBLE_EQ(mpu::toG(-16384, mpu::AccelRange::G2), -1.0);
  EXPECT_DOUBLE_EQ(mpu::toG(8192, mpu::AccelRange::G4), 1.0);
  EXPECT_DOUBLE_EQ(mpu::toG(4096, mpu::AccelRange::G8), 1.0);
  EXPECT_DOUBLE_EQ(mpu::toG(2048, mpu::AccelRange::G16), 1.0);
}

TEST(Convert, AngularRateMatchesDatasheetSensitivity) {
  EXPECT_DOUBLE_EQ(mpu::toDps(131, mpu::GyroRange::Dps250), 1.0);
  EXPECT_NEAR(mpu::toDps(655, mpu::GyroRange::Dps500), 10.0, 1e-9);
  EXPECT_NEAR(mpu::toDps(328, mpu::GyroRange::Dps1000), 10.0, 1e-9);
  EXPECT_NEAR(mpu::toDps(164, mpu::GyroRange::Dps2000), 10.0, 1e-9);
}

TEST(Convert, TemperatureUsesDatasheetFormula) {
  EXPECT_NEAR(mpu::toCelsius(0), 36.53, 1e-9);
  EXPECT_NEAR(mpu::toCelsius(340), 37.53, 1e-9);
  EXPECT_NEAR(mpu::toCelsius(-340), 35.53, 1e-9);
}

TEST(Convert, DecodesFullFrameInCorrectOrder) {
  // 0x3B..0x40 accel, 0x41..0x42 temp, 0x43..0x48 gyro.
  const std::array<std::uint8_t, mpu::kBurstLen> raw{
      0x00, 0x00,  // ax = 0
      0x40, 0x00,  // ay = 16384 => +1 g
      0xC0, 0x00,  // az = -16384 => -1 g
      0x01, 0x54,  // temp = 340 => 37.53 °C
      0x00, 0x83,  // gx = 131 => +1 °/с
      0xFF, 0x7D,  // gy = -131 => -1 °/с
      0x00, 0x00,  // gz = 0
  };
  const mpu::Sample s =
      mpu::decode(raw, mpu::AccelRange::G2, mpu::GyroRange::Dps250);

  EXPECT_DOUBLE_EQ(s.ax_g, 0.0);
  EXPECT_DOUBLE_EQ(s.ay_g, 1.0);
  EXPECT_DOUBLE_EQ(s.az_g, -1.0);
  EXPECT_NEAR(s.temp_c, 37.53, 1e-9);
  EXPECT_DOUBLE_EQ(s.gx_dps, 1.0);
  EXPECT_DOUBLE_EQ(s.gy_dps, -1.0);
  EXPECT_DOUBLE_EQ(s.gz_dps, 0.0);
}

// ---------------------------------------------------------------------------
// Драйвер: звірка ID, ініціалізація, читання
// ---------------------------------------------------------------------------

TEST(Driver, AcceptsCorrectWhoAmI) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus);

  std::uint8_t id = 0;
  EXPECT_TRUE(dev.checkId(id).ok());
  EXPECT_EQ(id, 0x68);
}

TEST(Driver, RejectsForeignChipId) {
  FakeBus bus;
  makeHealthy(bus);
  bus.regs[mpu::reg::kWhoAmI] = 0x71;  // MPU-9250
  mpu::Mpu6050 dev(bus);

  std::uint8_t id = 0;
  const i2c::Result r = dev.checkId(id);
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.code, i2c::Error::BadId);
  EXPECT_EQ(id, 0x71);
}

TEST(Driver, ReportsMissingDeviceAsNoAck) {
  FakeBus bus;
  makeHealthy(bus);
  bus.device_present = false;
  mpu::Mpu6050 dev(bus);

  std::uint8_t id = 0;
  const i2c::Result r = dev.checkId(id);
  EXPECT_EQ(r.code, i2c::Error::NoAck);
  EXPECT_NE(i2c::describe(r).find("ACK"), std::string::npos);
}

TEST(Driver, ReportsTruncatedBurstAsShortRead) {
  FakeBus bus;
  makeHealthy(bus);
  bus.truncate_reads = true;
  mpu::Mpu6050 dev(bus);

  mpu::Sample s;
  EXPECT_EQ(dev.read(s).code, i2c::Error::ShortRead);
}

TEST(Driver, WakeUpClearsSleepBit) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus);

  ASSERT_TRUE(dev.wakeUp().ok());
  ASSERT_EQ(bus.writes.size(), 1U);
  EXPECT_EQ(bus.writes[0].first, mpu::reg::kPwrMgmt1);
  EXPECT_EQ(bus.writes[0].second, 0x00);
  EXPECT_EQ(bus.regs[mpu::reg::kPwrMgmt1], 0x00);
}

TEST(Driver, ConfigurePutsRangeBitsInPositionFourThree) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus);

  ASSERT_TRUE(dev.configure(mpu::AccelRange::G8, mpu::GyroRange::Dps500).ok());
  EXPECT_EQ(bus.regs[mpu::reg::kGyroConfig], 0x08);   // FS_SEL  = 1 << 3
  EXPECT_EQ(bus.regs[mpu::reg::kAccelConfig], 0x10);  // AFS_SEL = 2 << 3
  EXPECT_EQ(dev.accelRange(), mpu::AccelRange::G8);
  EXPECT_EQ(dev.gyroRange(), mpu::GyroRange::Dps500);
}

TEST(Driver, ReadAppliesConfiguredRange) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus);
  ASSERT_TRUE(dev.configure(mpu::AccelRange::G8, mpu::GyroRange::Dps250).ok());

  bus.setWord(mpu::reg::kAccelXoutH, 4096);  // при ±8g це рівно 1 g
  mpu::Sample s;
  ASSERT_TRUE(dev.read(s).ok());
  EXPECT_DOUBLE_EQ(s.ax_g, 1.0);
}

TEST(Driver, ScalesByRangeDeviceConfirmedNotByRangeRequested) {
  // Відтворює поведінку курсового libi2csim.so: запис у 0x1C проходить,
  // але регістр лишається 0x00, а відліки — у шкалі ±2 g.
  FakeBus bus;
  makeHealthy(bus);
  bus.swallow_config_writes = true;
  mpu::Mpu6050 dev(bus);

  ASSERT_TRUE(dev.configure(mpu::AccelRange::G8, mpu::GyroRange::Dps2000).ok());
  EXPECT_FALSE(dev.rangesApplied());
  EXPECT_EQ(dev.requestedAccelRange(), mpu::AccelRange::G8);
  EXPECT_EQ(dev.accelRange(), mpu::AccelRange::G2);

  bus.setWord(mpu::reg::kAccelXoutH, 16384);
  mpu::Sample s;
  ASSERT_TRUE(dev.read(s).ok());
  EXPECT_DOUBLE_EQ(s.ax_g, 1.0);  // а не 4.0, як було б за «замовленим» ±8 g
}

TEST(Driver, ReportsRangesAppliedOnHonestDevice) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus);

  ASSERT_TRUE(dev.configure(mpu::AccelRange::G8, mpu::GyroRange::Dps500).ok());
  EXPECT_TRUE(dev.rangesApplied());
  EXPECT_EQ(dev.accelRange(), mpu::AccelRange::G8);
  EXPECT_EQ(dev.gyroRange(), mpu::GyroRange::Dps500);
}

TEST(Driver, DetectsDeviceStuckAsleep) {
  FakeBus bus;
  makeHealthy(bus);
  bus.stuck_asleep = true;
  mpu::Mpu6050 dev(bus);

  const i2c::Result r = dev.wakeUp();
  EXPECT_EQ(r.code, i2c::Error::NotApplied);
}

TEST(Driver, WakeUpSucceedsWhenSleepBitClears) {
  FakeBus bus;
  makeHealthy(bus);
  bus.regs[mpu::reg::kPwrMgmt1] = 0x40;  // як після подачі живлення
  mpu::Mpu6050 dev(bus);

  EXPECT_TRUE(dev.wakeUp().ok());
  EXPECT_EQ(bus.regs[mpu::reg::kPwrMgmt1], 0x00);
}

TEST(Convert, RangeBitsRoundTrip) {
  EXPECT_EQ(mpu::accelRangeFromBits(0x00), mpu::AccelRange::G2);
  EXPECT_EQ(mpu::accelRangeFromBits(0x08), mpu::AccelRange::G4);
  EXPECT_EQ(mpu::accelRangeFromBits(0x10), mpu::AccelRange::G8);
  EXPECT_EQ(mpu::accelRangeFromBits(0x18), mpu::AccelRange::G16);
  EXPECT_EQ(mpu::gyroRangeFromBits(0x18), mpu::GyroRange::Dps2000);
  // Сторонні біти регістра не повинні впливати на розбір.
  EXPECT_EQ(mpu::accelRangeFromBits(0xE7), mpu::AccelRange::G2);
}

TEST(Driver, SelectPassesConfiguredAddress) {
  FakeBus bus;
  makeHealthy(bus);
  mpu::Mpu6050 dev(bus, 0x69);

  ASSERT_TRUE(dev.select().ok());
  EXPECT_EQ(bus.selected, 0x69);
}

}  // namespace
