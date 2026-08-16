// Карта регістрів і типи MPU-6050. Ці константи взяті з даташита
// "MPU-6000/MPU-6050 Register Map and Descriptions, Rev. 4.2".
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mpu {

inline constexpr std::uint8_t kDefaultAddress = 0x68;  // AD0=0; 0x69 при AD0=1

namespace reg {
inline constexpr std::uint8_t kSmplrtDiv = 0x19;
inline constexpr std::uint8_t kConfig = 0x1A;
inline constexpr std::uint8_t kGyroConfig = 0x1B;   // FS_SEL  у бітах 4:3
inline constexpr std::uint8_t kAccelConfig = 0x1C;  // AFS_SEL у бітах 4:3
inline constexpr std::uint8_t kAccelXoutH = 0x3B;   // початок блоку вимірів
inline constexpr std::uint8_t kPwrMgmt1 = 0x6B;     // після подачі живлення = сон
inline constexpr std::uint8_t kWhoAmI = 0x75;
}  // namespace reg

/// WHO_AM_I має повернути саме це. Клони (MPU-6500/9250) повертають 0x70/0x71.
inline constexpr std::uint8_t kWhoAmIExpected = 0x68;

/// Блок 0x3B..0x48: accel XYZ (6) + temp (2) + gyro XYZ (6).
inline constexpr std::size_t kBurstLen = 14;

enum class AccelRange : std::uint8_t { G2 = 0, G4 = 1, G8 = 2, G16 = 3 };
enum class GyroRange : std::uint8_t {
  Dps250 = 0,
  Dps500 = 1,
  Dps1000 = 2,
  Dps2000 = 3
};

/// Один зчитаний кадр у фізичних величинах + сирі байти (для діагностики).
struct Sample {
  double ax_g = 0.0;
  double ay_g = 0.0;
  double az_g = 0.0;
  double gx_dps = 0.0;
  double gy_dps = 0.0;
  double gz_dps = 0.0;
  double temp_c = 0.0;
  std::array<std::uint8_t, kBurstLen> raw{};
};

}  // namespace mpu
