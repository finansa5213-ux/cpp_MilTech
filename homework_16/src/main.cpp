// ДЗ (Заняття 32) — робота з I²C-пристроєм MPU-6050 через i2c-dev.
//
// Запуск під симулятором:
//   LD_PRELOAD=./libi2csim.so ./homework_16 /dev/i2c-1 0x68
// На реальній платі — те саме без LD_PRELOAD.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "LinuxI2cBus.hpp"
#include "Mpu6050.hpp"
#include "Mpu6050Convert.hpp"

namespace {

// Прапорець зупинки. Ставиться з обробника сигналу, тому — sig_atomic_t.
volatile std::sig_atomic_t g_stop = 0;

void onSignal(int /*sig*/) { g_stop = 1; }

struct Options {
  std::string bus_path;
  std::uint8_t address = mpu::kDefaultAddress;
  double hz = 5.0;
  long samples = 0;  // 0 = поки не натиснуть Ctrl+C
  bool ignore_id = false;
  bool no_init = false;
  bool scan = false;
  bool show_raw = false;
  mpu::AccelRange accel = mpu::AccelRange::G2;
  mpu::GyroRange gyro = mpu::GyroRange::Dps250;
};

constexpr int kExitOk = 0;
constexpr int kExitBadArgs = 1;
constexpr int kExitBusError = 2;
constexpr int kExitBadId = 3;

void printUsage(const char* prog) {
  std::cout
      << "Використання:\n  " << prog << " <шина> <адреса> [опції]\n\n"
      << "Приклад:\n  LD_PRELOAD=./libi2csim.so " << prog
      << " /dev/i2c-1 0x68\n\n"
      << "Опції:\n"
      << "  --hz N            частота опитування, Гц (типово 5)\n"
      << "  --samples N       зупинитися після N кадрів (типово — до Ctrl+C)\n"
      << "  --accel-range N   2 | 4 | 8 | 16 (g)\n"
      << "  --gyro-range N    250 | 500 | 1000 | 2000 (°/с)\n"
      << "  --ignore-id       не зупинятися, якщо WHO_AM_I != 0x68\n"
      << "  --no-init         не писати в PWR_MGMT_1 / CONFIG-регістри\n"
      << "  --scan            пройти адреси 0x03..0x77 і вийти\n"
      << "  --raw             додатково показувати сирі байти\n"
      << "  -h, --help        ця довідка\n";
}

bool parseUInt(const std::string& s, long& out) {
  try {
    std::size_t pos = 0;
    const long v = std::stol(s, &pos, 0);  // 0 => розпізнає і 0x..
    if (pos != s.size() || v < 0) {
      return false;
    }
    out = v;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool parseArgs(int argc, char** argv, Options& opt) {
  if (argc < 3) {
    return false;
  }
  opt.bus_path = argv[1];

  long addr = 0;
  if (!parseUInt(argv[2], addr) || addr < 0x03 || addr > 0x77) {
    std::cerr << "Помилка: адреса має бути в діапазоні 0x03..0x77, отримано «"
              << argv[2] << "»\n";
    return false;
  }
  opt.address = static_cast<std::uint8_t>(addr);

  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    const bool has_value = (i + 1 < argc);

    if (a == "--ignore-id") {
      opt.ignore_id = true;
    } else if (a == "--no-init") {
      opt.no_init = true;
    } else if (a == "--scan") {
      opt.scan = true;
    } else if (a == "--raw") {
      opt.show_raw = true;
    } else if (a == "--hz" && has_value) {
      opt.hz = std::strtod(argv[++i], nullptr);
      if (opt.hz <= 0.0 || opt.hz > 1000.0) {
        std::cerr << "Помилка: --hz має бути в межах (0; 1000]\n";
        return false;
      }
    } else if (a == "--samples" && has_value) {
      if (!parseUInt(argv[++i], opt.samples)) {
        std::cerr << "Помилка: --samples очікує невід'ємне число\n";
        return false;
      }
    } else if (a == "--accel-range" && has_value) {
      const std::string v = argv[++i];
      if (v == "2") {
        opt.accel = mpu::AccelRange::G2;
      } else if (v == "4") {
        opt.accel = mpu::AccelRange::G4;
      } else if (v == "8") {
        opt.accel = mpu::AccelRange::G8;
      } else if (v == "16") {
        opt.accel = mpu::AccelRange::G16;
      } else {
        std::cerr << "Помилка: --accel-range очікує 2, 4, 8 або 16\n";
        return false;
      }
    } else if (a == "--gyro-range" && has_value) {
      const std::string v = argv[++i];
      if (v == "250") {
        opt.gyro = mpu::GyroRange::Dps250;
      } else if (v == "500") {
        opt.gyro = mpu::GyroRange::Dps500;
      } else if (v == "1000") {
        opt.gyro = mpu::GyroRange::Dps1000;
      } else if (v == "2000") {
        opt.gyro = mpu::GyroRange::Dps2000;
      } else {
        std::cerr << "Помилка: --gyro-range очікує 250, 500, 1000 або 2000\n";
        return false;
      }
    } else {
      std::cerr << "Помилка: невідома опція «" << a << "»\n";
      return false;
    }
  }
  return true;
}

/// Ширина рядка в символах, а не в байтах: std::setw рахує байти, тому
/// кирилиця й «°» ламали б вирівнювання шапки таблиці.
std::size_t displayWidth(const std::string& s) {
  std::size_t n = 0;
  for (const char c : s) {
    if ((static_cast<unsigned char>(c) & 0xC0U) != 0x80U) {
      ++n;
    }
  }
  return n;
}

std::string padLeft(const std::string& s, std::size_t width) {
  const std::size_t w = displayWidth(s);
  return (w >= width) ? s : std::string(width - w, ' ') + s;
}

std::string hex8(std::uint8_t v) {
  std::ostringstream os;
  os << "0x" << std::hex << std::setw(2) << std::setfill('0')
     << static_cast<int>(v);
  return os.str();
}

/// Спрощений аналог i2cdetect: пробуємо прочитати один байт з кожної адреси.
int runScan(i2c::LinuxBus& bus) {
  std::cout << "Сканування шини (аналог i2cdetect, читанням регістра 0x00):\n";
  int found = 0;
  for (int addr = 0x03; addr <= 0x77; ++addr) {
    if (!bus.selectDevice(static_cast<std::uint8_t>(addr))) {
      continue;
    }
    std::uint8_t dummy = 0;
    if (bus.readReg(0x00, dummy)) {
      std::cout << "  знайдено пристрій на " << hex8(static_cast<std::uint8_t>(addr))
                << "\n";
      ++found;
    }
  }
  if (found == 0) {
    std::cout << "  нічого не знайдено — перевірте шину, живлення й підтяжки\n";
  } else if (found > 16) {
    std::cout << "  відповіли " << found
              << " адрес — стільки пристроїв на шині не буває.\n"
                 "  Найімовірніше ви під симулятором, який відповідає на "
                 "будь-яку адресу.\n  Еталонна перевірка на залізі: "
                 "i2cdetect -y N\n";
  }
  return found;
}

void printHeader(const Options& opt, const mpu::Mpu6050& dev) {
  std::cout << "MPU-6050 на " << opt.bus_path << ", адреса "
            << hex8(opt.address) << ", "
            << "діапазони ±" << mpu::accelRangeG(dev.accelRange())
            << " g / ±" << mpu::gyroRangeDps(dev.gyroRange()) << " °/с, "
            << opt.hz
            << " Гц. Зупинка — Ctrl+C.\n\n";
  std::cout << padLeft("ax,g", 8) << padLeft("ay,g", 9) << padLeft("az,g", 9)
            << "  |" << padLeft("gx,°/с", 10) << padLeft("gy,°/с", 10)
            << padLeft("gz,°/с", 10) << "  |" << padLeft("t,°C", 8) << "\n";
  std::cout << std::string(70, '-') << "\n";
}

void printSample(const mpu::Sample& s, bool show_raw) {
  std::cout << std::fixed << std::right << std::setprecision(3) << std::setw(8)
            << s.ax_g << std::setw(9) << s.ay_g << std::setw(9) << s.az_g
            << "  |" << std::setprecision(2) << std::setw(10) << s.gx_dps
            << std::setw(10) << s.gy_dps << std::setw(10) << s.gz_dps << "  |"
            << std::setprecision(2) << std::setw(8) << s.temp_c;
  if (show_raw) {
    std::cout << "  |";
    for (const std::uint8_t b : s.raw) {
      std::cout << " " << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(b) << std::dec << std::setfill(' ');
    }
  }
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 2 &&
      (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
    printUsage(argv[0]);
    return kExitOk;
  }

  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    printUsage(argv[0]);
    return kExitBadArgs;
  }

  // SA_RESTART, щоб Ctrl+C не рвав системний виклик посеред транзакції.
  struct sigaction sa {};
  sa.sa_handler = onSignal;
  sa.sa_flags = SA_RESTART;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // Крок 1. Відкрити шину.
  i2c::LinuxBus bus;
  if (const i2c::Result r = bus.open(opt.bus_path); !r) {
    std::cerr << "Не вдалося відкрити " << opt.bus_path << ": "
              << i2c::describe(r) << "\n"
              << "Підказка: під симулятором запускайте з "
                 "LD_PRELOAD=./libi2csim.so\n";
    return kExitBusError;
  }

  if (opt.scan) {
    return runScan(bus) > 0 ? kExitOk : kExitBusError;
  }

  mpu::Mpu6050 dev(bus, opt.address);

  // Крок 2. Обрати адресу пристрою.
  if (const i2c::Result r = dev.select(); !r) {
    std::cerr << "Адреса " << hex8(opt.address)
              << " недоступна: " << i2c::describe(r) << "\n";
    return kExitBusError;
  }

  // Крок 3. Звірити ID (WHO_AM_I = 0x75 має віддати 0x68).
  std::uint8_t id = 0;
  const i2c::Result idr = dev.checkId(id);
  if (idr.code == i2c::Error::BadId) {
    std::cerr << "WHO_AM_I повернув " << hex8(id) << ", очікували "
              << hex8(mpu::kWhoAmIExpected)
              << ". Це не MPU-6050 (або клон MPU-6500/9250).\n";
    if (!opt.ignore_id) {
      std::cerr << "Продовжити попри це: --ignore-id\n";
      return kExitBadId;
    }
    std::cerr << "--ignore-id заданий, продовжую.\n";
  } else if (!idr) {
    std::cerr << "Не вдалося прочитати WHO_AM_I: " << i2c::describe(idr) << "\n";
    return kExitBusError;
  } else {
    std::cout << "WHO_AM_I = " << hex8(id) << " — пристрій підтверджено.\n";
  }

  // Крок 4. Розбудити й налаштувати діапазони.
  if (!opt.no_init) {
    if (const i2c::Result r = dev.wakeUp(); !r) {
      std::cerr << "Не вдалося вивести датчик зі сну: " << i2c::describe(r)
                << "\nБіт SLEEP лишився піднятим — перевірте живлення й "
                   "адресу; обійти перевірку: --no-init\n";
      return kExitBusError;
    }
    if (const i2c::Result r = dev.configure(opt.accel, opt.gyro); !r) {
      std::cerr << "Не вдалося записати діапазони: " << i2c::describe(r) << "\n";
      return kExitBusError;
    }
    if (!dev.rangesApplied()) {
      std::cerr << "Увага: пристрій проігнорував запит ±"
                << mpu::accelRangeG(opt.accel) << " g / ±"
                << mpu::gyroRangeDps(opt.gyro)
                << " °/с — перечитані регістри кажуть ±"
                << mpu::accelRangeG(dev.accelRange()) << " g / ±"
                << mpu::gyroRangeDps(dev.gyroRange())
                << " °/с.\nПерерахунок веду за фактичним діапазоном, інакше "
                   "величини були б хибними.\n";
    }
  }

  printHeader(opt, dev);

  // Крок 5. Цикл опитування.
  const auto period = std::chrono::duration<double>(1.0 / opt.hz);
  long taken = 0;
  int consecutive_errors = 0;
  constexpr int kMaxConsecutiveErrors = 5;

  while (g_stop == 0 && (opt.samples == 0 || taken < opt.samples)) {
    mpu::Sample s;
    if (const i2c::Result r = dev.read(s); !r) {
      std::cerr << "Помилка читання блоку: " << i2c::describe(r) << "\n";
      if (++consecutive_errors >= kMaxConsecutiveErrors) {
        std::cerr << "Поспіль " << kMaxConsecutiveErrors
                  << " невдалих читань — зупиняюся.\n";
        return kExitBusError;
      }
    } else {
      consecutive_errors = 0;
      printSample(s, opt.show_raw);
      ++taken;
    }
    std::this_thread::sleep_for(period);
  }

  std::cout << "\nЗавершено, кадрів прочитано: " << taken << "\n";
  return kExitOk;
}
