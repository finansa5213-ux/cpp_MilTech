#include "LinuxI2cBus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>

namespace i2c {
namespace {

/// Розрізняємо «пристрій не відповів» і «щось інше зламалося».
/// ENXIO / EREMOTEIO / ENODEV — саме те, що ядро віддає, коли на адресі
/// ніхто не підтвердив передачу (немає ACK).
Error classify(int e, Error fallback) {
  if (e == ENXIO || e == EREMOTEIO || e == ENODEV) {
    return Error::NoAck;
  }
  return fallback;
}

}  // namespace

LinuxBus::~LinuxBus() { close(); }

Result LinuxBus::open(const std::string& path) {
  close();
  fd_ = ::open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    return Result::fail(Error::OpenFailed, errno);
  }
  return Result::success();
}

void LinuxBus::close() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

Result LinuxBus::selectDevice(std::uint8_t addr) {
  if (fd_ < 0) {
    return Result::fail(Error::BadArgs);
  }
  // Адреса запам'ятовується ядром для цього дескриптора; далі звичайні
  // read/write ідуть саме на неї.
  if (::ioctl(fd_, I2C_SLAVE, static_cast<unsigned long>(addr)) < 0) {
    return Result::fail(Error::SelectAddressFailed, errno);
  }
  return Result::success();
}

Result LinuxBus::writeReg(std::uint8_t reg, std::uint8_t value) {
  if (fd_ < 0) {
    return Result::fail(Error::BadArgs);
  }
  const std::array<std::uint8_t, 2> buf{reg, value};
  const ssize_t n = ::write(fd_, buf.data(), buf.size());
  if (n < 0) {
    return Result::fail(classify(errno, Error::WriteFailed), errno);
  }
  if (static_cast<std::size_t>(n) != buf.size()) {
    return Result::fail(Error::WriteFailed);
  }
  return Result::success();
}

Result LinuxBus::readBlock(std::uint8_t reg, std::uint8_t* out,
                           std::size_t len) {
  if (fd_ < 0 || out == nullptr || len == 0) {
    return Result::fail(Error::BadArgs);
  }

  // Фаза 1: кажемо пристрою, з якого регістра читатимемо.
  const ssize_t w = ::write(fd_, &reg, 1);
  if (w < 0) {
    return Result::fail(classify(errno, Error::WriteFailed), errno);
  }
  if (w != 1) {
    return Result::fail(Error::WriteFailed);
  }

  // Фаза 2: читаємо дані. Внутрішній лічильник пристрою автоматично
  // інкрементується, тому один read забирає весь блок.
  const ssize_t r = ::read(fd_, out, len);
  if (r < 0) {
    return Result::fail(classify(errno, Error::ReadFailed), errno);
  }
  if (static_cast<std::size_t>(r) != len) {
    return Result::fail(Error::ShortRead);
  }
  return Result::success();
}

}  // namespace i2c
