#include "I2cError.hpp"

#include <cstring>

namespace i2c {
namespace {

const char* reason(Error e) {
  switch (e) {
    case Error::Ok:
      return "успішно";
    case Error::BadArgs:
      return "некоректні аргументи виклику";
    case Error::OpenFailed:
      return "не вдалося відкрити шину";
    case Error::SelectAddressFailed:
      return "ядро не дало обрати адресу (ioctl I2C_SLAVE)";
    case Error::NoAck:
      return "пристрій не відповідає на шині (немає ACK)";
    case Error::WriteFailed:
      return "обрив запису в регістр";
    case Error::ReadFailed:
      return "обрив читання";
    case Error::ShortRead:
      return "обрив читання: отримано менше байтів, ніж запитано";
    case Error::BadId:
      return "невірний ID пристрою";
    case Error::NotApplied:
      return "пристрій не прийняв записане значення (перевірено читанням)";
  }
  return "невідома помилка";
}

}  // namespace

std::string describe(const Result& r) {
  std::string out = reason(r.code);
  if (r.sys_errno != 0) {
    out += " [errno ";
    out += std::to_string(r.sys_errno);
    out += ": ";
    out += std::strerror(r.sys_errno);
    out += "]";
  }
  return out;
}

}  // namespace i2c
