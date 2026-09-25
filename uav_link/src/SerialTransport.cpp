#include "uav/SerialTransport.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace uav {
namespace {

speed_t toSpeed(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        default:      return 0;
    }
}

} // namespace

SerialTransport::SerialTransport(std::string name, std::string device, int baud)
    : name_(std::move(name)), device_(std::move(device)), baud_(baud) {}

SerialTransport::~SerialTransport() { close(); }

bool SerialTransport::open(std::string& err) {
    const speed_t speed = toSpeed(baud_);
    if (speed == 0) { err = "непідтримувана швидкість " + std::to_string(baud_); return false; }

    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) { err = device_ + ": " + std::strerror(errno); return false; }

    termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        err = device_ + ": tcgetattr: " + std::strerror(errno);
        close();
        return false;
    }

    ::cfmakeraw(&tio);                       // без обробки, без відлуння, без перекладів рядка
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;                 // апаратного керування потоком на стенді немає
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cc[VMIN]  = 0;                     // читання не блокує ніколи
    tio.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        err = device_ + ": tcsetattr: " + std::strerror(errno);
        close();
        return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialTransport::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

ssize_t SerialTransport::read(std::uint8_t* buf, std::size_t n) {
    if (fd_ < 0) return -1;
    const ssize_t r = ::read(fd_, buf, n);
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    return r;
}

ssize_t SerialTransport::write(const std::uint8_t* buf, std::size_t n) {
    if (fd_ < 0) return -1;
    std::size_t sent = 0;
    while (sent < n) {
        const ssize_t w = ::write(fd_, buf + sent, n - sent);
        if (w > 0) { sent += static_cast<std::size_t>(w); continue; }
        if (w < 0 && errno == EINTR) continue;
        // Порт переповнений: решту відкидаємо. Застаріла телеметрія гірша
        // за її відсутність, а накопичувати чергу тут нема куди.
        break;
    }
    return static_cast<ssize_t>(sent);
}

} // namespace uav
