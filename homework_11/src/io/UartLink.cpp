// ============================================================
// UartLink.cpp — робота з послідовним портом через termios.
// ============================================================
#include "io/UartLink.h"
#include "drone_link.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>

UartLink::~UartLink() { close(); }

bool UartLink::open(const std::string& dev)
{
    // O_NONBLOCK: read() повертає -1/EAGAIN, якщо даних нема, а не блокується.
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::perror(("[uart] open " + dev).c_str());
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::perror("[uart] tcgetattr");
        close();
        return false;
    }
    cfmakeraw(&tio);                 // 8N1, без обробки символів
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);      // швидкість з обох боків однакова!
    tio.c_cflag |= (CLOCAL | CREAD);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::perror("[uart] tcsetattr");
        close();
        return false;
    }
    tcflush(fd_, TCIOFLUSH);         // прибрати сміття, що могло лишитись у буфері
    std::printf("[uart] %s vidkryto (115200 8N1 raw)\n", dev.c_str());
    return true;
}

void UartLink::close()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int UartLink::readBytes(uint8_t* buf, size_t cap)
{
    if (fd_ < 0) return -1;
    ssize_t n = ::read(fd_, buf, cap);
    if (n > 0)  return (int)n;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0; // даних немає — норма
    if (n == 0) return 0;
    return -1;
}

bool UartLink::sendControl(float accel, float turnRate)
{
    if (fd_ < 0) return false;
    dlink::Control c{accel, turnRate};
    uint8_t out[64];
    size_t  m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof c, out);
    return ::write(fd_, out, m) == (ssize_t)m;
}
