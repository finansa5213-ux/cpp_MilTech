#pragma once
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <stdexcept>
#include <string>

// Неблокуючий UDP socket з RAII-володiнням ресурсом.
// У конструкторi робить bind на bind_port; при помилцi кидає std::runtime_error.
class UdpSocket {
public:
    explicit UdpSocket(uint16_t bind_port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
            throw std::runtime_error("socket() failed");

        ::fcntl(fd_, F_SETFL, O_NONBLOCK);

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(bind_port);

        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("bind() failed on port " + std::to_string(bind_port));
    }

    ~UdpSocket() { if (fd_ >= 0) ::close(fd_); }

    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    UdpSocket(UdpSocket&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }

    // Прочитати данi у buf. Повертає кiлькiсть байтiв або -1, якщо даних немає (EAGAIN).
    ssize_t recv(void* buf, std::size_t len) {
        return ::recvfrom(fd_, buf, len, 0, nullptr, nullptr);
    }

    // Прочитати данi разом з адресою sender, якщо потрiбно вiдповiдати назад.
    ssize_t recv(void* buf, std::size_t len, sockaddr_in& sender) {
        socklen_t slen = sizeof(sender);
        return ::recvfrom(fd_, buf, len, 0,
                          reinterpret_cast<sockaddr*>(&sender), &slen);
    }

    void send(const void* buf, std::size_t len, const sockaddr_in& to) {
        ::sendto(fd_, buf, len, 0,
                 reinterpret_cast<const sockaddr*>(&to), sizeof(to));
    }

    int fd() const { return fd_; }

private:
    int fd_ = -1;
};

// Створити sockaddr_in з IP-рядка у форматi a.b.c.d та порту.
inline sockaddr_in make_addr(const std::string& ip, uint16_t port) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &a.sin_addr) != 1)
        throw std::runtime_error("invalid IP: " + ip);
    return a;
}
