#include "udp_socket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace mavtel {

Endpoint parseEndpoint(const std::string& text) {
    const std::size_t colon = text.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= text.size()) {
        throw std::invalid_argument("Очікується формат host:port, отримано: " + text);
    }

    Endpoint ep;
    ep.host = text.substr(0, colon);

    const std::string port_text = text.substr(colon + 1);
    std::size_t consumed = 0;
    const int port = std::stoi(port_text, &consumed);
    if (consumed != port_text.size() || port <= 0 || port > 65535) {
        throw std::invalid_argument("Некоректний порт: " + port_text);
    }
    ep.port = static_cast<std::uint16_t>(port);
    return ep;
}

UdpSocket::UdpSocket(const std::string& host, std::uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        const std::string reason = std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("fcntl(O_NONBLOCK) failed: " + reason);
    }

    std::memset(&dest_, 0, sizeof(dest_));
    dest_.sin_family = AF_INET;
    dest_.sin_port = ::htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &dest_.sin_addr) != 1) {
        ::close(fd_);
        fd_ = -1;
        throw std::invalid_argument("Некоректна IPv4-адреса: " + host);
    }
}

UdpSocket::~UdpSocket() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool UdpSocket::send(const std::uint8_t* data, std::size_t len) {
    const ssize_t sent = ::sendto(fd_, data, len, 0,
                                  reinterpret_cast<const sockaddr*>(&dest_), sizeof(dest_));
    return sent == static_cast<ssize_t>(len);
}

std::size_t UdpSocket::receive(std::uint8_t* buffer, std::size_t capacity) {
    const ssize_t received = ::recvfrom(fd_, buffer, capacity, 0, nullptr, nullptr);
    if (received <= 0) {
        return 0; // EAGAIN/EWOULDBLOCK — даних немає, це нормально
    }
    return static_cast<std::size_t>(received);
}

} // namespace mavtel
