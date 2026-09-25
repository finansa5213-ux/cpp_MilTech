#include "uav/UdpTransport.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace uav {

UdpTransport::UdpTransport(std::string name, std::uint16_t bindPort,
                           std::string peerHost, std::uint16_t peerPort, bool peerFixed)
    : name_(std::move(name)), bindPort_(bindPort), peerHost_(std::move(peerHost)),
      peerPort_(peerPort), peerFixed_(peerFixed) {}

UdpTransport::~UdpTransport() { close(); }

bool UdpTransport::open(std::string& err) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd_ < 0) { err = std::string("socket: ") + std::strerror(errno); return false; }

    sockaddr_in local{};
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port        = htons(bindPort_);
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        err = "bind " + std::to_string(bindPort_) + ": " + std::strerror(errno);
        close();
        return false;
    }

    peer_.sin_family = AF_INET;
    peer_.sin_port   = htons(peerPort_);
    if (::inet_pton(AF_INET, peerHost_.c_str(), &peer_.sin_addr) != 1) {
        err = "нерозпізнана адреса " + peerHost_;
        close();
        return false;
    }
    return true;
}

void UdpTransport::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

ssize_t UdpTransport::read(std::uint8_t* buf, std::size_t n) {
    if (fd_ < 0) return -1;
    for (;;) {
        sockaddr_in from{};
        socklen_t   len = sizeof(from);
        const ssize_t r = ::recvfrom(fd_, buf, n, 0,
                                     reinterpret_cast<sockaddr*>(&from), &len);
        if (r < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        if (!peerFixed_ || from.sin_addr.s_addr == peer_.sin_addr.s_addr) return r;

        // Трафік повз концентратор: рахуємо й називаємо джерело один раз.
        bypass_ += static_cast<std::uint64_t>(r);
        char ip[INET_ADDRSTRLEN] = {};
        ::inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
        if (std::strncmp(ip, rejected_, sizeof(rejected_)) != 0) {
            std::snprintf(rejected_, sizeof(rejected_), "%s", ip);
            newRejection_ = true;
        }
    }
}

ssize_t UdpTransport::write(const std::uint8_t* buf, std::size_t n) {
    if (fd_ < 0) return -1;
    const ssize_t w = ::sendto(fd_, buf, n, 0,
                               reinterpret_cast<const sockaddr*>(&peer_), sizeof(peer_));
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
    return w;
}

} // namespace uav
