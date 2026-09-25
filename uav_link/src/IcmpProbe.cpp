#include "uav/IcmpProbe.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/ip_icmp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace uav {
namespace {

constexpr std::size_t kPayload = sizeof(std::uint64_t);

std::uint16_t checksum(const std::uint8_t* data, std::size_t n) {
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i + 1 < n; i += 2)
        sum += static_cast<std::uint32_t>(data[i]) | (static_cast<std::uint32_t>(data[i + 1]) << 8);
    if (n & 1) sum += data[n - 1];
    while (sum >> 16) sum = (sum & 0xFFFF) + (sum >> 16);
    return static_cast<std::uint16_t>(~sum);
}

std::uint64_t nanos(TimePoint t) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}

} // namespace

IcmpProbe::IcmpProbe(std::string host) : host_(std::move(host)) {}
IcmpProbe::~IcmpProbe() { close(); }

bool IcmpProbe::open(std::string& err) {
    // SOCK_DGRAM, а не SOCK_RAW: ядро саме проставляє ідентифікатор і
    // контрольну суму, а програмі не потрібні повні права на сирі сокети.
    fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, IPPROTO_ICMP);
    if (fd_ < 0) { err = std::string("сокет ICMP: ") + std::strerror(errno); return false; }

    dst_.sin_family = AF_INET;
    dst_.sin_port   = 0;
    if (::inet_pton(AF_INET, host_.c_str(), &dst_.sin_addr) != 1) {
        err = "нерозпізнана адреса " + host_;
        close();
        return false;
    }
    return true;
}

void IcmpProbe::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool IcmpProbe::ping(TimePoint now) {
    if (fd_ < 0) return false;

    std::uint8_t pkt[8 + kPayload] = {};
    pkt[0] = ICMP_ECHO;
    pkt[1] = 0;
    // байти 4-5 - ідентифікатор, його підставить ядро
    const std::uint16_t seq = ++seq_;
    pkt[6] = static_cast<std::uint8_t>(seq >> 8);
    pkt[7] = static_cast<std::uint8_t>(seq & 0xFF);
    const std::uint64_t ts = nanos(now);
    std::memcpy(pkt + 8, &ts, kPayload);

    const std::uint16_t crc = checksum(pkt, sizeof(pkt));
    std::memcpy(pkt + 2, &crc, 2);

    return ::sendto(fd_, pkt, sizeof(pkt), 0,
                    reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_)) > 0;
}

std::optional<double> IcmpProbe::readReply(TimePoint now) {
    if (fd_ < 0) return std::nullopt;

    std::uint8_t buf[128];
    const ssize_t r = ::recv(fd_, buf, sizeof(buf), 0);
    if (r < static_cast<ssize_t>(8 + kPayload)) return std::nullopt;
    if (buf[0] != ICMP_ECHOREPLY) return std::nullopt;

    std::uint64_t ts = 0;
    std::memcpy(&ts, buf + 8, kPayload);
    const std::uint64_t nowNs = nanos(now);
    if (ts == 0 || ts > nowNs) return std::nullopt;

    const double ms = static_cast<double>(nowNs - ts) / 1e6;
    if (ms <= 0.0 || ms > 5000.0) return std::nullopt;   // явно не наш пакет
    return ms;
}

} // namespace uav
