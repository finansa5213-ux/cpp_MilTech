#pragma once

#include <netinet/in.h>

#include <cstdint>
#include <string>

#include "transport.hpp"

namespace mavtel {

struct Endpoint {
    std::string host{"127.0.0.1"};
    std::uint16_t port{14550};
};

/// Розбір рядка виду "127.0.0.1:14550". Кидає std::invalid_argument при помилці.
Endpoint parseEndpoint(const std::string& text);

/// Неблокуючий UDP-сокет, який шле датаграми на фіксовану адресу.
/// Сокет навмисно НЕ прив'язується до локального порту: ОС видасть ефемерний,
/// і QGC/чекер відправлять COMMAND_ACK саме на нього (стандартна поведінка GCS).
class UdpSocket final : public ITransport {
public:
    UdpSocket(const std::string& host, std::uint16_t port);
    ~UdpSocket() override;

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;
    UdpSocket(UdpSocket&&) = delete;
    UdpSocket& operator=(UdpSocket&&) = delete;

    bool send(const std::uint8_t* data, std::size_t len) override;
    std::size_t receive(std::uint8_t* buffer, std::size_t capacity) override;

private:
    int fd_{-1};
    sockaddr_in dest_{};
};

} // namespace mavtel
