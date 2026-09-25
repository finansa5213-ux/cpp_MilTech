// Канал B: UDP у тунелі WireGuard до концентратора.
//
// Фільтр джерела тут не косметика. Поки борт відповідав тому, хто озвався
// останнім, станція-спостерігач могла говорити з ним прямо, повз концентратор,
// і розмежування прав, реалізоване лише на сервері, обходилося налаштуванням
// клієнта. Тепер це другий, незалежний рубіж: борт приймає висхідний трафік
// рівно з однієї адреси.
#pragma once

#include <cstdint>
#include <netinet/in.h>
#include <string>

#include "uav/ITransport.hpp"

namespace uav {

class UdpTransport final : public ITransport {
public:
    UdpTransport(std::string name, std::uint16_t bindPort,
                 std::string peerHost, std::uint16_t peerPort, bool peerFixed);
    ~UdpTransport() override;

    UdpTransport(const UdpTransport&)            = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;

    bool open(std::string& err);
    void close();

    int         fd() const override     { return fd_; }
    bool        isOpen() const override { return fd_ >= 0; }
    const char* name() const override   { return name_.c_str(); }

    /// Прочитати одну прийнятну датаграму. Чужі джерела відкидаються й
    /// зараховуються до bypassBytes(); читання триває до першої прийнятної
    /// або до вичерпання черги.
    ssize_t read(std::uint8_t* buf, std::size_t n) override;

    ssize_t write(const std::uint8_t* buf, std::size_t n) override;

    std::uint64_t bypassBytes() const { return bypass_; }
    void          resetBypass()       { bypass_ = 0; }
    /// Адреса першого відкинутого джерела - щоб назвати його в журналі один раз.
    const char*   lastRejected() const { return rejected_; }
    bool          hasNewRejection()    { const bool f = newRejection_; newRejection_ = false; return f; }

private:
    std::string   name_;
    std::uint16_t bindPort_;
    std::string   peerHost_;
    std::uint16_t peerPort_;
    bool          peerFixed_;

    int         fd_ = -1;
    sockaddr_in peer_{};

    std::uint64_t bypass_       = 0;
    char          rejected_[24] = {};
    bool          newRejection_ = false;
};

} // namespace uav
