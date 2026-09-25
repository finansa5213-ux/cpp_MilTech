// Послідовний порт: UART польотного контролера або радіомодем SiK.
#pragma once

#include <string>

#include "uav/ITransport.hpp"

namespace uav {

class SerialTransport final : public ITransport {
public:
    SerialTransport(std::string name, std::string device, int baud);
    ~SerialTransport() override;

    SerialTransport(const SerialTransport&)            = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    /// Відкрити порт у сирому режимі без блокування. err заповнюється при невдачі.
    bool open(std::string& err);
    void close();

    int         fd() const override     { return fd_; }
    bool        isOpen() const override { return fd_ >= 0; }
    const char* name() const override   { return name_.c_str(); }

    ssize_t read(std::uint8_t* buf, std::size_t n) override;
    ssize_t write(const std::uint8_t* buf, std::size_t n) override;

private:
    std::string name_;
    std::string device_;
    int         baud_;
    int         fd_ = -1;
};

} // namespace uav
