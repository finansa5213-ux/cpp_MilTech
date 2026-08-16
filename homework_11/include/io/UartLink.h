#pragma once
// ============================================================
// UartLink — послідовний порт (termios): 115200, 8N1, raw,
// неблокуюче читання. Однаково працює з віртуальним портом
// socat (/tmp/ttyA) і реальним /dev/ttyAMA* на платі.
// ============================================================
#include <cstdint>
#include <cstddef>
#include <string>

class UartLink {
public:
    UartLink() = default;
    ~UartLink();

    UartLink(const UartLink&)            = delete;
    UartLink& operator=(const UartLink&) = delete;

    // Відкрити і налаштувати порт. false — помилка (лог у stderr).
    bool open(const std::string& dev);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // Неблокуюче читання: >0 — к-сть байтів, 0 — даних немає, -1 — помилка.
    int readBytes(uint8_t* buf, size_t cap);

    // Надіслати PKT_CONTROL (accel, turnRate у [-1..1]).
    bool sendControl(float accel, float turnRate);

private:
    int fd_ = -1;
};
