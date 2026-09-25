// Спільний інтерфейс каналу передавання.
//
// Саме заради нього маршрутизатор не знає, чим саме він говорить: під ним
// однаково лежать послідовний порт радіомодема і сокет у тунелі. Додати
// третій канал - це новий клас, а не правка арбітра.
#pragma once

#include <cstddef>
#include <cstdint>
#include <sys/types.h>

namespace uav {

class ITransport {
public:
    virtual ~ITransport() = default;

    /// Дескриптор для epoll. -1, якщо канал не відкрито.
    virtual int fd() const = 0;

    /// Прочитати доступні байти. 0 - даних немає, -1 - помилка.
    virtual ssize_t read(std::uint8_t* buf, std::size_t n) = 0;

    /// Надіслати байти. Повертає скільки пішло або -1.
    virtual ssize_t write(const std::uint8_t* buf, std::size_t n) = 0;

    virtual const char* name() const = 0;
    virtual bool        isOpen() const = 0;
};

} // namespace uav
