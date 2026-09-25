// Вимірювання повного шляху борт <-> оператор.
//
// Прототип на Python запускав зовнішній ping і розбирав його вивід. Тут - сокет
// ICMP просто в epoll: ні породження процесів, ні розбору тексту, ні залежності
// від iputils в образі. На машині оператора нічого ставити не треба.
//
// Навіщо взагалі міряти, а не додавати константу: два прогони з різницею десять
// хвилин дали наземну ділянку 72,0 і 65,2 мс, а найгірший викид ночі (повний
// шлях 197,8 мс при тунелі 75,6) стався саме в ній, а не в стільниковій частині.
#pragma once

#include <cstdint>
#include <netinet/in.h>
#include <optional>
#include <string>

#include "uav/Types.hpp"

namespace uav {

class IcmpProbe {
public:
    explicit IcmpProbe(std::string host);
    ~IcmpProbe();

    IcmpProbe(const IcmpProbe&)            = delete;
    IcmpProbe& operator=(const IcmpProbe&) = delete;

    /// Потребує прав root або дозволу net.ipv4.ping_group_range.
    /// При невдачі повертає false - маршрутизатор працює далі з оцінкою.
    bool open(std::string& err);
    void close();

    int  fd() const     { return fd_; }
    bool isOpen() const { return fd_ >= 0; }

    /// Надіслати запит. Мітку часу кладемо в корисні дані: відповідь принесе
    /// її назад, і зберігати стан не доведеться.
    bool ping(TimePoint now);

    /// Прочитати відповідь. nullopt - це був не наш пакет.
    std::optional<double> readReply(TimePoint now);

private:
    std::string   host_;
    int           fd_  = -1;
    sockaddr_in   dst_{};
    std::uint16_t seq_ = 0;
};

} // namespace uav
