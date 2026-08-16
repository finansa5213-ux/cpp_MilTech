#pragma once

#include <cstdint>

#include "drone_link.h" // з homework_11
#include "mavlink_link.hpp"

namespace mavtel {

/// Конверсія кадру телеметрії ДЗ11 у зліпок для MAVLink.
///
/// Осі ДЗ11: напрямок польоту рахується як {cos(dir), sin(dir)} — отже dir це
/// математичний кут від осі X проти годинникової стрілки. Мапимо x -> схід,
/// y -> північ. Компасний курс (0 = північ, за годинниковою) беремо з вектора
/// швидкості: так він гарантовано узгоджений із напрямком руху, що й перевіряє
/// чекер. Якщо швидкість майже нульова — падаємо назад на t.dir.
TelemetrySample toSample(const dlink::Telemetry& telemetry);

/// Міст між автопілотом ДЗ11 і MAVLink 2 по UDP.
///
/// Виділений в окремий клас навмисно: уся конверсія одиниць, темпи HEARTBEAT
/// і логіка вибігу покриваються юніт-тестами без UART, GPIO і сокетів.
/// MissionProcessor лише викликає його методи у трьох місцях.
class Dz11Bridge {
public:
    static constexpr std::uint32_t kHeartbeatPeriodMs = 1000; ///< ~1 Гц
    static constexpr std::uint32_t kCoastPeriodMs = 100;      ///< 10 Гц у вибігу

    explicit Dz11Bridge(MavlinkLink& link);

    /// Кожен кадр PKT_TELEMETRY від чекера ДЗ11.
    void onTelemetry(const dlink::Telemetry& telemetry, std::uint32_t wall_ms);

    /// Момент скиду — поруч із gpio_.pulseDrop().
    void onDrop(const dlink::Telemetry& telemetry, std::uint32_t wall_ms);

    /// Щоітерації циклу місії, зокрема на тайм-ауті очікування телеметрії:
    /// HEARTBEAT за розкладом + приймання ACK + повтори COMMAND_LONG.
    void tick(std::uint32_t wall_ms);

    /// Режим вибігу: телеметрія ДЗ11 припинилась (місію оцінено), але треба
    /// добити повтори COMMAND_LONG, не зупиняючи потік MAVLink. Позиція
    /// екстраполюється по останній швидкості, тому узгодженість
    /// «зміна позиції vs швидкості» зберігається.
    void coast(std::uint32_t wall_ms);

    bool hasSample() const noexcept { return has_sample_; }
    bool dropFinished() const noexcept { return link_.dropFinished(); }
    DropState dropState() const noexcept { return link_.dropState(); }
    int dropAttempts() const noexcept { return link_.dropAttempts(); }

    /// Останній зліпок, відданий у MAVLink (для логів і тестів).
    const TelemetrySample& lastSample() const noexcept { return last_; }

private:
    void sendHeartbeatIfDue(std::uint32_t wall_ms);

    MavlinkLink& link_;
    TelemetrySample last_{};
    bool has_sample_{false};
    bool heartbeat_sent_{false};
    std::uint32_t last_heartbeat_ms_{0};
    std::uint32_t last_sample_ms_{0};
};

/// Монотонний час у мілісекундах від старту програми (steady_clock).
std::uint32_t nowMs();

} // namespace mavtel
