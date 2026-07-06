#pragma once
// fc_link.hpp: готовий допомiжний файл, змiнювати не потрiбно.
//
// Тонка C++ обгортка над MAVSDK для комунiкацiї з ArduPilot.
// Ховає налаштування з'єднання, плагiни MAVSDK та потокобезпечнiсть.
// Пiдключити цей заголовок i використати FcLink у c2_controller.cpp.
//
// Потрiбний API:
// - is_connected(): чи отримано перший HEARTBEAT
// - is_armed(): чи FC зараз armed
// - flight_mode(): поточний режим руху
// - hold(): перейти у HOLD
// - go_to_ned(north_m, east_m): передати точку маршруту

#include <chrono>
#include <cstdint>
#include <memory>

// FcLink пiдключається до ArduPilot FC через MAVSDK
// i надає простий API опитування для стану armed, режиму руху та MAVLink-команд.
//
// Типове використання:
//
//   FcLink fc(14551);                    // блокується до вiдповiдi автопiлота (30 s)
//
//   while (true) {
//       if (fc.is_armed() && fc.flight_mode() == FcLink::FlightMode::Guided)
//           fc.go_to_ned(50.0f, 0.0f);   // передати точку маршруту
//       else if (fc.is_armed())
//           fc.hold();                   // перейти у HOLD mode
//       std::this_thread::sleep_for(100ms);
//   }
class FcLink {
public:
    // Режим руху з ArduPilot HEARTBEAT.
    // У ArduPilot це режим GUIDED. MAVSDK може показувати його як Offboard,
    // тому FcLink нормалiзує це до FlightMode::Guided.
    enum class FlightMode { Unknown, Manual, Hold, Guided };

    // Пiдключитися до ArduPilot.
    // ArduPilot має надсилати MAVLink UDP-пакети на 127.0.0.1:<listen_port>.
    // Блокується до вiдповiдi автопiлота або до завершення часу очiкування.
    // Кидає std::runtime_error, якщо з'єднання не вiдкрилось або минув час очiкування.
    explicit FcLink(uint16_t listen_port,
                    std::chrono::seconds timeout = std::chrono::seconds{30});

    ~FcLink();

    FcLink(const FcLink&)            = delete;
    FcLink& operator=(const FcLink&) = delete;

    // Повертає true пiсля першого отриманого HEARTBEAT.
    // Використати це як сигнал для перевiрки стану (/tmp/c2_healthy).
    bool is_connected() const;

    // Повертає true, якщо FC зараз у станi armed.
    bool is_armed() const;

    // Повертає поточний режим руху. Оновлюється приблизно раз на 1 s через HEARTBEAT.
    FlightMode flight_mode() const;

    // Надiслати MAVLink-команду для переходу FC у HOLD mode.
    // Безпечно викликати з будь-якого стану; ArduPilot зупиниться i триматиме позицiю.
    void hold();

    // Надiслати цiльову позицiю у локальнiй системi координат NED (метри вiд home).
    // - north_m: додатне значення = пiвнiч
    // - east_m:  додатне значення = схiд
    // - down_m:  додатне значення = вниз (для наземного ровера лишити 0)
    // На першому виклику автоматично стартує сесiя MAVSDK Offboard.
    // Щоб команда спрацювала, FC має бути у режимi GUIDED.
    void go_to_ned(float north_m, float east_m, float down_m = 0.0f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
