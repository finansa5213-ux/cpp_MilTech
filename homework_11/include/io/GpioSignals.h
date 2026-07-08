#pragma once
// ============================================================
// GpioSignals — дві вихідні лінії GPIO через libgpiod:
//   START — «я готовий»: 1 одразу на старті, тримається весь політ;
//   DROP  — одноразовий імпульс 50-100 мс у момент скиду.
// Підтримує обидва API libgpiod:
//   v1 (Raspberry Pi OS Bookworm, Ubuntu 22.04) — як в умові ДЗ;
//   v2 (Ubuntu 24.04+) — обирається автоматично в CMake
//       (define HW11_GPIOD_V2).
// Працює і з gpio-sim (симуляція), і з реальним чипом плати.
// ============================================================
#include <string>

class GpioSignals {
public:
    GpioSignals() = default;
    ~GpioSignals();

    GpioSignals(const GpioSignals&)            = delete;
    GpioSignals& operator=(const GpioSignals&) = delete;

    // Відкрити чип і зарезервувати START/DROP як виходи (обидва = 0).
    bool init(const std::string& chipName, unsigned startLine, unsigned dropLine);

    void assertStart();              // START = 1 (сигнал чекеру починати)
    void pulseDrop(unsigned ms = 80);// імпульс DROP: 1 -> пауза -> 0
    void shutdown();                 // опустити лінії і звільнити ресурси

private:
    // Opaque-вказівники, щоб не тягнути gpiod.h у заголовок:
    // v1: chip_ = gpiod_chip*, start_/drop_ = gpiod_line*
    // v2: chip_ = gpiod_chip*, start_/drop_ = gpiod_line_request*
    void*    chip_      = nullptr;
    void*    start_     = nullptr;
    void*    drop_      = nullptr;
    unsigned startLine_ = 0;
    unsigned dropLine_  = 0;
};
