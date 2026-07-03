#pragma once
// ============================================================
// GpioSignals — дві вихідні лінії GPIO через libgpiod (API v1):
//   START — «я готовий»: 1 одразу на старті, тримається весь політ;
//   DROP  — одноразовий імпульс 50-100 мс у момент скиду.
// Працює і з реальним чипом плати.
// ============================================================
#include <string>

struct gpiod_chip;
struct gpiod_line;

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
    gpiod_chip* chip_  = nullptr;
    gpiod_line* start_ = nullptr;
    gpiod_line* drop_  = nullptr;
};
