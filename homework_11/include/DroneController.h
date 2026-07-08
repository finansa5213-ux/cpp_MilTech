#pragma once
// ============================================================
// DroneController — модуль керування дроном (вимога ДЗ11 п.2.2).
// Відокремлений від логіки наведення (MissionProcessor):
// перетворює рішення місії («тримати курс на точку aim»)
// на нормовані команди UART accel/turnRate у [-1..1].
// ============================================================
#include "Types.h"
#include "drone_link.h"

struct ControlCmd {
    float accel    = 0.f;
    float turnRate = 0.f;
};

class DroneController {
public:
    explicit DroneController(float kTurn = 3.0f) : kTurn_(kTurn) {}

    // t     — поточна телеметрія;
    // aim   — точка, на яку летимо;
    // brake — true, якщо ми ближче за мінімальну дальність скиду
    //         (треба гасити швидкість, щоб зменшити виніс).
    ControlCmd steer(const dlink::Telemetry& t, const Coord& aim, bool brake) const;

private:
    float kTurn_; // П-коефіцієнт повороту: turnRate = clamp(kTurn * кутова_помилка)
};
