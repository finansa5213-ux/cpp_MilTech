// ============================================================
// DroneController.cpp — P-регулятор курсу + логіка газу.
// ============================================================
#include "DroneController.h"

#include <algorithm>
#include <cmath>

ControlCmd DroneController::steer(const dlink::Telemetry& t,
                                  const Coord& aim, bool brake) const
{
    ControlCmd c;

    const Coord pos{t.x, t.y};
    const float want = std::atan2(aim.y - pos.y, aim.x - pos.x);
    const float err  = normalizeAngle(want - t.dir); // (-pi, pi], + = треба вліво

    // Поворот: пропорційно помилці курсу; знак умови: +1 = вліво.
    c.turnRate = std::clamp(kTurn_ * err, -1.f, 1.f);

    // Газ:
    if (brake) {
        c.accel = -1.f;                       // усередині мін. дальності — гальмо
    } else if (std::fabs(err) > 1.2f) {
        // сильно не в курсі: не розганяємось, на швидкості пригальмовуємо,
        // щоб радіус розвороту не тягнув повз ціль
        c.accel = (t.speed > 5.f) ? -0.5f : 0.2f;
    } else {
        c.accel = 1.f;                        // курс ок — повний газ
    }
    return c;
}
