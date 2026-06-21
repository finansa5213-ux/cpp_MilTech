// ============================================================
// AnalyticalSolver.cpp — імплементація аналітичної балістики.
// ============================================================

#include "AnalyticalSolver.h"
#include <cmath>

bool AnalyticalSolver::computeTimeOfFlight(const AmmoParams& ammo,
                                           float V0, float Z0, float& t) const {
    const float m = ammo.mass;
    const float d = ammo.drag;
    const float l = ammo.lift;

    const float a = d * G_ACCEL * m - 2.f * d * d * l * V0;
    const float b = -3.f * G_ACCEL * m * m + 3.f * d * l * m * V0;
    const float c = 6.f * m * m * Z0;

    if (fabs(a) < EPS) {
        if (fabs(b) > EPS && (-c / b) > 0.f) t = sqrt(-c / b);
        else                                  t = sqrt(2.f * Z0 / G_ACCEL);
        return t > 0.f;
    }

    const float p = -b * b / (3.f * a * a);
    const float q =  2.f * b * b * b / (27.f * a * a * a) + c / a;
    if (p >= 0.f) return false;

    float arg = (3.f * q / (2.f * p)) * sqrt(-3.f / p);
    if (arg >  1.f) arg =  1.f;
    if (arg < -1.f) arg = -1.f;

    const float phi = acos(arg);
    const float r   = sqrt(-p / 3.f);
    t = 2.f * r * cos((phi + 4.f * (float)M_PI) / 3.f) - b / (3.f * a);
    return t > 0.f;
}

float AnalyticalSolver::computeHorizontalDistance(const AmmoParams& ammo,
                                                   float V0, float t) const {
    const float m = ammo.mass, d = ammo.drag, l = ammo.lift;
    const float t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
    const float d2 = d*d, d3 = d2*d, d4 = d3*d;
    const float l2 = l*l, l3 = l2*l, l4 = l3*l;
    const float m2 = m*m, m3 = m2*m, m4 = m3*m;
    const float onePlusL2 = 1.f + l2;
    const float onePlusL2_sq = onePlusL2 * onePlusL2;
    const float onePlusL2plusL4 = 1.f + l2 + l4;

    const float C1 = V0;
    const float C2 = -(d * V0) / (2.f * m);
    const float C3 = (6.f * d * G_ACCEL * l * m - 6.f * d2 * (l2 - 1.f) * V0) / (36.f * m2);
    const float C4 = (-6.f * d2 * G_ACCEL * l * onePlusL2plusL4 * m
                      + 3.f * d3 * l2 * onePlusL2 * V0
                      + 6.f * d3 * l4 * onePlusL2 * V0)
                     / (36.f * onePlusL2_sq * m3);
    const float C5 = (3.f * d3 * G_ACCEL * l3 * m - 3.f * d4 * l2 * onePlusL2 * V0)
                     / (36.f * onePlusL2 * m4);

    return C1 * t + C2 * t2 + C3 * t3 + C4 * t4 + C5 * t5;
}

DropPoint AnalyticalSolver::solve(const Coord& dronePos,
                                   const Target& target,
                                   float altitude,
                                   const AmmoParams& ammo,
                                   float attackSpeed)
{
    DropPoint out;
    out.valid = false;

    float t_fall = 0.f;
    if (!computeTimeOfFlight(ammo, attackSpeed, altitude, t_fall)) return out;

    float h = computeHorizontalDistance(ammo, attackSpeed, t_fall);
    if (h <= 0.f) return out;

    // Прогноз цілі: де вона буде, коли бомба впаде.
    Coord predicted = target.pos + target.velocity * t_fall;

    Coord delta = predicted - dronePos;
    float D = length(delta);
    if (D < EPS) return out;

    Coord fire;
    if (D >= h) fire = predicted - normalize(delta) * h;
    else        fire = dronePos;  // дрон уже перейшов точку скиду

    out.firePoint       = fire;
    out.predictedTarget = predicted;
    out.totalTime       = t_fall;  // тут лише час падіння; час підльоту
                                   // дрона MissionProcessor може долічити
                                   // окремо, якщо буде потрібно
    out.valid           = true;
    // targetIndex заповнить MissionProcessor (solver його не знає)
    return out;
}
