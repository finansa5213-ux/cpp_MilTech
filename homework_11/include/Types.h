#pragma once
// ============================================================
// Types.h — доменна модель ДЗ11 (успадкована з ДЗ10, скорочена).
// Джерело даних тепер UART (drone_link.h), тому залишились лише
// геометрія, боєприпас, ціль і DropPoint для балістичного солвера.
// ============================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

const float G_ACCEL = 9.81f;
const float EPS     = 1e-6f;

// ------------------------------------------------------------
// Coord
// ------------------------------------------------------------
struct Coord {
    float x, y;

    Coord() : x(0.f), y(0.f) {}
    Coord(float ax, float ay) : x(ax), y(ay) {}

    Coord operator+(const Coord& o) const { return {x + o.x, y + o.y}; }
    Coord operator-(const Coord& o) const { return {x - o.x, y - o.y}; }
    Coord operator*(float s)        const { return {x * s,   y * s};   }
    Coord operator/(float s)        const {
        if (std::fabs(s) < EPS) return {};
        return {x / s, y / s};
    }
    Coord& operator+=(const Coord& o) { x += o.x; y += o.y; return *this; }
    Coord& operator-=(const Coord& o) { x -= o.x; y -= o.y; return *this; }
};

inline float length(const Coord& c)                   { return std::hypot(c.x, c.y); }
inline Coord normalize(const Coord& c) {
    float L = length(c);
    return (L < EPS) ? Coord{} : c / L;
}
inline float distance(const Coord& a, const Coord& b) { return length(b - a); }
inline float normalizeAngle(float a) {
    a = std::fmod(a + (float)M_PI, 2.f * (float)M_PI);
    if (a < 0.f) a += 2.f * (float)M_PI;
    return a - (float)M_PI;
}

// ------------------------------------------------------------
// AmmoParams — параметри балістики (з PKT_AMMO)
// ------------------------------------------------------------
struct AmmoParams {
    std::string name;
    float mass = 0.f;
    float drag = 0.f;
    float lift = 0.f;
};

// ------------------------------------------------------------
// Target — ціль (позиція з PKT_TARGET + оцінена швидкість)
// ------------------------------------------------------------
struct Target {
    Coord pos;
    Coord velocity;
    Target() {}
    Target(const Coord& p, const Coord& v) : pos(p), velocity(v) {}
};

// ------------------------------------------------------------
// DropPoint — рішення балістичного солвера.
// carry (нове у ДЗ11) — горизонтальний виніс боєприпасу, м:
// відстань, яку бомба пролетить уперед від точки скиду.
// ------------------------------------------------------------
struct DropPoint {
    int   targetIndex    = -1;
    Coord firePoint;              // де треба бути в момент скиду
    Coord predictedTarget;        // де буде ціль у момент падіння
    float fallTime       = 0.f;   // час падіння бомби, с
    float carry          = 0.f;   // горизонтальний виніс, м
    bool  valid          = false;
};
