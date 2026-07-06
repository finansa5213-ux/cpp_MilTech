#pragma once
// ============================================================
// Types.h — спільна доменна модель проекту.
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
    bool operator==(const Coord& o) const {
        return std::fabs(x-o.x) < EPS && std::fabs(y-o.y) < EPS;
    }
    bool operator!=(const Coord& o) const { return !(*this == o); }
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
// AmmoParams
// ------------------------------------------------------------
struct AmmoParams {
    std::string name;
    float mass = 0.f;
    float drag = 0.f;
    float lift = 0.f;
};

// ------------------------------------------------------------
// Target
// ------------------------------------------------------------
struct Target {
    Coord pos;
    Coord velocity;
    Target() {}
    Target(const Coord& p, const Coord& v) : pos(p), velocity(v) {}
};

// ------------------------------------------------------------
// DropPoint
// ------------------------------------------------------------
struct DropPoint {
    int   targetIndex    = -1;
    Coord firePoint;
    Coord predictedTarget;
    float totalTime      = 0.f;   // час польоту дрона до firePoint
    float fallTime       = 0.f;   // час падіння бомби
    bool  valid          = false;
};

// ------------------------------------------------------------
// MissionConfig
// ------------------------------------------------------------
struct MissionConfig {
    Coord       startPos;
    float       altitude      = 0.f;
    float       initialDir    = 0.f;
    float       attackSpeed   = 0.f;
    float       accelPath     = 0.f;
    std::string ammoName;
    float       arrayTimeStep = 0.f;
    float       simTimeStep   = 0.05f;
    float       hitRadius     = 5.f;
    float       angularSpeed  = 0.f;
    float       turnThreshold = 0.f;
    int         maxSteps      = 10000;
};