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
// DroneMode — режим руху дрона (ДЗ10).
// У ДЗ9 enum DroneState прибрали на користь патерна State; для
// команд між потоками та для логу зручно мати легкий enum.
// ------------------------------------------------------------
enum class DroneMode {
    IDLE,          // команди ще немає
    TURNING,
    ACCELERATING,
    CRUISE,
    ATTACK,
    STOP,
    ERROR
};

inline const char* droneModeName(DroneMode m) {
    switch (m) {
        case DroneMode::IDLE:         return "IDLE";
        case DroneMode::TURNING:      return "TURNING";
        case DroneMode::ACCELERATING: return "ACCELERATING";
        case DroneMode::CRUISE:       return "CRUISE";
        case DroneMode::ATTACK:       return "ATTACK";
        case DroneMode::STOP:         return "STOP";
        case DroneMode::ERROR:        return "ERROR";
    }
    return "UNKNOWN";
}

// ------------------------------------------------------------
// DroneCommand — команда від MissionProcessor до DronePhysics.
// Умова фіксує {state, angleSpeed}; щоб фізика могла інтегрувати
// кінематику ДЗ9 (поворот до цілі, розгін за дистанцією), команда
// несе ще firePoint та параметри руху сегмента.
// ------------------------------------------------------------
struct DroneCommand {
    DroneMode mode          = DroneMode::IDLE;  // бажаний режим
    Coord     firePoint;                        // ціль поточного сегмента
    float     angleSpeed    = 0.f;              // кутова швидкість повороту, рад/с
    float     attackSpeed   = 0.f;
    float     accelPath     = 0.f;
    float     turnThreshold = 0.f;
    float     hitRadius     = 5.f;
};

// ------------------------------------------------------------
// DroneTelemetry — знімок стану дрона, що віддає DronePhysics.
// timeSecSinceStart — модельний час останнього оновлення фізики
// (сума dt усіх фізичних кроків). Потрібен, щоб чекер не вважав
// кроки логу рівномірними у часі.
// ------------------------------------------------------------
struct DroneTelemetry {
    Coord     pos;                              // поточна позиція
    Coord     speed;                            // вектор швидкості
    float     dir               = 0.f;          // курс, рад (поле direction)
    DroneMode mode              = DroneMode::IDLE;
    float     timeSecSinceStart = 0.f;          // час останнього оновлення фізики
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

    // --- ДЗ10: багатопоточність ---
    float       physicsTimeStep = 0.01f; // крок інтегрування фізики дрона
    float       targetTimeStep  = 0.05f; // крок потоку провайдера (інформативно)
    float       timeScale       = 1.0f;  // прискорення часу: sleep = dt / timeScale
};
