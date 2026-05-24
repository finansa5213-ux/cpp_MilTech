#pragma once
// ============================================================
// Types.h — спільна доменна модель проекту.
// ============================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

// Глобальні константи фізики
const float G_ACCEL = 9.81f;
const float EPS     = 1e-6f;

// ------------------------------------------------------------
// Coord — 2D координата з перевантаженими арифметичними операторами.
// ------------------------------------------------------------
struct Coord {
    float x;
    float y;

    Coord() : x(0.f), y(0.f) {}
    Coord(float ax, float ay) : x(ax), y(ay) {}

    Coord operator+(const Coord& o) const { return Coord(x + o.x, y + o.y); }
    Coord operator-(const Coord& o) const { return Coord(x - o.x, y - o.y); }
    Coord operator*(float s)        const { return Coord(x * s,   y * s);   }
    Coord operator/(float s)        const {
        if (fabs(s) < EPS) return Coord();
        return Coord(x / s, y / s);
    }

    Coord& operator+=(const Coord& o) { x += o.x; y += o.y; return *this; }
    Coord& operator-=(const Coord& o) { x -= o.x; y -= o.y; return *this; }

    bool operator==(const Coord& o) const {
        return fabs(x - o.x) < EPS && fabs(y - o.y) < EPS;
    }
    bool operator!=(const Coord& o) const { return !(*this == o); }
};

inline float length(const Coord& c)                     { return hypot(c.x, c.y); }
inline Coord normalize(const Coord& c) {
    float L = length(c);
    if (L < EPS) return Coord();
    return c / L;
}
inline float distance(const Coord& a, const Coord& b)   { return length(b - a); }

inline float normalizeAngle(float a) {
    a = fmod(a + (float)M_PI, 2.f * (float)M_PI);
    if (a < 0.f) a += 2.f * (float)M_PI;
    return a - (float)M_PI;
}

// ------------------------------------------------------------
// AmmoParams — параметри обраного боєприпасу.
// ------------------------------------------------------------
struct AmmoParams {
    char  name[32];
    float mass;
    float drag;
    float lift;

    AmmoParams() : mass(0.f), drag(0.f), lift(0.f) { name[0] = '\0'; }
};

// ------------------------------------------------------------
// Target — одна ціль "як її бачить MissionProcessor".
// Містить позицію в момент t=0 та швидкість, обчислену кінцевими
// різницями. Цього достатньо для прогнозу позиції за час падіння
// бомби (tgt_pred = pos + vel * t_fall).
// ------------------------------------------------------------
struct Target {
    Coord pos;       // початкова позиція цілі
    Coord velocity;  // швидкість цілі (м/с)

    Target() {}
    Target(const Coord& p, const Coord& v) : pos(p), velocity(v) {}
};

// ------------------------------------------------------------
// DropPoint — результат роботи MissionProcessor::step().
// Це не просто координата: містить також метадані, потрібні
// для подальших етапів (індекс цілі, час підльоту, прогноз,
// маркер валідності).
// ------------------------------------------------------------
struct DropPoint {
    int   targetIndex;     // індекс цілі, для якої обчислено
    Coord firePoint;       // власне точка скиду (де "натиснути кнопку")
    Coord predictedTarget; // прогнозоване положення цілі в момент падіння
    float totalTime;       // повний час: підліт дрона + падіння бомби
    bool  valid;           // false, якщо балістика не розв'язана

    DropPoint() : targetIndex(-1), totalTime(0.f), valid(false) {}
};

// ------------------------------------------------------------
// MissionConfig — конфіг місії, який повертає IConfigLoader.
// Це повний "стартовий пакет": куди стартує дрон, на якій висоті,
// з якою швидкістю, який крок симуляції тощо.
// ------------------------------------------------------------
struct MissionConfig {
    Coord startPos;         // початкова позиція дрона (x, y)
    float altitude;         // висота
    float initialDir;       // початковий напрямок (рад)
    float attackSpeed;      // швидкість атаки (м/с)
    float accelPath;        // шлях розгону (м)
    char  ammoName[32];     // ім'я обраного боєприпасу (для пошуку в ammo.json)
    float arrayTimeStep;    // крок часу між кадрами масиву цілей
    float simTimeStep;      // крок симуляції
    float hitRadius;        // радіус влучення
    float angularSpeed;     // кутова швидкість (рад/с)
    float turnThreshold;    // поріг повороту (рад)
    int   maxSteps;         // ліміт кроків симуляції

    MissionConfig()
        : altitude(0.f), initialDir(0.f), attackSpeed(0.f),
          accelPath(0.f), arrayTimeStep(0.f), simTimeStep(0.f),
          hitRadius(0.f), angularSpeed(0.f), turnThreshold(0.f),
          maxSteps(10000)
    {
        ammoName[0] = '\0';
    }
};
