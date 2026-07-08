// ============================================================
// TableSolver.cpp — реалізація табличного солвера.
// ============================================================

#include "solvers/TableSolver.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace {

// Лінійна інтерполяція для Result (обидва поля паралельно)
BallisticTable::Result lerp(const BallisticTable::Result& a,
                            const BallisticTable::Result& b, float t)
{
    return { a.t     + (b.t     - a.t)     * t,
             a.hDist + (b.hDist - a.hDist) * t };
}

// Індекс нижнього вузла + коефіцієнт [0..1] для одного виміру.
// За межами таблиці — clamp до крайнього значення.
struct Interp { int lo; float frac; };

Interp findInterp(float val, const std::vector<float>& axis)
{
    if (axis.size() == 1) return {0, 0.0f};
    if (val <= axis.front()) return {0, 0.0f};
    if (val >= axis.back())  return {(int)axis.size() - 2, 1.0f};

    auto it = std::lower_bound(axis.begin(), axis.end(), val);
    int  i  = (int)(it - axis.begin()) - 1;
    if (i < 0) i = 0;
    float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
    return {i, frac};
}

} // namespace

// ------------------------------------------------------------
// BallisticTable::load
// Формат файлу:
//   nZ nV nM nD nL
//   <nZ значень осі Z0> <nV значень осі V0> ... <nL значень осі L>
//   далі nZ*nV*nM*nD*nL пар "t hDist"
//   Порядок: Z0 → V0 → m → d → l (зовнішній → внутрішній)
// ------------------------------------------------------------
bool BallisticTable::load(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "BallisticTable: не вдалось відкрити " << path << "\n";
        return false;
    }

    int nZ = 0, nV = 0, nM = 0, nD = 0, nL = 0;
    if (!(f >> nZ >> nV >> nM >> nD >> nL)) return false;
    if (nZ <= 0 || nV <= 0 || nM <= 0 || nD <= 0 || nL <= 0) return false;

    axisZ0.resize(nZ); for (auto& v : axisZ0) f >> v;
    axisV0.resize(nV); for (auto& v : axisV0) f >> v;
    axisM.resize(nM);  for (auto& v : axisM)  f >> v;
    axisD.resize(nD);  for (auto& v : axisD)  f >> v;
    axisL.resize(nL);  for (auto& v : axisL)  f >> v;

    size_t total = (size_t)nZ * nV * nM * nD * nL;
    data.resize(total);
    for (size_t i = 0; i < total; ++i)
        f >> data[i].t >> data[i].hDist;

    return f.good() || f.eof();
}

// ------------------------------------------------------------
// BallisticTable::lookup — 5D лінійна інтерполяція.
// Згортаємо гіперкуб 2^5: 32 → 16 → 8 → 4 → 2 → 1.
// ------------------------------------------------------------
BallisticTable::Result BallisticTable::lookup(
        float Z0, float V0, float m, float d, float l) const
{
    Interp iz = findInterp(Z0, axisZ0);
    Interp iv = findInterp(V0, axisV0);
    Interp im = findInterp(m,  axisM);
    Interp id = findInterp(d,  axisD);
    Interp il = findInterp(l,  axisL);

    // l: 32 → 16
    Result v[16];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       for (int e = 0; e < 2; ++e) {
           const Result& lo = at(iz.lo+a, iv.lo+b, im.lo+c, id.lo+e, il.lo);
           const Result& hi = at(iz.lo+a, iv.lo+b, im.lo+c, id.lo+e, il.lo+1);
           v[a*8 + b*4 + c*2 + e] = lerp(lo, hi, il.frac);
       }

    // d: 16 → 8
    Result w[8];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
          w[a*4 + b*2 + c] = lerp(v[a*8 + b*4 + c*2],
                                  v[a*8 + b*4 + c*2 + 1], id.frac);

    // m: 8 → 4
    Result u[4];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
         u[a*2 + b] = lerp(w[a*4 + b*2], w[a*4 + b*2 + 1], im.frac);

    // V0: 4 → 2
    Result s[2];
    for (int a = 0; a < 2; ++a)
        s[a] = lerp(u[a*2], u[a*2 + 1], iv.frac);

    // Z0: 2 → 1
    return lerp(s[0], s[1], iz.frac);
}

// ------------------------------------------------------------
// TableSolver
// ------------------------------------------------------------
TableSolver::TableSolver(const std::string& tablePath)
{
    loaded_ = table_.load(tablePath);
    if (!loaded_)
        std::cerr << "TableSolver: таблиця '" << tablePath
                  << "' не завантажена — solve() повертатиме invalid\n";
}

DropPoint TableSolver::solve(const Coord&      dronePos,
                             const Target&     target,
                             float             altitude,
                             const AmmoParams& ammo,
                             float             attackSpeed)
{
    DropPoint out;
    if (!loaded_ || table_.empty()) return out;

    // 1. Знаходимо результат у таблиці (з інтерполяцією)
    BallisticTable::Result r =
        table_.lookup(altitude, attackSpeed, ammo.mass, ammo.drag, ammo.lift);

    float t_fall = r.t;
    float h      = r.hDist;
    if (t_fall <= 0.f || h <= 0.f) return out;

    // 2. Та сама геометрія, що й в аналітичному солвері:
    //    лінійний прогноз цілі + відступ на h уздовж напрямку.
    Coord predicted = target.pos + target.velocity * t_fall;
    Coord delta     = predicted - dronePos;
    float D         = length(delta);
    if (D < EPS) return out;

    Coord fire = (D >= h) ? predicted - normalize(delta) * h : dronePos;

    out.firePoint       = fire;
    out.predictedTarget = predicted;
    out.fallTime        = t_fall;
    out.valid           = true;
    return out;
}
