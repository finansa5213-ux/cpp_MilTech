#pragma once

/// @file geo.hpp
/// Перетворення локальних координат симулятора (метри від старту) у геодезичні
/// координати (градуси), яких очікує MAVLink, та робота з курсом.

namespace mavtel {

/// Число pi (не покладаємось на M_PI: він не гарантований стандартом C++).
inline constexpr double kPi = 3.14159265358979323846;

/// Опорна точка старту (задана в умові ДЗ).
inline constexpr double kLat0Deg = 50.4501;
inline constexpr double kLon0Deg = 30.5234;

/// Довжина одного градуса широти в метрах (задана в умові ДЗ).
inline constexpr double kMetersPerDegLat = 111320.0;

struct GeoPoint {
    double lat_deg{};
    double lon_deg{};
};

/// Локальні координати -> градуси.
/// @param x_east_m  зміщення на схід від старту, метри
/// @param y_north_m зміщення на північ від старту, метри
GeoPoint localToGeo(double x_east_m, double y_north_m);

/// Нормалізація кута до діапазону [0, 360).
double wrapDeg360(double deg);

/// Нормалізація кута до діапазону (-180, 180].
double wrapDeg180(double deg);

/// Компасний курс (0 = північ, за годинниковою стрілкою) з вектора швидкості.
/// Якщо швидкість близька до нуля — повертає @p fallback_deg.
double headingFromVelocityDeg(double vx_east_mps, double vy_north_mps, double fallback_deg = 0.0);

/// Компасний курс у градусах -> yaw у радіанах для ATTITUDE, діапазон (-pi, pi].
double headingDegToYawRad(double heading_deg);

} // namespace mavtel
