#include "geo.hpp"

#include <cmath>

namespace mavtel {

GeoPoint localToGeo(double x_east_m, double y_north_m)
{
  const double lat = kLat0Deg + y_north_m / kMetersPerDegLat;
  const double meters_per_deg_lon = kMetersPerDegLat * std::cos(kLat0Deg * kPi / 180.0);
  const double lon = kLon0Deg + x_east_m / meters_per_deg_lon;
  return GeoPoint{lat, lon};
}

double wrapDeg360(double deg)
{
  double result = std::fmod(deg, 360.0);
  if (result < 0.0) {
    result += 360.0;
  }
  return result;
}

double wrapDeg180(double deg)
{
  double result = wrapDeg360(deg);
  if (result > 180.0) {
    result -= 360.0;
  }
  return result;
}

double headingFromVelocityDeg(double vx_east_mps, double vy_north_mps, double fallback_deg)
{
  constexpr double kMinSpeed = 1e-6;
  if (std::hypot(vx_east_mps, vy_north_mps) < kMinSpeed) {
    return wrapDeg360(fallback_deg);
  }
  // atan2(east, north) дає кут від півночі за годинниковою стрілкою.
  return wrapDeg360(std::atan2(vx_east_mps, vy_north_mps) * 180.0 / kPi);
}

double headingDegToYawRad(double heading_deg)
{
  return wrapDeg180(heading_deg) * kPi / 180.0;
}

}  // namespace mavtel
