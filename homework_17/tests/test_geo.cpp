#include <gtest/gtest.h>

#include <cmath>

#include "geo.hpp"

namespace {

TEST(GeoTest, OriginMapsToReferencePoint)
{
  const mavtel::GeoPoint point = mavtel::localToGeo(0.0, 0.0);
  EXPECT_DOUBLE_EQ(point.lat_deg, mavtel::kLat0Deg);
  EXPECT_DOUBLE_EQ(point.lon_deg, mavtel::kLon0Deg);
}

TEST(GeoTest, OneDegreeNorthEqualsMetersPerDegree)
{
  const mavtel::GeoPoint point = mavtel::localToGeo(0.0, mavtel::kMetersPerDegLat);
  EXPECT_NEAR(point.lat_deg, mavtel::kLat0Deg + 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(point.lon_deg, mavtel::kLon0Deg);
}

TEST(GeoTest, LongitudeIsCompressedByCosineOfLatitude)
{
  // На широті 50.45 один градус довготи коротший за градус широти.
  const mavtel::GeoPoint point = mavtel::localToGeo(mavtel::kMetersPerDegLat, 0.0);
  const double expected_delta = 1.0 / std::cos(mavtel::kLat0Deg * mavtel::kPi / 180.0);
  EXPECT_NEAR(point.lon_deg - mavtel::kLon0Deg, expected_delta, 1e-9);
  EXPECT_GT(point.lon_deg - mavtel::kLon0Deg, 1.0);
}

TEST(GeoTest, HeadingIsCompassAngleFromNorth)
{
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(0.0, 10.0), 0.0, 1e-9);     // північ
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(10.0, 0.0), 90.0, 1e-9);    // схід
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(0.0, -10.0), 180.0, 1e-9);  // південь
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(-10.0, 0.0), 270.0, 1e-9);  // захід
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(10.0, 10.0), 45.0, 1e-9);   // північний схід
}

TEST(GeoTest, HeadingFallsBackWhenSpeedIsZero)
{
  EXPECT_NEAR(mavtel::headingFromVelocityDeg(0.0, 0.0, 123.0), 123.0, 1e-9);
}

TEST(GeoTest, WrapKeepsAnglesInRange)
{
  EXPECT_NEAR(mavtel::wrapDeg360(-90.0), 270.0, 1e-9);
  EXPECT_NEAR(mavtel::wrapDeg360(450.0), 90.0, 1e-9);
  EXPECT_NEAR(mavtel::wrapDeg180(350.0), -10.0, 1e-9);
  EXPECT_NEAR(mavtel::wrapDeg180(190.0), -170.0, 1e-9);
}

TEST(GeoTest, YawIsHeadingInRadiansWithinPiRange)
{
  EXPECT_NEAR(mavtel::headingDegToYawRad(0.0), 0.0, 1e-9);
  EXPECT_NEAR(mavtel::headingDegToYawRad(90.0), mavtel::kPi / 2.0, 1e-9);
  EXPECT_NEAR(mavtel::headingDegToYawRad(270.0), -mavtel::kPi / 2.0, 1e-9);
}

}  // namespace
