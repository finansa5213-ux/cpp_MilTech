#include "dz11_bridge.hpp"

#include <chrono>

#include "geo.hpp"

namespace mavtel {

std::uint32_t nowMs() {
    using Clock = std::chrono::steady_clock;
    static const Clock::time_point started = Clock::now();
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started);
    return static_cast<std::uint32_t>(elapsed.count());
}

TelemetrySample toSample(const dlink::Telemetry& telemetry) {
    TelemetrySample sample{};
    sample.time_boot_ms = telemetry.t_ms;
    sample.x_east_m = telemetry.x;
    sample.y_north_m = telemetry.y;
    sample.alt_m = telemetry.z;
    sample.vx_east_mps = telemetry.vx;
    sample.vy_north_mps = telemetry.vy;
    sample.vz_up_mps = 0.0; // ДЗ11 літає на сталій висоті

    // Математичний кут dir -> компасний курс; використовується лише як запасний
    // варіант, коли швидкість близька до нуля і напрямок руху не визначений.
    const double heading_from_dir = wrapDeg360(90.0 - telemetry.dir * 180.0 / kPi);
    sample.heading_deg =
        headingFromVelocityDeg(telemetry.vx, telemetry.vy, heading_from_dir);

    sample.roll_rad = 0.0;
    sample.pitch_rad = 0.0;
    return sample;
}

Dz11Bridge::Dz11Bridge(MavlinkLink& link) : link_(link) {}

void Dz11Bridge::sendHeartbeatIfDue(std::uint32_t wall_ms) {
    if (heartbeat_sent_ && wall_ms - last_heartbeat_ms_ < kHeartbeatPeriodMs) {
        return;
    }
    link_.sendHeartbeat();
    last_heartbeat_ms_ = wall_ms;
    heartbeat_sent_ = true;
}

void Dz11Bridge::tick(std::uint32_t wall_ms) {
    sendHeartbeatIfDue(wall_ms);
    link_.poll(wall_ms);
}

void Dz11Bridge::onTelemetry(const dlink::Telemetry& telemetry, std::uint32_t wall_ms) {
    last_ = toSample(telemetry);
    has_sample_ = true;
    last_sample_ms_ = wall_ms;

    link_.sendTelemetry(last_);
    tick(wall_ms);
}

void Dz11Bridge::onDrop(const dlink::Telemetry& telemetry, std::uint32_t wall_ms) {
    const TelemetrySample sample = toSample(telemetry);
    const GeoPoint point = localToGeo(sample.x_east_m, sample.y_north_m);
    link_.requestDrop({point.lat_deg, point.lon_deg, sample.alt_m}, wall_ms);
}

void Dz11Bridge::coast(std::uint32_t wall_ms) {
    if (!has_sample_) {
        tick(wall_ms);
        return;
    }

    const std::uint32_t elapsed = wall_ms - last_sample_ms_;
    if (elapsed >= kCoastPeriodMs) {
        const double dt_s = elapsed / 1000.0;
        last_.time_boot_ms += elapsed;
        last_.x_east_m += last_.vx_east_mps * dt_s;
        last_.y_north_m += last_.vy_north_mps * dt_s;
        last_sample_ms_ = wall_ms;
        link_.sendTelemetry(last_);
    }
    tick(wall_ms);
}

} // namespace mavtel
