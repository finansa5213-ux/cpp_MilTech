// fc_link.cpp: готовий допомiжний файл, змiнювати не потрiбно.
#include "fc_link.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>

struct FcLink::Impl {
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::Mavsdk::ComponentType::GroundStation}};
    std::shared_ptr<mavsdk::System>    system;
    std::unique_ptr<mavsdk::Telemetry> telemetry;
    std::unique_ptr<mavsdk::Action>    action;
    std::unique_ptr<mavsdk::Offboard>  offboard;

    std::atomic<bool>            connected{false};
    std::atomic<bool>            armed{false};
    std::atomic<FcLink::FlightMode> mode{FcLink::FlightMode::Unknown};

    std::mutex offboard_mutex;
    bool       offboard_started = false;
};

FcLink::FcLink(uint16_t listen_port, std::chrono::seconds timeout)
    : impl_(std::make_unique<Impl>())
{
    std::string conn = "udp://:" + std::to_string(listen_port);
    const auto connection_result = impl_->mavsdk.add_any_connection(conn);
    if (connection_result != mavsdk::ConnectionResult::Success) {
        throw std::runtime_error("FcLink: cannot open " + conn + ", result="
                                 + std::to_string(static_cast<int>(connection_result)));
    }

    std::promise<std::shared_ptr<mavsdk::System>> prom;
    auto fut = prom.get_future();

    impl_->mavsdk.subscribe_on_new_system([this, &prom]() {
        for (auto& s : impl_->mavsdk.systems()) {
            if (s->has_autopilot()) {
                try { prom.set_value(s); } catch (...) {}
                return;
            }
        }
    });

    if (fut.wait_for(timeout) != std::future_status::ready)
        throw std::runtime_error("FcLink: timeout: is ArduPilot sending MAVLink to this UDP port?");

    impl_->system   = fut.get();
    impl_->telemetry = std::make_unique<mavsdk::Telemetry>(impl_->system);
    impl_->action    = std::make_unique<mavsdk::Action>(impl_->system);
    impl_->offboard  = std::make_unique<mavsdk::Offboard>(impl_->system);

    impl_->telemetry->subscribe_armed([this](bool a) {
        impl_->connected = true;
        impl_->armed     = a;
        if (!a) impl_->mode = FlightMode::Unknown;
    });

    impl_->telemetry->subscribe_flight_mode([this](mavsdk::Telemetry::FlightMode m) {
        using FM = mavsdk::Telemetry::FlightMode;
        FlightMode mapped = FlightMode::Unknown;
        switch (m) {
            case FM::Manual:   mapped = FlightMode::Manual;  break;
            case FM::Hold:     mapped = FlightMode::Hold;    break;
            case FM::Offboard: mapped = FlightMode::Guided;  break;
            default: break;
        }
        impl_->mode = mapped;
    });
}

FcLink::~FcLink() = default;

bool           FcLink::is_connected() const { return impl_->connected; }
bool           FcLink::is_armed()     const { return impl_->armed; }
FcLink::FlightMode FcLink::flight_mode() const { return impl_->mode; }

void FcLink::hold() {
    impl_->action->hold();
}

void FcLink::go_to_ned(float north_m, float east_m, float down_m) {
    {
        std::lock_guard lock(impl_->offboard_mutex);
        if (!impl_->offboard_started) {
            impl_->offboard->set_position_ned({});
            impl_->offboard->start();
            impl_->offboard_started = true;
        }
    }
    mavsdk::Offboard::PositionNedYaw pos{};
    pos.north_m = north_m;
    pos.east_m  = east_m;
    pos.down_m  = down_m;
    pos.yaw_deg = 0.0f;
    impl_->offboard->set_position_ned(pos);
}
