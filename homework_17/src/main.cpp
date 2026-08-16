#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "drone_sim.hpp"
#include "geo.hpp"
#include "mavlink_link.hpp"
#include "udp_socket.hpp"

namespace {

constexpr double kPhysicsStepS = 0.02;        ///< 50 Гц фізики
constexpr double kTelemetryPeriodS = 0.1;     ///< 10 Гц (умова вимагає >= 2 Гц)
constexpr double kHeartbeatPeriodS = 1.0;     ///< 1 Гц
constexpr double kTailAfterDropS = 3.0;       ///< скільки ще шлемо телеметрію після скиду
constexpr double kMaxFlightTimeS = 300.0;     ///< запобіжник від нескінченного циклу

struct Options {
    mavtel::Endpoint dest{};
    mavtel::DroneConfig drone{};
};

void printUsage(const char* program) {
    std::cout << "Використання: " << program << " [--dest host:port] [--target X,Y]\n"
              << "              [--alt METERS] [--speed MPS]\n"
              << "За замовчуванням: --dest 127.0.0.1:14550\n";
}

bool parseTarget(const std::string& text, mavtel::DroneConfig& config) {
    const std::size_t comma = text.find(',');
    if (comma == std::string::npos) {
        return false;
    }
    config.target_x_m = std::stod(text.substr(0, comma));
    config.target_y_m = std::stod(text.substr(comma + 1));
    return true;
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = (i + 1) < argc;

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        }
        if (arg == "--dest" && has_value) {
            options.dest = mavtel::parseEndpoint(argv[++i]);
        } else if (arg == "--target" && has_value) {
            if (!parseTarget(argv[++i], options.drone)) {
                std::cerr << "Очікується --target X,Y\n";
                return false;
            }
        } else if (arg == "--alt" && has_value) {
            options.drone.cruise_alt_m = std::stod(argv[++i]);
        } else if (arg == "--speed" && has_value) {
            options.drone.cruise_speed_mps = std::stod(argv[++i]);
        } else {
            std::cerr << "Невідомий аргумент: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        if (!parseArgs(argc, argv, options)) {
            return EXIT_FAILURE;
        }
    } catch (const std::exception& error) {
        std::cerr << "Помилка розбору аргументів: " << error.what() << "\n";
        return EXIT_FAILURE;
    }

    try {
        mavtel::UdpSocket socket(options.dest.host, options.dest.port);
        mavtel::MavlinkLink link(socket);
        mavtel::DroneSim drone(options.drone);

        std::cout << "MAVLink 2 -> " << options.dest.host << ":" << options.dest.port
                  << " | ціль (" << options.drone.target_x_m << ", "
                  << options.drone.target_y_m << ") м, висота "
                  << options.drone.cruise_alt_m << " м\n";

        using Clock = std::chrono::steady_clock;
        const auto started_at = Clock::now();
        auto next_tick = started_at;

        double sim_time_s = 0.0;
        double since_telemetry_s = kTelemetryPeriodS; // щоб перший кадр пішов одразу
        double since_heartbeat_s = kHeartbeatPeriodS;
        double time_after_drop_s = 0.0;
        bool drop_reported = false;

        while (sim_time_s < kMaxFlightTimeS) {
            // --- 1. Крок фізики ---
            const bool release_now = drone.step(kPhysicsStepS);
            sim_time_s += kPhysicsStepS;
            const auto now_ms = static_cast<std::uint32_t>(sim_time_s * 1000.0);

            // --- 2. Скид: одна команда + подальші повтори живуть у link.poll() ---
            if (release_now) {
                const mavtel::GeoPoint point = mavtel::localToGeo(drone.x(), drone.y());
                link.requestDrop({point.lat_deg, point.lon_deg, drone.altitude()}, now_ms);
                std::cout << "[скид] t=" << sim_time_s << " с, lat=" << point.lat_deg
                          << ", lon=" << point.lon_deg << ", alt=" << drone.altitude()
                          << " м -> COMMAND_LONG (MAV_CMD_USER_1)\n";
            }

            // --- 3. Телеметрія ---
            since_telemetry_s += kPhysicsStepS;
            if (since_telemetry_s >= kTelemetryPeriodS) {
                since_telemetry_s = 0.0;
                link.sendTelemetry(drone.sample(now_ms));
            }

            since_heartbeat_s += kPhysicsStepS;
            if (since_heartbeat_s >= kHeartbeatPeriodS) {
                since_heartbeat_s = 0.0;
                link.sendHeartbeat();
            }

            // --- 4. Приймання ACK + повтори за таймаутом ---
            link.poll(now_ms);

            if (!drop_reported && link.dropFinished()) {
                drop_reported = true;
                if (link.dropState() == mavtel::DropState::Acked) {
                    std::cout << "[скид] ACK отримано (спроб: " << link.dropAttempts() << ")\n";
                } else {
                    std::cout << "[скид] ACK не отримано після " << link.dropAttempts()
                              << " спроб\n";
                }
            }
            if (drop_reported) {
                time_after_drop_s += kPhysicsStepS;
                if (time_after_drop_s >= kTailAfterDropS) {
                    break;
                }
            }

            // --- 5. Тримаємо реальний час: темпи HEARTBEAT/телеметрії вимірюються ним ---
            next_tick += std::chrono::microseconds(static_cast<long long>(kPhysicsStepS * 1e6));
            std::this_thread::sleep_until(next_tick);
        }

        std::cout << "Політ завершено, t=" << sim_time_s << " с\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Помилка: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
}
