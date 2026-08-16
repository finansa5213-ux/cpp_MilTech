#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

namespace {

const char* to_string(C2State s) {
    switch (s) {
        case C2State::DISARMED:     return "DISARMED";
        case C2State::ARMED_HOLD:   return "ARMED_HOLD";
        case C2State::ARMED_GUIDED: return "ARMED_GUIDED";
        case C2State::ARMED_MANUAL: return "ARMED_MANUAL";
    }
    return "UNKNOWN";
}

}  // namespace

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;

    FcLink        fc;         // MAVLink-зв'язок з FC (блокується до HEARTBEAT)
    UdpSocket     stub_sock;  // JSON-точки маршруту вiд auto_stub
    std::ofstream log;        // /var/log/c2/c2.log
    bool          healthy_reported = false;

    explicit Impl(uint16_t fc_port)
        : fc(fc_port),
          stub_sock(STUB_PORT)
    {
        log.open("/var/log/c2/c2.log", std::ios::app);
        if (!log.is_open())
            std::cerr << "[C2] warn: cannot open /var/log/c2/c2.log\n";
    }

    // Записати рядок у stdout та лог-файл.
    void log_line(const std::string& line) {
        std::cout << line << std::endl;
        if (log.is_open())
            log << line << std::endl;
    }

    // Логування реальної змiни стану та оновлення state.
    void transition(C2State next) {
        if (next == state)
            return;
        std::ostringstream oss;
        oss << "[C2] state: " << to_string(state) << " -> " << to_string(next);
        log_line(oss.str());
        state = next;
    }

    // Визначити наступний стан C2 за armed-станом i режимом руху FC.
    void update_state() {
        if (!fc.is_armed()) {
            transition(C2State::DISARMED);
            return;
        }
        switch (fc.flight_mode()) {
            case FcLink::FlightMode::Guided:
                transition(C2State::ARMED_GUIDED);
                break;
            case FcLink::FlightMode::Hold:
                transition(C2State::ARMED_HOLD);
                break;
            case FcLink::FlightMode::Manual:
                transition(C2State::ARMED_MANUAL);
                break;
            case FcLink::FlightMode::Unknown:
                // Режим ще не отримано: стан не змiнюємо.
                break;
        }
    }

    // Прочитати всi точки маршруту вiд auto_stub, що накопичились,
    // i передати або заблокувати їх залежно вiд поточного стану.
    void handle_waypoints() {
        char buf[2048];
        ssize_t n;
        while ((n = stub_sock.recv(buf, sizeof(buf) - 1)) > 0) {
            buf[n] = '\0';
            try {
                const auto  j     = nlohmann::json::parse(buf);
                const float north = j.at("north_m").get<float>();
                const float east  = j.at("east_m").get<float>();

                if (state == C2State::ARMED_GUIDED) {
                    fc.go_to_ned(north, east);
                    std::ostringstream oss;
                    oss << "[C2] fwd: north=" << north << " east=" << east;
                    log_line(oss.str());
                } else {
                    std::ostringstream oss;
                    oss << "[C2] blocked: waypoint in " << to_string(state);
                    log_line(oss.str());
                }
            } catch (const nlohmann::json::exception& e) {
                log_line(std::string("[C2] error: bad waypoint json: ") + e.what());
            }
        }
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    // Healthcheck: створити /tmp/c2_healthy пiсля першого HEARTBEAT вiд FC.
    if (!impl_->healthy_reported && impl_->fc.is_connected()) {
        std::ofstream("/tmp/c2_healthy").close();
        impl_->healthy_reported = true;
    }

    const C2State prev = impl_->state;
    impl_->update_state();

    // Один раз на вхiд у ARMED_HOLD надiслати hold().
    if (impl_->state == C2State::ARMED_HOLD && prev != C2State::ARMED_HOLD)
        impl_->fc.hold();

    impl_->handle_waypoints();
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
