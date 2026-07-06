#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

static constexpr uint16_t STUB_PORT = 14560;

struct C2Controller::Impl {
    C2State state = C2State::DISARMED;

    // TODO: додати FcLink, UdpSocket, лог-файл та прапорцi стану.
    // FcLink потребує fc_port у конструкторi Impl.
    // UdpSocket має слухати STUB_PORT.

    void transition(C2State next) {
        // TODO: якщо next != state, записати "PREV -> NEW" у stdout i лог,
        // потiм оновити state. Якщо стан не змiнився, нiчого не писати.
        (void)next;
    }
};

C2Controller::C2Controller(uint16_t fc_port)
    : impl_(std::make_unique<Impl>())
{
    // TODO: передати fc_port в Impl та вiдкрити /var/log/c2/c2.log.
    (void)fc_port;
}

C2Controller::~C2Controller() = default;

void C2Controller::tick() {
    // TODO: healthcheck, оновлення C2State, читання точки маршруту,
    // передавання або блокування команди згiдно з поточним станом.
}

C2State C2Controller::current_state() const {
    return impl_->state;
}
