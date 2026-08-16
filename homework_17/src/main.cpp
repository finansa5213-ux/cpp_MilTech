// ============================================================
// main.cpp — ДЗ17: автопілот ДЗ11 + телеметрія MAVLink 2 по UDP.
//
// Той самий ланцюжок, що й у ДЗ11 (UART від чекера ДЗ11 + GPIO),
// але паралельно потік MAVLink іде на 127.0.0.1:14550 — у QGroundControl
// або в чекер ДЗ17.
//
//   sim: ./drone --uart /tmp/ttyA --gpiochip gpiochipN
//                --start-line 24 --drop-line 23 --dest 127.0.0.1:14550
// ============================================================
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include "LinkState.h"
#include "MavMissionProcessor.h"
#include "dz11_bridge.hpp"
#include "io/GpioSignals.h"
#include "io/UartLink.h"
#include "mavlink_link.hpp"
#include "solvers/AnalyticalSolver.h"
#include "udp_socket.hpp"

namespace {
LinkState* g_state = nullptr;

void onSignal(int) {
    if (g_state) {
        g_state->stop = true;
        g_state->cv.notify_all();
    }
}

void usage(const char* prog) {
    std::printf(
        "Vykorystannya: %s [--uart <dev>] [--gpiochip <n>] "
        "[--start-line <n>] [--drop-line <n>] [--dest <host:port>]\n"
        "  --uart <dev>       port UART (def: /tmp/ttyA; na plati /dev/ttyAMA1)\n"
        "  --gpiochip <n>     chip GPIO (def: gpiochip1; u sim — im'ya vid chekera)\n"
        "  --start-line <n>   liniya START (def: 24)\n"
        "  --drop-line <n>    liniya DROP  (def: 23)\n"
        "  --dest <host:port> kudy slaty MAVLink (def: 127.0.0.1:14550)\n",
        prog);
}
} // namespace

int main(int argc, char** argv) {
    std::string uartDev = "/tmp/ttyA";
    std::string chipName = "gpiochip1";
    unsigned startLine = 24;
    unsigned dropLine = 23;
    mavtel::Endpoint dest{};

    for (int i = 1; i < argc; ++i) {
        auto need = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "brakuye znachennya dlya %s\n", opt);
                std::exit(2);
            }
            return argv[++i];
        };
        if (!std::strcmp(argv[i], "--uart"))
            uartDev = need("--uart");
        else if (!std::strcmp(argv[i], "--gpiochip"))
            chipName = need("--gpiochip");
        else if (!std::strcmp(argv[i], "--start-line"))
            startLine = static_cast<unsigned>(std::stoul(need("--start-line")));
        else if (!std::strcmp(argv[i], "--drop-line"))
            dropLine = static_cast<unsigned>(std::stoul(need("--drop-line")));
        else if (!std::strcmp(argv[i], "--dest")) {
            try {
                dest = mavtel::parseEndpoint(need("--dest"));
            } catch (const std::exception& error) {
                std::fprintf(stderr, "--dest: %s\n", error.what());
                return 2;
            }
        } else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "nevidomyi argument: %s\n", argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    // Порядок як у ДЗ11: спершу UART і приймач, потім START — щоб не втратити
    // AMMO, який чекер шле одразу.
    UartLink uart;
    if (!uart.open(uartDev)) return 1;

    GpioSignals gpio;
    if (!gpio.init(chipName, startLine, dropLine)) return 1;

    LinkState state;
    g_state = &state;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    try {
        mavtel::UdpSocket socket(dest.host, dest.port);
        mavtel::MavlinkLink link(socket);
        mavtel::Dz11Bridge bridge(link);

        std::printf("[mavlink] telemetriya -> %s:%u\n", dest.host.c_str(), dest.port);

        MavMissionProcessor mission(uart, gpio, state,
                                    std::make_unique<AnalyticalSolver>(), bridge);

        std::thread rx(&MavMissionProcessor::runRx, &mission);
        mission.runMission();

        state.stop = true;
        state.cv.notify_all();
        rx.join();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "pomylka MAVLink: %s\n", error.what());
        gpio.shutdown();
        uart.close();
        return 1;
    }

    gpio.shutdown();
    uart.close();
    return 0;
}
