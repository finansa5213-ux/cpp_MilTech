// ============================================================
// main.cpp — ДЗ11: керування дроном по UART і автоматичний скид.
//
// Той самий бінарник для симуляції та плати:
//   sim:  ./homework_11 --uart /tmp/ttyA     --gpiochip gpiochipN --start-line 24 --drop-line 23
//   hw:   ./homework_11 --uart /dev/ttyAMA1  --gpiochip gpiochip0 --start-line 24 --drop-line 23
// ============================================================
#include <csignal>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "LinkState.h"
#include "MissionProcessor.h"
#include "io/GpioSignals.h"
#include "io/UartLink.h"
#include "solvers/AnalyticalSolver.h"

namespace {
LinkState* g_state = nullptr;

void onSignal(int)
{
    if (g_state) {
        g_state->stop = true;
        g_state->cv.notify_all();
    }
}

void usage(const char* prog)
{
    std::printf(
        "Vykorystannya: %s [--uart <dev>] [--gpiochip <name>] "
        "[--start-line <n>] [--drop-line <n>]\n"
        "  --uart <dev>       port UART (def: /tmp/ttyA; na plati /dev/ttyAMA1)\n"
        "  --gpiochip <name>  chip GPIO (def: gpiochip1; na plati gpiochip0;\n"
        "                     u sim — im'ya, yake drukuye cheker)\n"
        "  --start-line <n>   liniya START (def: 24)\n"
        "  --drop-line <n>    liniya DROP  (def: 23)\n",
        prog);
}
} // namespace

int main(int argc, char** argv)
{
    std::string uartDev  = "/tmp/ttyA";
    std::string chipName = "gpiochip1";
    unsigned    startLine = 24;
    unsigned    dropLine  = 23;

    for (int i = 1; i < argc; ++i) {
        auto need = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "brakuye znachennya dlya %s\n", opt);
                std::exit(2);
            }
            return argv[++i];
        };
        if      (!std::strcmp(argv[i], "--uart"))       uartDev  = need("--uart");
        else if (!std::strcmp(argv[i], "--gpiochip"))   chipName = need("--gpiochip");
        else if (!std::strcmp(argv[i], "--start-line")) startLine = (unsigned)std::stoul(need("--start-line"));
        else if (!std::strcmp(argv[i], "--drop-line"))  dropLine  = (unsigned)std::stoul(need("--drop-line"));
        else if (!std::strcmp(argv[i], "-h") || !std::strcmp(argv[i], "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "nevidomyi argument: %s\n", argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    // Порядок важливий: спершу відкриваємо UART і запускаємо приймач,
    // ПОТІМ піднімаємо START — щоб не втратити AMMO, який чекер шле одразу.
    UartLink uart;
    if (!uart.open(uartDev)) return 1;

    GpioSignals gpio;
    if (!gpio.init(chipName, startLine, dropLine)) return 1;

    LinkState state;
    g_state = &state;
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    MissionProcessor mission(uart, gpio, state,
                             std::make_unique<AnalyticalSolver>());

    std::thread rx(&MissionProcessor::runRx, &mission); // потік приймача
    mission.runMission();                               // цикл місії в main

    state.stop = true;
    state.cv.notify_all();
    rx.join();          // завершення лише через join(), без detach()

    gpio.shutdown();    // опустити START/DROP
    uart.close();
    return 0;
}
