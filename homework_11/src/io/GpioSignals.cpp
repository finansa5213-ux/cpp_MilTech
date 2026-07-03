// ============================================================
// GpioSignals.cpp — керування лініями START/DROP через libgpiod.
// ============================================================
#include "io/GpioSignals.h"

#include <gpiod.h>
#include <unistd.h>
#include <cstdio>

GpioSignals::~GpioSignals() { shutdown(); }

bool GpioSignals::init(const std::string& chipName,
                       unsigned startLine, unsigned dropLine)
{
    chip_ = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip_) {
        std::fprintf(stderr, "[gpio] ne vidkryv chip '%s' (perevirte gpiodetect)\n",
                     chipName.c_str());
        return false;
    }

    start_ = gpiod_chip_get_line(chip_, startLine);
    drop_  = gpiod_chip_get_line(chip_, dropLine);
    if (!start_ || !drop_) {
        std::fprintf(stderr, "[gpio] nema liniyi %u abo %u na %s\n",
                     startLine, dropLine, chipName.c_str());
        shutdown();
        return false;
    }

    if (gpiod_line_request_output(start_, "drone", 0) != 0 ||
        gpiod_line_request_output(drop_,  "drone", 0) != 0) {
        std::perror("[gpio] request_output");
        shutdown();
        return false;
    }

    std::printf("[gpio] %s: START=line%u DROP=line%u (vyhody)\n",
                chipName.c_str(), startLine, dropLine);
    return true;
}

void GpioSignals::assertStart()
{
    if (start_) {
        gpiod_line_set_value(start_, 1); // «готовий» -> чекер починає симуляцію
        std::printf("[gpio] START=1\n");
    }
}

void GpioSignals::pulseDrop(unsigned ms)
{
    if (!drop_) return;
    gpiod_line_set_value(drop_, 1);      // імпульс скиду
    usleep(ms * 1000u);
    gpiod_line_set_value(drop_, 0);
    std::printf("[gpio] DROP impuls %u ms\n", ms);
}

void GpioSignals::shutdown()
{
    if (start_) { gpiod_line_set_value(start_, 0); gpiod_line_release(start_); start_ = nullptr; }
    if (drop_)  { gpiod_line_set_value(drop_, 0);  gpiod_line_release(drop_);  drop_  = nullptr; }
    if (chip_)  { gpiod_chip_close(chip_); chip_ = nullptr; }
}
