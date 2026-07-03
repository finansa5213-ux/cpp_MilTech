// ============================================================
// GpioSignals.cpp — керування лініями START/DROP через libgpiod.
// Дві реалізації під одним інтерфейсом: API v1 (умова ДЗ, плата)
// і API v2 (нові дистрибутиви). Вибір — у CMake (HW11_GPIOD_V2).
// ============================================================
#include "io/GpioSignals.h"

#include <gpiod.h>
#include <unistd.h>
#include <cstdio>

#ifndef HW11_GPIOD_V2
// ============================================================
// libgpiod API v1 — Raspberry Pi OS / Ubuntu 22.04
// ============================================================

GpioSignals::~GpioSignals() { shutdown(); }

bool GpioSignals::init(const std::string& chipName,
                       unsigned startLine, unsigned dropLine)
{
    startLine_ = startLine;
    dropLine_  = dropLine;

    gpiod_chip* chip = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip) {
        std::fprintf(stderr, "[gpio] ne vidkryv chip '%s' (perevirte gpiodetect)\n",
                     chipName.c_str());
        return false;
    }
    chip_ = chip;

    gpiod_line* s = gpiod_chip_get_line(chip, startLine);
    gpiod_line* d = gpiod_chip_get_line(chip, dropLine);
    if (!s || !d) {
        std::fprintf(stderr, "[gpio] nema liniyi %u abo %u na %s\n",
                     startLine, dropLine, chipName.c_str());
        shutdown();
        return false;
    }
    start_ = s;
    drop_  = d;

    if (gpiod_line_request_output(s, "drone", 0) != 0 ||
        gpiod_line_request_output(d, "drone", 0) != 0) {
        std::perror("[gpio] request_output");
        shutdown();
        return false;
    }

    std::printf("[gpio] %s: START=line%u DROP=line%u (vyhody, API v1)\n",
                chipName.c_str(), startLine, dropLine);
    return true;
}

void GpioSignals::assertStart()
{
    if (start_) {
        gpiod_line_set_value((gpiod_line*)start_, 1); // «готовий»
        std::printf("[gpio] START=1\n");
    }
}

void GpioSignals::pulseDrop(unsigned ms)
{
    if (!drop_) return;
    gpiod_line_set_value((gpiod_line*)drop_, 1);      // імпульс скиду
    usleep(ms * 1000u);
    gpiod_line_set_value((gpiod_line*)drop_, 0);
    std::printf("[gpio] DROP impuls %u ms\n", ms);
}

void GpioSignals::shutdown()
{
    if (start_) {
        gpiod_line_set_value((gpiod_line*)start_, 0);
        gpiod_line_release((gpiod_line*)start_);
        start_ = nullptr;
    }
    if (drop_) {
        gpiod_line_set_value((gpiod_line*)drop_, 0);
        gpiod_line_release((gpiod_line*)drop_);
        drop_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close((gpiod_chip*)chip_);
        chip_ = nullptr;
    }
}

#else  // HW11_GPIOD_V2
// ============================================================
// libgpiod API v2 — Ubuntu 24.04+ (WSL тощо)
// ============================================================

namespace {
gpiod_line_request* requestOutput(gpiod_chip* chip, unsigned offset)
{
    gpiod_line_settings* ls = gpiod_line_settings_new();
    if (!ls) return nullptr;
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(ls, GPIOD_LINE_VALUE_INACTIVE);

    gpiod_line_config*    lc = gpiod_line_config_new();
    gpiod_request_config* rc = gpiod_request_config_new();
    gpiod_line_request*  req = nullptr;
    if (lc && rc &&
        gpiod_line_config_add_line_settings(lc, &offset, 1, ls) == 0) {
        gpiod_request_config_set_consumer(rc, "drone");
        req = gpiod_chip_request_lines(chip, rc, lc);
    }
    if (rc) gpiod_request_config_free(rc);
    if (lc) gpiod_line_config_free(lc);
    gpiod_line_settings_free(ls);
    return req;
}

void setLine(void* req, unsigned offset, int value)
{
    if (req)
        gpiod_line_request_set_value((gpiod_line_request*)req, offset,
                                     value ? GPIOD_LINE_VALUE_ACTIVE
                                           : GPIOD_LINE_VALUE_INACTIVE);
}
} // namespace

GpioSignals::~GpioSignals() { shutdown(); }

bool GpioSignals::init(const std::string& chipName,
                       unsigned startLine, unsigned dropLine)
{
    startLine_ = startLine;
    dropLine_  = dropLine;

    const std::string path = "/dev/" + chipName;
    gpiod_chip* chip = gpiod_chip_open(path.c_str());
    if (!chip) {
        std::fprintf(stderr, "[gpio] ne vidkryv chip '%s' (perevirte gpiodetect)\n",
                     path.c_str());
        return false;
    }
    chip_ = chip;

    start_ = requestOutput(chip, startLine);
    drop_  = requestOutput(chip, dropLine);
    if (!start_ || !drop_) {
        std::fprintf(stderr, "[gpio] ne zarezervuvav liniyi %u/%u na %s\n",
                     startLine, dropLine, chipName.c_str());
        shutdown();
        return false;
    }

    std::printf("[gpio] %s: START=line%u DROP=line%u (vyhody, API v2)\n",
                chipName.c_str(), startLine, dropLine);
    return true;
}

void GpioSignals::assertStart()
{
    if (start_) {
        setLine(start_, startLine_, 1); // «готовий»
        std::printf("[gpio] START=1\n");
    }
}

void GpioSignals::pulseDrop(unsigned ms)
{
    if (!drop_) return;
    setLine(drop_, dropLine_, 1);       // імпульс скиду
    usleep(ms * 1000u);
    setLine(drop_, dropLine_, 0);
    std::printf("[gpio] DROP impuls %u ms\n", ms);
}

void GpioSignals::shutdown()
{
    if (start_) {
        setLine(start_, startLine_, 0);
        gpiod_line_request_release((gpiod_line_request*)start_);
        start_ = nullptr;
    }
    if (drop_) {
        setLine(drop_, dropLine_, 0);
        gpiod_line_request_release((gpiod_line_request*)drop_);
        drop_ = nullptr;
    }
    if (chip_) {
        gpiod_chip_close((gpiod_chip*)chip_);
        chip_ = nullptr;
    }
}

#endif // HW11_GPIOD_V2
