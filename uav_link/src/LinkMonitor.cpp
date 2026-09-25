#include "uav/LinkMonitor.hpp"

#include <algorithm>
#include <limits>

namespace uav {
namespace {

/// Медіана з перших n елементів кільцевого буфера.
///
/// Умова n <= Cap - інваріант самого буфера, але перевірка стоїть явно:
/// без неї GCC 14 не може довести межу після вбудовування std::sort і
/// попереджає про вихід за масив. Мовчазно глушити таке попередження
/// прагмою не можна - або доводимо межу компіляторові, або її справді немає.
template <std::size_t Cap>
std::optional<double> medianOf(const std::array<double, Cap>& src, std::size_t n) {
    if (n == 0 || n > Cap) return std::nullopt;

    // Незаповнені комірки - нескінченність, і сортується ЗАВЖДИ весь масив.
    // Діапазон сортування сталий на етапі компіляції, тому питання про вихід
    // за межі не виникає взагалі, а нескінченності після сортування лягають
    // у хвіст і на медіану перших n не впливають.
    std::array<double, Cap> tmp;
    tmp.fill(std::numeric_limits<double>::infinity());
    for (std::size_t i = 0; i < Cap; ++i)
        if (i < n) tmp[i] = src[i];

    // Сортування вставками по масиву сталого розміру. std::sort тут зайвий:
    // для п'яти-десяти чисел introsort програє за швидкістю, а його
    // вбудовування збивало аналізатор меж GCC 14 - попередження про вихід
    // за масив надходило саме з бібліотечного коду, не з нашого. Тут усі
    // індекси обмежені Cap, константою етапу компіляції, і довести це
    // компіляторові нема чого: межа видима безпосередньо.
    for (std::size_t i = 1; i < Cap; ++i) {
        const double v = tmp[i];
        std::size_t  j = i;
        while (j > 0 && tmp[j - 1] > v) { tmp[j] = tmp[j - 1]; --j; }
        tmp[j] = v;
    }
    return tmp[n / 2];
}

} // namespace

LinkMonitor::LinkMonitor(const char* name, double tLostSec, double rttStaleSec)
    : name_(name), tLostSec_(tLostSec), rttStaleSec_(rttStaleSec) {}

void LinkMonitor::onGroundFrame(TimePoint now, std::size_t bytes) {
    heard_      = true;
    lastGround_ = now;
    rx_        += bytes;
    ++frames_;
}

void LinkMonitor::onRadioStatus(const RadioStatus& rs, TimePoint now) {
    radio_     = rs;
    lastRadio_ = now;
}

void LinkMonitor::onRttSample(double ms, TimePoint now) {
    rtt_[rttHead_] = ms;
    rttHead_       = (rttHead_ + 1) % kRttSamples;
    if (rttCount_ < kRttSamples) ++rttCount_;
    lastRtt_ = now;
}

bool LinkMonitor::alive(TimePoint now) const {
    return heard_ && secBetween(lastGround_, now) < tLostSec_;
}

std::optional<double> LinkMonitor::rttMedianMs(TimePoint now) const {
    if (secBetween(lastRtt_, now) > rttStaleSec_) return std::nullopt;
    return medianOf(rtt_, rttCount_);
}

void LinkMonitor::onFullPathSample(double ms, TimePoint now) {
    full_[rttFullHead_] = ms;
    rttFullHead_        = (rttFullHead_ + 1) % kFullSamples;
    if (rttFullCount_ < kFullSamples) ++rttFullCount_;
    lastFull_ = now;
}

bool LinkMonitor::fullPathMeasured(TimePoint now) const {
    return rttFullCount_ != 0 && secBetween(lastFull_, now) <= rttStaleSec_ * 2.0;
}

std::optional<double> LinkMonitor::fullPathMs(TimePoint now, double offsetMs) const {
    if (fullPathMeasured(now)) {
        if (const auto m = medianOf(full_, rttFullCount_)) return m;
    }
    const auto tunnel = rttMedianMs(now);
    if (!tunnel) return std::nullopt;
    return *tunnel + offsetMs;
}

bool LinkMonitor::radioFresh(TimePoint now) const {
    return lastRadio_ != TimePoint{} && secBetween(lastRadio_, now) < rttStaleSec_;
}

} // namespace uav
