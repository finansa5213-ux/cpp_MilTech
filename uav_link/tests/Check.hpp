// Мінімальний набір перевірок. Зовнішній фреймворк тут зайвий: у польотному
// образі його немає, а для десятка тверджень достатньо тридцяти рядків.
#pragma once

#include <cmath>
#include <cstdio>
#include <type_traits>

namespace uav::test {

inline int  gFailures = 0;
inline int  gChecks   = 0;

inline void report(bool ok, const char* what) {
    ++gChecks;
    if (!ok) ++gFailures;
    std::printf("  %s  %s\n", ok ? "OK " : "ПОМИЛКА", what);
}

inline int summary(const char* suite) {
    std::printf("\n%s: перевірок %d, помилок %d -> %s\n\n",
                suite, gChecks, gFailures, gFailures == 0 ? "ПРОЙДЕНО" : "Є ПОМИЛКИ");
    return gFailures == 0 ? 0 : 1;
}

} // namespace uav::test

#define CHECK(cond, what) ::uav::test::report((cond), (what))

#define CHECK_EQ(a, b, what)                                                        \
    do {                                                                            \
        const auto _a = (a);                                                        \
        const auto _b = (b);                                                        \
        const bool _ok = (_a == static_cast<std::remove_cv_t<decltype(_a)>>(_b));                     \
        ::uav::test::report(_ok, (what));                                           \
        if (!_ok) std::printf("        очікувано %lld, отримано %lld\n",            \
                              static_cast<long long>(_b), static_cast<long long>(_a)); \
    } while (0)

#define CHECK_NEAR(a, b, tol, what)                                                 \
    do {                                                                            \
        const double _a = (a), _b = (b);                                            \
        const bool _ok = std::fabs(_a - _b) <= (tol);                               \
        ::uav::test::report(_ok, (what));                                           \
        if (!_ok) std::printf("        очікувано %.3f +- %.3f, отримано %.3f\n",    \
                              _b, (tol), _a);                                       \
    } while (0)
