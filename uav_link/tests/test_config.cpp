// Конфігурація має або прочитатися правильно, або чесно поскаржитися.
// Мовчазне проковтування помилкового порога - найгірший можливий варіант:
// програма працюватиме, але за іншим правилом, ніж написано в роботі.
#include "uav/Config.hpp"
#include "tests/Check.hpp"

#include <cstdio>
#include <fstream>

using namespace uav;

namespace {

std::string writeTemp(const char* body) {
    const std::string path = "/tmp/uav_link_test.conf";
    std::ofstream     f(path);
    f << body;
    return path;
}

} // namespace

int main() {
    {
        const auto path = writeTemp(
            "# стенд, виміри 18.09\n"
            "peer_host = 10.8.0.1\n"
            "operator_host = 10.8.0.3\n"
            "t_lost_sec = 3.0\n"
            "rtt_switch_full_ms = 200\n"
            "rtt_back_full_ms   = 170\n"
            "failsafe_action = rtl\n"
            "control_allow = 127.0.0.1, 10.8.0.3\n");
        Config      c;
        std::string err;
        CHECK(Config::load(path, c, err), "коректний файл читається");
        CHECK(c.peerHost == "10.8.0.1", "адреса концентратора");
        CHECK_NEAR(c.rttSwitchFullMs, 200.0, 1e-9, "поріг відходу");
        CHECK_NEAR(c.rttBackFullMs, 170.0, 1e-9, "поріг повернення");
        CHECK(c.commandRtl(), "failsafe_action = rtl вмикає рівень 3");
        CHECK_EQ(c.controlAllow.size(), 2u, "список дозволених адрес розібрано");
        CHECK(c.controlAllow[1] == "10.8.0.3", "пробіли обрізано");
    }
    {
        const auto  path = writeTemp("rtt_switch_full_ms = 150\nrtt_back_full_ms = 170\n");
        Config      c;
        std::string err;
        CHECK(!Config::load(path, c, err),
              "поріг повернення вище за поріг відходу - відмова");
        CHECK(err.find("гістерезис") != std::string::npos, "помилка називає причину");
    }
    // Перевірка навігації перед поверненням: за умовчанням увімкнена, бо
    // вимірювання 20.09.2026 показало стан, у якому повернення нездійсненне.
    {
        Config c;
        CHECK(c.requireNavForRtl, "перевірка навігації увімкнена за умовчанням");
        // 12 с = три періоди виміряного потоку ESTIMATOR_STATUS (3,36 с).
        CHECK_NEAR(c.navStaleSec, 12.0, 1e-9, "строк придатності стану оцінювача");
        CHECK(c.navStaleSec > 3 * 3.36,
              "строк придатності покриває щонайменше три періоди потоку");
    }
    {
        const auto  path = writeTemp(
            "require_nav_for_rtl = 0\n"
            "nav_stale_sec = 2.5\n");
        Config      c;
        std::string err;
        CHECK(Config::load(path, c, err), "нові ключі приймаються");
        CHECK(!c.requireNavForRtl, "перевірку навігації можна вимкнути");
        CHECK_NEAR(c.navStaleSec, 2.5, 1e-9, "строк придатності читається");
    }
    {
        const auto  path = writeTemp("nav_stale_sec = 0\n");
        Config      c;
        std::string err;
        CHECK(!Config::load(path, c, err), "нульовий строк придатності - відмова");
    }
    {
        const auto  path = writeTemp("невідомий_ключ = 1\n");
        Config      c;
        std::string err;
        CHECK(!Config::load(path, c, err), "невідомий ключ - відмова, а не мовчання");
    }
    {
        Config      c;
        std::string err;
        CHECK(!Config::load("/nonexistent/uav.conf", c, err), "немає файла - повідомляємо");
        CHECK_NEAR(c.rttSwitchFullMs, 200.0, 1e-9, "вбудовані значення лишаються чинними");
    }

    std::remove("/tmp/uav_link_test.conf");
    return uav::test::summary("Config");
}
