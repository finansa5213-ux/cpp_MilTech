// Точка входу бортового маршрутизатора каналів керування БПЛА.
//
//   uav_link [шлях_до_конфігурації]
//
// За умовчанням /etc/uav_link/uav_link.conf. Якщо файла немає, беруться
// вбудовані значення - ті самі, що виміряні на стенді.
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>

#include "uav/Config.hpp"
#include "uav/Router.hpp"

namespace {

void onSignal(int) { uav::Router::requestStop(); }

void installHandlers() {
    struct sigaction sa {};
    sa.sa_handler = onSignal;
    ::sigemptyset(&sa.sa_mask);
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    // Розрив каналу не повинен убивати процес: запис у закритий сокет
    // або порт - звичайна подія, її обробляє код повернення write().
    ::signal(SIGPIPE, SIG_IGN);
}

} // namespace

int main(int argc, char** argv) {
    const std::string path = (argc > 1) ? argv[1] : "/etc/uav_link/uav_link.conf";

    uav::Config cfg;
    std::string err;
    if (!uav::Config::load(path, cfg, err)) {
        // Відсутній файл - не помилка: працюємо на вбудованих значеннях.
        std::fprintf(stderr, "конфігурація: %s (беру вбудовані значення)\n", err.c_str());
    }

    installHandlers();

    uav::Router router(cfg);
    if (!router.init(err)) {
        std::fprintf(stderr, "не вдалося запуститися: %s\n", err.c_str());
        return 1;
    }
    return router.run();
}
