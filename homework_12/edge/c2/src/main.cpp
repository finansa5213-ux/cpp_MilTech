#include "c2_controller.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

int main() {
    nlohmann::json cfg;
    {
        std::ifstream f("/etc/c2/c2_config.json");
        if (!f.is_open()) {
            std::cerr << "[C2] error: cannot open /etc/c2/c2_config.json\n";
            return 1;
        }
        try {
            f >> cfg;
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[C2] error: invalid config: " << e.what() << "\n";
            return 1;
        }
    }

    const uint16_t fc_port = cfg.at("fc_port").get<uint16_t>();

    std::cout << "[C2] config: fc_port=" << fc_port << "\n";

    C2Controller controller(fc_port);

    constexpr auto tick_interval = std::chrono::milliseconds(100);
    while (true) {
        controller.tick();
        std::this_thread::sleep_for(tick_interval);
    }
}
