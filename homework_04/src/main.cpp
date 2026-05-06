// Wheel odometry для НРК (4-колісний диференційний привід)
// Читає файл з показаннями енкодерів, виводить траєкторію (x, y, theta) на stdout.
// На stderr виводить детальні етапи розрахунків для кожного кроку (за замовчуванням).
//
// Прапорці:
//   --quiet / -q   вимкнути покрокове логування (тільки траєкторія)
//   --help  / -h   показати usage

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

    // ---- Параметри робота ----
    struct RobotParams {
        int    ticks_per_revolution = 1024;   // імпульсів на один оберт колеса
        double wheel_radius_m = 0.3;          // радіус колеса, м
        double wheelbase_m = 1.0;             // відстань між лівим і правим бортом, м

        // Похідне: скільки метрів проходить колесо за один тік
        double distance_per_tick() const {
            return 2.0 * M_PI * wheel_radius_m / ticks_per_revolution;
        }
    };

    // ---- Сире показання енкодерів з одного рядка вхідного файлу ----
    struct EncoderReading {
        long timestamp_ms = 0;
        long fl_ticks = 0;  // front-left
        long fr_ticks = 0;  // front-right
        long bl_ticks = 0;  // back-left
        long br_ticks = 0;  // back-right
    };

    // ---- Положення робота на площині ----
    struct LocationOfRobot {
        double x = 0.0;  // м
        double y = 0.0;  // м
        double theta = 0.0;  // рад
    };

    // ---- Результат парсингу одного рядка ----
    enum ParseStatus {
        Ok,
        Empty,    // порожній рядок — пропускаємо
        Invalid   // некоректний формат — виходимо з помилкою
    };

    // ---- Режим докладності виводу ----
    enum Verbosity {
        Quiet,    // тільки фінальна траєкторія на stdout
        Verbose   // + покрокові деталі обчислень на stderr
    };

    ParseStatus parse_line(const std::string& line, EncoderReading& out) {
        if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
            return ParseStatus::Empty;
        }
        std::istringstream iss(line);
        if (!(iss >> out.timestamp_ms
            >> out.fl_ticks >> out.fr_ticks
            >> out.bl_ticks >> out.br_ticks)) {
            return ParseStatus::Invalid;
        }
        return ParseStatus::Ok;
    }

    // ---- Один крок одометрії ----
    // При verbosity == Verbose — друкує деталі обчислень на stderr.
    LocationOfRobot  step(const LocationOfRobot& prev,
        long d_fl, long d_fr, long d_bl, long d_br,
        const RobotParams& p,
        Verbosity verbosity,
        int step_index)
    {
        // Крок 2. Усереднюємо борти (переднє і заднє колесо синхронні)
        const double d_left = (d_fl + d_bl) / 2.0;
        const double d_right = (d_fr + d_br) / 2.0;

        // Крок 3. Тіки -> метри
        const double dpt = p.distance_per_tick();
        const double dL = d_left * dpt;
        const double dR = d_right * dpt;

        // Крок 4. Рух центру і зміна орієнтації
        const double d = (dL + dR) / 2.0;
        const double dtheta = (dR - dL) / p.wheelbase_m;

        // Крок 5. Midpoint integration
        const double heading_mid = prev.theta + dtheta / 2.0;
        LocationOfRobot next;
        next.x = prev.x + d * std::cos(heading_mid);
        next.y = prev.y + d * std::sin(heading_mid);
        next.theta = prev.theta + dtheta;

        if (verbosity == Verbosity::Verbose) {
            auto& log = std::cerr;
            log << std::fixed << std::setprecision(6);
            log << "\n--- step #" << step_index << " ---\n";
            log << "  [1] delta ticks per wheel:\n"
                << "      d_fl=" << d_fl << "  d_fr=" << d_fr
                << "  d_bl=" << d_bl << "  d_br=" << d_br << "\n";
            log << "  [2] averaged sides:\n"
                << "      d_left  = (d_fl + d_bl) / 2 = " << d_left << " ticks\n"
                << "      d_right = (d_fr + d_br) / 2 = " << d_right << " ticks\n";
            log << "  [3] ticks -> meters (distance_per_tick = "
                << dpt << " m/tick):\n"
                << "      dL = " << dL << " m\n"
                << "      dR = " << dR << " m\n";
            log << "  [4] center motion:\n"
                << "      d      = (dL + dR) / 2          = " << d << " m\n"
                << "      dtheta = (dR - dL) / wheelbase  = " << dtheta << " rad\n";
            log << "  [5] midpoint integration:\n"
                << "      heading_mid = theta + dtheta/2  = " << heading_mid << " rad\n"
                << "      x: " << prev.x << " + " << d << "*cos(" << heading_mid
                << ") = " << next.x << "\n"
                << "      y: " << prev.y << " + " << d << "*sin(" << heading_mid
                << ") = " << next.y << "\n"
                << "      theta: " << prev.theta << " + " << dtheta
                << "  = " << next.theta << "\n";
            log << "  => location: (" << next.x << ", " << next.y
                << ", " << next.theta << ")\n";
        }
        return next;
    }
    
    void print_step(long timestamp_ms, const LocationOfRobot& location) {
    std::cout << timestamp_ms << ' '
        << std::fixed << std::setprecision(6)
        << location.x << ' ' << location.y << ' ' << location.theta << '\n';
    }
    
    void print_header(const RobotParams& p, const char* input_path) {
        auto& log = std::cerr;
        log << std::fixed << std::setprecision(6);
        log << "============================================================\n";
        log << " Wheel odometry — verbose trace\n";
        log << "============================================================\n";
        log << " input file:           " << input_path << "\n";
        log << " ticks_per_revolution: " << p.ticks_per_revolution << "\n";
        log << " wheel_radius_m:       " << p.wheel_radius_m << "\n";
        log << " wheelbase_m:          " << p.wheelbase_m << "\n";
        log << " distance_per_tick:    " << p.distance_per_tick() << " m/tick\n";
        log << " initial location:         (0, 0, 0)\n";
        log << "============================================================\n";
    }

    void print_summary(const LocationOfRobot& final_location, int steps) {
        auto& log = std::cerr;
        log << "\n============================================================\n";
        log << " Done. Steps processed: " << steps << "\n";
        log << " Final location: x=" << final_location.x
            << "  y=" << final_location.y
            << "  theta=" << final_location.theta << " rad"
            << " (" << (final_location.theta * 180.0 / M_PI) << " deg)\n";
        log << "============================================================\n";
    }

    void print_usage(const char* prog) {
        std::cerr << "Usage: " << prog << " [--quiet] <input_file>\n"
            << "  By default prints calculation steps to stderr.\n"
            << "  Use --quiet to disable verbose tracing.\n";
    }
}

int main(int argc, char** argv) {
    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    //
    // Model parameters:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Input: a text file with 5 whitespace-separated values per line:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Output: a table on stdout, starting from the second sample:
    //         timestamp_ms x y theta
    
    // ---- Розбір аргументів командного рядка ---- 
    Verbosity verbosity = Verbosity::Verbose;
    const char* input_path = nullptr;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--quiet" || arg == "-q") {
            verbosity = Verbosity::Quiet;
        }
        else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        else if (!input_path) {
            input_path = argv[i];
        }
        else {
            print_usage(argv[0]);
            return 1;
        }
    }
 
    // ---- Перевірка і підготовка ---- 
    if (!input_path) {
        print_usage(argv[0]);
        return 1;
    }

    std::ifstream in(input_path);
    if (!in) {
        std::cerr << "Error: cannot open file '" << input_path << "'\n";
        return 1;
    }

    const RobotParams params;
    LocationOfRobot  location;
    EncoderReading   prev;
    bool             have_prev = false;
    int              step_index = 0;

    if (verbosity == Verbosity::Verbose) {
        print_header(params, input_path);
    }

    std::string line;
    int line_no = 0;

    while (std::getline(in, line)) {
        ++line_no;
        EncoderReading cur;
        switch (parse_line(line, cur)) {
        case ParseStatus::Empty:
            continue;
        case ParseStatus::Invalid:
            std::cerr << "Error: malformed line " << line_no
                << ": " << line << '\n';
            return 1;
        case ParseStatus::Ok:
            break;
        }

        if (!have_prev) {
            prev = cur;
            have_prev = true;
            if (verbosity == Verbosity::Verbose) {
                std::cerr << "\nStart row (t=" << cur.timestamp_ms
                    << " ms): ticks ("
                    << cur.fl_ticks << ", " << cur.fr_ticks << ", "
                    << cur.bl_ticks << ", " << cur.br_ticks << ")\n";
            }
            continue;
    }

    ++step_index;
    const long d_fl = cur.fl_ticks - prev.fl_ticks;
    const long d_fr = cur.fr_ticks - prev.fr_ticks;
    const long d_bl = cur.bl_ticks - prev.bl_ticks;
    const long d_br = cur.br_ticks - prev.br_ticks;

    if (verbosity == Verbosity::Verbose) {
        std::cerr << "\nRow t=" << cur.timestamp_ms << " ms"
            << "  (prev t=" << prev.timestamp_ms << " ms,"
            << " dt=" << (cur.timestamp_ms - prev.timestamp_ms) << " ms)";
    }

    location = step(location, d_fl, d_fr, d_bl, d_br, params, verbosity, step_index);
    print_step(cur.timestamp_ms, location);

    prev = cur;
    }

    if (verbosity == Verbosity::Verbose) {
        print_summary(location, step_index);
    }

    return 0;

}
