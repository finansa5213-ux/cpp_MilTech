// =============================================================================
//  cMiltech — C++ для військових технологій
//  ДЗ18 (Заняття 36): пристрій на двох інтерфейсах
//
//  Інтерфейси:
//    UART0  — звіт рядками + приймання команд (кільцевий буфер, побайтово)
//    I2C    — MPU-6050: акселерометр, гіроскоп, температура
//    ШІМ    — серво (LEDC), кут задається командою або веде нахил у режимі auto
//
//  Дисципліна:
//    * Обробник таймера — лише прапорець + мітка часу + пробудження циклу.
//      Жодних printf, жодних обмінів по шинах, жодних затримок усередині.
//    * Спільні змінні обробник↔цикл — volatile, багатослівні читаються під
//      критичною секцією (int64 на 32-бітному ядрі не атомарний).
//    * Період задає таймер, а не vTaskDelay у циклі.
// =============================================================================

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "app_config.hpp"
#include "mpu6050.hpp"
#include "servo.hpp"
#include "uart_link.hpp"

// Бонус із методички: додати у рядок стану фактичний інтервал між спрацюваннями
// таймера (поточний + min/max). Вимкнено, щоб формат звіту лишався мінімальним.
#define REPORT_DT 0

namespace {

constexpr const char* TAG = "dz18";

// ------------------------------------------------- стан обробник ↔ головний цикл
portMUX_TYPE      s_tick_mux    = portMUX_INITIALIZER_UNLOCKED;
volatile bool     s_tick_flag   = false;   // «пора працювати»
volatile uint32_t s_tick_seq    = 0;       // скільки разів спрацював таймер
volatile int64_t  s_tick_us     = 0;       // мітка часу останнього спрацювання
volatile uint32_t s_tick_missed = 0;       // цикл не встиг забрати попередній тік

TaskHandle_t     s_main_task = nullptr;
gptimer_handle_t s_timer     = nullptr;

// ---------------------------------------------------------- стан головного циклу
enum class Mode : uint8_t { Manual, Auto };

Mpu6050        s_imu;
Servo          s_servo;
Mpu6050::Sample s_last{};
Mode           s_mode      = Mode::Auto;
uint32_t       s_period_ms = cfg::kPeriodMsDefault;
bool           s_i2c_ok    = false;

#if REPORT_DT
int64_t  s_prev_tick_us = 0;
uint32_t s_dt_min_us    = UINT32_MAX;
uint32_t s_dt_max_us    = 0;
uint32_t s_dt_cur_us    = 0;
#endif

inline int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// -----------------------------------------------------------------------------
//  Обробник переривання таймера. Все, що тут дозволено: підняти прапорець,
//  зафіксувати час і розбудити цикл.
// -----------------------------------------------------------------------------
bool IRAM_ATTR on_timer_alarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*)
{
    BaseType_t higher_woken = pdFALSE;

    portENTER_CRITICAL_ISR(&s_tick_mux);
    // Пишемо явними присвоєннями: ++ і += над volatile у сучасному C++ депрековані.
    if (s_tick_flag) s_tick_missed = s_tick_missed + 1;   // попередній тік ще не оброблений
    s_tick_flag = true;
    s_tick_seq  = s_tick_seq + 1;
    s_tick_us   = esp_timer_get_time();
    portEXIT_CRITICAL_ISR(&s_tick_mux);

    vTaskNotifyGiveFromISR(s_main_task, &higher_woken);
    return higher_woken == pdTRUE;
}

// Атомарно забрати тік у головному циклі.
bool take_tick(uint32_t& seq, int64_t& t_us)
{
    bool has = false;
    portENTER_CRITICAL(&s_tick_mux);
    if (s_tick_flag) {
        s_tick_flag = false;
        seq  = s_tick_seq;
        t_us = s_tick_us;
        has  = true;
    }
    portEXIT_CRITICAL(&s_tick_mux);
    return has;
}

// -----------------------------------------------------------------------------
//  Таймер
// -----------------------------------------------------------------------------
esp_err_t timer_set_period(uint32_t period_ms)
{
    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count  = static_cast<uint64_t>(period_ms) * 1000ULL;  // роздільність 1 мкс
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = true;

    esp_err_t err = gptimer_set_alarm_action(s_timer, &alarm);
    if (err != ESP_OK) return err;

    // Нова уставка діє з наступного тіку, а не «дораховуючи» старий інтервал.
    gptimer_set_raw_count(s_timer, 0);
    s_period_ms = period_ms;

#if REPORT_DT
    s_dt_min_us = UINT32_MAX;
    s_dt_max_us = 0;
#endif
    return ESP_OK;
}

esp_err_t timer_start()
{
    gptimer_config_t tcfg = {};
    tcfg.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
    tcfg.direction     = GPTIMER_COUNT_UP;
    tcfg.resolution_hz = 1000000;   // 1 тік = 1 мкс

    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &s_timer));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = on_timer_alarm;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, nullptr));

    ESP_ERROR_CHECK(gptimer_enable(s_timer));
    ESP_ERROR_CHECK(timer_set_period(s_period_ms));
    ESP_ERROR_CHECK(gptimer_start(s_timer));
    return ESP_OK;
}

// -----------------------------------------------------------------------------
//  Друк дробових без %f: у прошивці не варто тягнути float-форматування newlib.
// -----------------------------------------------------------------------------
void fmt_fixed(char* dst, std::size_t n, float value, int decimals)
{
    long scale = 1;
    for (int i = 0; i < decimals; ++i) scale *= 10;

    long scaled = lroundf(value * static_cast<float>(scale));
    const char* sign = (scaled < 0) ? "-" : "";
    if (scaled < 0) scaled = -scaled;

    snprintf(dst, n, "%s%ld.%0*ld", sign, scaled / scale, decimals, scaled % scale);
}

const char* mode_name(Mode m) { return (m == Mode::Auto) ? "auto" : "manual"; }

// -----------------------------------------------------------------------------
//  Рядок стану. Формат стабільний: пари key=value, розділені пробілами.
//    t=12345 ms  ax=0.02 ay=-0.01 az=1.00  temp=27.4 C  servo=90 deg  mode=auto  i2c=ok
// -----------------------------------------------------------------------------
void report(int64_t t_us)
{
    char ax[12], ay[12], az[12], tc[12];
    fmt_fixed(ax, sizeof(ax), s_last.ax, 2);
    fmt_fixed(ay, sizeof(ay), s_last.ay, 2);
    fmt_fixed(az, sizeof(az), s_last.az, 2);
    fmt_fixed(tc, sizeof(tc), s_last.temp_c, 1);

#if REPORT_DT
    uart_link::printf_line(
        "t=%lld ms  ax=%s ay=%s az=%s  temp=%s C  servo=%d deg  mode=%s  i2c=%s  "
        "dt=%lu us dtmin=%lu dtmax=%lu",
        static_cast<long long>(t_us / 1000), ax, ay, az, tc,
        s_servo.angle(), mode_name(s_mode),
        s_i2c_ok ? "ok" : "err",
        static_cast<unsigned long>(s_dt_cur_us),
        static_cast<unsigned long>(s_dt_min_us == UINT32_MAX ? 0 : s_dt_min_us),
        static_cast<unsigned long>(s_dt_max_us));
#else
    uart_link::printf_line(
        "t=%lld ms  ax=%s ay=%s az=%s  temp=%s C  servo=%d deg  mode=%s  i2c=%s",
        static_cast<long long>(t_us / 1000), ax, ay, az, tc,
        s_servo.angle(), mode_name(s_mode),
        s_i2c_ok ? "ok" : "err");
#endif
}

// -----------------------------------------------------------------------------
//  Періодична робота: вимір → перерахунок виконавця → звіт.
//  Викликається рівно за прапорцем від таймера.
// -----------------------------------------------------------------------------
void do_cycle(int64_t t_us)
{
#if REPORT_DT
    if (s_prev_tick_us != 0) {
        s_dt_cur_us = static_cast<uint32_t>(t_us - s_prev_tick_us);
        if (s_dt_cur_us < s_dt_min_us) s_dt_min_us = s_dt_cur_us;
        if (s_dt_cur_us > s_dt_max_us) s_dt_max_us = s_dt_cur_us;
    }
    s_prev_tick_us = t_us;
#endif

    Mpu6050::Sample s{};
    s_i2c_ok = s_imu.read(s);
    if (s_i2c_ok) s_last = s;          // при збої тримаємо останні валідні дані

    if (s_mode == Mode::Auto) {
        // Нахил по осі X → кут серво. atan2 стійкіший за просте масштабування ax.
        const float pitch_deg =
            atan2f(-s_last.ax, sqrtf(s_last.ay * s_last.ay + s_last.az * s_last.az)) *
            57.29577951f;
        s_servo.set_angle(clamp_int(90 + static_cast<int>(lroundf(pitch_deg)), 0, 180));
    }

    report(t_us);
}

// -----------------------------------------------------------------------------
//  Команди з комп'ютера. Кожна підтверджується рядком у відповідь.
// -----------------------------------------------------------------------------
void print_help()
{
    uart_link::write_line("commands:");
    uart_link::write_line("  p <ms>          period 50..5000 (default 200)");
    uart_link::write_line("  s <deg>         servo 0..180, switches mode to manual");
    uart_link::write_line("  m auto|manual   auto = servo follows tilt");
    uart_link::write_line("  r               report once");
    uart_link::write_line("  ?               this help");
}

void handle_command(const char* line)
{
    char cmd[16] = {};
    char arg[24] = {};
    const int parsed = sscanf(line, "%15s %23s", cmd, arg);
    if (parsed < 1) return;

    for (char* p = cmd; *p; ++p) {
        if (*p >= 'A' && *p <= 'Z') *p = static_cast<char>(*p - 'A' + 'a');
    }

    // ---- період -------------------------------------------------------------
    if (std::strcmp(cmd, "p") == 0 || std::strcmp(cmd, "period") == 0) {
        if (parsed < 2) { uart_link::printf_line("ok p=%lu ms", (unsigned long)s_period_ms); return; }

        char* end = nullptr;
        const long v = std::strtol(arg, &end, 10);
        if (end == arg || *end != '\0' ||
            v < static_cast<long>(cfg::kPeriodMsMin) || v > static_cast<long>(cfg::kPeriodMsMax)) {
            uart_link::printf_line("err p: '%s' (range %lu..%lu)", arg,
                                   (unsigned long)cfg::kPeriodMsMin,
                                   (unsigned long)cfg::kPeriodMsMax);
            return;
        }
        if (timer_set_period(static_cast<uint32_t>(v)) == ESP_OK) {
            uart_link::printf_line("ok p=%ld ms", v);
        } else {
            uart_link::write_line("err p: timer");
        }
        return;
    }

    // ---- кут серво ----------------------------------------------------------
    if (std::strcmp(cmd, "s") == 0 || std::strcmp(cmd, "servo") == 0) {
        if (parsed < 2) { uart_link::printf_line("ok s=%d deg", s_servo.angle()); return; }

        char* end = nullptr;
        const long v = std::strtol(arg, &end, 10);
        if (end == arg || *end != '\0' || v < 0 || v > 180) {
            uart_link::printf_line("err s: '%s' (range 0..180)", arg);
            return;
        }
        s_mode = Mode::Manual;
        s_servo.set_angle(static_cast<int>(v));
        uart_link::printf_line("ok s=%d deg mode=manual", s_servo.angle());
        return;
    }

    // ---- режим --------------------------------------------------------------
    if (std::strcmp(cmd, "m") == 0 || std::strcmp(cmd, "mode") == 0) {
        if (parsed < 2) { uart_link::printf_line("ok mode=%s", mode_name(s_mode)); return; }

        if (std::strncmp(arg, "a", 1) == 0)      s_mode = Mode::Auto;
        else if (std::strncmp(arg, "m", 1) == 0) s_mode = Mode::Manual;
        else { uart_link::printf_line("err m: '%s' (auto|manual)", arg); return; }

        uart_link::printf_line("ok mode=%s", mode_name(s_mode));
        return;
    }

    // ---- разовий звіт -------------------------------------------------------
    if (std::strcmp(cmd, "r") == 0 || std::strcmp(cmd, "report") == 0) {
        report(esp_timer_get_time());
        return;
    }

    // ---- довідка ------------------------------------------------------------
    if (std::strcmp(cmd, "?") == 0 || std::strcmp(cmd, "h") == 0 ||
        std::strcmp(cmd, "help") == 0) {
        print_help();
        return;
    }

    uart_link::printf_line("err unknown: '%s' (try ?)", cmd);
}

}  // namespace

// =============================================================================
extern "C" void app_main()
{
    s_main_task = xTaskGetCurrentTaskHandle();

    uart_link::init();
    uart_link::write_line("");
    uart_link::write_line("# cMiltech DZ18: UART + I2C(MPU-6050) + PWM(servo)");

    if (s_servo.init(cfg::kServoGpio, cfg::kServoMinUs, cfg::kServoMaxUs,
                     cfg::kServoFreq) != ESP_OK) {
        uart_link::write_line("# servo: init failed");
    }

    if (s_imu.init(cfg::kI2cSda, cfg::kI2cScl, cfg::kI2cHz, cfg::kMpuAddr) == ESP_OK) {
        uart_link::printf_line("# mpu6050: who_am_i=0x%02X", s_imu.who_am_i());
    } else {
        uart_link::write_line("# mpu6050: init failed (перевір SDA/SCL і живлення)");
    }

    ESP_ERROR_CHECK(timer_start());
    uart_link::printf_line("# period=%lu ms mode=%s. '?' for help",
                           (unsigned long)s_period_ms, mode_name(s_mode));

    char line[cfg::kCmdMaxLen];
    uint32_t seq = 0;
    int64_t  t_us = 0;
    uint32_t reported_missed = 0;

    for (;;) {
        // Чекаємо нотифікацію від таймера. Таймаут потрібен не для періоду, а щоб
        // цикл прокидався й розбирав команди навіть за довгого періоду виміру.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(cfg::kLoopWakeMs));

        // 1) команди: розбираємо все, що встигло накопичитись у кільці
        for (;;) {
            const auto st = uart_link::read_line(line, sizeof(line));
            if (st == uart_link::LineStatus::None) break;
            if (st == uart_link::LineStatus::TooLong) {
                uart_link::printf_line("err line too long (max %u)",
                                       (unsigned)(cfg::kCmdMaxLen - 1));
                continue;
            }
            handle_command(line);
        }

        // 2) періодична робота — тільки за прапорцем від таймера
        if (take_tick(seq, t_us)) {
            do_cycle(t_us);
        }

        // 3) діагностика: якщо цикл не встигає за таймером — скажемо про це один раз
        const uint32_t missed = s_tick_missed;
        if (missed != reported_missed) {
            reported_missed = missed;
            uart_link::printf_line("# warn: missed ticks=%lu (period too short?)",
                                   (unsigned long)missed);
        }
    }
}
