// Хостова перевірка логіки (не частина прошивки): підміняємо IDF-заглушками
// і ганяємо розбір команд, формат звіту, кільцевий буфер і збирач рядків.
#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

// ----------------------------------------------------------------- заглушки IDF
static int64_t g_fake_time_us = 1'000'000;
int64_t esp_timer_get_time(void) { return g_fake_time_us; }
const char* esp_err_to_name(esp_err_t) { return "ESP_OK"; }

void portENTER_CRITICAL(portMUX_TYPE*) {}
void portEXIT_CRITICAL(portMUX_TYPE*) {}
void portENTER_CRITICAL_ISR(portMUX_TYPE*) {}
void portEXIT_CRITICAL_ISR(portMUX_TYPE*) {}

TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
void vTaskDelay(TickType_t) {}
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t* w) { if (w) *w = pdFALSE; }
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t) { return 0; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*,
                                  UBaseType_t, TaskHandle_t*, BaseType_t) { return pdTRUE; }

static uint64_t g_alarm_count = 0;
esp_err_t gptimer_new_timer(const gptimer_config_t*, gptimer_handle_t* h) { *h = (gptimer_handle_t)1; return ESP_OK; }
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t, const gptimer_event_callbacks_t*, void*) { return ESP_OK; }
esp_err_t gptimer_enable(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_start(gptimer_handle_t) { return ESP_OK; }
esp_err_t gptimer_set_alarm_action(gptimer_handle_t, const gptimer_alarm_config_t* c) { g_alarm_count = c->alarm_count; return ESP_OK; }
esp_err_t gptimer_set_raw_count(gptimer_handle_t, uint64_t) { return ESP_OK; }

static uint32_t g_duty = 0;
esp_err_t ledc_timer_config(const ledc_timer_config_t*) { return ESP_OK; }
esp_err_t ledc_channel_config(const ledc_channel_config_t*) { return ESP_OK; }
esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t, uint32_t d) { g_duty = d; return ESP_OK; }
esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t) { return ESP_OK; }

// Датчик: ax=0.02 ay=-0.01 az=1.00, temp=27.4
static bool g_i2c_fail = false;
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t* h) { *h = (i2c_master_bus_handle_t)1; return ESP_OK; }
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t* h) { *h = (i2c_master_dev_handle_t)1; return ESP_OK; }
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t*, size_t, int) { return ESP_OK; }
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t* tx, size_t,
                                      uint8_t* rx, size_t len, int)
{
    if (g_i2c_fail) return ESP_FAIL;
    if (tx[0] == 0x75) { rx[0] = 0x68; return ESP_OK; }
    auto put = [&](int i, int16_t v) { rx[i] = uint8_t(v >> 8); rx[i + 1] = uint8_t(v & 0xFF); };
    if (len >= 14) {
        put(0, 328); put(2, -164); put(4, 16384);   // 0.02 / -0.01 / 1.00 g
        put(6, -3104);                              // ≈27.4 °C
        put(8, 0); put(10, 0); put(12, 0);
    }
    return ESP_OK;
}

// -------------------------------------------------------- підмінений uart_link
#include "uart_link.hpp"
static std::vector<std::string> g_out;
namespace uart_link {
void init() {}
LineStatus read_line(char*, std::size_t) { return LineStatus::None; }
void write(const char* s) { g_out.emplace_back(s); }
void write_line(const char* s) { g_out.emplace_back(s); }
void printf_line(const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    g_out.emplace_back(buf);
}
uint32_t dropped_bytes() { return 0; }
}  // namespace uart_link

#include "main.cpp"   // тягне анонімний namespace з handle_command/report/do_cycle

// --------------------------------------------------------------------- helpers
static int g_fail = 0;
static void check(bool cond, const char* what)
{
    printf("%s  %s\n", cond ? "  ok  " : "  FAIL", what);
    if (!cond) ++g_fail;
}
static std::string last() { return g_out.empty() ? std::string() : g_out.back(); }
static void cmd(const char* s) { g_out.clear(); handle_command(s); }

int main()
{
    printf("== серво ==\n");
    s_servo.init(18, 500, 2500, 50);
    s_servo.set_angle(0);   check(g_duty == uint32_t(500ull * 16384 / 20000), "0deg -> 500us duty");
    s_servo.set_angle(180); check(g_duty == uint32_t(2500ull * 16384 / 20000), "180deg -> 2500us duty");
    s_servo.set_angle(90);  check(g_duty == uint32_t(1500ull * 16384 / 20000), "90deg -> 1500us duty");
    s_servo.set_angle(999); check(s_servo.angle() == 180, "кут підтискається зверху");
    s_servo.set_angle(-5);  check(s_servo.angle() == 0,   "кут підтискається знизу");

    printf("\n== датчик і звіт ==\n");
    s_imu.init(21, 22, 400000, 0x68);
    check(s_imu.who_am_i() == 0x68, "WHO_AM_I=0x68");
    s_mode = Mode::Manual;
    s_servo.set_angle(90);
    g_out.clear();
    do_cycle(12'345'678);
    printf("       > %s\n", last().c_str());
    check(last() == "t=12345 ms  ax=0.02 ay=-0.01 az=1.00  temp=27.4 C  servo=90 deg  "
                    "mode=manual  i2c=ok",
          "рядок стану точно за форматом методички");

    printf("\n== режим auto: серво веде нахил ==\n");
    s_mode = Mode::Auto;
    do_cycle(20'000'000);
    check(s_servo.angle() == 89, "ax=+0.02 -> кут 89 (нахил ≈ -1.1°)");

    printf("\n== збій I2C: тримаємо останні валідні дані ==\n");
    g_i2c_fail = true;
    g_out.clear();
    do_cycle(30'000'000);
    check(last().find("i2c=err") != std::string::npos, "у звіті i2c=err");
    check(last().find("ax=0.02") != std::string::npos, "показані останні валідні дані");
    g_i2c_fail = false;

    printf("\n== команди ==\n");
    timer_start();
    cmd("p 100");    check(last() == "ok p=100 ms" && g_alarm_count == 100000, "p 100");
    cmd("p 1000");   check(last() == "ok p=1000 ms" && g_alarm_count == 1000000, "p 1000");
    cmd("p 10");     check(last().rfind("err p", 0) == 0, "p 10 -> err (нижче межі)");
    cmd("p 99999");  check(last().rfind("err p", 0) == 0, "p 99999 -> err (вище межі)");
    cmd("p abc");    check(last().rfind("err p", 0) == 0, "p abc -> err (не число)");
    cmd("p");        check(last() == "ok p=1000 ms", "p без аргументу показує поточний");

    cmd("s 30");     check(last() == "ok s=30 deg mode=manual" && s_servo.angle() == 30, "s 30");
    check(s_mode == Mode::Manual, "s перемикає режим у manual");
    cmd("s 200");    check(last().rfind("err s", 0) == 0, "s 200 -> err");
    cmd("S 45");     check(s_servo.angle() == 45, "регістр команди не має значення");

    cmd("m auto");   check(last() == "ok mode=auto" && s_mode == Mode::Auto, "m auto");
    cmd("m manual"); check(last() == "ok mode=manual" && s_mode == Mode::Manual, "m manual");
    cmd("m xyz");    check(last().rfind("err m", 0) == 0, "m xyz -> err");

    cmd("r");        check(last().rfind("t=", 0) == 0, "r -> разовий рядок стану");
    cmd("?");        check(g_out.size() >= 6, "? -> довідка");
    cmd("qqq");      check(last().rfind("err unknown", 0) == 0, "невідома команда -> err");

    printf("\n== обробник таймера ↔ цикл ==\n");
    s_tick_flag = false; s_tick_seq = 0; s_tick_missed = 0;
    uint32_t seq = 0; int64_t t = 0;
    check(!take_tick(seq, t), "без тіку цикл нічого не забирає");
    g_fake_time_us = 5'000'000;
    on_timer_alarm(nullptr, nullptr, nullptr);
    check(take_tick(seq, t) && seq == 1 && t == 5'000'000, "тік забрано з міткою часу");
    check(!take_tick(seq, t), "прапорець скинуто після забору");
    on_timer_alarm(nullptr, nullptr, nullptr);
    on_timer_alarm(nullptr, nullptr, nullptr);   // цикл не встиг
    check(s_tick_missed == 1, "пропущений тік порахований");

    printf("\n== fmt_fixed (друк без %%f) ==\n");
    char b[16];
    fmt_fixed(b, sizeof(b), 0.0f, 2);     check(std::strcmp(b, "0.00") == 0, "0.00");
    fmt_fixed(b, sizeof(b), -0.014f, 2);  check(std::strcmp(b, "-0.01") == 0, "-0.01");
    fmt_fixed(b, sizeof(b), 1.0f, 2);     check(std::strcmp(b, "1.00") == 0, "1.00");
    fmt_fixed(b, sizeof(b), -12.345f, 1); check(std::strcmp(b, "-12.3") == 0, "-12.3");
    fmt_fixed(b, sizeof(b), 9.999f, 2);   check(std::strcmp(b, "10.00") == 0, "10.00 (округлення)");

    printf("\n%s (%d помилок)\n", g_fail ? "ПРОВАЛЕНО" : "ВСІ ПЕРЕВІРКИ ПРОЙДЕНО", g_fail);
    return g_fail ? 1 : 0;
}
