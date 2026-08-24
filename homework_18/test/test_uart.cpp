// Хостова перевірка кільцевого буфера і збирача рядків (бонус ДЗ).
#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

const char* esp_err_to_name(esp_err_t) { return "ESP_OK"; }
esp_err_t uart_driver_install(uart_port_t, int, int, int, void*, int) { return ESP_OK; }
esp_err_t uart_param_config(uart_port_t, const uart_config_t*) { return ESP_OK; }
esp_err_t uart_set_pin(uart_port_t, int, int, int, int) { return ESP_OK; }
int uart_read_bytes(uart_port_t, void*, uint32_t, TickType_t) { return 0; }
static std::string g_tx;
int uart_write_bytes(uart_port_t, const void* p, size_t n) {
    g_tx.append(static_cast<const char*>(p), n); return int(n);
}
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*,
                                  UBaseType_t, TaskHandle_t*, BaseType_t) { return pdTRUE; }

#include "uart_link.cpp"   // отримуємо доступ до s_rx з анонімного namespace

static int g_fail = 0;
static void check(bool c, const char* what) {
    printf("%s  %s\n", c ? "  ok  " : "  FAIL", what);
    if (!c) ++g_fail;
}

// Імітуємо продюсера: те саме, що робить помпа над драйвером.
static void feed(const char* s) {
    for (const char* p = s; *p; ++p) uart_link::s_rx.push(uint8_t(*p));
}

int main()
{
    using uart_link::LineStatus;
    char line[64];

    printf("== кільцевий буфер ==\n");
    {
        RingBuffer<8> rb;
        check(rb.capacity() == 7, "capacity = N-1");
        uint8_t b;
        check(!rb.pop(b), "порожній не віддає");
        for (int i = 0; i < 7; ++i) check(rb.push(uint8_t(i)) || i < 0, "");
        check(!rb.push(99), "повний не приймає");
        bool seq_ok = true;
        for (int i = 0; i < 7; ++i) { rb.pop(b); if (b != i) seq_ok = false; }
        check(seq_ok, "FIFO-порядок збережено");
        // 1000 обертів через межу масиву
        bool wrap_ok = true;
        for (int i = 0; i < 1000; ++i) {
            rb.push(uint8_t(i & 0xFF));
            uint8_t v; rb.pop(v);
            if (v != uint8_t(i & 0xFF)) wrap_ok = false;
        }
        check(wrap_ok, "коректний перехід через кінець буфера (1000 циклів)");
    }

    printf("\n== збирач рядків ==\n");
    feed("p 100\r\n");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::Ready &&
          std::strcmp(line, "p 100") == 0, "CRLF: рядок зібрано без керівних символів");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::None, "далі порожньо");

    feed("s 30\nm auto\n");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::Ready &&
          std::strcmp(line, "s 30") == 0, "дві команди поспіль: перша");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::Ready &&
          std::strcmp(line, "m auto") == 0, "дві команди поспіль: друга");

    feed("\n\r\n");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::None, "порожні рядки ігноруються");

    feed("p\t100\x01 \n");
    check(uart_link::read_line(line, sizeof(line)) == LineStatus::Ready &&
          std::strcmp(line, "p100 ") == 0, "непечатні символи відкинуто");

    printf("\n== задовгий рядок ==\n");
    {
        std::string longcmd(200, 'x');
        longcmd += "\n";
        // кільце менше за рядок — годуємо порціями, як це робить помпа
        for (char c : longcmd) {
            uart_link::s_rx.push(uint8_t(c));
            uart_link::read_line(line, sizeof(line));
        }
        // фінальний '\n' уже спожито; статус віддається на ньому
        feed("p 250\n");
        auto st = uart_link::read_line(line, sizeof(line));
        check(st == LineStatus::Ready && std::strcmp(line, "p 250") == 0,
              "після задовгого рядка наступна команда читається нормально");
    }
    {
        std::string longcmd(100, 'y');
        feed(longcmd.c_str());
        uart_link::read_line(line, sizeof(line));
        feed("\n");
        check(uart_link::read_line(line, sizeof(line)) == LineStatus::TooLong,
              "задовгий рядок -> LineStatus::TooLong");
    }

    printf("\n== переповнення кільця рахується ==\n");
    {
        const uint32_t before = uart_link::dropped_bytes();
        for (int i = 0; i < 400; ++i) uart_link::s_rx.push('z');   // кільце на 256
        check(uart_link::dropped_bytes() == before, "push() сам не рахує — рахує помпа");
        // очистимо
        uint8_t b; while (uart_link::s_rx.pop(b)) {}
        check(uart_link::s_rx.empty(), "кільце спорожнене");
    }

    printf("\n== вивід ==\n");
    g_tx.clear();
    uart_link::write_line("ok p=100 ms");
    check(g_tx == "ok p=100 ms\r\n", "write_line додає CRLF");
    g_tx.clear();
    uart_link::printf_line("t=%d ms", 42);
    check(g_tx == "t=42 ms\r\n", "printf_line форматує і термінує рядок");

    printf("\n%s (%d помилок)\n", g_fail ? "ПРОВАЛЕНО" : "ВСІ ПЕРЕВІРКИ ПРОЙДЕНО", g_fail);
    return g_fail ? 1 : 0;
}
