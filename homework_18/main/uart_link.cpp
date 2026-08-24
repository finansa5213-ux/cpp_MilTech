#include "uart_link.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "app_config.hpp"
#include "ring_buffer.hpp"

namespace uart_link {
namespace {

constexpr const char* TAG = "uart";
constexpr uart_port_t kPort = static_cast<uart_port_t>(cfg::kUartPort);

// Спільний стан продюсер↔споживач.
RingBuffer<cfg::kRxRingSize> s_rx;
std::atomic<uint32_t>        s_dropped{0};

// Стан збирача рядка — належить ТІЛЬКИ головному циклу.
char        s_line[cfg::kCmdMaxLen] = {};
std::size_t s_len       = 0;
bool        s_overflow  = false;

// Помпа приймання. Робить рівно дві речі: забрати байти з драйвера і покласти
// їх у кільце. Жодного розбору, жодних printf, жодних затримок — так само, як
// має поводитись обробник переривання.
void rx_pump_task(void*)
{
    uint8_t chunk[64];
    for (;;) {
        const int n = uart_read_bytes(kPort, chunk, sizeof(chunk), portMAX_DELAY);
        for (int i = 0; i < n; ++i) {
            if (!s_rx.push(chunk[i])) {
                s_dropped.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
}

}  // namespace

void init()
{
    uart_config_t cfg_uart = {};
    cfg_uart.baud_rate = cfg::kUartBaud;
    cfg_uart.data_bits = UART_DATA_8_BITS;
    cfg_uart.parity    = UART_PARITY_DISABLE;
    cfg_uart.stop_bits = UART_STOP_BITS_1;
    cfg_uart.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg_uart.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(kPort, cfg::kUartRxDrvBuf, cfg::kUartTxDrvBuf,
                                        0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(kPort, &cfg_uart));
    ESP_ERROR_CHECK(uart_set_pin(kPort, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(rx_pump_task, "uart_rx", 2560, nullptr,
                            configMAX_PRIORITIES - 3, nullptr, 0);
    ESP_LOGI(TAG, "UART%d @ %d", cfg::kUartPort, cfg::kUartBaud);
}

LineStatus read_line(char* out, std::size_t max_len)
{
    uint8_t b = 0;
    while (s_rx.pop(b)) {
        if (b == '\n' || b == '\r') {
            if (s_len == 0 && !s_overflow) continue;      // порожній рядок / другий символ CRLF

            if (s_overflow) {                              // рядок був задовгий — віддаємо статус
                s_overflow = false;
                s_len = 0;
                return LineStatus::TooLong;
            }

            s_line[s_len] = '\0';
            const std::size_t n = (s_len < max_len - 1) ? s_len : max_len - 1;
            std::memcpy(out, s_line, n);
            out[n] = '\0';
            s_len = 0;
            return LineStatus::Ready;
        }

        if (b < 0x20 || b > 0x7E) continue;                // ігноруємо керівні символи

        if (s_len + 1 >= sizeof(s_line)) {                 // переповнення — чекаємо кінця рядка
            s_overflow = true;
            continue;
        }
        s_line[s_len++] = static_cast<char>(b);
    }
    return LineStatus::None;
}

void write(const char* text)
{
    uart_write_bytes(kPort, text, std::strlen(text));
}

void write_line(const char* text)
{
    uart_write_bytes(kPort, text, std::strlen(text));
    uart_write_bytes(kPort, "\r\n", 2);
}

void printf_line(const char* fmt, ...)
{
    char buf[192];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        uart_write_bytes(kPort, buf, (static_cast<std::size_t>(n) < sizeof(buf))
                                         ? static_cast<std::size_t>(n)
                                         : sizeof(buf) - 1);
    }
    uart_write_bytes(kPort, "\r\n", 2);
}

uint32_t dropped_bytes()
{
    return s_dropped.load(std::memory_order_relaxed);
}

}  // namespace uart_link
