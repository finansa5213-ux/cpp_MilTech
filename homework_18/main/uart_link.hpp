// uart_link.hpp — інтерфейс №1: рядковий протокол з комп'ютером по UART.
//
// Приймання: байти забирає окремий короткий обробник (задача-помпа над драйвером)
// і кладе їх у наш кільцевий буфер. Розбір рядків і команд — у головному циклі.
#pragma once

#include <cstddef>
#include <cstdint>

namespace uart_link {

enum class LineStatus : uint8_t {
    None,      // повного рядка ще немає
    Ready,     // out містить рядок без '\r'/'\n'
    TooLong    // рядок перевищив ліміт і був відкинутий
};

void init();

// Забирає з кільця стільки байтів, скільки треба до кінця рядка.
// Викликати з головного циклу в while-циклі, поки повертає != None.
LineStatus read_line(char* out, std::size_t max_len);

void write(const char* text);
void write_line(const char* text);                 // додає "\r\n"
void printf_line(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Діагностика: скільки байтів довелось викинути через переповнення кільця.
uint32_t dropped_bytes();

}  // namespace uart_link
