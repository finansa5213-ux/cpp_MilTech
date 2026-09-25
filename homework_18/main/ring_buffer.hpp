// ring_buffer.hpp — кільцевий буфер на одного продюсера й одного споживача (SPSC).
//
// Продюсер — обробник/драйвер UART (кладе байти по одному).
// Споживач  — головний цикл (забирає байти й збирає з них рядки-команди).
//
// Чому std::atomic, а не volatile: ESP32 двоядерний, продюсер і споживач можуть
// фізично виконуватись на різних ядрах, і потрібні бар'єри пам'яті, а не лише
// заборона оптимізації. На однопроцесорному МК (STM32 з ДЗ) для тієї самої
// схеми ISR↔цикл достатньо volatile-індексів — семантика ідентична.
//
// Ні push, ні pop не вимикають переривань і не беруть м'ютексів: при одному
// продюсері та одному споживачі гонки немає за побудовою.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

template <std::size_t N>
class RingBuffer {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "Розмір має бути степенем двійки");

public:
    // Викликається ТІЛЬКИ з боку продюсера. false = буфер повний, байт втрачено.
    bool push(uint8_t byte)
    {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & kMask;
        if (next == tail_.load(std::memory_order_acquire)) return false;   // повний
        buf_[head] = byte;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Викликається ТІЛЬКИ з боку споживача. false = порожньо.
    bool pop(uint8_t& byte)
    {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;   // порожньо
        byte = buf_[tail];
        tail_.store((tail + 1) & kMask, std::memory_order_release);
        return true;
    }

    bool empty() const
    {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    std::size_t size() const
    {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & kMask;
    }

    static constexpr std::size_t capacity() { return N - 1; }   // один слот службовий

private:
    static constexpr std::size_t kMask = N - 1;

    uint8_t buf_[N] = {};
    std::atomic<std::size_t> head_{0};   // пише продюсер
    std::atomic<std::size_t> tail_{0};   // пише споживач
};
