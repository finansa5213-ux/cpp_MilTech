// app_config.hpp — усі «магічні числа» проєкту в одному місці.
#pragma once

#include <cstdint>
#include <cstddef>

namespace cfg {

// ---------------------------------------------------------------- інтерфейс №1
// UART: зв'язок з комп'ютером (звіт + команди).
// UART0 = той самий порт, що йде на USB і на $serialMonitor у Wokwi.
inline constexpr int         kUartPort      = 0;
inline constexpr int         kUartBaud      = 115200;
inline constexpr int         kUartRxDrvBuf  = 512;   // кільце всередині драйвера IDF
inline constexpr int         kUartTxDrvBuf  = 1024;  // щоб uart_write_bytes не блокував цикл
inline constexpr std::size_t kRxRingSize    = 256;   // НАШЕ кільце (степінь двійки!)
inline constexpr std::size_t kCmdMaxLen     = 64;    // максимальна довжина рядка команди

// ---------------------------------------------------------------- інтерфейс №2
// I2C: акселерометр/гіроскоп MPU-6050.
inline constexpr int      kI2cSda    = 21;
inline constexpr int      kI2cScl    = 22;
inline constexpr uint32_t kI2cHz     = 400000;
inline constexpr uint8_t  kMpuAddr   = 0x68;         // AD0 = GND

// ---------------------------------------------------------------- інтерфейс №3
// ШІМ: серво (бонус — два додаткові інтерфейси замість одного).
inline constexpr int kServoGpio  = 18;
inline constexpr int kServoMinUs = 500;              // 0°
inline constexpr int kServoMaxUs = 2500;             // 180°
inline constexpr int kServoFreq  = 50;               // 20 мс період

// -------------------------------------------------------------------- таймер
inline constexpr uint32_t kPeriodMsDefault = 200;
inline constexpr uint32_t kPeriodMsMin     = 50;
inline constexpr uint32_t kPeriodMsMax     = 5000;

// Скільки чекати на нотифікацію від таймера, перш ніж прокинутись «просто так»
// і розібрати накопичені команди. Це НЕ спосіб задати період — період дає таймер.
inline constexpr uint32_t kLoopWakeMs = 5;

}  // namespace cfg
