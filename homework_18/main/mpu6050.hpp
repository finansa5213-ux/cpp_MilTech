// mpu6050.hpp — мінімальний драйвер акселерометра/гіроскопа по I2C.
//
// Робота з шиною відбувається ТІЛЬКИ з головного циклу: жодних обмінів
// усередині обробників переривань (див. дисципліну з заняття).
#pragma once

#include <cstdint>
#include <cstddef>

#include "esp_err.h"
#include "esp_idf_version.h"

// ESP-IDF ≥ 5.2 має новий драйвер i2c_master; на старіших — legacy driver/i2c.h.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#  define MPU_NEW_I2C_API 1
#  include "driver/i2c_master.h"
#else
#  define MPU_NEW_I2C_API 0
#  include "driver/i2c.h"
#endif

class Mpu6050 {
public:
    struct Sample {
        float ax = 0.f, ay = 0.f, az = 0.f;   // g
        float gx = 0.f, gy = 0.f, gz = 0.f;   // °/с
        float temp_c = 0.f;                   // °C
    };

    // Піднімає шину, будить датчик, налаштовує діапазони і DLPF.
    esp_err_t init(int sda, int scl, uint32_t hz, uint8_t addr);

    // Один блоковий обмін: 14 байт з 0x3B (accel + temp + gyro).
    // ~0.4 мс на 400 кГц. Викликати з циклу, не з ISR.
    bool read(Sample& out);

    uint8_t who_am_i() const { return whoami_; }

private:
    esp_err_t write_reg(uint8_t reg, uint8_t val);
    esp_err_t read_regs(uint8_t reg, uint8_t* dst, std::size_t len);

#if MPU_NEW_I2C_API
    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
#else
    i2c_port_t port_ = I2C_NUM_0;
#endif
    uint8_t addr_   = 0x68;
    uint8_t whoami_ = 0;

    // Масштаби для обраних діапазонів (±2 g, ±250 °/с).
    static constexpr float kAccelLsbPerG   = 16384.0f;
    static constexpr float kGyroLsbPerDps  = 131.0f;
};
