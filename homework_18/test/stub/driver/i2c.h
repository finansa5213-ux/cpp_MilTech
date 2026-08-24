#pragma once
#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef int i2c_port_t;
#define I2C_NUM_0 0
typedef enum { I2C_MODE_MASTER = 1 } i2c_mode_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef struct {
    i2c_mode_t mode;
    int sda_io_num;
    int scl_io_num;
    gpio_pullup_t sda_pullup_en;
    gpio_pullup_t scl_pullup_en;
    struct { uint32_t clk_speed; } master;
    uint32_t clk_flags;
} i2c_config_t;
esp_err_t i2c_param_config(i2c_port_t, const i2c_config_t*);
esp_err_t i2c_driver_install(i2c_port_t, i2c_mode_t, size_t, size_t, int);
esp_err_t i2c_master_write_to_device(i2c_port_t, uint8_t, const uint8_t*, size_t, TickType_t);
esp_err_t i2c_master_write_read_device(i2c_port_t, uint8_t, const uint8_t*, size_t, uint8_t*, size_t, TickType_t);
