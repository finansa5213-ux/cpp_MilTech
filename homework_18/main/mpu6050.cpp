#include "mpu6050.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "mpu";

// Регістри MPU-6050
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr int kI2cTimeoutMs = 100;

inline int16_t be16(const uint8_t* p) {
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

esp_err_t Mpu6050::init(int sda, int scl, uint32_t hz, uint8_t addr)
{
    addr_ = addr;

#if MPU_NEW_I2C_API
    // --- новий драйвер (IDF ≥ 5.2) ---
    // Поля заповнюємо присвоєннями, а не designated-initializers: у C++ порядок
    // ініціалізаторів жорсткий, і будь-яка зміна структури в IDF ламала б збірку.
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port                     = I2C_NUM_0;
    bus_cfg.sda_io_num                   = static_cast<gpio_num_t>(sda);
    bus_cfg.scl_io_num                   = static_cast<gpio_num_t>(scl);
    bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt            = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus_);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err)); return err; }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = addr_;
    dev_cfg.scl_speed_hz    = hz;

    err = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_);
    if (err != ESP_OK) { ESP_LOGE(TAG, "add_device: %s", esp_err_to_name(err)); return err; }
#else
    // --- legacy драйвер (IDF < 5.2) ---
    port_ = I2C_NUM_0;

    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = sda;
    conf.scl_io_num       = scl;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = hz;

    esp_err_t err = i2c_param_config(port_, &conf);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(err)); return err; }

    err = i2c_driver_install(port_, conf.mode, 0, 0, 0);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(err)); return err; }
#endif

    // Прокидаємо датчик: скидаємо SLEEP, беремо такт від гіроскопа X (стабільніший).
    err = write_reg(REG_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) { ESP_LOGE(TAG, "wake: %s", esp_err_to_name(err)); return err; }
    vTaskDelay(pdMS_TO_TICKS(20));

    // DLPF ~44 Гц (CONFIG=3) => частота вибірки гіро 1 кГц; SMPLRT_DIV=7 => 125 Гц.
    (void)write_reg(REG_SMPLRT_DIV, 0x07);
    (void)write_reg(REG_CONFIG,     0x03);
    (void)write_reg(REG_GYRO_CONFIG,  0x00);  // ±250 °/с
    (void)write_reg(REG_ACCEL_CONFIG, 0x00);  // ±2 g

    uint8_t id = 0;
    if (read_regs(REG_WHO_AM_I, &id, 1) == ESP_OK) {
        whoami_ = id;
        if (id != 0x68) ESP_LOGW(TAG, "WHO_AM_I=0x%02X (очікували 0x68)", id);
    }
    return ESP_OK;
}

esp_err_t Mpu6050::write_reg(uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = {reg, val};
#if MPU_NEW_I2C_API
    return i2c_master_transmit(dev_, buf, sizeof(buf), kI2cTimeoutMs);
#else
    return i2c_master_write_to_device(port_, addr_, buf, sizeof(buf),
                                      pdMS_TO_TICKS(kI2cTimeoutMs));
#endif
}

esp_err_t Mpu6050::read_regs(uint8_t reg, uint8_t* dst, std::size_t len)
{
#if MPU_NEW_I2C_API
    return i2c_master_transmit_receive(dev_, &reg, 1, dst, len, kI2cTimeoutMs);
#else
    return i2c_master_write_read_device(port_, addr_, &reg, 1, dst, len,
                                        pdMS_TO_TICKS(kI2cTimeoutMs));
#endif
}

bool Mpu6050::read(Sample& out)
{
    uint8_t raw[14] = {};
    if (read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw)) != ESP_OK) return false;

    out.ax = be16(&raw[0])  / kAccelLsbPerG;
    out.ay = be16(&raw[2])  / kAccelLsbPerG;
    out.az = be16(&raw[4])  / kAccelLsbPerG;

    out.temp_c = be16(&raw[6]) / 340.0f + 36.53f;

    out.gx = be16(&raw[8])  / kGyroLsbPerDps;
    out.gy = be16(&raw[10]) / kGyroLsbPerDps;
    out.gz = be16(&raw[12]) / kGyroLsbPerDps;
    return true;
}
