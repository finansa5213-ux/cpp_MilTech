#include "servo.hpp"

#include "esp_log.h"

namespace {
constexpr const char* TAG = "servo";

inline int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

esp_err_t Servo::init(int gpio, int min_us, int max_us, int freq_hz,
                      ledc_timer_t timer, ledc_channel_t channel)
{
    timer_   = timer;
    channel_ = channel;
    min_us_  = min_us;
    max_us_  = max_us;
    freq_hz_ = freq_hz;

    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = LEDC_LOW_SPEED_MODE;
    tcfg.duty_resolution = kResolution;
    tcfg.timer_num       = timer_;
    tcfg.freq_hz         = static_cast<uint32_t>(freq_hz_);
    tcfg.clk_cfg         = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(err)); return err; }

    ledc_channel_config_t ccfg = {};
    ccfg.gpio_num   = gpio;
    ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
    ccfg.channel    = channel_;
    ccfg.intr_type  = LEDC_INTR_DISABLE;
    ccfg.timer_sel  = timer_;
    ccfg.duty       = 0;
    ccfg.hpoint     = 0;

    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ledc_channel_config: %s", esp_err_to_name(err)); return err; }

    ready_ = true;
    set_angle(angle_);          // стартова позиція — середина
    return ESP_OK;
}

uint32_t Servo::us_to_duty(int us) const
{
    // duty = us / період_у_мкс * повна_шкала
    const uint32_t period_us = 1000000u / static_cast<uint32_t>(freq_hz_);
    uint64_t duty = static_cast<uint64_t>(us) * (kMaxDuty + 1u) / period_us;
    if (duty > kMaxDuty) duty = kMaxDuty;
    return static_cast<uint32_t>(duty);
}

void Servo::set_angle(int deg)
{
    angle_ = clamp_int(deg, 0, 180);
    if (!ready_) return;

    const int us = min_us_ + (max_us_ - min_us_) * angle_ / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_, us_to_duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_);
}
