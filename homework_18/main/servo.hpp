// servo.hpp — виконавець на ШІМ (LEDC). Керується командами по UART або режимом auto.
#pragma once

#include "driver/ledc.h"
#include "esp_err.h"

class Servo {
public:
    esp_err_t init(int gpio, int min_us, int max_us, int freq_hz,
                   ledc_timer_t timer = LEDC_TIMER_0,
                   ledc_channel_t channel = LEDC_CHANNEL_0);

    // Кут 0..180°, значення поза діапазоном підтискаються.
    void set_angle(int deg);

    int angle() const { return angle_; }

private:
    uint32_t us_to_duty(int us) const;

    ledc_timer_t   timer_   = LEDC_TIMER_0;
    ledc_channel_t channel_ = LEDC_CHANNEL_0;
    int  min_us_  = 500;
    int  max_us_  = 2500;
    int  freq_hz_ = 50;
    int  angle_   = 90;
    bool ready_   = false;

    // 14 біт на 50 Гц => крок ≈1.22 мкс, цього більш ніж досить для серво.
    static constexpr ledc_timer_bit_t kResolution = LEDC_TIMER_14_BIT;
    static constexpr uint32_t kMaxDuty = (1u << 14) - 1u;
};
