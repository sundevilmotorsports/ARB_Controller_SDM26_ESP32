#include "servo.h"

static const char *TAG = "PWM";

void initialize_servo_timer(ledc_timer_t led_tim){
    const ledc_timer_config_t timer_config = {
        .speed_mode         = SERVO_PWM_SPEED_MODE,
        .duty_resolution    = SERVO_PWM_DUTY_RES,
        .timer_num          = led_tim,
        .freq_hz            = SERVO_PWM_FREQUENCY_HZ,
        .clk_cfg            = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
}
    
void initialize_servo_pwm(int gpio, ledc_timer_t led_tim, ledc_channel_t led_chan){
    const ledc_channel_config_t channel_config = {
        .gpio_num   = gpio,
        .speed_mode = SERVO_PWM_SPEED_MODE,
        .channel    = led_chan,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = led_tim,
        .duty       = 0,
        .hpoint     = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    ESP_LOGI(TAG, "Servo PWM initialized on GPIO %d at %u Hz", gpio, SERVO_PWM_FREQUENCY_HZ);
}

void servo_task(void *arg){
    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t last_applied_position = 0;
    ServoTaskArgs *args = (ServoTaskArgs *) arg;
    uint32_t refresh_rate_hz = args->frequency;
    uint8_t *requested_pos = args->REQ_ARB_POS;
    initialize_servo_pwm(args->gpio, args->led_timer, args->led_channel);
    for (;;) {
        const uint8_t requested_position = *requested_pos;
        if ((requested_position >= 1) && (requested_position <= args->max_pos)) {
            ledc_set_duty(SERVO_PWM_SPEED_MODE, args->led_channel, (((SERVO_MIN_PULSE_US + ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * requested_position) / args->max_pos) * ((1 << SERVO_PWM_DUTY_RES) - 1)) / (1000000 / SERVO_PWM_FREQUENCY_HZ)));
            ledc_update_duty(SERVO_PWM_SPEED_MODE, args->led_channel);
        } else {
            ESP_LOGW(TAG, "NOT IN VALID RANGE");
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS((uint64_t) 1000 / refresh_rate_hz));
    }
}