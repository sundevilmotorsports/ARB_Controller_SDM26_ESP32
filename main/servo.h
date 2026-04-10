#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#define SERVO_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE
#define SERVO_PWM_TIMER         LEDC_TIMER_0
#define SERVO_PWM_CHANNEL       LEDC_CHANNEL_0
#define SERVO_PWM_FREQUENCY_HZ  333U
#define SERVO_PWM_DUTY_RES      LEDC_TIMER_14_BIT
#define SERVO_PWM_DUTY_SCALE    (1U << SERVO_PWM_DUTY_RES)
#define SERVO_PWM_PERIOD_US     (1000000U / SERVO_PWM_FREQUENCY_HZ)

#define SERVO_POSITION_MIN 1U
#define SERVO_POSITION_MAX 100U
#define SERVO_MIN_PULSE_US 900U
#define SERVO_MAX_PULSE_US 2100U

#define ESP_ERR_SERVO_BASE                        0xE200

typedef struct {
  int frequency;
  uint8_t *REQ_ARB_POS;
  uint16_t gpio;
  ledc_channel_t led_channel;
  ledc_timer_t led_timer;
  uint16_t max_pos;
} ServoTaskArgs;

void initialize_servo_timer(ledc_timer_t led_tim);
void initialize_servo_pwm(int gpio, ledc_timer_t led_tim, ledc_channel_t led_chan);
void servo_task(void *args);

#endif