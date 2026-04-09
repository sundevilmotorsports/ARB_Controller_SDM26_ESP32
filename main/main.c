#include <stdint.h>

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "servo_can";

static twai_node_handle_t s_can_node = NULL;
static volatile uint8_t s_requested_position = 0;

#define CAN1_TX GPIO_NUM_17
#define CAN1_RX GPIO_NUM_18
#define CAN1_BAUD 1000000U

#define SERVO_POSITION_CAN_ID 0x100U

#define SERVO_PWM_GPIO GPIO_NUM_16
#define SERVO_PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define SERVO_PWM_TIMER LEDC_TIMER_0
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_0
#define SERVO_PWM_FREQUENCY_HZ 50U
#define SERVO_PWM_DUTY_RES LEDC_TIMER_14_BIT
#define SERVO_PWM_DUTY_SCALE (1U << SERVO_PWM_DUTY_RES)
#define SERVO_PWM_PERIOD_US (1000000U / SERVO_PWM_FREQUENCY_HZ)

#define SERVO_POSITION_MIN 1U
#define SERVO_POSITION_MAX 7U
#define SERVO_MIN_PULSE_US 1000U
#define SERVO_MAX_PULSE_US 2000U
#define SERVO_CONTROL_TASK_PERIOD_MS 20U

static uint32_t servo_position_to_pulse_width_us(uint8_t position)
{
    const uint32_t position_span = SERVO_POSITION_MAX - SERVO_POSITION_MIN;
    const uint32_t pulse_span_us = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    const uint32_t position_index = position - SERVO_POSITION_MIN;

    return SERVO_MIN_PULSE_US + ((pulse_span_us * position_index) + (position_span / 2U)) / position_span;
}

static uint32_t servo_pulse_width_to_duty(uint32_t pulse_width_us)
{
    return (pulse_width_us * SERVO_PWM_DUTY_SCALE) / SERVO_PWM_PERIOD_US;
}

static esp_err_t servo_apply_position(uint8_t position)
{
    if ((position < SERVO_POSITION_MIN) || (position > SERVO_POSITION_MAX)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t pulse_width_us = servo_position_to_pulse_width_us(position);
    const uint32_t duty = servo_pulse_width_to_duty(pulse_width_us);

    ESP_RETURN_ON_ERROR(ledc_set_duty(SERVO_PWM_SPEED_MODE, SERVO_PWM_CHANNEL, duty), TAG, "failed to set PWM duty");
    return ledc_update_duty(SERVO_PWM_SPEED_MODE, SERVO_PWM_CHANNEL);
}

static void initialize_servo_pwm(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = SERVO_PWM_SPEED_MODE,
        .duty_resolution = SERVO_PWM_DUTY_RES,
        .timer_num = SERVO_PWM_TIMER,
        .freq_hz = SERVO_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    const ledc_channel_config_t channel_config = {
        .gpio_num = SERVO_PWM_GPIO,
        .speed_mode = SERVO_PWM_SPEED_MODE,
        .channel = SERVO_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = SERVO_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    ESP_LOGI(TAG, "Servo PWM initialized on GPIO %d at %u Hz", SERVO_PWM_GPIO, SERVO_PWM_FREQUENCY_HZ);
}

static bool can_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    (void)user_ctx;

    uint8_t rx_data[TWAI_FRAME_MAX_LEN] = {0};
    twai_frame_t rx_frame = {
        .buffer = rx_data,
        .buffer_len = sizeof(rx_data),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        return false;
    }

    if (rx_frame.header.ide || rx_frame.header.rtr || rx_frame.header.fdf) {
        return false;
    }

    if ((rx_frame.header.id != SERVO_POSITION_CAN_ID) || (rx_frame.header.dlc < 1U)) {
        return false;
    }

    if ((rx_data[0] < SERVO_POSITION_MIN) || (rx_data[0] > SERVO_POSITION_MAX)) {
        return false;
    }

    s_requested_position = rx_data[0];
    return false;
}

static void initialize_can(void)
{
    const twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN1_TX,
            .rx = CAN1_RX,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN1_BAUD,
        },
        .fail_retry_cnt = -1,
        .tx_queue_depth = 5,
        .flags = {
            .no_receive_rtr = 1,
        },
    };

    const twai_event_callbacks_t callbacks = {
        .on_rx_done = can_rx_done_callback,
    };

    const twai_mask_filter_config_t position_filter = {
        .id = SERVO_POSITION_CAN_ID,
        .mask = TWAI_STD_ID_MASK,
        .is_ext = 0,
        .no_classic = 0,
        .no_fd = 1,
        .dual_filter = 0,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_can_node));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_can_node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_config_mask_filter(s_can_node, 0, &position_filter));
    ESP_ERROR_CHECK(twai_node_enable(s_can_node));

    ESP_LOGI(TAG, "CAN initialized on TX=%d, RX=%d @ %u bps, accepting ID 0x%03lX",
             CAN1_TX, CAN1_RX, CAN1_BAUD, (unsigned long)SERVO_POSITION_CAN_ID);
}

static void servo_control_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t last_applied_position = 0;

    while (true) {
        const uint8_t requested_position = s_requested_position;

        if ((requested_position >= SERVO_POSITION_MIN) &&
            (requested_position <= SERVO_POSITION_MAX) &&
            (requested_position != last_applied_position)) {
            esp_err_t ret = servo_apply_position(requested_position);
            if (ret == ESP_OK) {
                last_applied_position = requested_position;
                ESP_LOGI(TAG, "Applied servo position %u", requested_position);
            } else {
                ESP_LOGE(TAG, "Failed to apply servo position %u: %s", requested_position, esp_err_to_name(ret));
            }
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SERVO_CONTROL_TASK_PERIOD_MS));
    }
}

void app_main(void)
{
    initialize_servo_pwm();

    if (xTaskCreate(servo_control_task, "servo_control", 3072, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create servo control task");
        return;
    }

    initialize_can();
}
