#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_twai.h"
#include "can.h"
#include "servo.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "MAIN";

uint8_t REQ_F_ARB_POS;
uint8_t REQ_R_ARB_POS;

static uint8_t can_data[2];

void save_pos(const char *key, uint8_t value){
    nvs_handle_t handle;
    if (nvs_open("storage", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, key, value);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

bool process_can_message(twai_frame_t *msg, void *user){
    uint8_t PREV_REQ_F_ARB_POS = REQ_F_ARB_POS;
    uint8_t PREV_REQ_R_ARB_POS = REQ_R_ARB_POS;
    if (msg->header.id == STEERING_WHEEL_ID) {
        REQ_F_ARB_POS = msg->buffer[1];
        REQ_R_ARB_POS = msg->buffer[2];
        if(PREV_REQ_F_ARB_POS != REQ_F_ARB_POS) save_pos("f_pos", REQ_F_ARB_POS);
        if(PREV_REQ_R_ARB_POS != REQ_R_ARB_POS) save_pos("r_pos", REQ_R_ARB_POS);
    }
    return true;
}

uint8_t load_pos(const char *key, uint8_t def){
    nvs_handle_t handle;
    uint8_t val = def;

    if (nvs_open("storage", NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, key, &val);
        nvs_close(handle);
    }
    return val;
}

void app_main(void){
    ESP_ERROR_CHECK(nvs_flash_init());
    while(CAN_init() != ESP_OK){
        ESP_LOGW(TAG, "Retrying CAN init");
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    CANTaskArgs *can_args = malloc(sizeof(CANTaskArgs));
    twai_frame_t can_msg = {
        .header = {
            .id = ARB_ID,
            .dlc = 2,
            .ide = 0,
            .rtr = 0,
            .fdf = 0,
            .brs = 0,
            .esi = 0,
        },
        .buffer = can_data,
        .buffer_len = 2
    };
    
    can_args->can_msg = can_msg;
    can_args->frequency = 10;
    can_args->callback_function = process_can_message;
    can_args->user_ctx = NULL;
    
    initialize_servo_timer(LEDC_TIMER_0);
    
    ServoTaskArgs *f_servo_args = malloc(sizeof(ServoTaskArgs));
    f_servo_args->frequency = 100;
    f_servo_args->gpio = GPIO_NUM_14;
    f_servo_args->REQ_ARB_POS = &REQ_F_ARB_POS;
    f_servo_args->led_channel = LEDC_CHANNEL_0;
    f_servo_args->led_timer = LEDC_TIMER_0;
    f_servo_args->max_pos = 15;
    
    ServoTaskArgs *r_servo_args = malloc(sizeof(ServoTaskArgs));
    r_servo_args->frequency = 100;
    r_servo_args->gpio = GPIO_NUM_15;
    r_servo_args->REQ_ARB_POS = &REQ_R_ARB_POS;
    r_servo_args->led_channel = LEDC_CHANNEL_1;
    r_servo_args->led_timer = LEDC_TIMER_0;
    r_servo_args->max_pos = 15;
    
    xTaskCreate(can_tx_task, "can_tx_task", 4096, can_args, 10, NULL);
    xTaskCreate(servo_task, "f_arb_task", 4096, f_servo_args, 10, NULL);
    xTaskCreate(servo_task, "r_arb_task", 4096, r_servo_args, 10, NULL);
    
    REQ_F_ARB_POS = load_pos("f_pos", f_servo_args->max_pos);
    REQ_R_ARB_POS = load_pos("r_pos", r_servo_args->max_pos);
    ESP_LOGI(TAG, "Attemped pos load %d %d", REQ_F_ARB_POS, REQ_F_ARB_POS);
    
    for(int i = 0;;i++){
        // Motor test code
        // REQ_F_ARB_POS   =  i % f_servo_args->max_pos + 1;
        // REQ_R_ARB_POS   =  i % r_servo_args->max_pos + 1;
        // save_pos("f_pos", REQ_F_ARB_POS);
        // save_pos("r_pos", REQ_R_ARB_POS);
        // ESP_LOGI(TAG, "Incrementing servos %d", REQ_F_ARB_POS);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}