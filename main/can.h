#ifndef ARB_CAN_H
#define ARB_CAN_H

#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define ESP_ERR_CAN_BASE                        0xE100
#define ESP_ERR_CAN_FAIL_EN                     ESP_ERR_CAN_BASE
#define ESP_ERR_CAN_FAIL_TX                     (ESP_ERR_CAN_BASE + 1)
#define ESP_ERR_CAN_FAIL_RX_INIT                (ESP_ERR_CAN_BASE + 2)
#define ESP_ERR_CAN_BUS_RECOVERING              (ESP_ERR_CAN_BASE + 3)


#define TERMINATION_GPIO    GPIO_NUM_10
#define CAN_TX_PIN          GPIO_NUM_4
#define CAN_RX_PIN          GPIO_NUM_6
#define CAN_BITRATE         1000000U     // 1Mbps
#define CAN_QUEUE_SIZE      16
#define TRANSMIT_TIMEOUT     0
#define STEERING_WHEEL_ID   0x3b0
#define ARB_ID   0x3c0

typedef bool (*can_message_callback_t)(
    twai_frame_t *msg,
    void *user
);
typedef struct {
    int frequency;
    twai_frame_t can_msg;
    can_message_callback_t callback_function;
    void *user_ctx;
} CANTaskArgs;

twai_node_status_t get_can_status();
twai_node_record_t get_can_record();
void CAN_set_rx_queue(QueueHandle_t q);
void enable_can_debug();
esp_err_t CAN_init();
esp_err_t CAN_transmit(twai_frame_t data);
void can_tx_task(void *arg);
void can_rx_task(void *args);
#endif