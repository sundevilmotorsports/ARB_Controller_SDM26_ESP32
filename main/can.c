#include "can.h"

static const char *TAG = "CAN";

static twai_node_handle_t can_handle = NULL;
static twai_node_status_t node_status;
static twai_node_record_t node_record;

static bool CAN_DEBUG = false;
static CANTaskArgs *g_args = NULL;

static bool rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user){
    CANTaskArgs *args = (CANTaskArgs *)user;
    if (!args || !args->callback_function) return false;

    twai_frame_t msg;
    memcpy(&msg, edata, sizeof(twai_frame_t));
    return args->callback_function(&msg, args->user_ctx);
}

twai_node_status_t get_can_status() {return node_status;}
twai_node_record_t get_can_record() {return node_record;}
void enable_can_debug(){CAN_DEBUG = true;}

esp_err_t CAN_init(){
    gpio_reset_pin(TERMINATION_GPIO);
    gpio_set_direction(TERMINATION_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TERMINATION_GPIO, 0);

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = {
            .bitrate = CAN_BITRATE,
        },
        .tx_queue_depth = CAN_QUEUE_SIZE,
        .flags.enable_self_test = 0,
        .flags.enable_loopback = 0,
    };

    esp_err_t ret = twai_new_node_onchip(&node_config, &can_handle);
    if (ret != ESP_OK) return ret;

    twai_event_callbacks_t cbs = {.on_rx_done = rx_done_cb};

    ret = twai_node_register_event_callbacks(can_handle, &cbs, NULL);
    if (ret != ESP_OK) return ret;

    ret = twai_node_enable(can_handle);
    if (ret != ESP_OK) return ESP_ERR_CAN_FAIL_EN;

    twai_node_get_info(can_handle, &node_status, &node_record);

    return ESP_OK;
}

esp_err_t CAN_transmit(twai_frame_t data){
    twai_node_get_info(can_handle, &node_status, &node_record);
    
    if (node_status.state != TWAI_ERROR_ACTIVE) {
        if (CAN_DEBUG) ESP_LOGW(TAG, "Recovering CAN bus...");
        twai_node_recover(can_handle);
        return ESP_ERR_CAN_BUS_RECOVERING;
    }

    esp_err_t ret = twai_node_transmit(can_handle, &data, TRANSMIT_TIMEOUT);

    if (ret != ESP_OK) {
        if (CAN_DEBUG)
            ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(ret));
        return ESP_ERR_CAN_FAIL_TX;
    }

    return ESP_OK;
}


void can_tx_task(void *arg){
    TickType_t last_wake = xTaskGetTickCount();
    CANTaskArgs *args = (CANTaskArgs *)arg;

    uint32_t hz = args->frequency;
    twai_frame_t msg = args->can_msg;

    for (;;) {
        CAN_transmit(msg);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / hz));
    }
}