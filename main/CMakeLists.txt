idf_component_register(SRCS "main.c" "servo.c" "can.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer esp_driver_twai esp_driver_gpio esp_driver_ledc freertos nvs_flash)
