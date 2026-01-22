idf_component_register(
    SRCS 
        "src/app_main.c"
        "src/motor_test.c"
        "src/imu_test.c"
        "src/sensor_test.c"
        "src/balance_test.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos common device control algorithm esp_timer console wit_imu wifi_remote
)
