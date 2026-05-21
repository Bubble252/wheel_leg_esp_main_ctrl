idf_component_register(
    SRCS
        "src/app_main.c"
        "src/motor_test.c"
        "src/imu_test.c"
        "src/sensor_test.c"
        "src/balance_test.c"
        "src/balance_cli.c"
        "src/balance_plot.c"
        "src/balance_leg.c"
        "src/balance_control.c"
        "src/balance_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos common device control algorithm esp_timer console wit_imu wifi_remote pi_comm esp_driver_usb_serial_jtag vfs balance
)
