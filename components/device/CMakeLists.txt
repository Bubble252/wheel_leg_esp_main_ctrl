idf_component_register(
    SRCS 
        "src/can_motor.c"
        "src/imu_driver.c"
        "src/button_driver.c"
        "src/power_detect.c"
        "src/sht30_driver.c"
        "src/commander_parser.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos common esp_timer wit_imu
)
