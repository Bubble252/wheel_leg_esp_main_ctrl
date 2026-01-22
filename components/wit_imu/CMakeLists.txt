idf_component_register(
    SRCS 
        "src/wit_c_sdk.c"
        "src/wit_imu.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos common esp_timer
)
