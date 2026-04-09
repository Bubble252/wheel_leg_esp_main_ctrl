idf_component_register(
    SRCS 
        "src/pid_controller.c"
        "src/balance_algorithm.c"
        "src/lowpass_filter.c"
        "src/lqr_balance.c"
        "src/full_lqr.c"
        "src/leg_kinematics.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos common
    PRIV_REQUIRES esp_timer device
)
