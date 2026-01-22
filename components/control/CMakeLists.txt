idf_component_register(
    SRCS 
        "src/state_machine.c"
        "src/robot_controller.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos common device algorithm esp_timer
)
