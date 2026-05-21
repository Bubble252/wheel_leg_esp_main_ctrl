idf_component_register(
    SRCS
        "src/balance_roll.c"
        "src/balance_jump.c"
        "src/balance_standup.c"
        "src/balance_observer.c"
        "src/balance_vmc.c"
    INCLUDE_DIRS "include"
    REQUIRES algorithm device
)
