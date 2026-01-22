idf_component_register(
    SRCS "hello_world_main.c"
    INCLUDE_DIRS ""
    PRIV_REQUIRES spi_flash common device algorithm control app
)
