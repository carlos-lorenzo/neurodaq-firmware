idf_component_register(
    SRCS "ads1299.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_spi esp_driver_gpio esp_ringbuf esp_timer
)
