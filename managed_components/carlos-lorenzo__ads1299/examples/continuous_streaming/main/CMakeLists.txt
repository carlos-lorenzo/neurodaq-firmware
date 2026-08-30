idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES ads1299-esp esp_driver_spi esp_driver_gpio esp_ringbuf esp_timer esp_driver_uart esp_driver_usb_serial_jtag
)
