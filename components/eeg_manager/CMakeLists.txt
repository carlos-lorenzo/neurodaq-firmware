idf_component_register(
    SRCS "src/eeg_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES eeg_core carlos-lorenzo__ads1299 esp_driver_gpio esp_driver_spi esp_timer freertos
)
