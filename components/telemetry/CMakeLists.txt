idf_component_register(
    SRCS "src/usb_jtag_transport.cpp" "src/udp_transport.cpp" "src/eeg_streamer.cpp"
    INCLUDE_DIRS "include"
    REQUIRES eeg_core esp_driver_usb_serial_jtag lwip freertos
)
