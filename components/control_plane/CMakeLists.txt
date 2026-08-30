idf_component_register(
    SRCS "src/control_server.cpp"
    INCLUDE_DIRS "include"
    REQUIRES eeg_core eeg_manager dsp telemetry lwip freertos
)
