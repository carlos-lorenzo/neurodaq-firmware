idf_component_register(
    SRCS "src/app_context.cpp"
    INCLUDE_DIRS "include"
    REQUIRES eeg_core eeg_manager dsp ml_engine telemetry control_plane esp_timer telemetry
)
