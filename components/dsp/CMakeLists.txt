idf_component_register(
    SRCS "src/biquad.cpp" "src/iir_filter_chain.cpp" "src/dsp_task.cpp"
    INCLUDE_DIRS "include"
    REQUIRES eeg_core freertos
)
