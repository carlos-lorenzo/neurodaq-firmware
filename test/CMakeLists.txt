# Native host build (plain gcc/clang) for the logic that doesn't need
# FreeRTOS/ESP-IDF: FramePool bookkeeping, filter math, window accumulation.
# Not part of the ESP-IDF build; build/run separately, e.g.:
#   cmake -S test -B test/build && cmake --build test/build && ctest --test-dir test/build
cmake_minimum_required(VERSION 3.16)
project(eeg_firmware_tests CXX)

include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.4
)
FetchContent_MakeAvailable(Catch2)

add_executable(eeg_firmware_tests
    test_frame_pool.cpp
    test_biquad.cpp
    test_window_accumulator.cpp
)
target_link_libraries(eeg_firmware_tests PRIVATE Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(eeg_firmware_tests)
