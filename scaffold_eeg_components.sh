#!/usr/bin/env bash
#
# scaffold_eeg_components.sh
#
# Creates the components/ tree (eeg_core, eeg_manager, dsp, ml_engine,
# telemetry, control_plane, app) plus a host-side test/ tree, matching the
# EEG firmware architecture plan. The ads1299 driver itself is left alone —
# it's already pulled in as managed_components/carlos-lorenzo__ads1299.
#
# Safe to re-run: existing files are left untouched unless --force is passed.
# Run from the repo root (same directory as the top-level CMakeLists.txt).

set -euo pipefail

FORCE=0
if [[ "${1:-}" == "--force" ]]; then
    FORCE=1
fi

REPO_ROOT="$(pwd)"

if [[ ! -f "${REPO_ROOT}/CMakeLists.txt" || ! -d "${REPO_ROOT}/main" ]]; then
    echo "error: run this from the ESP-IDF project root (expected ./CMakeLists.txt and ./main)" >&2
    exit 1
fi

COMPONENTS_DIR="${REPO_ROOT}/components"
TEST_DIR="${REPO_ROOT}/test"

write_file() {
    # write_file <path> <<'EOF' ... EOF
    local path="$1"
    if [[ -e "$path" && "$FORCE" -ne 1 ]]; then
        echo "  skip (exists): ${path#$REPO_ROOT/}"
        cat >/dev/null
        return
    fi
    mkdir -p "$(dirname "$path")"
    cat > "$path"
    echo "  wrote:         ${path#$REPO_ROOT/}"
}

header_stub() {
    # header_stub <path> <one-line description>
    local path="$1" note="$2"
    write_file "$path" <<EOF
#pragma once

// ${note}
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

namespace eeg {

} // namespace eeg
EOF
}

src_stub() {
    # src_stub <path> <matching-header-include>
    local path="$1" inc="$2"
    write_file "$path" <<EOF
#include "${inc}"

namespace eeg {

// TODO: implement per the architecture plan.

} // namespace eeg
EOF
}

component_cmake() {
    # component_cmake <path> <srcs...> -- <requires...>
    local path="$1"; shift
    local srcs=() requires=()
    local mode="srcs"
    for arg in "$@"; do
        if [[ "$arg" == "--" ]]; then mode="requires"; continue; fi
        if [[ "$mode" == "srcs" ]]; then srcs+=("\"$arg\""); else requires+=("$arg"); fi
    done
    local srcs_str requires_str
    srcs_str=$(IFS=' '; echo "${srcs[*]:-}")
    requires_str=$(IFS=' '; echo "${requires[*]:-}")
    write_file "$path" <<EOF
idf_component_register(
    SRCS ${srcs_str}
    INCLUDE_DIRS "include"
    REQUIRES ${requires_str}
)
EOF
}

echo "== eeg_core =="
header_stub "${COMPONENTS_DIR}/eeg_core/include/eeg_core/status.hpp"          "eeg::Status - non-throwing esp_err_t wrapper"
header_stub "${COMPONENTS_DIR}/eeg_core/include/eeg_core/eeg_types.hpp"       "FrameHeader / FrameView / PacketType / ContaminationFlags"
header_stub "${COMPONENTS_DIR}/eeg_core/include/eeg_core/frame_pool.hpp"      "eeg::FramePool - ref-counted fan-out frame slots"
header_stub "${COMPONENTS_DIR}/eeg_core/include/eeg_core/i_stream_source.hpp" "eeg::IStreamSource - pull-based consumer interface"
src_stub    "${COMPONENTS_DIR}/eeg_core/src/status.cpp"     "eeg_core/status.hpp"
src_stub    "${COMPONENTS_DIR}/eeg_core/src/frame_pool.cpp" "eeg_core/frame_pool.hpp"
component_cmake "${COMPONENTS_DIR}/eeg_core/CMakeLists.txt" \
    src/status.cpp src/frame_pool.cpp \
    -- carlos-lorenzo__ads1299 freertos

echo "== eeg_manager =="
header_stub "${COMPONENTS_DIR}/eeg_manager/include/eeg_manager/eeg_manager.hpp" "eeg::EEGManager - C++ facade over the ads1299 driver"
src_stub    "${COMPONENTS_DIR}/eeg_manager/src/eeg_manager.cpp" "eeg_manager/eeg_manager.hpp"
component_cmake "${COMPONENTS_DIR}/eeg_manager/CMakeLists.txt" \
    src/eeg_manager.cpp \
    -- eeg_core carlos-lorenzo__ads1299 esp_driver_gpio esp_driver_spi esp_timer freertos

echo "== dsp =="
header_stub "${COMPONENTS_DIR}/dsp/include/dsp/biquad.hpp"           "eeg::dsp::BiquadCascade / DspConfig"
header_stub "${COMPONENTS_DIR}/dsp/include/dsp/iir_filter_chain.hpp" "eeg::dsp::IirFilterChain"
header_stub "${COMPONENTS_DIR}/dsp/include/dsp/dsp_task.hpp"         "eeg::DSPTask - filters raw frames, feeds ML + filtered IStreamSource"
src_stub    "${COMPONENTS_DIR}/dsp/src/biquad.cpp"           "dsp/biquad.hpp"
src_stub    "${COMPONENTS_DIR}/dsp/src/iir_filter_chain.cpp" "dsp/iir_filter_chain.hpp"
src_stub    "${COMPONENTS_DIR}/dsp/src/dsp_task.cpp"         "dsp/dsp_task.hpp"
component_cmake "${COMPONENTS_DIR}/dsp/CMakeLists.txt" \
    src/biquad.cpp src/iir_filter_chain.cpp src/dsp_task.cpp \
    -- eeg_core freertos

echo "== ml_engine =="
header_stub "${COMPONENTS_DIR}/ml_engine/include/ml_engine/i_model.hpp"            "eeg::IModel - inference interface"
header_stub "${COMPONENTS_DIR}/ml_engine/include/ml_engine/window_accumulator.hpp" "eeg::WindowAccumulator - sliding window feeding a model"
src_stub    "${COMPONENTS_DIR}/ml_engine/src/window_accumulator.cpp" "ml_engine/window_accumulator.hpp"
component_cmake "${COMPONENTS_DIR}/ml_engine/CMakeLists.txt" \
    src/window_accumulator.cpp \
    -- eeg_core freertos

echo "== telemetry =="
header_stub "${COMPONENTS_DIR}/telemetry/include/telemetry/i_transport.hpp"        "eeg::ITransport - send-only transport interface"
header_stub "${COMPONENTS_DIR}/telemetry/include/telemetry/wire_protocol.hpp"      "eeg::WireSampleHeader / WireSample - packed wire structs"
header_stub "${COMPONENTS_DIR}/telemetry/include/telemetry/usb_jtag_transport.hpp" "eeg::UsbJtagTransport"
header_stub "${COMPONENTS_DIR}/telemetry/include/telemetry/udp_transport.hpp"      "eeg::UdpTransport"
header_stub "${COMPONENTS_DIR}/telemetry/include/telemetry/eeg_streamer.hpp"       "eeg::EEGStreamer - packs+sends frames from one active IStreamSource"
src_stub    "${COMPONENTS_DIR}/telemetry/src/usb_jtag_transport.cpp" "telemetry/usb_jtag_transport.hpp"
src_stub    "${COMPONENTS_DIR}/telemetry/src/udp_transport.cpp"      "telemetry/udp_transport.hpp"
src_stub    "${COMPONENTS_DIR}/telemetry/src/eeg_streamer.cpp"       "telemetry/eeg_streamer.hpp"
component_cmake "${COMPONENTS_DIR}/telemetry/CMakeLists.txt" \
    src/usb_jtag_transport.cpp src/udp_transport.cpp src/eeg_streamer.cpp \
    -- eeg_core esp_driver_usb_serial_jtag lwip freertos

echo "== control_plane =="
header_stub "${COMPONENTS_DIR}/control_plane/include/control_plane/command_types.hpp"  "eeg::CommandId / Command / CommandResult"
header_stub "${COMPONENTS_DIR}/control_plane/include/control_plane/control_server.hpp" "eeg::ControlServer - isolated TCP command server"
src_stub    "${COMPONENTS_DIR}/control_plane/src/control_server.cpp" "control_plane/control_server.hpp"
component_cmake "${COMPONENTS_DIR}/control_plane/CMakeLists.txt" \
    src/control_server.cpp \
    -- eeg_core eeg_manager dsp telemetry lwip freertos

echo "== app =="
header_stub "${COMPONENTS_DIR}/app/include/app/app_context.hpp" "eeg::AppContext - owns construction/teardown order for everything above"
src_stub    "${COMPONENTS_DIR}/app/src/app_context.cpp" "app/app_context.hpp"
component_cmake "${COMPONENTS_DIR}/app/CMakeLists.txt" \
    src/app_context.cpp \
    -- eeg_core eeg_manager dsp ml_engine telemetry control_plane

echo "== test (host-side, not an ESP-IDF component) =="
write_file "${TEST_DIR}/CMakeLists.txt" <<'EOF'
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
EOF

for t in test_frame_pool test_biquad test_window_accumulator; do
    write_file "${TEST_DIR}/${t}.cpp" <<EOF
#include <catch2/catch_test_macros.hpp>

// TODO: implement per the architecture plan (Part 6, Phase 1/4/5).
TEST_CASE("${t} placeholder", "[placeholder]") {
    REQUIRE(true);
}
EOF
done

echo
echo "Done."
echo
echo "components/ and test/ created. Two things left to do by hand:"
echo
echo "1. main/CMakeLists.txt doesn't depend on the new tree yet."
echo "   Add 'app' to its REQUIRES (the rest comes in transitively):"
echo
echo "     idf_component_register("
echo "         SRCS \"main.cpp\""
echo "         INCLUDE_DIRS \".\""
echo "         REQUIRES carlos-lorenzo__ads1299 esp_driver_spi esp_driver_gpio \\"
echo "                  esp_ringbuf esp_timer esp_wifi nvs_flash app"
echo "     )"
echo
echo "2. Top-level ./include and ./src already exist and aren't referenced by"
echo "   main/CMakeLists.txt's INCLUDE_DIRS/SRCS — this script left them"
echo "   untouched. Worth confirming what they're for before main.cpp starts"
echo "   pulling in app_context.hpp, in case they overlap or should be merged."
