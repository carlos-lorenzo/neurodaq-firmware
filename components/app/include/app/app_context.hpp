#pragma once

// eeg::AppContext — the ownership root. Constructs and owns the frame pools, queues,
// EEGManager, EEGStreamer and ControlServer by value; every other component holds
// references into it. Member init order matters: pools and queues must exist before the
// manager and streamer that reference them. Heap-allocated once in main.cpp and never
// freed (see main.cpp).

#include <cstdint>
#include <memory>
#include <array>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "eeg_manager/eeg_manager.hpp"
#include "eeg_core/frame_pool.hpp"
#include "eeg_core/eeg_types.hpp"
#include "telemetry/eeg_streamer.hpp"
#include "control_plane/control_server.hpp"

namespace eeg {

    constexpr std::size_t n_consumers = 1;
    constexpr std::size_t control_queue_capacity = 10;
    constexpr std::size_t frame_pool_capacity = 10;

    class AppContext {
    public:
        AppContext(
            const eeg::PinConfig& pin_config,
            const eeg::DeviceConfig& device_config
        );

        ~AppContext() = default;

        AppContext(const AppContext&) = delete;
        AppContext& operator=(const AppContext&) = delete;
        AppContext(AppContext&&) = delete;
        AppContext& operator=(AppContext&&) = delete;

    private:
        constexpr static auto TAG = "AppContext";

        // -------------------------------------------------------------
        // These are owned by AppContext.
        // -------------------------------------------------------------

        FramePool<
            eeg::EEGFrame,
            frame_pool_capacity,
            n_consumers
        > raw_pool_{};

        FramePool<
            eeg::EEGFrame,
            frame_pool_capacity,
            n_consumers
        > filtered_pool_{};

        std::array<QueueHandle_t, n_consumers> raw_frame_queue_{};
        std::array<QueueHandle_t, n_consumers> filtered_frame_queue_{};

        QueueHandle_t control_command_queue_{nullptr};
        QueueHandle_t control_response_queue_{nullptr};

        // -------------------------------------------------------------
        // These depend on the resources above.
        // -------------------------------------------------------------

        EEGManager<
            frame_pool_capacity,
            n_consumers
        > eeg_manager_;

        EEGStreamer<
            TransportType::UDP,
            frame_pool_capacity,
            n_consumers
        > eeg_streamer_;

        std::unique_ptr<ControlServer> control_server_;
    };

} // namespace eeg
