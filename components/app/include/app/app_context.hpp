#pragma once
#include "eeg_manager/eeg_manager.hpp"
#include "eeg_core/frame_pool.hpp"
#include "eeg_core/eeg_types.hpp"

// eeg::AppContext - owns construction/teardown order for everything above
// See: EEG firmware architecture plan, Part 4.


namespace eeg {
    constexpr std::size_t n_consumers = 2; // TODO: make this configurable at compile time or runtime
    constexpr std::size_t frame_pool_capacity = 10; // TODO: make this configurable at compile time or runtime
    class AppContext {
    public:
        AppContext(const eeg::PinConfig &pin_config, const eeg::DeviceConfig &device_config);
        // ~AppContext();


    private:
        FramePool<eeg::FrameType, frame_pool_capacity, n_consumers> raw_pool_{};
        FramePool<eeg::FrameType, frame_pool_capacity, n_consumers> filtered_pool_{};
        std::array<QueueHandle_t, n_consumers> raw_frame_queue_; // Array of queues for each consumer
        std::array<QueueHandle_t, n_consumers> filtered_frame_queue_; // Array of queues for each consumer
        EEGManager<frame_pool_capacity, n_consumers> eeg_manager_;

    };
} // namespace eeg
