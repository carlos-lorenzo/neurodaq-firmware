#include "app/app_context.hpp"

namespace eeg {

// TODO: implement per the architecture plan.






AppContext::AppContext(
    const eeg::PinConfig &pin_config,
    const eeg::DeviceConfig &device_config
    ) :
    raw_frame_queue_(),
    filtered_frame_queue_(),
    eeg_manager_(pin_config, device_config, raw_pool_, raw_frame_queue_)
{
    for (auto i=0; i < n_consumers; ++i) {
        raw_frame_queue_[i] = xQueueCreate(frame_pool_capacity, sizeof(eeg::FrameType*));
        filtered_frame_queue_[i] = xQueueCreate(frame_pool_capacity, sizeof(eeg::FrameType*));
    }
    // Initialize EEGManager with provided pin and device configurations
    eeg_manager_.start_acquisition();
}


} // namespace eeg
