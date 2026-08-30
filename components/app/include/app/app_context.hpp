#pragma once
#include "eeg_manager/eeg_manager.hpp"

// eeg::AppContext - owns construction/teardown order for everything above
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

namespace eeg {
    class AppContext {
    public:
        AppContext(EEGManager::PinConfig pin_config, EEGManager::DeviceConfig device_config);
        // ~AppContext();

    private:
        EEGManager eeg_manager_;

    };
} // namespace eeg
