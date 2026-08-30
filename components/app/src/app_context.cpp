#include "app/app_context.hpp"

namespace eeg {

// TODO: implement per the architecture plan.



AppContext::AppContext(EEGManager::PinConfig pin_config, EEGManager::DeviceConfig device_config) : eeg_manager_(pin_config, device_config) {
    // Initialize EEGManager with provided pin and device configurations

}
} // namespace eeg
