#pragma once

// eeg::EEGManager - C++ facade over the ads1299 driver
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include "ads1299.h"
#include "ads1299_defs.h"

namespace eeg {
    class EEGManager {
    public:
        struct PinConfig {
            gpio_num_t drdy_pin;
            gpio_num_t miso_pin;
            gpio_num_t sclk_pin;
            gpio_num_t cs_pin;
            gpio_num_t start_pin;
            gpio_num_t reset_pin;
            gpio_num_t mosi_pin;
            gpio_num_t analog_ldo_enable_pin;
        };

        struct DeviceConfig {
            ads1299_sample_rate_t sample_rate;
            // Add other device configuration parameters as needed
        };

        explicit EEGManager(PinConfig pin_config, DeviceConfig device_config); // Will also take a reference to an app-owned frame pool
        ~EEGManager();

        // Owned by app context, so we delete copy/move constructors and assignment operators to prevent accidental copies
        EEGManager(const EEGManager&) = delete;
        EEGManager& operator=(const EEGManager&) = delete;
        EEGManager(EEGManager&&) = delete;
        EEGManager& operator=(EEGManager&&) = delete;

        void start_acquisition();
        void stop_acquisition();

    private:
        const PinConfig pin_config_;
        DeviceConfig device_config_; // The TCP controller will be allowed to change this at runtime, which will trigger reconfiguration of the device through register writes
        //FramePool& frame_pool_; // Reference to the app-owned frame pool, not yet defined
        ads1299_t dev_;
        constexpr static auto TAG = "EEGManager";

        static void chunk_callback(const ads1299_chunk_t* chunk, void* ctx);
        void on_chunk(const ads1299_chunk_t* chunk);


    };
} // namespace eeg


