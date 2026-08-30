#pragma once

// FrameHeader / FrameView / PacketType / ContaminationFlags
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include "ads1299.h"
#include "ads1299_defs.h"

namespace eeg {
    typedef ads1299_chunk_t FrameType;

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

        std::uint8_t channel_pd_mask; // Bitmask for choosing if channels are powered down (bit 0 = CH1, bit 1 = CH2, ..., bit 7 = CH8)
        std::uint8_t channel_bias_sensp_mask; // Bitmask for choosing if channels are connected to the bias sense positive input (bit 0 = CH1, bit 1 = CH2, ..., bit 7 = CH8)

        std::array<ads1299_pga_gain_t, 8> channel_gains; // Array of gains for each channel (CH1 to CH8)
        bool use_srb1;

        DeviceConfig() :
            sample_rate(ADS1299_DR_250SPS),
              channel_pd_mask(0xFF), // All channels powered up by default
              channel_bias_sensp_mask(0x00), // No channels connected to bias sense positive input by default
              channel_gains({
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1,
                  ADS1299_PGA_GAIN_1}
                  ),
              use_srb1(true) {}

        DeviceConfig(ads1299_sample_rate_t sample_rate, ads1299_pga_gain_t global_gain, std::uint8_t bias_mask, bool use_srb1) :
            sample_rate(sample_rate),
            channel_pd_mask(0xFF),
            channel_bias_sensp_mask(bias_mask),
            channel_gains({
                global_gain,
                global_gain,
                global_gain,
                global_gain,
                global_gain,
                global_gain,
                global_gain,
                global_gain}
                ),
            use_srb1(use_srb1) {}
        };

} // namespace eeg
