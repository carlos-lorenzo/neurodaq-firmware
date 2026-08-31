#pragma once

// FrameHeader / FrameView / PacketType / ContaminationFlags
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include <cstdint>
#include <array>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>

#include "ads1299.h"
#include "ads1299_defs.h"

namespace eeg {
    // typedef ads1299_chunk_t EEGFrame;

    // struct EEGFrame {
    //     ads1299_chunk_t chunk{};
    //     EEGFrame() = default;
    //     explicit EEGFrame(const ads1299_chunk_t& chunk) : chunk(chunk) {}
    // };

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

        DeviceConfig(ads1299_sample_rate_t sample_rate, ads1299_pga_gain_t global_gain, std::uint8_t pd_mask, std::uint8_t bias_mask, bool use_srb1) :
            sample_rate(sample_rate),
            channel_pd_mask(pd_mask),
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


    enum class TransportType {
        UDP,
        USB,
    };

    struct EEGFrame {
        std::array<ads1299_sample_t, 25> samples;
        std::size_t n_samples;
        int64_t first_timestamp_us;
        int64_t last_timestamp_us;
        int64_t dropped_count;
        int64_t overflow_count;

        EEGFrame() : n_samples(98), first_timestamp_us(123), last_timestamp_us(574), dropped_count(0), overflow_count(0) {}

        EEGFrame(
            const ads1299_sample_t* src,
            std::size_t count,
            int64_t first_timestamp,
            int64_t last_timestamp,
            int64_t dropped,
            int64_t overflow
        )
            : n_samples(count),
              first_timestamp_us(first_timestamp),
              last_timestamp_us(last_timestamp),
              dropped_count(dropped),
              overflow_count(overflow)
        {
            std::copy_n(src, count, samples.begin());
        }
    };


    struct __attribute__((packed)) TelemetryHeader {
        uint32_t magic_header;    // MAGIC_HEADER = 0x21474545 (ASCII for "EEG!" - little-endian) :)
        uint32_t sequence_number;

        TelemetryHeader()
            : magic_header(0x21474545), sequence_number(0) {}

        TelemetryHeader(const uint32_t seq_num)
            : magic_header(0x21474545), sequence_number(seq_num) {}
    };


} // namespace eeg
