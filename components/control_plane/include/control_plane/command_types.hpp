#pragma once

// eeg::CommandId / Command / CommandResult
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include <cstdint>
#include <string>
#include <variant>

#include "ads1299.h"
#include "ads1299_defs.h"

namespace eeg {
    struct CommandStart {};
    struct CommandStop {};
    struct CommandReset {};
    struct CommandStandBy {};
    struct CommandWakeUp {};

    struct CommandReadRegister {
        uint8_t reg_address;
    };
    struct CommandWriteRegister {
        uint8_t reg_address;
        uint8_t value;
    };


    struct CommandConfigGlobal {
        uint8_t sample_rate; // 0-6, corresponding to 16kSPS, 8kSPS, 4kSPS, 2kSPS, 1kSPS, 500SPS, 250SPS
        bool srb1_enabled;
        bool srb2_enabled;
    };

    struct CommandConfigLeadOff {
        bool lead_off_enabled; // Also powers up/down the loff comparator
        uint8_t lead_off_threshold; // 0-7, 95%, 92.5%, 90%, 87.5%, 85%, 80%, 75%, 70%
        uint8_t lead_off_current; // 0-3, corresponding to 6nA, 24nA, 6uA, 24uA
        uint8_t lead_off_frequency; // 0-3, corresponding to DC, 7.8Hz, 31.2Hz, fDR / 4
        uint8_t loff_sensp; // bitmask for channels 1-8, 1 = enabled, 0 = disabled
        uint8_t loff_sensn; // bitmask for channels 1-8, 1 = enabled, 0 = disabled
        uint8_t loff_flip; // bitmask for channels 1-8, 1 = flip, 0 = normal
    };

    struct CommandConfigBias {
        bool bias_p_enabled;
        bool bias_n_enabled;
        uint8_t bias_sensp; // bitmask for channels 1-8, 1 = enabled, 0 = disabled
        uint8_t bias_sensn; // bitmask for channels 1-8, 1 = enabled, 0 = disabled
    };

    struct CommandConfigChannel {
        uint8_t channel_number;
        uint8_t channel_power_down; // 0 = enabled, 1 = powered down
        uint8_t channel_gain; // 0-7, corresponding to 1x, 2x, 4x, 6x, 8x, 12x, 24x
        uint8_t input_mux; // 0-7, corresponding to normal electrode, shorted, BIAS_MES, MVDD, TEMP, TESTSIG, BIAS_P, BIAS_N
    };

    using ControlCommand = std::variant<
        CommandStart,
        CommandStop,
        CommandReset,
        CommandStandBy,
        CommandWakeUp,
        CommandReadRegister,
        CommandWriteRegister,
        CommandConfigGlobal,
        CommandConfigLeadOff,
        CommandConfigBias,
        CommandConfigChannel
    >;

    struct ControlRequest {
        uint32_t request_id;
        ControlCommand command;
    };


    struct ControlResponse {
        uint32_t request_id;
        bool success;
        char message[64];
    };


} // namespace eeg
