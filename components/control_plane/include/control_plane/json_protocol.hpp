#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <memory>
#include "cJSON.h"
#include "control_plane/command_types.hpp"

namespace eeg {

    struct CJsonDeleter {
        void operator()(cJSON* ptr) const noexcept {
            if (ptr != nullptr) {
                cJSON_Delete(ptr);
            }
        }
    };

    using UniqueCJson = std::unique_ptr<cJSON, CJsonDeleter>;

    class JsonProtocol {
    public:
        JsonProtocol() = delete; // Pure static utility class

        // Parses a single \n-terminated JSON string into a ControlRequest.
        // Guaranteed non-throwing to prevent dynamic stack unwinding cost.
        [[nodiscard]] static std::optional<ControlRequest> parse_request(const char* json_str, std::size_t length) noexcept;

        // Serializes a ControlResponse to standard JSON string with newline termination.
        [[nodiscard]] static std::string serialize_response(const ControlResponse& response);

    private:
        [[nodiscard]] static std::optional<ControlCommand> parse_command(const cJSON* root) noexcept;
    };

} // namespace eeg