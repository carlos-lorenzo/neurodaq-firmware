#include <cstring>
#include "esp_log.h"

#include "control_plane/json_protocol.hpp"

namespace eeg {

namespace {
constexpr auto TAG = "JsonProtocol";

[[nodiscard]] inline std::optional<int> get_json_int(const cJSON* obj, const char* key) noexcept {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item)) {
        return item->valueint;
    }
    return std::nullopt;
}

[[nodiscard]] inline std::optional<bool> get_json_bool(const cJSON* obj, const char* key) noexcept {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsBool(item)) {
        return cJSON_IsTrue(item) != 0;
    }
    return std::nullopt;
}
} // namespace

std::optional<ControlRequest> JsonProtocol::parse_request(const char* json_str, std::size_t length) noexcept {
    if (json_str == nullptr || length == 0) {
        return std::nullopt;
    }

    UniqueCJson root(cJSON_ParseWithLength(json_str, length));
    if (!root) {
        ESP_LOGE(TAG, "JSON parse error before: [%s]", cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown");
        return std::nullopt;
    }

    auto req_id = get_json_int(root.get(), "id");
    if (!req_id) {
        ESP_LOGE(TAG, "Missing or invalid 'id' field");
        return std::nullopt;
    }

    auto cmd_opt = parse_command(root.get());
    if (!cmd_opt) {
        ESP_LOGE(TAG, "Failed to parse command contents");
        return std::nullopt;
    }

    return ControlRequest{
        .request_id = static_cast<uint32_t>(*req_id),
        .command = std::move(*cmd_opt)
    };
}

std::optional<ControlCommand> JsonProtocol::parse_command(const cJSON* root) noexcept {
    const cJSON* cmd_item = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(cmd_item) || (cmd_item->valuestring == nullptr)) {
        return std::nullopt;
    }

    const char* cmd_str = cmd_item->valuestring;
    const cJSON* params = cJSON_GetObjectItemCaseSensitive(root, "params");

    if (std::strcmp(cmd_str, "start") == 0)   return CommandStart{};
    if (std::strcmp(cmd_str, "stop") == 0)    return CommandStop{};
    if (std::strcmp(cmd_str, "reset") == 0)   return CommandReset{};
    if (std::strcmp(cmd_str, "standby") == 0) return CommandStandBy{};
    if (std::strcmp(cmd_str, "wakeup") == 0)  return CommandWakeUp{};

    if (std::strcmp(cmd_str, "read_reg") == 0) {
        if (!params) return std::nullopt;
        auto addr = get_json_int(params, "address");
        if (!addr) return std::nullopt;
        return CommandReadRegister{ static_cast<uint8_t>(*addr) };
    }

    if (std::strcmp(cmd_str, "write_reg") == 0) {
        if (!params) return std::nullopt;
        auto addr = get_json_int(params, "address");
        auto val = get_json_int(params, "value");
        if (!addr || !val) return std::nullopt;
        return CommandWriteRegister{ static_cast<uint8_t>(*addr), static_cast<uint8_t>(*val) };
    }

    if (std::strcmp(cmd_str, "config_global") == 0) {
        if (!params) return std::nullopt;
        auto sr = get_json_int(params, "sample_rate");
        auto srb1 = get_json_bool(params, "srb1_enabled");
        auto srb2 = get_json_bool(params, "srb2_enabled");
        if (!sr || !srb1 || !srb2) return std::nullopt;
        return CommandConfigGlobal{ static_cast<uint8_t>(*sr), *srb1, *srb2 };
    }

    if (std::strcmp(cmd_str, "config_leadoff") == 0) {
        if (!params) return std::nullopt;
        auto en = get_json_bool(params, "enabled");
        auto thresh = get_json_int(params, "threshold");
        auto curr = get_json_int(params, "current");
        auto freq = get_json_int(params, "frequency");
        auto sensp = get_json_int(params, "sensp");
        auto sensn = get_json_int(params, "sensn");
        auto flip = get_json_int(params, "flip");
        if (!en || !thresh || !curr || !freq || !sensp || !sensn || !flip) return std::nullopt;
        return CommandConfigLeadOff{ *en, static_cast<uint8_t>(*thresh), static_cast<uint8_t>(*curr),
                                     static_cast<uint8_t>(*freq), static_cast<uint8_t>(*sensp),
                                     static_cast<uint8_t>(*sensn), static_cast<uint8_t>(*flip) };
    }

    if (std::strcmp(cmd_str, "config_bias") == 0) {
        if (!params) return std::nullopt;
        auto bp = get_json_bool(params, "bias_p");
        auto bn = get_json_bool(params, "bias_n");
        auto sensp = get_json_int(params, "sensp");
        auto sensn = get_json_int(params, "sensn");
        if (!bp || !bn || !sensp || !sensn) return std::nullopt;
        return CommandConfigBias{ *bp, *bn, static_cast<uint8_t>(*sensp), static_cast<uint8_t>(*sensn) };
    }

    if (std::strcmp(cmd_str, "config_channel") == 0) {
        if (!params) return std::nullopt;
        auto ch = get_json_int(params, "channel");
        auto pd = get_json_int(params, "power_down");
        auto gain = get_json_int(params, "gain");
        auto mux = get_json_int(params, "mux");
        if (!ch || !pd || !gain || !mux) return std::nullopt;
        return CommandConfigChannel{ static_cast<uint8_t>(*ch), static_cast<uint8_t>(*pd),
                                    static_cast<uint8_t>(*gain), static_cast<uint8_t>(*mux) };
    }

    return std::nullopt;
}

std::string JsonProtocol::serialize_response(const ControlResponse& response) {
    UniqueCJson root(cJSON_CreateObject());
    if (!root) return "{\"success\":false,\"message\":\"OOM\"}\n";

    cJSON_AddNumberToObject(root.get(), "id", response.request_id);
    cJSON_AddBoolToObject(root.get(), "success", response.success);
    cJSON_AddStringToObject(root.get(), "message", response.message);

    char* unformatted = cJSON_PrintUnformatted(root.get());
    if (!unformatted) {
        return "{\"success\":false,\"message\":\"Formatting Failed\"}\n";
    }

    std::string result(unformatted);
    cJSON_free(unformatted);
    result.push_back('\n');
    return result;
}

} // namespace eeg