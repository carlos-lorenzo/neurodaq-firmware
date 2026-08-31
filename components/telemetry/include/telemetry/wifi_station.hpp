#pragma once

#include <algorithm>
#include <string_view>
#include <span>
#include <string_view>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

namespace eeg {
    class WifiStation {
    public:
        enum class Status {
            Connected,
            Failed,
            Connecting
        };

        WifiStation(std::string_view ssid, std::string_view password, int max_retries = 5);
        ~WifiStation();
        WifiStation(const WifiStation&) = delete;
        WifiStation& operator=(const WifiStation&) = delete;
        Status connect();
    private:
        static constexpr EventBits_t CONNECTED_BIT = BIT0;
        static constexpr EventBits_t FAIL_BIT = BIT1;
        static constexpr const char* TAG = "WifiStation";
        static void event_handler_trampoline(void* arg, esp_event_base_t base, int32_t id, void* data);
        void handle_event(esp_event_base_t base, int32_t id, void* data);
        void unregister_handlers();

        int max_retries_;
        int retry_count_{0};
        std::string_view ssid_;
        std::string_view password_;
        EventGroupHandle_t event_group_{nullptr};
        esp_event_handler_instance_t instance_any_id_{nullptr};
        esp_event_handler_instance_t instance_got_ip_{nullptr};

    };
}