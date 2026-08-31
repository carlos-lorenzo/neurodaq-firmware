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

#include "telemetry/wifi_station.hpp"


namespace eeg {

    WifiStation::WifiStation(std::string_view ssid, std::string_view password, int max_retries)
        : max_retries_(max_retries), ssid_(ssid), password_(password)
    {
        event_group_ = xEventGroupCreate();
    }

    WifiStation::~WifiStation()
    {
        unregister_handlers();
        if (event_group_) {
            vEventGroupDelete(event_group_);
        }
    }

    WifiStation::Status WifiStation::connect()
    {
        ESP_ERROR_CHECK(esp_netif_init());

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Initialise Event handlres
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &WifiStation::event_handler_trampoline,
            this,
            &instance_any_id_));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &WifiStation::event_handler_trampoline,
            this,
            &instance_got_ip_));

        wifi_config_t wifi_config{};
        std::copy_n(ssid_.begin(), std::min(ssid_.size(), sizeof(wifi_config.sta.ssid) - 1), wifi_config.sta.ssid);
        std::copy_n(password_.begin(), std::min(password_.size(), sizeof(wifi_config.sta.password) - 1), wifi_config.sta.password);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        // Wait for connection or failure
        EventBits_t bits = xEventGroupWaitBits(event_group_, CONNECTED_BIT | FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & CONNECTED_BIT) {
            return Status::Connected;
        } else if (bits & FAIL_BIT) {
            return Status::Failed;
        } else {
            return Status::Connecting; // Should not reach here
        }
    }

    void WifiStation::event_handler_trampoline(void *arg, esp_event_base_t base, int32_t id, void *data) {
        // Safe cast back to C++ instance reference
        auto* self = static_cast<WifiStation*>(arg);
        self->handle_event(base, id, data);
    }

    void WifiStation::handle_event(esp_event_base_t base, int32_t id, void *data) {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_count_ < max_retries_) {
                esp_wifi_connect();
                retry_count_++;
                ESP_LOGI(TAG, "Retrying connection (%d/%d)...", retry_count_, max_retries_);
            } else {
                xEventGroupSetBits(event_group_, FAIL_BIT);
            }
        } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
            auto* event = static_cast<ip_event_got_ip_t*>(data);
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            retry_count_ = 0;
            xEventGroupSetBits(event_group_, CONNECTED_BIT);
        }
    }

    void WifiStation::unregister_handlers() {
        if (instance_any_id_) esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id_);
        if (instance_got_ip_) esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip_);
    }
}
