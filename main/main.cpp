#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "nvs_flash.h"

#include "ads1299.h"
#include "ads1299_defs.h"

#include "app/app_context.hpp"
#include "eeg_manager/eeg_manager.hpp"
#include "eeg_core/eeg_types.hpp"
#include "telemetry/i_transport.hpp"
#include "telemetry/eeg_streamer.hpp"
#include "telemetry/udp_transport.hpp"
#include "telemetry/wifi_station.hpp"

constexpr const char *TAG = "main";


extern  "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Create the Wi-Fi station instance on the stack.
    eeg::WifiStation wifi_station(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_MAXIMUM_RETRY);
    auto status = wifi_station.connect();
    // Wi-Fi is required to boot: there is no offline fallback. If association fails,
    // app_main returns and the device does nothing until reset.
    if (status != eeg::WifiStation::Status::Connected) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
        return;
    }
    ESP_LOGI(TAG, "Connected to Wi-Fi");

    constexpr eeg::PinConfig pin_config = {
        .drdy_pin = GPIO_NUM_8,
        .miso_pin = GPIO_NUM_9,
        .sclk_pin = GPIO_NUM_10,
        .cs_pin = GPIO_NUM_11,
        .start_pin = GPIO_NUM_12,
        .reset_pin = GPIO_NUM_13,
        .mosi_pin = GPIO_NUM_14,
        .analog_ldo_enable_pin = GPIO_NUM_6
    };
    eeg::DeviceConfig device_config{ADS1299_DR_250SPS, ADS1299_PGA_GAIN_24, 0xFF, 0x01, true};

    // AppContext owns all pools, queues and tasks. It is heap-allocated and
    // deliberately never freed: it must outlive app_main's return, and the tasks it
    // starts run for the lifetime of the device. The intentional "leak" is permanent,
    // not a TODO.
    new eeg::AppContext(pin_config, device_config);
}
