#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "ads1299.h"
#include "ads1299_defs.h"

#include "app/app_context.hpp"
#include "eeg_manager/eeg_manager.hpp"

constexpr const char *TAG = "main";


extern  "C" void app_main(void)
{
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
    eeg::DeviceConfig device_config{ADS1299_DR_250SPS, ADS1299_PGA_GAIN_24, 0xFF, true};

    // Move the creation of AppContext to the heap to avoid accessing a destroyed object when the main function exits.
    new eeg::AppContext(pin_config, device_config);
}
