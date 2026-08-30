#include "esp_log.h"

#include "ads1299.h"
#include "ads1299_defs.h"

#include "eeg_manager/eeg_manager.hpp"

namespace eeg {

    // TODO: implement per the architecture plan.
    EEGManager::EEGManager(PinConfig pin_config, DeviceConfig device_config)
        : pin_config_(pin_config), device_config_(device_config), dev_({}) {

        // Turn Analog LDO on
        gpio_config_t analog_ldo_cfg = {
            .pin_bit_mask = (1ULL << pin_config_.analog_ldo_enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&analog_ldo_cfg));
        ESP_ERROR_CHECK(gpio_set_level(pin_config_.analog_ldo_enable_pin, 1));  // Enable LDO
        vTaskDelay(pdMS_TO_TICKS(250));

        // Init SPI bus
        spi_bus_config_t bus_cfg = {};
        bus_cfg.miso_io_num = pin_config_.miso_pin;
        bus_cfg.mosi_io_num = pin_config_.mosi_pin;
        bus_cfg.sclk_io_num = pin_config_.sclk_pin;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25; // TODO: adjust this size based on expected chunk size


        ESP_ERROR_CHECK(
            spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO)
        );

        ads1299_config_t cfg = {};
        cfg.spi_host = SPI2_HOST;
        cfg.cs_pin = pin_config_.cs_pin;
        cfg.drdy_pin = pin_config_.drdy_pin;
        cfg.start_pin = pin_config_.start_pin;
        cfg.reset_pin = pin_config_.reset_pin;
        cfg.sample_rate = device_config_.sample_rate;
        dev_ = ads1299_create(&cfg);

        ESP_ERROR_CHECK(ads1299_init(&dev_));

        ESP_LOGI(TAG, "EEGManager initialized device");
    }

    EEGManager::~EEGManager() {
        // Stop the device and deinitialize
        ESP_ERROR_CHECK(ads1299_deinit(&dev_));
        ESP_LOGI(TAG, "EEGManager deinitialized device");

        // Deinitialize SPI bus
        ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));

        // Turn Analog LDO off
        ESP_ERROR_CHECK(gpio_set_level(pin_config_.analog_ldo_enable_pin, 0));
    }

    void EEGManager::start_acquisition() {
        ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev_));
        ads1299_continuous_config_t cont_cfg = {};
        cont_cfg.on_chunk = &EEGManager::chunk_callback;
        cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms
        cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
        cont_cfg.task_priority = configMAX_PRIORITIES - 2;
        cont_cfg.task_core = 0;

        // Start task to handle continuous acquisition


        ESP_ERROR_CHECK(ads1299_start_continuous(&dev_, &cont_cfg));
        ESP_ERROR_CHECK(ads1299_start(&dev_)); // Start conversions so the ADC begins pulsing DRDY and triggering interrupts

        ESP_LOGI(TAG, "EEGManager started acquisition");
    }

    void EEGManager::stop_acquisition() {
        ESP_ERROR_CHECK(ads1299_stop(&dev_));
        ESP_ERROR_CHECK(ads1299_stop_continuous(&dev_));
        ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev_));
        ESP_LOGI(TAG, "EEGManager stopped acquisition");
    }

    void EEGManager::chunk_callback(const ads1299_chunk_t* chunk, void* ctx)
    {
        auto* self = static_cast<EEGManager*>(ctx);
        self->on_chunk(chunk);
    }

    void EEGManager::on_chunk(const ads1299_chunk_t* chunk)
    {
        //frame_pool_.push(chunk);
    }

} // namespace eeg


// ads1299_continuous_config_t cont_cfg = {};
// cont_cfg.on_chunk = on_chunk;
// cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms chunks
// cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
// cont_cfg.task_priority = configMAX_PRIORITIES - 2;
// cont_cfg.task_core = 0;
//
// telemetry_init();
//
// ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));
//
// // 3. CRITICAL: Start conversions so the ADC begins pulsing DRDY and triggering interrupts!
// ESP_ERROR_CHECK(ads1299_start(&dev1));