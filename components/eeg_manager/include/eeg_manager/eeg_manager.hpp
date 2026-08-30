#pragma once

// eeg::EEGManager - C++ facade over the ads1299 driver
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include <cstdint>
#include <array>

#include "esp_log.h"

#include "ads1299.h"
#include "ads1299_defs.h"
#include "eeg_core/eeg_types.hpp"

#include "eeg_core/frame_pool.hpp"

namespace eeg {

    template <std::size_t Capacity, std::size_t Consumers>
    class EEGManager {

    private:
        const PinConfig pin_config_;
        DeviceConfig device_config_; // The TCP controller will be allowed to change this at runtime, which will trigger reconfiguration of the device through register writes

        FramePool<eeg::FrameType, Capacity, Consumers> &raw_pool_; // Reference to the app-owned frame pool
        std::array<QueueHandle_t, Consumers> &raw_frame_queue_; // Array of queues for each consumer

        ads1299_t dev_;
        constexpr static auto TAG = "EEGManager";

        static void chunk_callback(const ads1299_chunk_t* chunk, void* ctx) {
            auto* self = static_cast<EEGManager*>(ctx);
            self->on_chunk(chunk);
        }
        void on_chunk(const ads1299_chunk_t* chunk) {
            auto* frame = raw_pool_.allocate(
                chunk->samples,
                chunk->n_samples,
                chunk->first_timestamp_us,
                chunk->last_timestamp_us,
                chunk->dropped_count,
                chunk->overflow_count
            );

            if (frame == nullptr) {
                ESP_LOGW(TAG, "Frame pool full, dropping chunk");
                return;
            }

            for (std::size_t i = 0; i < Consumers; ++i) {
                if (xQueueSend(raw_frame_queue_[i], &frame, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Consumer queue %zu full, dropping frame", i);
                    raw_pool_.release(frame);
                    // Important: release this consumer's reference here.
                    // Otherwise, the frame can remain permanently retained.
                }
            }
        }


    public:

        explicit EEGManager(
            PinConfig pin_config,
            DeviceConfig device_config,
            FramePool<eeg::FrameType, Capacity, Consumers> &raw_pool,
            std::array<QueueHandle_t, Consumers> &consumer_queues
            )
            :
        pin_config_(pin_config),
        device_config_(device_config),
        raw_pool_(raw_pool),
        raw_frame_queue_(consumer_queues),
        dev_({}) {



            // Constructor implementation (initialization of the EEGManager)
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
        ~EEGManager() {
            // Stop the device and deinitialize
            ESP_ERROR_CHECK(ads1299_deinit(&dev_));
            ESP_LOGI(TAG, "EEGManager deinitialized device");

            // Deinitialize SPI bus
            ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));

            // Turn Analog LDO off
            ESP_ERROR_CHECK(gpio_set_level(pin_config_.analog_ldo_enable_pin, 0));
        }

        // Owned by app context, so we delete copy/move constructors and assignment operators to prevent accidental copies
        EEGManager(const EEGManager&) = delete;
        EEGManager& operator=(const EEGManager&) = delete;
        EEGManager(EEGManager&&) = delete;
        EEGManager& operator=(EEGManager&&) = delete;

        void start_acquisition() {
            ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev_));
            ads1299_continuous_config_t cont_cfg = {};
            cont_cfg.on_chunk = &EEGManager::chunk_callback;
            cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms
            cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
            cont_cfg.task_priority = configMAX_PRIORITIES - 2;
            cont_cfg.task_core = 0;
            cont_cfg.ctx = this; // Pass the EEGManager instance as context to the callback

            // Start task to handle continuous acquisition


            ESP_ERROR_CHECK(ads1299_start_continuous(&dev_, &cont_cfg));
            ESP_ERROR_CHECK(ads1299_start(&dev_)); // Start conversions so the ADC begins pulsing DRDY and triggering interrupts

            ESP_LOGI(TAG, "EEGManager started acquisition");
        }
        void stop_acquisition() {
            ESP_ERROR_CHECK(ads1299_stop(&dev_));
            ESP_ERROR_CHECK(ads1299_stop_continuous(&dev_));
            ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev_));
            ESP_LOGI(TAG, "EEGManager stopped acquisition");
        }
    };
} // namespace eeg


