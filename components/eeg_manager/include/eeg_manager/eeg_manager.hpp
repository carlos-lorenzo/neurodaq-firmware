#pragma once

// eeg::EEGManager - C++ facade over the ads1299 driver
// See: EEG firmware architecture plan, Part 4.
// TODO: implement per the architecture plan.

#include <cstdint>
#include <array>
#include <variant>
#include "esp_log.h"

#include "ads1299.h"
#include "ads1299_defs.h"

#include "eeg_core/eeg_types.hpp"
#include "eeg_core/frame_pool.hpp"
#include "control_plane/command_types.hpp"

namespace eeg {

    template <std::size_t Capacity, std::size_t Consumers>
    class EEGManager {

    private:
        const PinConfig pin_config_;
        DeviceConfig device_config_; // The TCP controller will be allowed to change this at runtime, which will trigger reconfiguration of the device through register writes

        FramePool<eeg::EEGFrame, Capacity, Consumers>& raw_pool_;
        std::array<QueueHandle_t, Consumers>& raw_frame_queue_;

        QueueHandle_t control_command_queue_;
        QueueHandle_t control_response_queue_; // Queue for control responses to the TCP controller

        TaskHandle_t control_task_handle_ = nullptr; // Task handle for the control command processing task
        ads1299_t dev_;
        constexpr static auto TAG = "EEGManager";

        static void chunk_callback(const ads1299_chunk_t* chunk, void* ctx) {
            auto self = static_cast<EEGManager*>(ctx);
            self->on_chunk(chunk);
        }
        void on_chunk(const ads1299_chunk_t* chunk) {
            EEGFrame* frame = raw_pool_.allocate(
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
            FramePool<eeg::EEGFrame, Capacity, Consumers> &raw_pool,
            std::array<QueueHandle_t, Consumers> &consumer_queues,
            QueueHandle_t control_command_queue,
            QueueHandle_t control_response_queue
            )
            :
        pin_config_(pin_config),
        device_config_(device_config),
        raw_pool_(raw_pool),
        raw_frame_queue_(consumer_queues),
        control_command_queue_(control_command_queue),
        control_response_queue_(control_response_queue),
        dev_({})
        {
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
            ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev_)); // Ensure we're in SDATAC mode before writing registers

            // Configure channels based on device_config_
            //  for (uint8_t ch = 1; ch <= 8; ++ch) {
            //      // bool powerdown = (device_config_.channel_pd_mask & (1 << ch)) != 0;
            //      // ESP_ERROR_CHECK(ads1299_set_channel_powerdown(&dev_, ch, powerdown));
            //      bool bias_sense = (device_config_.channel_bias_sensp_mask & (1 << ch)) != 0;
            //      ESP_ERROR_CHECK(ads1299_set_bias_sense(&dev_, ch, true, bias_sense));
            //      ESP_ERROR_CHECK(ads1299_set_channel_gain(&dev_, ch, device_config_.channel_gains[ch]));
            // }
            // ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_CONFIG2, 0xC0));
            // for (uint8_t i = 1; i <= 8; i++) {
            //     ESP_ERROR_CHECK(ads1299_write_register(&dev_, (ADS1299_REG_CH1SET + (i - 1)), 0x01));
            // }
            // Set SRB1 routing if specified
            // Run your desired configs (gain, bias, shorting inputs to srb...)
            ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_MISC1, ADS1299_MISC1_SRB1_ON));


            ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_CONFIG3, ADS1299_CONFIG3_BIAS_ON));
            ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_BIAS_SENSP, 0x01));

            vTaskDelay(pdMS_TO_TICKS(25));

            ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev_, device_config_.channel_gains[0])); // Assuming all channels have the same gain for simplicity
            ESP_ERROR_CHECK(ads1299_set_srb1(&dev_, device_config_.use_srb1));
        }

        ~EEGManager() {
            if (control_task_handle_ != nullptr) {
                vTaskDelete(control_task_handle_);
                control_task_handle_ = nullptr;
            }

            ESP_ERROR_CHECK(ads1299_stop(&dev_));
            ESP_ERROR_CHECK(ads1299_deinit(&dev_));

            ESP_LOGI(TAG, "EEGManager deinitialized device");

            ESP_ERROR_CHECK(spi_bus_free(SPI2_HOST));

            ESP_ERROR_CHECK(
                gpio_set_level(pin_config_.analog_ldo_enable_pin, 0)
            );
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

        void start_control_task() {
            configASSERT(control_command_queue_ != nullptr);
            configASSERT(control_response_queue_ != nullptr);

            BaseType_t result = xTaskCreate(
                &EEGManager::control_task_entry,
                "eeg_control",
                4096,
                this,
                configMAX_PRIORITIES - 3,
                &control_task_handle_
            );

            configASSERT(result == pdPASS);

            ESP_LOGI(TAG, "Control task started");
        }

        static void control_task_entry(void* arg) {
            auto* self = static_cast<EEGManager*>(arg);
            self->control_task();
        }

        void control_task() {
            ControlRequest request;
            ControlResponse response;
            for (;;) {
                if (xQueueReceive(control_command_queue_, &request, portMAX_DELAY) == pdTRUE) {
                    handle_command(request, response);
                    xQueueSend(control_response_queue_, &response, 0);
                }
            }
        }

        void handle_command(const ControlRequest& request, ControlResponse& response)
        {
            response.request_id = request.request_id;
            std::visit(
                [this, &response](const auto& cmd) {
                    handle(cmd, response);
                    ESP_LOGI(TAG, "Handling command:");
                },
                request.command
            );
            ESP_LOGI(TAG, "Command response: %s", response.success ? "success" : "failure");
        }

        void handle(const CommandStart&, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Acquisition already started");
                response.success = false;
                return;
            }
            start_acquisition();
            response.success = true;

        }
        void handle(const CommandStop&, ControlResponse& response) {
            if (dev_.dma_ctx == nullptr) {
                ESP_LOGW(TAG, "Acquisition not started");
                response.success = false;
                return;
            }
            stop_acquisition();
            response.success = true;
        }
        void handle(const CommandReset&, ControlResponse& response) {
            ESP_ERROR_CHECK(ads1299_reset_software(&dev_));
            response.success = true;
        }
        void handle(const CommandStandBy&, ControlResponse& response) {
            ESP_ERROR_CHECK(ads1299_standby(&dev_));
            response.success = true;
        }
        void handle(const CommandWakeUp&, ControlResponse& response) {
            ESP_ERROR_CHECK(ads1299_wakeup(&dev_));
            response.success = true;
        }
        void handle(const CommandReadRegister& cmd, ControlResponse& response) {
            // Check if stadac mode
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }

            uint8_t value;
            ESP_ERROR_CHECK(ads1299_read_register(&dev_, cmd.reg_address, &value));
            ESP_LOGI(TAG, "Read register 0x%02X: 0x%02X", cmd.reg_address, value);
            response.success = true;
        }
        void handle(const CommandWriteRegister& cmd, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }
            ESP_ERROR_CHECK(ads1299_write_register(&dev_, cmd.reg_address, cmd.value));
            ESP_LOGI(TAG, "Wrote register 0x%02X: 0x%02X", cmd.reg_address, cmd.value);
            response.success = true;
        }
        void handle(const CommandConfigGlobal& cmd, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }
            auto sample_rate = static_cast<ads1299_sample_rate_t>(cmd.sample_rate);
            device_config_.sample_rate = sample_rate;
            ESP_ERROR_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG1, 0x03, sample_rate));
            ESP_ERROR_CHECK(ads1299_set_srb1(&dev_, cmd.srb1_enabled));
            // ESP_ERROR_CHECK(ads1299_set_srb2(&dev_, cmd.srb2_enabled));
            response.success = true;

            ESP_LOGI(TAG, "Configured global settings: sample_rate=%u, srb1=%d, srb2=%d", cmd.sample_rate, cmd.srb1_enabled, cmd.srb2_enabled);
        }
        void handle(const CommandConfigLeadOff& cmd, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }
            ESP_ERROR_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0xE0, cmd.lead_off_threshold));
            ESP_ERROR_CHECK(ads1299_set_config4_loff_comp(&dev_, cmd.lead_off_enabled));
            ESP_ERROR_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0x0C, cmd.lead_off_current));
            ESP_ERROR_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0x03, cmd.lead_off_frequency));;
            ESP_ERROR_CHECK(ads1299_set_all_loff_sense(&dev_, true, cmd.loff_sensp));
            ESP_ERROR_CHECK(ads1299_set_all_loff_sense  (&dev_, false, cmd.loff_sensn));
            ESP_ERROR_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_LOFF_FLIP, 0xFF, cmd.loff_flip));
            ESP_LOGI(TAG, "Configured lead-off settings");
            response.success = true;
        }

        void handle(const CommandConfigBias& cmd, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }
            ESP_ERROR_CHECK(ads1299_set_bias_enabled(&dev_, cmd.bias_p_enabled || cmd.bias_n_enabled));
            ESP_ERROR_CHECK(ads1299_set_all_bias_sense(&dev_, true, cmd.bias_sensp));
            ESP_ERROR_CHECK(ads1299_set_all_bias_sense(&dev_, false, cmd.bias_sensn));
            ESP_LOGI(TAG, "Configured bias settings");
            response.success = true;
        }
        void handle(const CommandConfigChannel& cmd, ControlResponse& response) {
            if (dev_.dma_ctx != nullptr) {
                ESP_LOGW(TAG, "Cannot read register while in RDATAC mode");
                response.success = false;
                return;
            }
            ESP_ERROR_CHECK(ads1299_set_channel_powerdown(&dev_, cmd.channel_number, cmd.channel_power_down != 0));
            ESP_ERROR_CHECK(ads1299_set_channel_gain(&dev_, cmd.channel_number, static_cast<ads1299_pga_gain_t>(cmd.channel_gain)));
            ESP_ERROR_CHECK(ads1299_set_channel_mux(&dev_, cmd.channel_number, static_cast<ads1299_input_mux_t>(cmd.input_mux)));
            ESP_LOGI(TAG, "Configured channel %u settings: power_down=%d, gain=%u, mux=%u", cmd.channel_number, cmd.channel_power_down, cmd.channel_gain, cmd.input_mux);
            response.success = true;
        }
    };
} // namespace eeg


