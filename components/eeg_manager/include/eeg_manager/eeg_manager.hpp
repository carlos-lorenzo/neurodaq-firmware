#pragma once

// eeg::EEGManager — owns ADS1299 bring-up and the control-command loop. Configures the
// AFE over SPI2, registers the chunk callback that fills EEGFrames from the frame pool,
// and services ControlRequests from the TCP server. Header-only: all logic lives here
// because the class is a template, so there is no matching .cpp.

#include <cstdint>
#include <array>
#include <variant>
#include <cstdio>
#include "esp_log.h"
#include "esp_err.h"

#include "ads1299.h"
#include "ads1299_defs.h"

#include "eeg_core/eeg_types.hpp"
#include "eeg_core/frame_pool.hpp"
#include "control_plane/command_types.hpp"

namespace eeg {

// Helper macro to catch runtime driver errors without crashing standard execution
#define EEG_EXEC_CHECK(expr, res) \
    do { \
        esp_err_t err_ = (expr); \
        if (err_ != ESP_OK) { \
            ESP_LOGE(TAG, "%s failed: %s", #expr, esp_err_to_name(err_)); \
            (res).success = false; \
            snprintf((res).message, sizeof((res).message), "%s", esp_err_to_name(err_)); \
            return; \
        } \
    } while (0)

template <std::size_t Capacity, std::size_t Consumers>
class EEGManager {
private:
    const PinConfig pin_config_;
    DeviceConfig device_config_;

    FramePool<eeg::EEGFrame, Capacity, Consumers>& raw_pool_;
    std::array<QueueHandle_t, Consumers>& raw_frame_queue_;

    QueueHandle_t control_command_queue_;
    QueueHandle_t control_response_queue_;

    TaskHandle_t control_task_handle_ = nullptr;
    ads1299_t dev_;
    bool is_acquiring_ = false;

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
    : pin_config_(pin_config),
      device_config_(device_config),
      raw_pool_(raw_pool),
      raw_frame_queue_(consumer_queues),
      control_command_queue_(control_command_queue),
      control_response_queue_(control_response_queue),
      dev_({})
    {
        gpio_config_t analog_ldo_cfg = {
            .pin_bit_mask = (1ULL << pin_config_.analog_ldo_enable_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&analog_ldo_cfg));
        ESP_ERROR_CHECK(gpio_set_level(pin_config_.analog_ldo_enable_pin, 1));
        vTaskDelay(pdMS_TO_TICKS(250));

        spi_bus_config_t bus_cfg = {};
        bus_cfg.miso_io_num = pin_config_.miso_pin;
        bus_cfg.mosi_io_num = pin_config_.mosi_pin;
        bus_cfg.sclk_io_num = pin_config_.sclk_pin;
        bus_cfg.quadwp_io_num = -1;
        bus_cfg.quadhd_io_num = -1;
        bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25;

        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

        ads1299_config_t cfg = {};
        cfg.spi_host = SPI2_HOST;
        cfg.cs_pin = pin_config_.cs_pin;
        cfg.drdy_pin = pin_config_.drdy_pin;
        cfg.start_pin = pin_config_.start_pin;
        cfg.reset_pin = pin_config_.reset_pin;
        cfg.sample_rate = device_config_.sample_rate;
        dev_ = ads1299_create(&cfg);

        ESP_ERROR_CHECK(ads1299_init(&dev_));
        ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev_));
        ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_MISC1, ADS1299_MISC1_SRB1_ON));
        ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_CONFIG3, ADS1299_CONFIG3_BIAS_ON));
        ESP_ERROR_CHECK(ads1299_write_register(&dev_, ADS1299_REG_BIAS_SENSP, 0x01));

        vTaskDelay(pdMS_TO_TICKS(25));
        ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev_, device_config_.channel_gains[0]));
        ESP_ERROR_CHECK(ads1299_set_srb1(&dev_, device_config_.use_srb1));
    }

    ~EEGManager() {
        if (control_task_handle_ != nullptr) {
            vTaskDelete(control_task_handle_);
            control_task_handle_ = nullptr;
        }
        ads1299_stop(&dev_);
        ads1299_deinit(&dev_);
        spi_bus_free(SPI2_HOST);
        gpio_set_level(pin_config_.analog_ldo_enable_pin, 0);
    }

    EEGManager(const EEGManager&) = delete;
    EEGManager& operator=(const EEGManager&) = delete;
    EEGManager(EEGManager&&) = delete;
    EEGManager& operator=(EEGManager&&) = delete;

    esp_err_t start_acquisition() noexcept {
        esp_err_t err = ads1299_enable_continuous_read(&dev_);
        if (err != ESP_OK) return err;

        ads1299_continuous_config_t cont_cfg = {};
        cont_cfg.on_chunk = &EEGManager::chunk_callback;
        cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS;
        cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;
        cont_cfg.task_priority = configMAX_PRIORITIES - 2;
        cont_cfg.task_core = 0;
        cont_cfg.ctx = this;

        err = ads1299_start_continuous(&dev_, &cont_cfg);
        if (err != ESP_OK) return err;

        err = ads1299_start(&dev_);
        ESP_LOGI(TAG, "Acquisition started");
        if (err == ESP_OK) is_acquiring_ = true;
        return err;
    }

    esp_err_t stop_acquisition() noexcept {
        ads1299_stop(&dev_);
        ads1299_stop_continuous(&dev_);
        esp_err_t err = ads1299_disable_continuous_read(&dev_);
        is_acquiring_ = false;
        return err;
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
                xQueueSend(control_response_queue_, &response, portMAX_DELAY);
            }
        }
    }

    void handle_command(const ControlRequest& request, ControlResponse& response) {
        response.request_id = request.request_id;
        response.success = true;
        snprintf(response.message, sizeof(response.message), "OK");

        std::visit(
            [this, &response](const auto& cmd) {
                this->handle(cmd, response);
            },
            request.command
        );
    }

    // --- Safe Handlers ---
    void handle(const CommandStart&, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Acquisition already active");
            return;
        }
        EEG_EXEC_CHECK(start_acquisition(), response);
    }

    void handle(const CommandStop&, ControlResponse& response) {
        if (!is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Acquisition not running");
            return;
        }
        EEG_EXEC_CHECK(stop_acquisition(), response);
    }

    void handle(const CommandReset&, ControlResponse& response) {
        EEG_EXEC_CHECK(ads1299_reset_software(&dev_), response);
    }

    void handle(const CommandStandBy&, ControlResponse& response) {
        EEG_EXEC_CHECK(ads1299_standby(&dev_), response);
    }

    void handle(const CommandWakeUp&, ControlResponse& response) {
        EEG_EXEC_CHECK(ads1299_wakeup(&dev_), response);
    }

    void handle(const CommandReadRegister& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot read register during acquisition");
            return;
        }
        uint8_t value = 0;
        EEG_EXEC_CHECK(ads1299_read_register(&dev_, cmd.reg_address, &value), response);
        snprintf(response.message, sizeof(response.message), "Reg[0x%02X] = %u", cmd.reg_address, value);
    }

    void handle(const CommandWriteRegister& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot write register during acquisition");
            return;
        }
        EEG_EXEC_CHECK(ads1299_write_register(&dev_, cmd.reg_address, cmd.value), response);
    }

    void handle(const CommandConfigGlobal& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot configure device during acquisition");
            return;
        }
        auto sample_rate = static_cast<ads1299_sample_rate_t>(cmd.sample_rate);
        device_config_.sample_rate = sample_rate;
        EEG_EXEC_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG1, 0x03, sample_rate), response);
        EEG_EXEC_CHECK(ads1299_set_srb1(&dev_, cmd.srb1_enabled), response);
    }

    void handle(const CommandConfigLeadOff& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot configure device during acquisition");
            return;
        }
        EEG_EXEC_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0xE0, cmd.lead_off_threshold), response);
        EEG_EXEC_CHECK(ads1299_set_config4_loff_comp(&dev_, cmd.lead_off_enabled), response);
        EEG_EXEC_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0x0C, cmd.lead_off_current), response);
        EEG_EXEC_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_CONFIG4, 0x03, cmd.lead_off_frequency), response);
        EEG_EXEC_CHECK(ads1299_set_all_loff_sense(&dev_, true, cmd.loff_sensp), response);
        EEG_EXEC_CHECK(ads1299_set_all_loff_sense(&dev_, false, cmd.loff_sensn), response);
        EEG_EXEC_CHECK(ads1299_update_register_masked(&dev_, ADS1299_REG_LOFF_FLIP, 0xFF, cmd.loff_flip), response);
    }

    void handle(const CommandConfigBias& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot configure device during acquisition");
            return;
        }
        EEG_EXEC_CHECK(ads1299_set_bias_enabled(&dev_, cmd.bias_p_enabled || cmd.bias_n_enabled), response);
        EEG_EXEC_CHECK(ads1299_set_all_bias_sense(&dev_, true, cmd.bias_sensp), response);
        EEG_EXEC_CHECK(ads1299_set_all_bias_sense(&dev_, false, cmd.bias_sensn), response);
    }

    void handle(const CommandConfigChannel& cmd, ControlResponse& response) {
        if (is_acquiring_) {
            response.success = false;
            snprintf(response.message, sizeof(response.message), "Cannot configure device during acquisition");
            return;
        }
        EEG_EXEC_CHECK(ads1299_set_channel_powerdown(&dev_, cmd.channel_number, cmd.channel_power_down != 0), response);
        EEG_EXEC_CHECK(ads1299_set_channel_gain(&dev_, cmd.channel_number, static_cast<ads1299_pga_gain_t>(cmd.channel_gain)), response);
        EEG_EXEC_CHECK(ads1299_set_channel_mux(&dev_, cmd.channel_number, static_cast<ads1299_input_mux_t>(cmd.input_mux)), response);
    }
};

} // namespace eeg