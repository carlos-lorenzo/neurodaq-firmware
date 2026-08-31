#include "app/app_context.hpp"

#include <cstdlib>

#include "esp_log.h"

namespace eeg {

    namespace {

        std::array<QueueHandle_t, n_consumers>
        create_frame_queues()
        {
            std::array<QueueHandle_t, n_consumers> queues{};

            for (std::size_t i = 0; i < n_consumers; ++i) {
                queues[i] = xQueueCreate(
                    frame_pool_capacity,
                    sizeof(eeg::EEGFrame*)
                );

                if (queues[i] == nullptr) {
                    ESP_LOGE(
                        "AppContext",
                        "Failed to create frame queue %zu",
                        i
                    );
                    abort();
                }
            }

            return queues;
        }


        QueueHandle_t create_control_command_queue()
        {
            QueueHandle_t queue = xQueueCreate(
                control_queue_capacity,
                sizeof(ControlRequest)
            );

            if (queue == nullptr) {
                ESP_LOGE(
                    "AppContext",
                    "Failed to create control command queue"
                );
                abort();
            }

            return queue;
        }


        QueueHandle_t create_control_response_queue()
        {
            QueueHandle_t queue = xQueueCreate(
                control_queue_capacity,
                sizeof(ControlResponse)
            );

            if (queue == nullptr) {
                ESP_LOGE(
                    "AppContext",
                    "Failed to create control response queue"
                );
                abort();
            }

            return queue;
        }

    } // namespace


    AppContext::AppContext(
        const eeg::PinConfig& pin_config,
        const eeg::DeviceConfig& device_config
    )
        :
        raw_pool_{},
        filtered_pool_{},

        raw_frame_queue_(create_frame_queues()),
        filtered_frame_queue_(create_frame_queues()),

        control_command_queue_(create_control_command_queue()),
        control_response_queue_(create_control_response_queue()),

        eeg_manager_(
            pin_config,
            device_config,
            raw_pool_,
            raw_frame_queue_,
            control_command_queue_,
            control_response_queue_
        ),

        eeg_streamer_(
            raw_pool_,
            raw_frame_queue_[0]
        ),

        control_server_(nullptr)
    {
        ESP_LOGI(TAG, "AppContext resources created");

        // -------------------------------------------------------------
        // Now that EEGManager is fully constructed, start its control
        // task. The queue handles are guaranteed to be valid.
        // -------------------------------------------------------------

        eeg_manager_.start_control_task();

        // -------------------------------------------------------------
        // Start acquisition.
        // -------------------------------------------------------------

        eeg_manager_.start_acquisition();

        // -------------------------------------------------------------
        // Start streaming.
        // -------------------------------------------------------------

        eeg_streamer_.start_streaming();

        // -------------------------------------------------------------
        // Finally start the TCP control server.
        // -------------------------------------------------------------

        control_server_ = std::make_unique<ControlServer>(
            CONFIG_ESP_TCP_PORT,
            control_command_queue_,
            control_response_queue_
        );

        if (!control_server_->is_open()) {
            ESP_LOGE(
                TAG,
                "Control server failed to open"
            );
            abort();
        }

        ESP_LOGI(TAG, "AppContext started");
    }

} // namespace eeg
