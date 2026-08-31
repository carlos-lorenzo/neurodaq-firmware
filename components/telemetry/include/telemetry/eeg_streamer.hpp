#pragma once

// eeg::EEGStreamer - packs+sends frames from one active IStreamSource

#include <memory>
#include <span>

#include "esp_log.h"

#include "eeg_core/frame_pool.hpp"
#include "eeg_core/eeg_types.hpp"
#include "telemetry/i_transport.hpp"
#include "telemetry/udp_transport.hpp"
#include "telemetry/endpoint.hpp"


namespace eeg {
    template <eeg::TransportType T, std::size_t Capacity, std::size_t Consumers>
    class EEGStreamer {
    public:

        explicit  EEGStreamer(
            eeg::FramePool<eeg::EEGFrame, Capacity, Consumers>
            &raw_pool, QueueHandle_t&frame_queue
            ) :
        raw_pool_(raw_pool),
        frame_queue_(frame_queue),
        task_handle_(nullptr)
        {

            if constexpr (T == eeg::TransportType::UDP) {
                Endpoint endpoint(CONFIG_ESP_UDP_IP, CONFIG_ESP_UDP_PORT);
                transport_ = std::make_unique<UdpTransport>(endpoint);

                ESP_LOGI(TAG, "Using UDP transport");
            } else if constexpr (T == eeg::TransportType::USB) {
                // transport_ = std::make_unique<UsbTransport>();
                transport_ = nullptr; // Placeholder for actual USB transport implementation
                ESP_LOGI(TAG, "Using USB transport");
            } else {
                static_assert(T == eeg::TransportType::UDP || T == eeg::TransportType::USB, "Unsupported transport type");
            }

        }
        ~EEGStreamer() {
            vTaskDelete(task_handle_);
            vQueueDelete(frame_queue_);
        }

        void start_streaming() {
            xTaskCreate(
                stream_task,
                "EEGStreamerTask",
                4096,
                this,
                5,
                &task_handle_
            );
        }

        void stop_streaming() {
            if (task_handle_) {
                vTaskDelete(task_handle_);
                task_handle_ = nullptr;
            }
        }

        [[noreturn]] void static stream_task(void* arg) {
            auto* self = static_cast<EEGStreamer*>(arg);
            while (true) {
                eeg::EEGFrame* frame;
                if (xQueueReceive(self->frame_queue_, &frame, portMAX_DELAY) == pdTRUE) {
                    if (self->transport_) {
                        struct iovec chunks[3]; // Expand to 4 to include IMU quaternion data
                        auto header = eeg::TelemetryHeader(++self->sequence_number_);
                        chunks[0].iov_base = &header;
                        chunks[0].iov_len = sizeof(header);
                        chunks[1].iov_base = frame->samples.data();
                        chunks[1].iov_len = frame->n_samples * sizeof(ads1299_sample_t);
                        uint8_t checksum = 0;
                        for (std::size_t i = 0; i < frame->n_samples * sizeof(ads1299_sample_t); ++i) {
                            checksum ^= reinterpret_cast<uint8_t*>(frame->samples.data())[i];
                        }
                        chunks[2].iov_base = &checksum;
                        chunks[2].iov_len = sizeof(checksum);

                        self->transport_->send(chunks);
                    }
                    self->raw_pool_.release(frame); // Release the frame back to the pool after sending

                }
            }
        }

    private:
        eeg::FramePool<eeg::EEGFrame, Capacity, Consumers> &raw_pool_; // Reference to the app-owned frame pool
        QueueHandle_t &frame_queue_;
        std::unique_ptr<ITransport> transport_;
        TaskHandle_t task_handle_;
        constexpr static auto TAG = "EEGStreamer";
        std::uint32_t sequence_number_{0};
    };
} // namespace eeg
