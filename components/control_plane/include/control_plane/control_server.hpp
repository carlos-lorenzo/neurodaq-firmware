#pragma once

#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>

#include "esp_log.h"

#include "esp_task.h"

#include "control_plane/command_types.hpp"

namespace eeg {
    class ControlServer {
    public:
        explicit ControlServer(std::uint16_t port, QueueHandle_t command_queue, QueueHandle_t response_queue);
        ~ControlServer();
        ControlServer(const ControlServer&) = delete;
        ControlServer& operator=(const ControlServer&) = delete;
        ControlServer(ControlServer&&) = delete;
        ControlServer& operator=(ControlServer&&) = delete;
        [[nodiscard]] bool is_open() const noexcept;

        static void control_task(void* arg);

    private:
        int fd_ = -1;
        TaskHandle_t control_task_handle_ = nullptr;
        constexpr static auto TAG = "ControlServer";
        QueueHandle_t command_queue_;
        QueueHandle_t response_queue_;

    };
} // namespace eeg
