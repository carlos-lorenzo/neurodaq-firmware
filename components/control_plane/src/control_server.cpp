#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "control_plane/control_server.hpp"

namespace eeg {
    ControlServer::ControlServer(const uint16_t port, QueueHandle_t command_queue, QueueHandle_t response_queue) : command_queue_(command_queue), response_queue_(response_queue) {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) {
            ESP_LOGE(TAG, "Failed to create socket");
            return;
        }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        int opt = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        if (bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ESP_LOGE(TAG, "Failed to bind socket");
            close(fd_);
            fd_ = -1;
            return;
        }

        if (listen(fd_, 5) < 0) {
            ESP_LOGE(TAG, "Failed to listen on socket");
            close(fd_);
            fd_ = -1;
            return;
        }

        xTaskCreate(
            control_task,
            "ControlServerTask",
            4096,
            this,
            5,
            &control_task_handle_
            );



        ESP_LOGI(TAG, "Control server listening on port %d", port);
    }

    ControlServer::~ControlServer() {
        if (control_task_handle_) {
            vTaskDelete(control_task_handle_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    bool ControlServer::is_open() const noexcept {
        return fd_ >= 0;
    }

    void ControlServer::control_task(void *arg) {
        auto *server = static_cast<ControlServer *>(arg);
        if (!server->is_open()) {
            ESP_LOGE(TAG, "Control server socket is not open");
            return;
        }

        while (true) {
            struct sockaddr_in client_addr{};
            socklen_t client_addr_len = sizeof(client_addr);
            int client_fd = accept(server->fd_, reinterpret_cast<struct sockaddr *>(&client_addr), &client_addr_len);
            if (client_fd < 0) {
                ESP_LOGE(TAG, "Failed to accept connection");
                continue;
            }

            ESP_LOGI(TAG, "Accepted connection from %s:%d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

            // Here you would typically spawn a new task or thread to handle the client connection
            // For simplicity, we'll just close the connection immediately
            // Send example control command to the queue
            CommandStop stop_command;
            ControlRequest request{1, stop_command}; // Example request_id = 1
            if (xQueueSend(server->command_queue_, &request, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to send control command to queue");
            }

            ControlResponse response;
            if (xQueueReceive(server->response_queue_, &response, pdMS_TO_TICKS(CONFIG_ESP_TCP_TIMEOUT)) == pdTRUE) {
                ESP_LOGI(TAG, "Received response for request_id %d: %s", response.request_id, response.success ? "success" : "failure");
            } else {
                ESP_LOGE(TAG, "Failed to receive response from queue");
            }

            close(client_fd);
        }
    }

} // namespace eeg
