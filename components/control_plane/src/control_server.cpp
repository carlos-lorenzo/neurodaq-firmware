#include "control_plane/control_server.hpp"
#include "control_plane/json_protocol.hpp"
#include <unistd.h>
#include <cstring>
#include <array>

namespace eeg {

ControlServer::ControlServer(const uint16_t port, QueueHandle_t command_queue, QueueHandle_t response_queue)
    : command_queue_(command_queue), response_queue_(response_queue) {

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

    if (listen(fd_, 1) < 0) {
        ESP_LOGE(TAG, "Failed to listen on socket");
        close(fd_);
        fd_ = -1;
        return;
    }

    xTaskCreate(
        control_task,
        "ControlServerTask",
        6144,
        this,
        5,
        &control_task_handle_
    );

    ESP_LOGI(TAG, "Control server listening on port %d", port);
}

ControlServer::~ControlServer() {
    if (control_task_handle_ != nullptr) {
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
    if (!server->is_open()) return;

    // Line accumulation framing buffer (Zero dynamic allocation on hot path)
    constexpr std::size_t RX_BUF_CAPACITY = 2048;
    std::array<char, RX_BUF_CAPACITY> rx_buffer{};
    std::size_t rx_len = 0;

    while (true) {
        struct sockaddr_in client_addr{};
        socklen_t client_addr_len = sizeof(client_addr);
        int client_fd = accept(server->fd_, reinterpret_cast<struct sockaddr *>(&client_addr), &client_addr_len);

        if (client_fd < 0) {
            ESP_LOGE(TAG, "Failed to accept connection");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ESP_LOGI(TAG, "Client connected");
        rx_len = 0;

        while (true) {
            ssize_t bytes_read = recv(client_fd, rx_buffer.data() + rx_len, RX_BUF_CAPACITY - rx_len - 1, 0);
            if (bytes_read <= 0) {
                ESP_LOGI(TAG, "Client disconnected or socket read error");
                break;
            }

            rx_len += static_cast<std::size_t>(bytes_read);
            rx_buffer[rx_len] = '\0';

            // Extract line-by-line using \n frame boundaries
            std::size_t processed_idx = 0;
            for (std::size_t i = 0; i < rx_len; ++i) {
                if (rx_buffer[i] == '\n') {
                    std::size_t line_len = i - processed_idx;

                    if (line_len > 0) {
                        auto req_opt = JsonProtocol::parse_request(&rx_buffer[processed_idx], line_len);

                        ControlResponse response{};
                        if (!req_opt.has_value()) {
                            response.request_id = 0;
                            response.success = false;
                            strncpy(response.message, "Malformed JSON Request", sizeof(response.message));
                        } else {
                            if (xQueueSend(server->command_queue_, &req_opt.value(), pdMS_TO_TICKS(100)) == pdTRUE) {
                                if (xQueueReceive(server->response_queue_, &response, pdMS_TO_TICKS(1000)) != pdTRUE) {
                                    response.request_id = req_opt->request_id;
                                    response.success = false;
                                    strncpy(response.message, "Execution Timeout", sizeof(response.message));
                                }
                            } else {
                                response.request_id = req_opt->request_id;
                                response.success = false;
                                strncpy(response.message, "Command Queue Full", sizeof(response.message));
                            }
                        }

                        // Send back \n-terminated response JSON frame
                        std::string resp_json = JsonProtocol::serialize_response(response);
                        send(client_fd, resp_json.data(), resp_json.size(), 0);
                    }
                    processed_idx = i + 1;
                }
            }

            // Move leftover unparsed stream bytes to start of buffer
            if (processed_idx > 0) {
                std::memmove(rx_buffer.data(), rx_buffer.data() + processed_idx, rx_len - processed_idx);
                rx_len -= processed_idx;
            }

            // Buffer Overflow Guard: reset on garbage frame larger than buffer
            if (rx_len >= RX_BUF_CAPACITY - 1) {
                ESP_LOGE(TAG, "Framing RX Buffer overflow; dropping buffer stream");
                rx_len = 0;
            }
        }

        close(client_fd);
    }
}

} // namespace eeg