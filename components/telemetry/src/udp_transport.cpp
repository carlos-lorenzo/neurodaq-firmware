#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/uio.h>
#include "esp_log.h"

#include "telemetry/udp_transport.hpp"
#include "telemetry/endpoint.hpp"

namespace eeg {



UdpTransport::UdpTransport(const eeg::Endpoint &endpoint): endpoint_(endpoint) {

    // Create a UDP socket
    fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    // Connect the socket to the endpoint
    if (connect(fd_, reinterpret_cast<const struct sockaddr*>(&endpoint.addr), sizeof(endpoint.addr)) < 0) {
        ESP_LOGE(TAG, "Failed to connect to endpoint");
        close(fd_);
        fd_ = -1;
        return;
    }

    ESP_LOGI(TAG, "UDP socket connected to %s:%d", inet_ntoa(endpoint_.addr.sin_addr), endpoint_.addr.sin_port);
}

UdpTransport::~UdpTransport() {
    close(fd_);
}

bool UdpTransport::is_open() const noexcept {
    return fd_ >= 0;
}

void UdpTransport::send(std::span<const struct iovec> buffers) {
// 1. Calculate total size for validation and logging
size_t total_expected_bytes = 0;
for (const auto&[_, iov_len] : buffers) {
    total_expected_bytes += iov_len;
}

// 2. Set up the msghdr structure required by sendmsg
struct msghdr msg{};
msg.msg_name = const_cast<sockaddr*>(reinterpret_cast<const sockaddr*>(&endpoint_.addr));
msg.msg_namelen = sizeof(endpoint_.addr);
msg.msg_iov = const_cast<struct iovec*>(buffers.data());
msg.msg_iovlen = static_cast<msg_iovlen_t>(buffers.size());
msg.msg_control = nullptr;
msg.msg_controllen = 0;
msg.msg_flags = 0;

for (int attempt = 0; attempt < 5; ++attempt) {
    // sendmsg handles the scatter-gather natively under the hood
    ssize_t sent_bytes = sendmsg(fd_, &msg, 0);

    if (sent_bytes == static_cast<ssize_t>(total_expected_bytes)) {
        return; // Success!
    }

    if (sent_bytes < 0) {
        int err = errno;

        // Handle temporary LwIP buffer exhaustion
        if (err == ENOBUFS || err == ENOMEM) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGE(TAG, "sendmsg() failed: errno=%d (%s)", err, strerror(err));
        return;
    }

    // UDP is packet-oriented: partial sends are rare but mean truncation/packet drop
    ESP_LOGE(TAG, "Partial UDP sendmsg: %zd/%zu bytes", sent_bytes, total_expected_bytes);
    return;
}
}
} // namespace eeg
