#pragma once

// eeg::UdpTransport — ITransport implementation that sends frames over a UDP socket
// via scatter-gather sendmsg(). Constructed with a destination Endpoint.

#include "eeg_core/eeg_types.hpp"
#include "telemetry/i_transport.hpp"
#include "telemetry/endpoint.hpp"

namespace eeg {
    class UdpTransport : public ITransport {
    public:
        explicit UdpTransport(const eeg::Endpoint& endpoint);
        ~UdpTransport() override;
        // Explicitly disable copies to prevent network socket slicing
        UdpTransport(const UdpTransport&) = delete;
        UdpTransport& operator=(const UdpTransport&) = delete;
        [[nodiscard]] bool is_open() const noexcept;
        void send(std::span<const struct iovec> buffers) override;

    private:
        int fd_ = -1;
        const Endpoint endpoint_;
        constexpr static auto TAG = "UdpSocket";
    };
} // namespace eeg