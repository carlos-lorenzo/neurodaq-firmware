#include <string_view>
#include <cstdint>
#include <memory>
#include <cstring>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

#include "telemetry/endpoint.hpp"

namespace eeg {
    Endpoint::Endpoint(const std::string_view ip, uint16_t port) {
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        // This line sets the IP address:
        inet_pton(AF_INET, ip.data(), &addr.sin_addr);
    }

}