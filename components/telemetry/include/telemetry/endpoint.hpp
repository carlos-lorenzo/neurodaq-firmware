#pragma once

#include <string_view>
#include <cstdint>
#include <memory>
#include <cstring>

#include <lwip/sockets.h>  // FIX 2: Provides sockaddr_in and AF_INET
#include <lwip/netdb.h>

namespace eeg {
    class Endpoint {
    public:
        Endpoint(const std::string_view ip, uint16_t port);

        struct sockaddr_in addr;
    };
}