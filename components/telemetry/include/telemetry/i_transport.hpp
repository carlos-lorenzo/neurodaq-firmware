#pragma once

#include <span>

#include "freertos/ringbuf.h"
#include "ads1299.h"
#include "ads1299_defs.h"
#include "eeg_core/eeg_types.hpp"

namespace eeg {
    class ITransport {
    public:
        virtual ~ITransport() = default;
        virtual void send(std::span<const struct iovec> buffers) = 0;
    };
} // namespace eeg
