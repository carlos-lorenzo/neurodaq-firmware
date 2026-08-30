# ADS1299 ESP-IDF Driver

[![Component Registry](https://components.espressif.com/components/carlos-lorenzo/ads1299/badge.svg)](https://components.espressif.com/components/carlos-lorenzo/ads1299)
[![Documentation Status](https://readthedocs.org/projects/ads1299-esp/badge/?version=latest)](https://ads1299-esp.readthedocs.io/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An ESP-IDF driver for Texas Instruments' [ADS1299](https://www.ti.com/product/ADS1299)
analog front-end — an 8-channel, 24-bit ADC purpose-built for EEG and other
biopotential acquisition. The driver talks to the ADS1299 over SPI, handles
DMA-driven continuous acquisition in the background, and hands your
application fully parsed, timestamped samples through a callback. Protocol
details (RDATAC framing, DRDY timing, frame parsing) stay inside the driver;
your application only ever sees `ads1299_sample_t`.

## Features

- DMA-driven continuous acquisition at rates up to 16 kSPS, with an
  interrupt/DMA pipeline that never blocks in ISR context
- Multiple simultaneous ADS1299 devices, each with its own handle and SPI
  device — no shared global state
- Chunked delivery via an internal ring buffer, with dropped-sample and
  overflow counters exposed to the application for monitoring signal
  integrity
- Safe masked register updates using `mask & value` semantics, so unrelated
  control bits are preserved during RMW operations
- RDATAC awareness in the driver state: writes are rejected with
  `ESP_ERR_INVALID_STATE` while streaming mode is active
- Full configuration API for gain, mux, SRB1 routing, bias, lead-off, and
  other ADS1299 control fields, alongside the raw register read/write layer
- Clean separation between driver internals and application-layer signal
  processing

## Installation

```console
idf.py add-dependency "carlos-lorenzo/ads1299^0.1.0"
```

The application is responsible for initializing the SPI bus
(`spi_bus_initialize`) before creating and initializing an ADS1299 device
handle; the driver adds and removes its own SPI device on that bus.

## Quick start

```c
#include "ads1299.h"

ads1299_config_t config = {
    .spi_host = SPI2_HOST,
    .cs_pin = GPIO_NUM_5,
    .drdy_pin = GPIO_NUM_4,
    .reset_pin = GPIO_NUM_16,
    .start_pin = GPIO_NUM_17,
    .sample_rate = ADS1299_DR_250SPS,
};

ads1299_t dev = ads1299_create(&config);
ESP_ERROR_CHECK(ads1299_init(&dev));

/* Safe, bit-preserving configuration using helper APIs. */
ESP_ERROR_CHECK(ads1299_set_srb1(&dev, true));
ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev, ADS1299_PGA_GAIN_24));
ESP_ERROR_CHECK(ads1299_set_all_channels_mux(&dev, ADS1299_INPUT_NORMAL));
ESP_ERROR_CHECK(ads1299_set_all_channels_powerdown(&dev, false));
```

`ads1299_init()` runs the full power-up sequence (hardware reset, device ID
check, baseline short and test-signal verification) and leaves the device
ready to stream. The configuration helpers preserve unrelated bits in a
register and reject writes while the device is in RDATAC/streaming mode.

## Continuous acquisition

```c
void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{
    // chunk->samples is valid only for the duration of this callback —
    // copy anything you need to keep.
}

ads1299_continuous_config_t cont_cfg = {
    .on_chunk = on_chunk,
    .chunk_duration_ms = 100,
    .ring_buffer_chunks = 8,
    .task_priority = configMAX_PRIORITIES - 2,
    .task_core = 0,
};

ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev));
ESP_ERROR_CHECK(ads1299_start_continuous(&dev, &cont_cfg));
ESP_ERROR_CHECK(ads1299_start(&dev));
```

`on_chunk` runs from the driver's own handler task, not from ISR context —
keep it short, or hand samples off to an application task if further
processing is expensive.

## Examples

- [`examples/continuous_streaming`](examples/continuous_streaming) — streams
  ADS1299 sample chunks over USB Serial/JTAG as length-prefixed binary frames.
  The example shows the driver's continuous acquisition path and demonstrates
  the recommended masked-register configuration pattern for channel setup.
- [`examples/leadoff_detection`](examples/leadoff_detection) — configures
  lead-off detection and monitors positive/negative electrode disconnect
  status for each channel over the active acquisition stream.
- [`examples/snr`](examples/snr) — measures the ADC's shorted-input noise floor
  and compares it against the on-chip test signal to compute per-channel SNR.
- [`examples/wifi_streaming`](examples/wifi_streaming) — configures the ADS1299,
  streams raw ADC data over UDP/Wi-Fi, and demonstrates how to fan out the
  driver's chunk callback into a network transport.

## Documentation

Full API reference, generated from the headers, is published at
[ads1299-esp.readthedocs.io](https://ads1299-esp.readthedocs.io/).

## License

MIT — see [LICENSE](LICENSE).