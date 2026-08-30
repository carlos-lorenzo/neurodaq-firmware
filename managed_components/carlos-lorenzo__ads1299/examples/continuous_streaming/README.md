# ADS1299 Continuous Streaming Example

This example demonstrates how to configure the ADS1299 with the safe masked-register API and stream sample chunks over the ESP32's native USB Serial/JTAG interface.

## What it does

- Initializes the ADS1299 with a typical 250 SPS configuration.
- Enables SRB1 and bias routing using the high-level helper API instead of raw byte writes.
- Applies a uniform PGA gain to all channels with `ads1299_set_all_channels_gain()`.
- Enters RDATAC mode, then starts continuous acquisition.
- Packages each chunk into a binary frame and sends it over the USB device endpoint for downstream capture.

## How it works

The example uses a dual-core pattern:

1. Core 0 handles SPI acquisition and the driver's chunk callback.
2. Core 1 drains the ring buffer and writes raw bytes to the USB Serial/JTAG TX FIFO.

This keeps the acquisition path fast and avoids mixing ASCII log output into the binary stream.

## Binary frame format

Each output frame is:

- Header (8 bytes)
  - `sync[0] = 0xAA`
  - `sync[1] = 0x55`
  - `length` = payload length in bytes
  - `chunk_seq` = monotonic sequence number
- Payload = `ads1299_sample_t` objects from a driver chunk
- Checksum = simple XOR over the payload bytes

## Safe configuration convention used here

This example intentionally uses the new masked-update convention instead of direct `write_register()` calls for bitfields:

```c
ESP_ERROR_CHECK(ads1299_set_srb1(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_bias_enabled(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, ADS1299_PGA_GAIN_24));
```

The driver enforces `RDATAC` state checks and preserves unrelated bits in each register.

## Quick start

```console
idf.py set-target esp32s3
idf.py build flash
```

The pin mapping and hardware wiring remain in `main.cpp`, and the example assumes the ADS1299 analog rail is enabled before SPI traffic begins.
