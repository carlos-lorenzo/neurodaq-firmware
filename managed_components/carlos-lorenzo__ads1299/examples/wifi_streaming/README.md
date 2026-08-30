# ADS1299 Wi-Fi Streaming Example

This example demonstrates how to configure the ADS1299 with the protected register-update API and then stream chunked samples out over a Wi-Fi UDP socket.

## What it does

- Initializes the ADS1299 in the same pattern as the other examples.
- Uses the safe config helpers (`set_srb1`, `set_bias_enabled`, `set_all_channels_gain`) instead of raw register writes.
- Enables RDATAC and continuous acquisition.
- Sends each chunk to a Wi-Fi socket as a UDP payload.

## Network flow

The application creates a Wi-Fi station connection and then uses a lightweight UDP socket abstraction to pass sensor data to a remote host. The chunk callback serializes the `ads1299_sample_t` data into the payload and transmits it with minimal CPU overhead.

## Safe configuration convention used here

The example follows the driver convention that all register updates preserve unrelated bits and reject writes while the device is in RDATAC mode:

```c
ESP_ERROR_CHECK(ads1299_set_srb1(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_bias_enabled(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, ADS1299_PGA_GAIN_24));
```

## Quick start

```console
idf.py set-target esp32s3
idf.py build flash monitor
```

Before running, update the Wi-Fi credentials in `main.cpp` to match the network that will receive the UDP packets.
