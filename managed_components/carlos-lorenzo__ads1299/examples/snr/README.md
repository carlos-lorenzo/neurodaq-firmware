# ADS1299 SNR Measurement Example

This example measures the per-channel noise floor and compares it with the ADS1299's internal test signal to estimate signal-to-noise ratio (SNR).

## What it does

- Configures the ADS1299 for a low-noise shorted-input baseline measurement.
- Reconfigures CONFIG2 to enable the internal test signal.
- Applies uniform gain to all channels with the driver helper API.
- Captures a fixed number of chunks from the ring buffer.
- Computes AC RMS noise and signal power per channel and logs the resulting SNR.

## Procedure

The example runs two successive measurements:

1. Shorted-input measurement determines the baseline noise floor.
2. Internal test-signal measurement determines the signal power.

For each channel, it calculates:

- DC offset = mean of the sample set
- AC variance = $E[x^2] - E[x]^2$
- AC RMS = $\sqrt{variance}$
- SNR = signal variance / noise variance, reported in dB

## Safe configuration pattern used here

The example uses the new API conventions instead of manually packing raw register values:

```c
ESP_ERROR_CHECK(ads1299_set_bias_enabled(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, CHANNEL_GAIN));
ESP_ERROR_CHECK(ads1299_set_all_channels_mux(&dev1, ADS1299_INPUT_SHORTED));
```

The device is returned to SDATAC before register configuration changes, and the driver rejects WREG writes while RDATAC is active.

## Quick start

```console
idf.py set-target esp32s3
idf.py build flash monitor
```

The example expects the ESP32 SPI and ADS1299 control pins to match the constants defined at the top of `main.cpp`.
