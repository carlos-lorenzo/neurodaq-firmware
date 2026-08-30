# ADS1299 Lead-Off Detection Example

This example configures the ADS1299 lead-off detection path and monitors each channel for positive or negative electrode disconnect conditions while the device is streaming.

## What it does

- Enables the lead-off compensation path in CONFIG4.
- Configures SRB1 and the ADS1299 lead-off sense registers for all channels.
- Enables all-channel gain and starts normal continuous acquisition.
- Reads the driver callbacks and inspects the lead-off status bits to determine whether an electrode is disconnected on either side.

## How it works

The example uses the driver callbacks to receive chunked samples and then checks the status bytes with the channel bit masks described in the ADS1299 datasheet:

- `ADS1299_REG_LOFF_SENSP` / `ADS1299_REG_LOFF_SENSN` select which channels participate in lead-off detection.
- The status registers `ADS1299_REG_LOFF_STATP` and `ADS1299_REG_LOFF_STATN` report per-channel disconnect events.
- The application interprets each bit as a positive or negative electrode status flag.

The configuration is established in SDATAC mode before streaming begins, and then the driver is switched to RDATAC for the acquisition loop.

## Safe configuration pattern used here

The example uses the high-level helpers instead of raw writes for bitfield updates:

```c
ESP_ERROR_CHECK(ads1299_set_srb1(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_config4_loff_comp(&dev1, true));
ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, ADS1299_PGA_GAIN_24));
ESP_ERROR_CHECK(ads1299_set_all_loff_sense(&dev1, true, 0xFF));
ESP_ERROR_CHECK(ads1299_set_all_loff_sense(&dev1, false, 0xFF));
```

These helpers keep unrelated bits untouched and respect the driver's RDATAC guard.

## Quick start

```console
idf.py set-target esp32s3
idf.py build flash monitor
```

The example expects the analog power pin and SPI pin mapping exactly as defined at the top of `main.cpp`.
