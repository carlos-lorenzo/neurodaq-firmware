# neurodaq-firmware

ESP-IDF firmware for [NeuroDAQ](https://github.com/carlos-lorenzo/neurodaq): drives a
Texas Instruments **ADS1299** 8-channel 24-bit EEG front-end from an **ESP32-S3** over
SPI2, batches samples into 100 ms frames, and streams them over Wi-Fi (UDP) while
accepting configuration over a TCP JSON control server.

> **Status: under active development.** Acquisition, streaming, and control run
> end-to-end. On-device DSP and ML are scaffolded but not implemented. See
> [Component status](#component-status) and [Known limitations](#known-limitations).

- **Target:** ESP32-S3-DevKitC-1-**N8R2** (dual-core Xtensa LX7 @ 160 MHz)
- **Toolchain:** ESP-IDF **v6.0.2**
- **AFE:** ADS1299IPAGR over SPI2, via the [`carlos-lorenzo/ads1299`][drv] driver
  (pulled from the ESP component registry — not vendored in-tree)

[drv]: https://github.com/carlos-lorenzo/ads1299

## Quick start

```bash
source /opt/esp-idf/export.sh      # or: ./activate_idf.sh
idf.py set-target esp32s3
idf.py menuconfig                  # REQUIRED first step — see Configuration
idf.py -p /dev/ttyACM0 flash monitor
```

`sdkconfig` is **not tracked** — it is generated on first configure and gitignored
(it previously held Wi-Fi credentials). `menuconfig` is therefore a required step, not
an optional one: at minimum set the Wi-Fi SSID/password and the host UDP IP before the
first flash.

## Configuration

All options live under **Neurodaq Configuration** in `menuconfig`
(`main/Kconfig.projbuild`).

| Option | Default | Effect |
|---|---|---|
| `ESP_WIFI_SSID` | `myssid` | Wi-Fi network name. |
| `ESP_WIFI_PASSWORD` | `mypassword` | Wi-Fi password. |
| `ESP_WIFI_SAE_MODE` | BOTH | WPA3 SAE PWE mode. |
| `ESP_MAXIMUM_RETRY` | 5 | STA reconnect attempts before giving up. |
| `ESP_UDP_IP` | `192.168.1.57` | **Host IP for UDP telemetry — must match your host.** Compile-time pinned; reflash to change. |
| `ESP_UDP_PORT` | 3333 | UDP telemetry port (ESP32 → host). |
| `ESP_TCP_PORT` | 3334 | TCP control port (host → ESP32). |
| `ESP_TCP_TIMEOUT` | 5 | Control-socket timeout (s). |

### Task Core Affinity

The dual core split is configurable (`EEG_MANAGER_CORE`, `DSP_CORE`, `EDGE_ML_CORE`,
`EEG_STREAMER_CORE`, `CONTROL_SERVER_CORE`; each 0 or 1). Defaults keep the EEG
manager and streamer on core 1 and the control server on core 0. `DSP_CORE` and
`EDGE_ML_CORE` are reserved for components that are not yet implemented, but the split
documents the intended design.

## Wire protocol

Telemetry and control are specified once, in the umbrella repo:
[`neurodaq/docs/PROTOCOL.md`](https://github.com/carlos-lorenzo/neurodaq/blob/main/docs/PROTOCOL.md).
The format is currently defined implicitly by `TelemetryHeader` in
`components/eeg_core/include/eeg_core/eeg_types.hpp` and the scatter-gather assembly in
`components/telemetry/include/telemetry/eeg_streamer.hpp`; `telemetry/wire_protocol.hpp`
is a placeholder.

## Pin mapping

See [`pin_mappings.md`](pin_mappings.md) for the full ESP32-S3 ↔ ADS1299 / IMU /
battery table. Note `CS2_ADC` (GPIO21) is routed for a **second cascaded ADS1299 that
is neither populated nor driven by firmware**; the IMU and battery-management I2C pins
are mapped but currently unused.

## Component status

| Component | Status |
|---|---|
| `eeg_core`, `eeg_manager`, `telemetry`, `control_plane`, `app` | **Working** — acquisition, streaming, and control run end-to-end |
| `dsp` (biquad, IIR chain, DSP task) | Scaffolded, not implemented — filtering is host-side |
| `ml_engine` (window accumulator, model interface) | Scaffolded, not implemented |
| `telemetry/usb_jtag_transport` | Scaffolded — **Wi-Fi is the only working link** |
| `test/` (Catch2) | Placeholders — the harness builds and runs but asserts nothing |

`filtered_pool_` and `filtered_frame_queue_` in `AppContext` are allocated but unused,
reserved for the future DSP task.

## Testing

Host-side unit tests use Catch2:

```bash
cmake -S test -B test/build && cmake --build test/build && ctest --test-dir test/build
```

All three tests currently assert `true` — the harness exists so the first real test is
one commit away.

## Known limitations

These are documented, not yet fixed (tracked as issues):

- **Sample-rate switching is unreliable.** The CONFIG1 data-rate mask is too narrow, so
  1 k/500/250 SPS selections are truncated.
- **Lead-off configuration writes to the wrong register** (CONFIG4 instead of LOFF).
- **Frames are fixed at 25 samples**, so sample rates above 250 SPS overrun the frame
  array — 250 SPS is the only safe rate today.
- **Flash size is configured 32 MB** while the N8R2 module has 8 MB.
- **Wi-Fi failure is a silent no-op boot** — there is no offline fallback; if the STA
  can't associate, `app_main` returns and nothing runs.
- **Acquisition auto-starts** and most reconfiguration is rejected while running; use
  `stop` → `config_*` → `start`.

## Licence

MIT. See [`LICENSE`](LICENSE).
