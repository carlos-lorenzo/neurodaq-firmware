# Testing

Host-side unit tests use Catch2 (v3) and build off-target with plain CMake — no ESP32
required:

```bash
cmake -S test -B test/build
cmake --build test/build
ctest --test-dir test/build --output-on-failure
```

## Status

All three tests (`test_frame_pool`, `test_biquad`, `test_window_accumulator`) are
placeholders that assert nothing. The harness exists so the first real test is one
commit away, and so CI stays green.

## What is testable off-target

Pure, hardware-independent logic can be unit-tested on the host:

- `eeg_core/frame_pool` — the lock-free refcount/free-list logic.
- `dsp/biquad`, `dsp/iir_filter_chain` — once implemented (filter math is pure).
- `ml_engine/window_accumulator` — once implemented (windowing is pure).

Anything that touches SPI, GPIO, FreeRTOS tasks, or the ADS1299 needs hardware and is
out of scope for these tests.
