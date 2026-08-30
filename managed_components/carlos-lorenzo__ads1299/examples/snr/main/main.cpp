#include <inttypes.h>
#include <bitset>
#include <array>
#include <cmath>

#include "esp_check.h"
#include "esp_log.h"

#include "ads1299.h"
#include "ads1299_defs.h"

#define DRDY_PIN GPIO_NUM_8
#define MISO_PIN GPIO_NUM_9
#define SCLK_PIN GPIO_NUM_10
#define CS1_PIN GPIO_NUM_11
#define START_PIN GPIO_NUM_12
#define RESET_PIN GPIO_NUM_13
#define MOSI_PIN GPIO_NUM_14
#define CS2_PIN GPIO_NUM_21
#define ANPWREN_PIN GPIO_NUM_6



// Sampling config
#define RINGBUF_SIZE_BYTES     (1024 * 16)
#define DRIVER_TASK_CORE 1
#define N_SAMPLE_CHUNKS 10 // Number of chunks to be sampled. If chunk duration set to ADS1299_DEFAULT_CHUNK_MS, sample will be 10 * 100ms = 1s long
#define CHANNEL_GAIN ADS1299_PGA_GAIN_24


// Global State
static RingbufHandle_t s_sampling_ringuf = nullptr;

static const char *TAG = "ADS1299";

void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{
    if (!chunk || !chunk->samples || chunk->n_samples == 0) {
        return;
    }

    uint16_t payload_len = chunk->n_samples * sizeof(ads1299_sample_t);
    size_t total_frame_size = payload_len;
    void *frame_ptr = nullptr;
    // Use xRingbufferSendAcquire to grab a block of memory directly from the ring buffer.
    // This avoids needing a secondary intermediate buffer on the stack.
    xRingbufferSendAcquire(s_sampling_ringuf, &frame_ptr, total_frame_size, pdMS_TO_TICKS(5));

    if (frame_ptr == nullptr) {
        // Buffer overflow! The TX task isn't draining fast enough.
        ESP_LOGW(TAG, "Ringbuffer full! Dropped %d samples", chunk->n_samples);
        return;
    }

    // ESP_LOGI(TAG, "Dropped count: %d | Overflow count: %d", chunk->dropped_count, chunk->overflow_count);

    // Cast the acquired memory pointer for sequential writing
    auto *write_ptr = static_cast<uint8_t *>(frame_ptr);

    // 2. Write Payload
    memcpy(write_ptr, chunk->samples, payload_len);


    // Release the acquired memory back to the ring buffer for the TX task to consume
    xRingbufferSendComplete(s_sampling_ringuf, frame_ptr);
}



extern "C" void app_main(void) {


    // Before touching ANPWREN at all:
    gpio_config_t predrive_cfg = {
        .pin_bit_mask = (1ULL << RESET_PIN) | (1ULL << START_PIN) | (1ULL << CS1_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&predrive_cfg);
    gpio_set_level(RESET_PIN, 0);   // hold in reset through power-up
    gpio_set_level(START_PIN, 0);
    gpio_set_level(CS1_PIN, 1);     // deasserted


    // Configuring and enabling the LDO which regulates analog power
    gpio_config_t analog_power_cfg = {
        .pin_bit_mask = (1ULL << ANPWREN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&analog_power_cfg));

    ESP_ERROR_CHECK(gpio_set_level(ANPWREN_PIN, 1));

    gpio_config_t miso_cfg = {
        .pin_bit_mask = (1ULL << MISO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&miso_cfg);

    // Wait for analog rails to settle
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure the SPI bus (user's responsibility)
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = MISO_PIN;
    bus_cfg.mosi_io_num = MOSI_PIN;
    bus_cfg.sclk_io_num = SCLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25;


    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));


    // Configure and initialize the ADS1299
    ads1299_config_t cfg1  = {};
    cfg1.spi_host = SPI2_HOST;
    cfg1.cs_pin = CS1_PIN;
    cfg1.drdy_pin = DRDY_PIN;
    cfg1.start_pin = START_PIN;
    cfg1.reset_pin = RESET_PIN;
    cfg1.sample_rate = ADS1299_DR_250SPS;

    auto dev1 = ads1299_create(&cfg1);

    ESP_ERROR_CHECK(ads1299_init(&dev1));


    // Buffer config
    // Create the No-Split ring buffer for IPC between Core 0 and Core 1
    s_sampling_ringuf = xRingbufferCreate(RINGBUF_SIZE_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_sampling_ringuf == nullptr) {
        printf("Failed to create sampling ring buffer\n");
        abort();
    }



    // ADS1299 must be in SDATAC mode to be configured
    ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));

    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG3, ADS1299_CONFIG3_BIAS_ON));
    vTaskDelay(pdMS_TO_TICKS(250));

    // Shorting the inputs to measure baseline noise
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG2, 0xC0));
    for (uint8_t i = 1; i <= 8; i++) {
        ESP_ERROR_CHECK(ads1299_write_register(&dev1, (ADS1299_REG_CH1SET + (i - 1)), 0x01));
    }
    ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, CHANNEL_GAIN));




    // ADS1299 must be in RDATAC mode to be read
    ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev1));

    // Configure the chunk size (how often a chunk is outputted by the driver) and the core it will run one
    ads1299_continuous_config_t cont_cfg = {};
    cont_cfg.on_chunk = on_chunk; // What happens when a chunk is created. In this example, the data is streamed as raw bytes over serial
    cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms chunks
    cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = DRIVER_TASK_CORE;


    // Apply the config. Different from setting RDATAC. RDATAC is a register inside the ADS1299, this configures the required memory to efficiently transfer data
    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));

    // CRITICAL: Start conversions so the ADC begins pulsing DRDY and triggering interrupts. RDATAC can be enabled but if it's not started, DRDY (data ready) won't pulse.
    // Similarly, you can call ads1299_stop to stop DRDY from pulsing
    ESP_ERROR_CHECK(ads1299_start(&dev1));
    vTaskDelay(pdMS_TO_TICKS(25));


    // Shorted Inputs (Noise Floor)
    std::array<double, ADS1299_NUM_CHANNELS> shorted_sum_uv{};
    std::array<double, ADS1299_NUM_CHANNELS> shorted_sum_sq_uv{};
    size_t total_shorted_samples = 0;

    for (int n_chunk = 0; n_chunk < N_SAMPLE_CHUNKS; n_chunk++) {
        size_t item_size = 0;
        void *item = xRingbufferReceive(s_sampling_ringuf, &item_size, portMAX_DELAY);
        if (item != nullptr) {
            auto chunk_samples = static_cast<ads1299_sample_t *>(item);
            size_t num_samples = item_size / sizeof(ads1299_sample_t);

            for (size_t sample = 0; sample < num_samples; sample++) {
                for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
                    int32_t raw_count = chunk_samples[sample].channels[ch];
                    double uV = ads1299_count_to_microvolts(raw_count, 4500, CHANNEL_GAIN);

                    shorted_sum_uv[ch] += uV;
                    shorted_sum_sq_uv[ch] += (uV * uV);
                }
            }
            total_shorted_samples += num_samples;
            vRingbufferReturnItem(s_sampling_ringuf, item);
        }
    }

    // Compute & Log True AC Noise Floor
    ESP_LOGI(TAG, "Shorted Inputs (AC Noise Floor)");
    std::array<double, ADS1299_NUM_CHANNELS> ac_noise_var_uv{};
    std::array<double, ADS1299_NUM_CHANNELS> ac_noise_rms_uv{};

    if (total_shorted_samples > 0) {
        for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
            double dc_mean = shorted_sum_uv[ch] / total_shorted_samples;
            double total_mean_sq = shorted_sum_sq_uv[ch] / total_shorted_samples;

            // True AC Variance = E[X^2] - (E[X])^2
            double variance = total_mean_sq - (dc_mean * dc_mean);
            if (variance < 0.0) variance = 0.0; // Protection against floating-point precision error

            ac_noise_var_uv[ch] = variance;
            ac_noise_rms_uv[ch] = sqrt(variance);

            ESP_LOGI(TAG, "CH%d - DC Offset: %.2f uV | AC Noise RMS: %.3f uV_rms (Var: %.4f uV^2)",
                     ch + 1, dc_mean, ac_noise_rms_uv[ch], variance);
        }
    }




    ESP_ERROR_CHECK(ads1299_stop(&dev1));

    ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));


    // Test Signal (2mv Amplitude)
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG2, 0xD0));
    vTaskDelay(pdMS_TO_TICKS(25));
    for (uint8_t i = 1; i <= 8; i++) {
        ESP_ERROR_CHECK(ads1299_write_register(&dev1, (ADS1299_REG_CH1SET + (i - 1)), 0x05));
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, CHANNEL_GAIN));

    // ADS1299 must be in RDATAC mode to be read
    ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev1));


    ESP_ERROR_CHECK(ads1299_start(&dev1));
    vTaskDelay(pdMS_TO_TICKS(25));

    // Test Signal
    std::array<double, ADS1299_NUM_CHANNELS> signal_sum_uv{};
    std::array<double, ADS1299_NUM_CHANNELS> signal_sum_sq_uv{};
    size_t total_signal_samples = 0;

    for (int n_chunk = 0; n_chunk < N_SAMPLE_CHUNKS; n_chunk++) {
        size_t item_size = 0;
        void *item = xRingbufferReceive(s_sampling_ringuf, &item_size, portMAX_DELAY);
        if (item != nullptr) {
            auto chunk_samples = static_cast<ads1299_sample_t *>(item);
            size_t num_samples = item_size / sizeof(ads1299_sample_t);

            for (size_t sample = 0; sample < num_samples; sample++) {
                for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
                    int32_t raw_count = chunk_samples[sample].channels[ch];
                    double uV = ads1299_count_to_microvolts(raw_count, 4500, CHANNEL_GAIN);

                    signal_sum_uv[ch] += uV;
                    signal_sum_sq_uv[ch] += (uV * uV);
                }
            }
            total_signal_samples += num_samples;
            vRingbufferReturnItem(s_sampling_ringuf, item);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(ads1299_stop(&dev1));

    // Compute & Log Test Signal AC RMS Power
    ESP_LOGI(TAG, "Test Signal Evaluation");
    std::array<double, ADS1299_NUM_CHANNELS> signal_ac_var_uv{};
    std::array<double, ADS1299_NUM_CHANNELS> signal_ac_rms_uv{};

    if (total_signal_samples > 0) {
        for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
            double dc_mean = signal_sum_uv[ch] / total_signal_samples;
            double total_mean_sq = signal_sum_sq_uv[ch] / total_signal_samples;

            // True AC Signal Variance = E[X^2] - (E[X])^2
            double variance = total_mean_sq - (dc_mean * dc_mean);
            if (variance < 0.0) variance = 0.0;

            signal_ac_var_uv[ch] = variance;
            signal_ac_rms_uv[ch] = sqrt(variance);

            ESP_LOGI(TAG, "CH%d - Signal DC Offset: %.2f uV | Signal AC RMS: %.2f uV_rms",
                     ch + 1, dc_mean, signal_ac_rms_uv[ch]);
        }
    }

    // ==========================================
    // 3. COMPUTE TRUE SNR
    // ==========================================
    ESP_LOGI(TAG, "Computed True SNR");
    if (total_shorted_samples > 0 && total_signal_samples > 0) {
        for (int ch = 0; ch < ADS1299_NUM_CHANNELS; ch++) {
            if (ac_noise_var_uv[ch] > 0.0) {
                // Power Ratio SNR = Signal AC Variance / Noise AC Variance
                double snr_ratio = signal_ac_var_uv[ch] / ac_noise_var_uv[ch];

                // Decibel SNR = 10 * log10(Power Ratio) OR 20 * log10(Voltage Ratio)
                double snr_db = 10.0 * log10(snr_ratio);

                ESP_LOGI(TAG, "CH%d - True SNR Ratio: %.2f | SNR dB: %.2f dB",
                         ch + 1, snr_ratio, snr_db);
            } else {
                ESP_LOGW(TAG, "CH%d - Noise variance was 0, cannot compute SNR", ch + 1);
            }
        }
    }

}
