#include <inttypes.h>
#include <bitset>
#include <array>
#include <cmath>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

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
#define RINGBUF_SIZE_BYTES     (1024 * 64)
#define N_SAMPLE_CHUNKS 50
#define DRIVER_TASK_CORE       0
#define LEADOFF_TASK_CORE      1
#define CHANNEL_GAIN           ADS1299_PGA_GAIN_24

// Global State
static RingbufHandle_t s_sampling_ringbuf = nullptr;

static const char *TAG = "ADS1299";
static const char *TASK_TAG = "LEADOFF";

void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{
    if (!chunk || !chunk->samples || chunk->n_samples == 0) {
        return;
    }

    uint16_t payload_len = chunk->n_samples * sizeof(ads1299_sample_t);
    void *frame_ptr = nullptr;

    // Acquire a block directly from the ring buffer
    xRingbufferSendAcquire(s_sampling_ringbuf, &frame_ptr, payload_len, pdMS_TO_TICKS(5));

    if (frame_ptr == nullptr) {
        ESP_LOGW(TAG, "Ringbuffer full! Dropped %d samples", chunk->n_samples);
        return;
    }

    // Write Payload
    memcpy(frame_ptr, chunk->samples, payload_len);

    // Release memory to reader task
    xRingbufferSendComplete(s_sampling_ringbuf, frame_ptr);
}
[[noreturn]] void check_dc_leadoff_task(void *arg) {
    size_t item_size;

    // Rate limiting configuration
    const TickType_t log_interval_ticks = pdMS_TO_TICKS(1000); // Log once per second (1000 ms)
    TickType_t last_log_time = xTaskGetTickCount();

    for (;;) {
        // Continuous, non-blocking drain: consume items as fast as they arrive
        void *item = xRingbufferReceive(s_sampling_ringbuf, &item_size, portMAX_DELAY);

        if (item != nullptr) {
            TickType_t now = xTaskGetTickCount();

            // Only print if the logging interval has elapsed
            if ((now - last_log_time) >= log_interval_ticks) {
                last_log_time = now;

                auto *chunk_samples = static_cast<ads1299_sample_t *>(item);

                // Extract the 8-bit status values from the sample struct
                uint8_t sensp = chunk_samples[1].status[1];
                uint8_t sensn = chunk_samples[1].status[2];

                ESP_LOGI(TASK_TAG, "=== ADS1299 LEAD-OFF STATUS ===");
                ESP_LOGI(TASK_TAG, "CH  |  INxP (Positive)  |  INxN (Negative)");
                ESP_LOGI(TASK_TAG, "------------------------------------------");

                for (int ch = 0; ch < 8; ch++) {
                    // Bits are ordered Ch1 (bit 0) to Ch8 (bit 7)
                    bool p_disconnected = (sensp >> ch) & 1;
                    bool n_disconnected = (sensn >> ch) & 1;

                    ESP_LOGI(TASK_TAG, "CH%d |     %s     |     %s",
                             ch + 1,
                             p_disconnected ? "🚨 DISCONNECTED" : "🟢 CONNECTED    ",
                             n_disconnected ? "🚨 DISCONNECTED" : "🟢 CONNECTED    ");
                }
            }

            // IMMEDIATELY return the buffer item on every iteration to keep memory free
            vRingbufferReturnItem(s_sampling_ringbuf, item);
        }
    }
}

extern "C" [[noreturn]] void app_main(void) {

    // 1. Pre-drive hardware control lines before powering rails
    gpio_config_t predrive_cfg = {
        .pin_bit_mask = (1ULL << RESET_PIN) | (1ULL << START_PIN) | (1ULL << CS1_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&predrive_cfg);
    gpio_set_level(RESET_PIN, 0);   // Hold reset low
    gpio_set_level(START_PIN, 0);
    gpio_set_level(CS1_PIN, 1);     // Deassert chip select

    // 2. Configure and enable analog power rail LDO
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

    // 3. Initialize SPI Bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = MISO_PIN;
    bus_cfg.mosi_io_num = MOSI_PIN;
    bus_cfg.sclk_io_num = SCLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // 4. Configure & initialize ADS1299 device
    ads1299_config_t cfg1 = {};
    cfg1.spi_host = SPI2_HOST;
    cfg1.cs_pin = CS1_PIN;
    cfg1.drdy_pin = DRDY_PIN;
    cfg1.start_pin = START_PIN;
    cfg1.reset_pin = RESET_PIN;
    cfg1.sample_rate = ADS1299_DR_250SPS;

    auto dev1 = ads1299_create(&cfg1);
    ESP_ERROR_CHECK(ads1299_init(&dev1));

    // 5. Create No-Split ring buffer
    s_sampling_ringbuf = xRingbufferCreate(RINGBUF_SIZE_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_sampling_ringbuf == nullptr) {
        ESP_LOGE(TAG, "Failed to create sampling ring buffer");
        abort();
    }

    // 6. Register Configuration (Must be in SDATAC mode)
    ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_MISC1, ADS1299_MISC1_SRB1_ON));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG4, ADS1299_CONFIG4_LOFF_COMP_ON));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_LOFF, 0x00));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_LOFF_SENSP, 0xFF));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_LOFF_SENSN, 0xFF));
    vTaskDelay(pdMS_TO_TICKS(250));

    ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, CHANNEL_GAIN));

    // 7. Enable continuous read mode (RDATAC)
    ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev1));

    // 8. Configure driver task execution parameters
    ads1299_continuous_config_t cont_cfg = {};
    cont_cfg.on_chunk = on_chunk;
    cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS;
    cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = DRIVER_TASK_CORE;

    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));
    ESP_ERROR_CHECK(ads1299_start(&dev1));

    // 9. Spawn the processing task on Core 1
    xTaskCreatePinnedToCore(
        check_dc_leadoff_task,
        "dc_task",
        4096,
        nullptr,
        5,
        nullptr,
        LEADOFF_TASK_CORE
    );

    // 10. Prevent app_main from returning and destroying its execution context
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}