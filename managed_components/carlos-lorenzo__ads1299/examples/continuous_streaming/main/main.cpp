#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"



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


/* ── Hardware & Task Config ────────────────────────────────────────────── */
// Remove the UART1 defines, keep the Ring Buffer defines
#define TELEMETRY_TASK_CORE    1
#define TELEMETRY_TASK_PRIO    5
#define RINGBUF_SIZE_BYTES     (1024 * 16)

/* ── Protocol Config ───────────────────────────────────────────────────── */
#define FRAME_SYNC_0           0xAA
#define FRAME_SYNC_1           0x55


#define CHANNEL_GAIN ADS1299_PGA_GAIN_24
#define SAMPLE_RATE ADS1299_DR_250SPS

/**
 * @brief Binary telemetry frame header
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /**< Synchronization bytes [0xAA, 0x55] */
    uint16_t length;        /**< Payload length in bytes */
    uint32_t chunk_seq;     /**< Monotonically increasing sequence number */
} telemetry_header_t;


/* ── Global State ──────────────────────────────────────────────────────── */
static RingbufHandle_t s_telemetry_ringbuf = nullptr;
static uint32_t s_chunk_sequence = 0;



/**
 * @brief Dedicated task to drain the ring buffer and transmit via native USB
 */
[[noreturn]] static void telemetry_tx_task(void *arg)
{
    size_t item_size;

    for (;;) {
        void *item = xRingbufferReceive(s_telemetry_ringbuf, &item_size, portMAX_DELAY);

        if (item != nullptr) {
            // Bypass stdout and VFS completely. Write directly to the USB hardware FIFO.
            // This prevents the OS from corrupting binary 0x0A bytes into 0x0D 0x0A.
            usb_serial_jtag_write_bytes((const char *)item, item_size, portMAX_DELAY);

            vRingbufferReturnItem(s_telemetry_ringbuf, item);
        }
    }
}

void telemetry_init()
{
    // 1. Install the explicit USB Serial/JTAG driver for raw binary output
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 4096 * 4, // Generous TX buffer for high-speed streaming
        .rx_buffer_size = 256   // Minimal RX buffer (we are mostly transmitting)
    };

    // Install the driver. Ignore the error if it was already installed by a bootloader/console config.
    esp_err_t err = usb_serial_jtag_driver_install(&usb_config);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("Failed to install USB Serial/JTAG driver: %s\n", esp_err_to_name(err));
        abort();
    }

    // 2. Silence ESP_LOGs so they don't periodically corrupt the binary stream
    esp_log_level_set("*", ESP_LOG_WARN);

    // 3. Create the No-Split ring buffer for IPC between Core 0 and Core 1
    s_telemetry_ringbuf = xRingbufferCreate(RINGBUF_SIZE_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_telemetry_ringbuf == nullptr) {
        printf("Failed to create telemetry ring buffer\n");
        abort();
    }

    // 4. Spawn the dedicated TX task on Core 1
    xTaskCreatePinnedToCore(
        telemetry_tx_task,
        "tlm_tx_task",
        4096,
        nullptr,
        TELEMETRY_TASK_PRIO,
        nullptr,
        TELEMETRY_TASK_CORE
    );
}

void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
{
    if (!chunk || !chunk->samples || chunk->n_samples == 0) {
        return;
    }

    uint16_t payload_len = chunk->n_samples * sizeof(ads1299_sample_t);
    size_t total_frame_size = sizeof(telemetry_header_t) + payload_len + 1; // +1 for checksum
    void *frame_ptr = nullptr;
    // Use xRingbufferSendAcquire to grab a block of memory directly from the ring buffer.
    // This avoids needing a secondary intermediate buffer on the stack.
    xRingbufferSendAcquire(s_telemetry_ringbuf, &frame_ptr, total_frame_size, pdMS_TO_TICKS(5));

    if (frame_ptr == nullptr) {
        // Buffer overflow! The TX task isn't draining fast enough.
        //ESP_LOGW(TAG_TLM, "Ringbuffer full! Dropped %d samples", chunk->n_samples);
        return;
    }

    // Cast the acquired memory pointer for sequential writing
    auto *write_ptr = static_cast<uint8_t *>(frame_ptr);

    // 1. Write Header
    telemetry_header_t header = {
        .sync = {FRAME_SYNC_0, FRAME_SYNC_1},
        .length = payload_len,
        .chunk_seq = s_chunk_sequence++
    };
    memcpy(write_ptr, &header, sizeof(telemetry_header_t));
    write_ptr += sizeof(telemetry_header_t);

    // 2. Write Payload
    memcpy(write_ptr, chunk->samples, payload_len);

    // 3. Calculate and Write Checksum
    uint8_t checksum = 0;
    const auto *payload_ptr = reinterpret_cast<const uint8_t *>(chunk->samples);
    for (size_t i = 0; i < payload_len; i++) {
        checksum ^= payload_ptr[i];
    }
    write_ptr += payload_len;
    *write_ptr = checksum;

    // Release the acquired memory back to the ring buffer for the TX task to consume
    xRingbufferSendComplete(s_telemetry_ringbuf, frame_ptr);
}

extern "C" [[noreturn]] void app_main(void)
{
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

    // Wait for analog rails to settle
    vTaskDelay(pdMS_TO_TICKS(250));

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
    cfg1.sample_rate = SAMPLE_RATE;

    auto dev1 = ads1299_create(&cfg1);

    ESP_ERROR_CHECK(ads1299_init(&dev1));

    // ADS1299 must be in SDATAC mode to be configured
    ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));

    // Run your desired configs using the safe masked-register helpers.
    ESP_ERROR_CHECK(ads1299_set_srb1(&dev1, true));
    ESP_ERROR_CHECK(ads1299_set_bias_enabled(&dev1, true));
    ESP_ERROR_CHECK(ads1299_set_bias_sense(&dev1, 1, true, true));

    vTaskDelay(pdMS_TO_TICKS(25));

    ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, CHANNEL_GAIN));


    // ADS1299 must be in RDATAC mode be to read
    ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev1));

    // Configure the chunk size (how often a chunk is outputted by the driver) and the core it will run one
    ads1299_continuous_config_t cont_cfg = {};
    cont_cfg.on_chunk = on_chunk; // What happens when a chunk is created. In this example, the data is streamed as raw bytes over serial
    cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms chunks
    cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = 0;

    // Stream raw bits over serial
    telemetry_init();

    // Apply the config. Different from setting RDATAC. RDATAC is a register inside the ADS1299, this configures the required memory to efficiently transfer data
    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));

    // CRITICAL: Start conversions so the ADC begins pulsing DRDY and triggering interrupts. RDATAC can be enabled but if it's not started, DRDY (data ready) won't pulse.
    // Similarly, you can call ads1299_stop to stop DRDY from pulsing
    ESP_ERROR_CHECK(ads1299_start(&dev1));

    // Make the program run endlessly
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

