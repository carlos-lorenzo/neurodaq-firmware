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
#include "driver/uart.h"
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

static const char *TAG = "ADS1299";


/**
 * @brief Binary telemetry frame header
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /**< Synchronization bytes [0xAA, 0x55] */
    uint16_t length;        /**< Payload length in bytes */
    uint32_t chunk_seq;     /**< Monotonically increasing sequence number */
} telemetry_header_t;

/* ── Global State ──────────────────────────────────────────────────────── */
static RingbufHandle_t s_telemetry_ringbuf = NULL;
static uint32_t s_chunk_sequence = 0;

static const char *TAG_TLM = "TELEMETRY";

/**
 * @brief Dedicated task to drain the ring buffer and transmit via UART
 */
/**
 * @brief Dedicated task to drain the ring buffer and transmit via native USB
 */
static void telemetry_tx_task(void *arg)
{
    size_t item_size;

    while (1) {
        void *item = xRingbufferReceive(s_telemetry_ringbuf, &item_size, portMAX_DELAY);

        if (item != NULL) {
            // Bypass stdout and VFS completely. Write directly to the USB hardware FIFO.
            // This prevents the OS from corrupting binary 0x0A bytes into 0x0D 0x0A.
            usb_serial_jtag_write_bytes((const char *)item, item_size, portMAX_DELAY);

            vRingbufferReturnItem(s_telemetry_ringbuf, item);
        }
    }
}

void telemetry_init(void)
{
    // 1. Install the explicit USB Serial/JTAG driver for raw binary output
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 4096, // Generous TX buffer for high-speed streaming
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
    if (s_telemetry_ringbuf == NULL) {
        printf("Failed to create telemetry ring buffer\n");
        abort();
    }

    // 4. Spawn the dedicated TX task on Core 1
    xTaskCreatePinnedToCore(
        telemetry_tx_task,
        "tlm_tx_task",
        4096,
        NULL,
        TELEMETRY_TASK_PRIO,
        NULL,
        TELEMETRY_TASK_CORE
    );
}


// void on_chunk(const ads1299_chunk_t *chunk, void *ctx)
// {
//     // Defensive check to avoid null pointer dereferencing
//     if (!chunk || !chunk->samples || chunk->n_samples == 0) {
//         return;
//     }
//
//     // 1. Correctly log chunk metadata using 64-bit specifiers for timestamps
//     ESP_LOGI(TAG, "Chunk: %zu samples | Time: %" PRId64 " to %" PRId64 " us | Dropped: %" PRId64,
//              chunk->n_samples,
//              chunk->first_timestamp_us,
//              chunk->last_timestamp_us,
//              chunk->dropped_count);
//
//     // 2. FIXED: Print multiple channels from the FIRST sample (index 0) of this chunk.
//     // Explicitly use %"PRId32" to match the int32_t channel array precisely.
//     ESP_LOGI(TAG, "Sample[0] Data -> CH1: %" PRId32 " | CH2: %" PRId32 " | CH3: %" PRId32 " | CH4: %" PRId32 " | CH5: %" PRId32 ,
//              chunk->samples[0].channels[0],
//              chunk->samples[0].channels[1],
//              chunk->samples[0].channels[2],
//              chunk->samples[0].channels[3],
//               chunk->samples[0].channels[4]);
// }


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

    if (frame_ptr == NULL) {
        // Buffer overflow! The TX task isn't draining fast enough.
        //ESP_LOGW(TAG_TLM, "Ringbuffer full! Dropped %d samples", chunk->n_samples);
        return;
    }

    // Cast the acquired memory pointer for sequential writing
    uint8_t *write_ptr = (uint8_t *)frame_ptr;

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
    const uint8_t *payload_ptr = (const uint8_t *)chunk->samples;
    for (size_t i = 0; i < payload_len; i++) {
        checksum ^= payload_ptr[i];
    }
    write_ptr += payload_len;
    *write_ptr = checksum;

    // Release the acquired memory back to the ring buffer for the TX task to consume
    xRingbufferSendComplete(s_telemetry_ringbuf, frame_ptr);
}

extern "C" void app_main(void)
{
    gpio_config_t analog_power_cfg = {
        .pin_bit_mask = (1ULL << ANPWREN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&analog_power_cfg));

    ESP_ERROR_CHECK(gpio_set_level(ANPWREN_PIN, 1));  // Enable LDO

    // Wait for analog rails to settle
    vTaskDelay(pdMS_TO_TICKS(250));

    spi_bus_config_t bus_cfg = {};
    bus_cfg.miso_io_num = MISO_PIN;
    bus_cfg.mosi_io_num = MOSI_PIN;
    bus_cfg.sclk_io_num = SCLK_PIN;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = ADS1299_FRAME_SIZE * 25;


    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO)
    );

    ads1299_config_t cfg1  = {};
    cfg1.spi_host = SPI2_HOST;
    cfg1.cs_pin = CS1_PIN;
    cfg1.drdy_pin = DRDY_PIN;
    cfg1.start_pin = START_PIN;
    cfg1.reset_pin = RESET_PIN;
    cfg1.sample_rate = ADS1299_DR_250SPS;
    ads1299_t dev1 = ads1299_create(&cfg1);

    ESP_ERROR_CHECK(ads1299_init(&dev1));

    // Run some configs...
    // ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));
    // uint8_t device_id = 1;
    // ESP_ERROR_CHECK(ads1299_read_register(&dev1, ADS1299_REG_ID, &device_id));
    // ESP_LOGI(TAG, "Device ID: %d", device_id);

    ESP_ERROR_CHECK(ads1299_disable_continuous_read(&dev1));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_MISC1, ADS1299_MISC1_SRB1_ON));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG3, ADS1299_CONFIG3_BIAS_ON));
    ESP_ERROR_CHECK(ads1299_set_all_channels_gain(&dev1, ADS1299_PGA_GAIN_12));

    // 1. Enter RDATAC (Read Data Continuous) mode
    ESP_ERROR_CHECK(ads1299_enable_continuous_read(&dev1));

    // 2. Configure continuous acquisition parameters
    ads1299_continuous_config_t cont_cfg = {};
    cont_cfg.on_chunk = on_chunk;
    cont_cfg.chunk_duration_ms = ADS1299_DEFAULT_CHUNK_MS; // 100 ms chunks
    cont_cfg.ring_buffer_chunks = ADS1299_RING_BUF_SLOTS;  // 8 chunks in ring buffer
    cont_cfg.task_priority = configMAX_PRIORITIES - 2;
    cont_cfg.task_core = 0;

    telemetry_init();

    ESP_ERROR_CHECK(ads1299_start_continuous(&dev1, &cont_cfg));

    // 3. CRITICAL: Start conversions so the ADC begins pulsing DRDY and triggering interrupts!
    ESP_ERROR_CHECK(ads1299_start(&dev1));



    // // 2. Configure a dedicated microsecond-resolution hardware timer
    // esp_timer_create_args_t periodic_timer_args = {}; // Zero-initialize everything first
    // periodic_timer_args.callback = &emulated_drdy_timer_callback;
    // periodic_timer_args.name = "emulated_drdy";
    //
    // esp_timer_handle_t periodic_timer;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    //
    // // 3. Start the timer at exactly 4000 microseconds (250 Hz)
    // // This runs completely independently of FreeRTOS task slicing
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 4000));

    // Fall into standard execution loop
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

