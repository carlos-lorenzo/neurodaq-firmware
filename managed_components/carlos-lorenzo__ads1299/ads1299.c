#include <stdio.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"


#include "ads1299.h"

static const char *TAG = "ADS1299";

// ----------------------------------------
// Private helper functions to be used by the driver and not the user
// ----------------------------------------
struct ads1299_dma_ctx {

    /* ── DMA staging ─────────────────────────────────────────────── */
    uint8_t           *dma_buf;           /* 27-byte receive buffer; MALLOC_CAP_DMA  */
    spi_transaction_t  trans;             /* reused for every sample; configured once */
    volatile bool      spi_busy;          /* true while DMA in-flight                */
    int64_t            current_timestamp; /* DRDY edge time; written ISR, read post_cb */

    /* ── Ring buffer ──────────────────────────────────────────────── */
    ads1299_chunkring_t ring;
    uint32_t            sample_count;     /* samples written into ring's current slot */

    /* ── Handler task ─────────────────────────────────────────────── */
    TaskHandle_t        handler_task;
    SemaphoreHandle_t   done_sem;         /* task gives this before vTaskDelete       */
    volatile bool       stop_requested;

    /* ── Callbacks ────────────────────────────────────────────────── */
    ads1299_chunk_cb_t  on_chunk;
    ads1299_error_cb_t  on_error;
    void               *cb_ctx;

    /* ── Statistics ───────────────────────────────────────────────── */
    volatile uint32_t   dropped_count;    /* DRDY pulses dropped; spi_busy was set   */
    volatile uint32_t   overflow_count;   /* chunks dropped; ring was full            */
};


// Ring functions
/**
 *
 * @param r pointer to uninitialised struct
 * @param capacity number of chunk slots; must be power-of-2, >= 2
 * @param chunk_samples  samples per chunk; must be > 0
 * @return
 *
 *
* Validate: r != NULL, chunk_samples > 0, capacity >= 2, capacity & (capacity-1) == 0. Return ESP_ERR_INVALID_ARG otherwise.
Allocate capacity * chunk_samples * sizeof(ads1299_sample_t) bytes with malloc. Return ESP_ERR_NO_MEM if NULL.
Set r->buf = <allocated>, r->capacity = capacity, r->chunk_samples = chunk_samples, r->mask = capacity - 1.
Set r->head = 0, r->tail = 0.
Return ESP_OK.
 */
static esp_err_t ads1299_chunkring_init(ads1299_chunkring_t *r, uint32_t capacity, uint32_t chunk_samples)
{
    if (!r) {
        return ESP_ERR_INVALID_ARG;
    }
    if (chunk_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (capacity == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (capacity & (capacity - 1)) {
        return ESP_ERR_INVALID_ARG;   /* not power of 2 */
    }

    r->capacity = capacity;
    r->chunk_samples = chunk_samples;
    r->mask = capacity - 1;
    r->head = 0;
    r->tail = 0;
    r->buf = calloc(capacity * r->chunk_samples, sizeof(*r->buf));
    if (!r->buf) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void ads1299_chunkring_free(ads1299_chunkring_t *r)
{
    free(r->buf);
    memset(r, 0, sizeof(*r));
}
static uint32_t ads1299_chunkring_available(const ads1299_chunkring_t *r)
{
    return r->head - r->tail;   /* unsigned arithmetic; wraps correctly */
}

static uint32_t ads1299_chunkring_free_slots(const ads1299_chunkring_t *r)
{
    return (r->capacity - 1u) - (r->head - r->tail);
}
static ads1299_sample_t *ads1299_chunkring_write_ptr(const ads1299_chunkring_t *r, uint32_t sample_idx)
{
    return &r->buf[(r->head & r->mask) * r->chunk_samples + sample_idx];
}
static bool ads1299_chunkring_commit(ads1299_chunkring_t *r)
{
    if (ads1299_chunkring_free_slots(r) == 0) {
        return false;   /* ring full; caller increments overflow_count */
    }
    /* Release fence: all sample writes to buf[] must be visible to the
     * consumer before head is incremented. On Xtensa this emits MEMW. */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    r->head++;
    return true;
}

static const ads1299_sample_t *ads1299_chunkring_read_ptr(const ads1299_chunkring_t *r)
{
    /* Acquire fence: read all sample data written before the ISR's
     * release fence in commit(). */
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return &r->buf[(r->tail & r->mask) * r->chunk_samples];
}

static void ads1299_chunkring_release(ads1299_chunkring_t *r)
{
    /* Release fence: ensure tail increment is visible to the ISR's
     * free_slots() check before the ISR observes the freed space. */
    __atomic_thread_fence(__ATOMIC_RELEASE);
    r->tail++;
}



static void IRAM_ATTR spi_post_transfer_cb(spi_transaction_t *trans)
{
    if (!trans->user) return;   /* guard for non-DMA transactions (SDATAC etc.) */
    ads1299_dma_ctx_t *ctx = (ads1299_dma_ctx_t *)trans->user;

    /* ── 1. Parse DMA bytes into current ring slot ─────────────────── */
    /* dma_buf is stable: spi_busy has been true since the DRDY ISR
     * queued this transaction, preventing any new DMA from starting. */
    ads1299_sample_t *dst = ads1299_chunkring_write_ptr(&ctx->ring, ctx->sample_count);
    ads1299_parse_frame(ctx->dma_buf, ctx->current_timestamp, dst);

    /* ── 2. Advance sample position ────────────────────────────────── */
    /* Increment before clearing spi_busy so the next DRDY ISR
     * sees the correct write position if it fires immediately after. */
    ctx->sample_count++;

    /* ── 3. Release DMA buffer ──────────────────────────────────────── */
    /* Cleared AFTER parse. From this point, the next DRDY ISR may
     * set spi_busy and queue a new transaction that overwrites dma_buf. */
    ctx->spi_busy = false;

    /* ── 4. Check chunk completion ──────────────────────────────────── */
    if (ctx->sample_count < ctx->ring.chunk_samples) {
        return;   /* chunk still in progress */
    }

    ctx->sample_count = 0;

    /* ── 5. Commit slot to ring ─────────────────────────────────────── */
    if (!ads1299_chunkring_commit(&ctx->ring)) {
        /* Ring full: newest chunk dropped.
         * Same physical slot will be overwritten by the next chunk.
         * The handler task's tail pointer is untouched. */
        ctx->overflow_count++;
        return;
    }

    /* ── 6. Notify handler task ─────────────────────────────────────── */
    /* eSetValueWithOverwrite: if task has not woken yet and a previous
     * notification is pending, this overwrites it (same value = 1).
     * No notification is lost: the task drains all available chunks
     * in a loop regardless of how many notifications triggered it. */
    BaseType_t higher_woken = pdFALSE;
    xTaskNotifyFromISR(ctx->handler_task,
                       1u,
                       eSetValueWithOverwrite,
                       &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
    /* portYIELD_FROM_ISR sets a deferred flag; the SPI ISR completes
     * normally (puts trans on ret_queue), then the context switch fires
     * at ISR exit. Safe to call from within post_cb on ESP32/Xtensa. */
}


ads1299_t ads1299_create(const ads1299_config_t *cfg)
{
    ads1299_t dev = {0};
    if (cfg) {
        dev.config = *cfg;
    }

    return dev;
}

/* init */
esp_err_t ads1299_init(ads1299_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (dev->initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGI(TAG, "Initializing ADS1299");

    /* 1. GPIO setup */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << dev->config.reset_pin) | (1ULL << dev->config.start_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_cfg), TAG, "GPIO config failed");

    gpio_config_t drdy_config = {
        .pin_bit_mask = (1ULL << dev->config.drdy_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&drdy_config), TAG, "DRDY GPIO config failed");

    gpio_set_level(dev->config.reset_pin, 1);
    gpio_set_level(dev->config.start_pin, 0);

    /* 2. SPI bus device add (SPI Mode 1: CPOL=0, CPHA=1) */
    spi_device_interface_config_t dev_cfg = {
        .mode = 1,
        .clock_speed_hz = 4 * 1000 * 1000,
        .spics_io_num = dev->config.cs_pin,
        .cs_ena_pretrans = 4,
        .cs_ena_posttrans = 8,
        .queue_size = 25,
        .post_cb = NULL /* Set dynamically during continuous start */
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_device(dev->config.spi_host, &dev_cfg, &dev->spi_handle), TAG, "SPI add failed");

    dev->mutex = xSemaphoreCreateMutex();
    if (!dev->mutex) return ESP_ERR_NO_MEM;

    /* 3. Allow analog rails and VCAP1 to settle post power-on */
    vTaskDelay(pdMS_TO_TICKS(250));

    /* 4. Hardware reset pulse + mandatory post-reset delay */
    ESP_RETURN_ON_ERROR(ads1299_reset_hardware(dev), TAG, "Hardware reset failed");

    /* 5. Issue SDATAC to exit power-on default RDATAC mode before register access */
    ESP_RETURN_ON_ERROR(ads1299_disable_continuous_read(dev), TAG, "SDATAC failed");

    vTaskDelay(pdMS_TO_TICKS(50));

    /* 6. Verify Device ID */
    uint8_t dev_id = 0;
    ESP_RETURN_ON_ERROR(ads1299_read_register(dev, ADS1299_REG_ID, &dev_id), TAG, "Read ID failed");
    ESP_LOGI(TAG, "Hardware Device ID: 0x%02X (%d)", dev_id, dev_id);
    switch (dev_id & 0x1F) {
        case 0x1C:
            ESP_LOGI(TAG, "ADS1299-4 detected");
            break;

        case 0x1D:
            ESP_LOGI(TAG, "ADS1299-6 detected");
            break;

        case 0x1E:
            ESP_LOGI(TAG, "ADS1299-8 detected");
            break;

        default:
            ESP_LOGW(TAG, "Unknown device ID: 0x%02X", dev_id);
            break;
    }

    // uint8_t dev_id = 0;
    int attempts;
    for (attempts = 1; attempts <= 50; attempts++) {
        ESP_RETURN_ON_ERROR(ads1299_read_register(dev, ADS1299_REG_ID, &dev_id), TAG, "Read ID failed");
        if ((dev_id & 0x1F) != 0x00) break;
        // vTaskDelay(pdMS_TO_TICKS(5));
        esp_rom_delay_us(5000);
    }
    ESP_LOGI(TAG, "Hardware Device ID: 0x%02X after %d attempt(s)", dev_id, attempts);

    /* 7. Configure Data Rate and Internal Reference Buffer */
    ESP_RETURN_ON_ERROR(ads1299_write_register(dev, ADS1299_REG_CONFIG1, 0x90 | dev->config.sample_rate), TAG, "Write CONFIG1 failed");
    ESP_RETURN_ON_ERROR(ads1299_write_register(dev, ADS1299_REG_CONFIG3, 0xE0), TAG, "Write CONFIG3 failed");
    vTaskDelay(pdMS_TO_TICKS(200)); /* Allow internal VREF buffer to settle */

    /* 8. Baseline Short Evaluation (CHnSET = 0x01) */
    ESP_RETURN_ON_ERROR(ads1299_write_register(dev, ADS1299_REG_CONFIG2, 0xC0), TAG, "Write CONFIG2 failed");
    for (uint8_t i = 1; i <= 8; i++) {
        ESP_RETURN_ON_ERROR(ads1299_write_register(dev, (ADS1299_REG_CH1SET + (i - 1)), 0x01), TAG, "Write CHnSET failed");
    }

    /* Start active conversion to generate hardware DRDY pulses */
    ESP_RETURN_ON_ERROR(ads1299_start(dev), TAG, "START assertion failed");

    /* FIX: Allow internal digital filter to settle (4+ cycles at 250 SPS ~ 20ms) */
    vTaskDelay(pdMS_TO_TICKS(25));

    /* FIX: Wait for a fresh DRDY edge (high-to-low transition) */
    while (gpio_get_level(dev->config.drdy_pin) == 0); // Make sure it isn't already low
    while (gpio_get_level(dev->config.drdy_pin) == 1); // Wait for the drop edge

    ads1299_sample_t noise_sample;
    if (ads1299_read_sample(dev, &noise_sample) == ESP_OK) {
        ESP_LOGI(TAG, "Baseline Short Status Bytes: 0x%02X 0x%02X 0x%02X",
                 noise_sample.status[0], noise_sample.status[1], noise_sample.status[2]);
        ESP_LOGI(TAG, "CH1 Short Raw Counts: %ld", noise_sample.channels[0]);
    } else {
        ESP_LOGE(TAG, "Failed to read baseline short sample");
    }

    /* 9. Test Signal Verification (CHnSET = 0x05) */
    ESP_RETURN_ON_ERROR(ads1299_write_register(dev, ADS1299_REG_CONFIG2, 0xD0), TAG, "Write CONFIG2 test failed");
    for (uint8_t i = 1; i <= 8; i++) {
        ESP_RETURN_ON_ERROR(ads1299_write_register(dev, (ADS1299_REG_CH1SET + (i - 1)), 0x05), TAG, "Write CHnSET test failed");
    }

    /* FIX: Allow internal digital filter to settle to the test signal wave */
    vTaskDelay(pdMS_TO_TICKS(25));

    /* FIX: Wait for a fresh DRDY edge */
    while (gpio_get_level(dev->config.drdy_pin) == 0);
    while (gpio_get_level(dev->config.drdy_pin) == 1);

    ads1299_sample_t test_sample;
    if (ads1299_read_sample(dev, &test_sample) == ESP_OK) {
        ESP_LOGI(TAG, "Test Signal Status Bytes: 0x%02X 0x%02X 0x%02X",
                 test_sample.status[0], test_sample.status[1], test_sample.status[2]);
        ESP_LOGI(TAG, "CH1 Test Signal Raw Counts: %ld", test_sample.channels[0]);
    }

    /* 10. Stop conversions and restore normal channel configuration (0x00) */
    ESP_RETURN_ON_ERROR(ads1299_stop(dev), TAG, "STOP failed");
    ESP_RETURN_ON_ERROR(ads1299_write_register(dev, ADS1299_REG_CONFIG2, 0xC0), TAG, "Restore CONFIG2 failed");
    for (uint8_t i = 1; i <= 8; i++) {
        ESP_RETURN_ON_ERROR(ads1299_write_register(dev, (ADS1299_REG_CH1SET + (i - 1)), 0x00), TAG, "Restore CHnSET failed");
    }

    ESP_LOGI(TAG, "ADS1299 successfully initialized and ready for streaming");
    dev->initialized = true;
    return ESP_OK;
}

/* deinit */
esp_err_t ads1299_deinit(ads1299_t *dev)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Deinitializing ADS1299");

    ads1299_stop_continuous(dev);

    /* stop device */
    gpio_set_level(dev->config.start_pin, 0);

    /* remove SPI device */
    if (dev->spi_handle) {
        spi_bus_remove_device(dev->spi_handle);
        dev->spi_handle = NULL;
    }


    /* free mutex */
    if (dev->mutex) {
        vSemaphoreDelete(dev->mutex);
        dev->mutex = NULL;
    }

    dev->initialized = false;


    free(dev->dma_ctx);
    dev->dma_ctx = NULL;

    ESP_LOGI(TAG, "ADS1299 deinitialized");

    return ESP_OK;
}

esp_err_t ads1299_write_register(ads1299_t* dev, uint8_t reg, uint8_t value)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    /* During initialization the driver may call this helper before
     * dev->initialized is set. Ensure the SPI device has been added
     * (spi_handle) so that the SPI transactions can proceed. */
    if (!dev->spi_handle) return ESP_ERR_INVALID_STATE;

    if (ADS1299_REG_IS_READONLY(reg)) {
        ESP_LOGE(TAG, "Attempt to write to read-only register 0x%02X", reg);
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->rdatac_active) {
        ESP_LOGW(TAG, "WREG rejected: device is in RDATAC (streaming) mode");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(dev->spi_handle, portMAX_DELAY), TAG, "Acquire bus failed");

    uint8_t tx_cmd[2] = { (uint8_t)(ADS1299_CMD_WREG | (reg & 0x1F)), 0x00 };
    spi_transaction_t t_cmd = {0};
    t_cmd.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    t_cmd.length = 16;
    t_cmd.tx_buffer = tx_cmd;

    esp_err_t err = spi_device_polling_transmit(dev->spi_handle, &t_cmd);
    if (err != ESP_OK) {
        spi_device_release_bus(dev->spi_handle);
        return err;
    }

    esp_rom_delay_us(5);

    uint8_t tx_data[1] = {value};
    spi_transaction_t t_data = {0};
    t_data.length = 8;
    t_data.tx_buffer = tx_data;

    err = spi_device_polling_transmit(dev->spi_handle, &t_data);
    spi_device_release_bus(dev->spi_handle);

    return err;
}

esp_err_t ads1299_read_register(ads1299_t* dev, uint8_t reg, uint8_t* value)
{
    if (!dev || !value) return ESP_ERR_INVALID_ARG;

    /* 1. Lock the bus to prevent background tasks from interrupting */
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(dev->spi_handle, portMAX_DELAY), TAG, "Acquire bus failed");

    /* 2. Send Opcode (16 bits) and force hardware to KEEP CS LOW */
    uint8_t tx_cmd[2] = { (uint8_t)(ADS1299_CMD_RREG | (reg & 0x1F)), 0x00 };
    spi_transaction_t t_cmd = {0};
    t_cmd.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS asserted
    t_cmd.length = 16;
    t_cmd.tx_buffer = tx_cmd;

    esp_err_t err = spi_device_polling_transmit(dev->spi_handle, &t_cmd);
    if (err != ESP_OK) {
        spi_device_release_bus(dev->spi_handle);
        return err;
    }

    /* 3. Mandatory t_SDECODE delay while CS is held low */
    esp_rom_delay_us(5); // 5us safely exceeds 4 tCLKs

    /* 4. Clock out the payload and allow CS to naturally release */
    uint8_t rx_data[1] = {0};
    spi_transaction_t t_data = {0};
    t_data.length = 8;
    t_data.rxlength = 8;
    t_data.rx_buffer = rx_data;

    err = spi_device_polling_transmit(dev->spi_handle, &t_data);
    spi_device_release_bus(dev->spi_handle);

    if (err == ESP_OK) {
        *value = rx_data[0];
    }
    return err;
}


esp_err_t ads1299_write_registers(ads1299_t* dev, uint8_t start_reg, const uint8_t* data, size_t count)
{
    if (!dev || !data || count == 0) return ESP_ERR_INVALID_ARG;
    /* During initialization the driver may need to perform multi-write
     * sequences before dev->initialized is set. Require the SPI device
     * to be present instead of the initialized flag. */
    if (!dev->spi_handle) return ESP_ERR_INVALID_STATE;

    /* Reject if any target register is read-only */
    for (size_t i = 0; i < count; i++) {
        uint8_t r = start_reg + (uint8_t)i;
        if (ADS1299_REG_IS_READONLY(r)) {
            ESP_LOGE(TAG, "Attempt to write read-only register 0x%02X in multi-write", r);
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (dev->rdatac_active) {
        ESP_LOGW(TAG, "WREG rejected: device is in RDATAC (streaming) mode");
        return ESP_ERR_INVALID_STATE;
    }

    size_t total_len = 2 + count;
    uint8_t* tx_buf = (uint8_t*)calloc(1, total_len);
    if (!tx_buf) return ESP_ERR_NO_MEM;

    tx_buf[0] = (uint8_t)(ADS1299_CMD_WREG | (start_reg & 0x1F));
    tx_buf[1] = (uint8_t)(count - 1);
    for (size_t i = 0; i < count; i++) {
        tx_buf[2 + i] = data[i];
    }

    spi_transaction_t t = {0};
    t.length = total_len * 8;
    t.tx_buffer = tx_buf;

    esp_err_t err = spi_device_polling_transmit(dev->spi_handle, &t);
    free(tx_buf);
    esp_rom_delay_us(ADS1299_T_SDECODE);
    return err;
}

esp_err_t ads1299_read_registers(ads1299_t* dev, uint8_t start_reg, uint8_t* data, size_t count)
{
    if (!dev || !data || count == 0) return ESP_ERR_INVALID_ARG;

    /* For multi-byte reads, allocate a temporary buffer for Full-Duplex TX/RX */
    size_t total_len = 2 + count;
    uint8_t* tx_buf = (uint8_t*)calloc(1, total_len);
    uint8_t* rx_buf = (uint8_t*)calloc(1, total_len);
    if (!tx_buf || !rx_buf) {
        free(tx_buf);
        free(rx_buf);
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = (uint8_t)(ADS1299_CMD_RREG | (start_reg & 0x1F));
    tx_buf[1] = (uint8_t)(count - 1);

    spi_transaction_t t = {0};
    t.length = total_len * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;

    esp_err_t err = spi_device_polling_transmit(dev->spi_handle, &t);
    if (err == ESP_OK) {
        /* Copy payloads starting from byte index 2 */
        for (size_t i = 0; i < count; i++) {
            data[i] = rx_buf[2 + i];
        }
    }

    free(tx_buf);
    free(rx_buf);
    esp_rom_delay_us(ADS1299_T_SDECODE);
    return err;
}

/**
 * @brief Safely update a register using mask/value semantics.
 *
 * Performs: new = (current & ~mask) | (value & mask).
 * Acquires dev->mutex to make the multi-step SPI sequence atomic against
 * concurrent tasks. Rejects writes to read-only registers and when RDATAC is
 * active.
 */
esp_err_t ads1299_update_register_masked(ads1299_t *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    if (ADS1299_REG_IS_READONLY(reg)) {
        ESP_LOGE(TAG, "Attempt to update read-only register 0x%02X", reg);
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->rdatac_active) {
        ESP_LOGW(TAG, "WREG rejected: device is in RDATAC (streaming) mode");
        return ESP_ERR_INVALID_STATE;
    }

    if (!dev->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t current = 0;
    esp_err_t err = ads1299_read_register(dev, reg, &current);
    if (err != ESP_OK) {
        xSemaphoreGive(dev->mutex);
        return err;
    }

    uint8_t newv = (current & (uint8_t)~mask) | (value & mask);
    if (newv != current) {
        err = ads1299_write_register(dev, reg, newv);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write reg 0x%02X: %s", reg, esp_err_to_name(err));
            xSemaphoreGive(dev->mutex);
            return err;
        }
        ESP_LOGD(TAG, "Register 0x%02X updated: 0x%02X -> 0x%02X", reg, current, newv);
    } else {
        ESP_LOGD(TAG, "Register 0x%02X unchanged (mask indicates no-op)", reg);
    }

    xSemaphoreGive(dev->mutex);
    return ESP_OK;
}

esp_err_t ads1299_send_command(ads1299_t* dev, uint8_t cmd)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    uint8_t cmd_byte = cmd;
    spi_transaction_t t = {0};
    t.length = 8;
    t.tx_buffer = &cmd_byte;

    ESP_RETURN_ON_ERROR(spi_device_polling_transmit(dev->spi_handle, &t), TAG, "SPI command failed");
    esp_rom_delay_us(5); // Enforce t_SCCS delay (>= 4 tCLKs)
    return ESP_OK;
}
esp_err_t ads1299_read_data(ads1299_t* dev, uint8_t* buffer)
{
    if (!dev || !buffer) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. Explicitly acquire exclusive SPI bus lock before keeping CS active across transactions */
    ESP_RETURN_ON_ERROR(spi_device_acquire_bus(dev->spi_handle, portMAX_DELAY), TAG, "Failed to acquire SPI bus");

    /* Transaction 1: Send RDATA opcode (0x12) and KEEP CS ASSERTED (LOW) */
    uint8_t cmd = ADS1299_CMD_RDATA;
    spi_transaction_t t_cmd = {
        .length = 8,
        .tx_buffer = &cmd,
        .flags = SPI_TRANS_CS_KEEP_ACTIVE
    };

    esp_err_t err = spi_device_polling_transmit(dev->spi_handle, &t_cmd);
    if (err != ESP_OK) {
        spi_device_release_bus(dev->spi_handle); /* Always release lock on early error */
        ESP_LOGE(TAG, "RDATA cmd failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Wait mandatory t_SDECODE (4 tCLKs ~ 2 us) while CS remains held LOW */
    esp_rom_delay_us(ADS1299_T_SDECODE);

    /* Transaction 2: Clock out 27 data bytes. CS deasserts automatically after this finishes. */
    spi_transaction_t t_data = {
        .length = ADS1299_FRAME_SIZE * 8,
        .rxlength = ADS1299_FRAME_SIZE * 8,
        .tx_buffer = NULL,
        .rx_buffer = buffer
    };
    err = spi_device_polling_transmit(dev->spi_handle, &t_data);

    /* 2. Release bus lock regardless of transaction 2 outcome */
    spi_device_release_bus(dev->spi_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RDATA payload read failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t ads1299_read_sample(ads1299_t* dev, ads1299_sample_t* sample)
{
    if (!dev || !sample) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[ADS1299_FRAME_SIZE] = {0};
    ESP_RETURN_ON_ERROR(ads1299_read_data(dev, buffer), TAG, "Read frame failed");

    ads1299_parse_frame(buffer, esp_timer_get_time(), sample);
    return ESP_OK;
}

esp_err_t ads1299_start(ads1299_t* dev)
{
    return gpio_set_level(dev->config.start_pin, 1);
}

esp_err_t ads1299_stop(ads1299_t* dev)
{
    gpio_set_level(dev->config.start_pin, 0);
    return ads1299_send_command(dev, ADS1299_CMD_STOP);
}

esp_err_t ads1299_reset_hardware(ads1299_t* dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_ERROR(gpio_set_level(dev->config.reset_pin, 0), TAG, "RESET low failed");
    esp_rom_delay_us(ADS1299_T_RESET); /* Hold LOW >= 18 tCLKs (~9 us) */
    ESP_RETURN_ON_ERROR(gpio_set_level(dev->config.reset_pin, 1), TAG, "RESET high failed");

    /* CRITICAL FIX: Section 10.1.1 mandates waiting t_POR (2^18 tCLKs ~ 128 ms)
     * post-reset before issuing SDATAC or any register commands.
     * We delay 150 ms to guarantee crystal stability and core boot completion. */
    vTaskDelay(pdMS_TO_TICKS(150));
    return ESP_OK;
}

esp_err_t ads1299_reset_software(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RESET);
    /* Software reset also requires t_POR stabilization before issuing SDATAC */
    vTaskDelay(pdMS_TO_TICKS(150));
    return ret;
}

esp_err_t ads1299_standby(ads1299_t* dev)
{
    return ads1299_send_command(dev, ADS1299_CMD_STANDBY);
}

esp_err_t ads1299_wakeup(ads1299_t* dev)
{
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_WAKEUP);
    esp_rom_delay_us(ADS1299_T_WAKEUP);
    return ret;
}

esp_err_t ads1299_enable_continuous_read(ads1299_t* dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_RDATAC);
    if (ret == ESP_OK) {
        /* Mark RDATAC/streaming mode active to reject R/WREG while streaming */
        if (dev->mutex) {
            /* Acquire mutex briefly to avoid races with concurrent writes */
            if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                dev->rdatac_active = true;
                xSemaphoreGive(dev->mutex);
            } else {
                /* best-effort: set without mutex if unavailable */
                dev->rdatac_active = true;
            }
        } else {
            dev->rdatac_active = true;
        }
    }
    esp_rom_delay_us(ADS1299_T_RDATAC);
    return ret;
}

esp_err_t ads1299_disable_continuous_read(ads1299_t* dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = ads1299_send_command(dev, ADS1299_CMD_SDATAC);
    if (ret == ESP_OK) {
        if (dev->mutex) {
            if (xSemaphoreTake(dev->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                dev->rdatac_active = false;
                xSemaphoreGive(dev->mutex);
            } else {
                dev->rdatac_active = false;
            }
        } else {
            dev->rdatac_active = false;
        }
    }
    esp_rom_delay_us(ADS1299_T_SDATAC);
    return ret;
}


/* Interrupt Service Routine (ISR) for DRDY pin.
 * This will be called when the DRDY pin goes low, indicating that new data is ready to be read from the ADS1299.
 * The ISR will send the GPIO number to a FreeRTOS queue for processing in a separate task.
 * Declared within ads1299.c to keep it private to the driver implementation.
 */
static void IRAM_ATTR drdy_isr_handler(void *arg)
{
    ads1299_t         *dev = (ads1299_t *)arg;
    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    /* ── Guard ────────────────────────────────────────────────────── */
    if (ctx->spi_busy) {
        /* Previous DMA not yet parsed. This DRDY pulse is dropped.
         * At 250 SPS / 4 MHz SPI, the DMA takes ~54 µs; inter-sample
         * gap is 4000 µs. This path fires only under hardware fault. */
        ctx->dropped_count++;
        return;
    }

    /* ── Timestamp ─────────────────────────────────────────────────── */
    /* Capture at DRDY edge — the most accurate possible timestamp.
     * current_timestamp is read in post_cb. Race is impossible:
     * post_cb (SPI ISR, higher priority) cannot interrupt the GPIO ISR;
     * the next GPIO ISR cannot fire until spi_busy is cleared in post_cb. */
    ctx->current_timestamp = esp_timer_get_time();

    /* ── Arm DMA ───────────────────────────────────────────────────── */
    ctx->spi_busy = true;

    /* trans.rx_buffer = ctx->dma_buf (set once at start_continuous).
     * trans.user      = ctx            (set once at start_continuous).
     * No fields to modify; just re-queue. */
    esp_err_t err = spi_device_queue_trans(dev->spi_handle, &ctx->trans, 0);
    if (err != ESP_OK) {
        /* queue_trans failed (SPI trans_queue full or bus error).
         * Clear spi_busy so the next DRDY can retry. */
        ctx->spi_busy = false;
        ctx->dropped_count++;
    }
}

// parse ready_buf into ads1299_sample_t[] (local/static array)
// build ads1299_chunk_t
// call ctx->on_chunk(&chunk, ctx->ctx)
static void ads1299_handler_task(void *arg)
{
    ads1299_t         *dev = (ads1299_t *)arg;
    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    for (;;) {

        /* ── 1. Block until notified ────────────────────────────────── */
        xTaskNotifyWait(
            0,            /* do not clear bits on entry */
            ULONG_MAX,    /* clear all bits on exit     */
            NULL,         /* notification value unused  */
            portMAX_DELAY
        );

        /* ── 2. Check stop signal ───────────────────────────────────── */
        if (ctx->stop_requested) {
            break;
        }

        /* ── 3. Drain SPI ret_queue ─────────────────────────────────── */
        /* post_cb does all data work. get_trans_result here is purely
         * bookkeeping: it frees slots in the SPI driver's internal
         * ret_queue. Without this drain, ret_queue fills after
         * chunk_samples transactions and new queue_trans calls silently
         * fail (timeout 0 returns immediately), breaking the pipeline. */
        {
            spi_transaction_t *result;
            while (spi_device_get_trans_result(dev->spi_handle,
                                               &result, 0) == ESP_OK) {
                /* Nothing to do. Data was handled in post_cb. */
            }
        }

        /* ── 4. Drain all available complete chunks ─────────────────── */
        /* Loop — do not return to xTaskNotifyWait after each chunk.
         * Multiple chunks may have accumulated while this task was
         * processing on_chunk. Draining here avoids needing one
         * notification per chunk and handles backlog naturally. */
        while (ads1299_chunkring_available(&ctx->ring) > 0) {

            /* ── 4a. Acquire pointer into ring (zero-copy) ─────────── */
            const ads1299_sample_t *samples =
                ads1299_chunkring_read_ptr(&ctx->ring);

            /* ── 4b. Build chunk descriptor ─────────────────────────── */
            ads1299_chunk_t chunk = {
                .samples = samples,
                .n_samples = ctx->ring.chunk_samples,
                .first_timestamp_us = samples[0].timestamp_us,
                .last_timestamp_us = samples[ctx->ring.chunk_samples - 1].timestamp_us,
                .dropped_count = ctx->dropped_count,
                .overflow_count = ctx->overflow_count,
            };

            /* ── 4c. Deliver to user ─────────────────────────────────── */
            /* chunk.samples points directly into the ring buffer.
             * It is valid until ads1299_chunkring_release() is called.
             * The user must copy samples if they need them beyond
             * the duration of this call. */
            ctx->on_chunk(&chunk, ctx->cb_ctx);

            /* ── 4d. Release slot ────────────────────────────────────── */
            /* After release, the ISR may reclaim this slot for a
             * future chunk. Do not access chunk.samples after this. */
            ads1299_chunkring_release(&ctx->ring);
        }

        /* Loop back to xTaskNotifyWait. */
    }

    /* ── Shutdown ────────────────────────────────────────────────────── */
    xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}

esp_err_t ads1299_start_continuous(ads1299_t  *dev, const ads1299_continuous_config_t *cfg)
{
    /* ── Validate ────────────────────────────────────────────────────── */
    if (!dev || !cfg || !cfg->on_chunk)  return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (dev->dma_ctx) return ESP_ERR_INVALID_STATE; /* already running */

    /* ── Allocate context ────────────────────────────────────────────── */
    ads1299_dma_ctx_t *ctx = calloc(1, sizeof(ads1299_dma_ctx_t));
    if (!ctx) return ESP_ERR_NO_MEM;
    dev->dma_ctx = ctx;

    /* ── Compute sizing ──────────────────────────────────────────────── */
    uint32_t ms = cfg->chunk_duration_ms ? cfg->chunk_duration_ms
                                         : ADS1299_DEFAULT_CHUNK_MS;
    uint32_t chunk_samples = ads1299_sample_rate_to_hz(dev->config.sample_rate)
                             * ms / 1000;
    if (chunk_samples == 0) {
        free(ctx); dev->dma_ctx = NULL;
        return ESP_ERR_INVALID_ARG;
    }

    /* ── Determine ring buffer capacity ─────────────────────────────── */
    uint32_t ring_chunks = cfg->ring_buffer_chunks
                           ? cfg->ring_buffer_chunks
                           : ADS1299_RING_BUF_SLOTS;
    /* Round up to next power of 2 */
    ring_chunks--;
    ring_chunks |= ring_chunks >> 1;
    ring_chunks |= ring_chunks >> 2;
    ring_chunks |= ring_chunks >> 4;
    ring_chunks |= ring_chunks >> 8;
    ring_chunks |= ring_chunks >> 16;
    ring_chunks++;
    if (ring_chunks < 2) ring_chunks = 2;

    /* ── Allocate DMA staging buffer ────────────────────────────────── */
    ctx->dma_buf = heap_caps_malloc(ADS1299_FRAME_SIZE, MALLOC_CAP_DMA);
    if (!ctx->dma_buf) goto fail;


    /* ── Allocate ring buffer ────────────────────────────────────────── */
    esp_err_t err = ads1299_chunkring_init(&ctx->ring, ring_chunks, chunk_samples);
    if (err != ESP_OK) goto fail;

    /* ── Populate context ────────────────────────────────────────────── */
    ctx->sample_count   = 0;
    ctx->spi_busy       = false;
    ctx->stop_requested = false;
    ctx->on_chunk       = cfg->on_chunk;
    ctx->on_error       = cfg->on_error;
    ctx->cb_ctx         = cfg->ctx;
    ctx->dropped_count  = 0;
    ctx->overflow_count = 0;

    ctx->done_sem = xSemaphoreCreateBinary();
    if (!ctx->done_sem) goto fail;

    /* ── Configure SPI transaction (set once; never modified at runtime) */
    memset(&ctx->trans, 0, sizeof(spi_transaction_t));
    ctx->trans.length    = ADS1299_FRAME_SIZE * 8;
    ctx->trans.rxlength  = ADS1299_FRAME_SIZE * 8;
    ctx->trans.tx_buffer = NULL;
    ctx->trans.rx_buffer = ctx->dma_buf;
    ctx->trans.user      = ctx;

    /* ── Re-add SPI device with queue_size = chunk_samples ──────────── */
    /* The device was added in ads1299_init() with queue_size = 1 for
     * polling. For async DMA, queue_size must be >= chunk_samples to
     * prevent the SPI driver's ret_queue from filling between handler
     * task wake-ups and silently dropping queue_trans calls. */
    spi_bus_remove_device(dev->spi_handle);
    dev->spi_handle = NULL;

    spi_device_interface_config_t dev_cfg = {
        .mode             = 1,
        .clock_speed_hz   = 4 * 1000 * 1000,
        .spics_io_num     = dev->config.cs_pin,
        .cs_ena_pretrans  = 4,
        .cs_ena_posttrans = 8,
        .queue_size       = chunk_samples,   /* must hold all in-flight trans */
        .post_cb          = spi_post_transfer_cb,
    };
    err = spi_bus_add_device(dev->config.spi_host, &dev_cfg, &dev->spi_handle);
    if (err != ESP_OK) goto fail;

    /* ── Create handler task ─────────────────────────────────────────── */
    /* Task must exist before ISR is armed so that post_cb has a valid
     * handler_task handle for xTaskNotifyFromISR. */
    BaseType_t ok = xTaskCreatePinnedToCore(
        ads1299_handler_task,
        "ads1299_handler",
        4096,                       /* stack; adjust if on_chunk does heavy work */
        dev,
        cfg->task_priority,
        &ctx->handler_task,
        cfg->task_core
    );
    if (ok != pdPASS) goto fail;

    /* ── Arm DRDY ISR ────────────────────────────────────────────────── */
    /* Armed last: handler_task is guaranteed valid when ISR fires. */
    gpio_install_isr_service(0);    /* no-op if already installed */
    err = gpio_isr_handler_add(dev->config.drdy_pin, drdy_isr_handler, dev);
    if (err != ESP_OK) goto fail;

    return ESP_OK;

    fail:
        /* Partial cleanup: free whatever was allocated */
        if (ctx->handler_task) { vTaskDelete(ctx->handler_task); ctx->handler_task = NULL; }
        if (ctx->done_sem)     { vSemaphoreDelete(ctx->done_sem); }
        ads1299_chunkring_free(&ctx->ring);
        if (ctx->dma_buf)      { free(ctx->dma_buf); }
        free(ctx);
        dev->dma_ctx = NULL;
        return ESP_ERR_NO_MEM;
}

esp_err_t ads1299_stop_continuous(ads1299_t *dev)
{
    if (!dev)               return ESP_ERR_INVALID_ARG;
    if (!dev->initialized)  return ESP_ERR_INVALID_STATE;
    if (!dev->dma_ctx)      return ESP_OK;   /* already stopped; clean no-op */

    ads1299_dma_ctx_t *ctx = dev->dma_ctx;

    /* ── 1. Disable DRDY ISR ─────────────────────────────────────────── */
    /* No new DRDY pulses will be processed after this returns. Any ISR
     * that had already entered drdy_isr_handler before this call
     * will complete, but no further ones fire. */
    gpio_isr_handler_remove(dev->config.drdy_pin);

    /* ── 2. Wait for any in-flight DMA to complete ───────────────────── */
    /* Spin on spi_busy. The in-flight transaction completes in < 100 µs
     * at 4 MHz SPI. post_cb will fire, parse, and clear spi_busy. */
    while (ctx->spi_busy) {
        esp_rom_delay_us(10);
    }

    /* ── 3. Signal handler task to exit ─────────────────────────────── */
    ctx->stop_requested = true;
    xTaskNotify(ctx->handler_task, 1u, eSetValueWithOverwrite);

    /* ── 4. Wait for handler task to exit ────────────────────────────── */
    xSemaphoreTake(ctx->done_sem, portMAX_DELAY);

    /* ── 5. Final SPI ret_queue drain ────────────────────────────────── */
    {
        spi_transaction_t *result;
        while (spi_device_get_trans_result(dev->spi_handle,
                                           &result, 0) == ESP_OK) {}
    }

    /* ── 6. Restore SPI device to polling configuration ──────────────── */
    spi_bus_remove_device(dev->spi_handle);
    dev->spi_handle = NULL;

    spi_device_interface_config_t poll_cfg = {
        .mode             = 1,
        .clock_speed_hz   = 4 * 1000 * 1000,
        .spics_io_num     = dev->config.cs_pin,
        .cs_ena_pretrans  = 4,
        .cs_ena_posttrans = 8,
        .queue_size       = 1,
        .post_cb          = NULL,
    };
    spi_bus_add_device(dev->config.spi_host, &poll_cfg, &dev->spi_handle);

    /* ── 7. Free resources ────────────────────────────────────────────── */
    vSemaphoreDelete(ctx->done_sem);
    ads1299_chunkring_free(&ctx->ring);
    free(ctx->dma_buf);
    free(ctx);
    dev->dma_ctx = NULL;

    return ESP_OK;
}

bool ads1299_is_running(const ads1299_t* dev)
{
    if (!dev) {
        return false;
    }
    return dev->dma_ctx != NULL;
}

void ads1299_parse_frame(const uint8_t* raw, int64_t timestamp, ads1299_sample_t* out)
{
    out->status[0] = raw[0];
    out->status[1] = raw[1];
    out->status[2] = raw[2];

    for (int ch = 0; ch < 8; ch++) {
        int offset = 3 + ch * 3;

        // Temporary 24-bit value stored in a 32-bit int for sign extension
        int32_t val = ((int32_t)raw[offset] << 16) |
                      ((int32_t)raw[offset + 1] << 8)  |
                       (int32_t)raw[offset + 2];

        // Bit manipulation magic to properly fill the left most bits based on the sign
        out->channels[ch] = (val << 8) >> 8;
    }
    out->timestamp_us = timestamp;

}

esp_err_t ads1299_set_channel_gain(ads1299_t *dev, uint8_t channel, ads1299_pga_gain_t gain)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (channel < 1 || channel > ADS1299_NUM_CHANNELS) {
        ESP_LOGE(TAG, "Invalid channel: %d. Must be 1 to %d", channel, ADS1299_NUM_CHANNELS);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_addr = ADS1299_REG_CH1SET + (channel - 1);

    /* Use the safe masked RMW helper which acquires the driver mutex and
     * enforces RDATAC/read-only guarding. The gain enum values are already
     * aligned to the CHnSET gain bit positions, but wrap with the helper macro
     * to be explicit. */
    uint8_t gain_bits = ADS1299_CHSET_GAIN_VAL((uint8_t)gain);
    return ads1299_update_register_masked(dev, reg_addr, ADS1299_CHSET_GAIN_MASK, gain_bits);
}

esp_err_t ads1299_set_all_channels_gain(ads1299_t *dev, ads1299_pga_gain_t gain)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    /* Reject while in RDATAC mode */
    if (dev->rdatac_active) return ESP_ERR_INVALID_STATE;

    /* Acquire mutex once for the entire multi-channel update to make it
     * atomic and reduce SPI bus overhead. */
    if (!dev->mutex) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t err = ESP_OK;
    for (uint8_t ch = 1; ch <= ADS1299_NUM_CHANNELS; ch++) {
        uint8_t reg = ADS1299_REG_CH1SET + (ch - 1);
        uint8_t cur = 0;
        err = ads1299_read_register(dev, reg, &cur);
        if (err != ESP_OK) break;
        uint8_t newv = (cur & ~ADS1299_CHSET_GAIN_MASK) | ADS1299_CHSET_GAIN_VAL((uint8_t)gain);
        err = ads1299_write_register(dev, reg, newv);
        if (err != ESP_OK) break;
    }

    xSemaphoreGive(dev->mutex);
    return err;
}

/* Channel MUX helpers */
esp_err_t ads1299_set_channel_mux(ads1299_t *dev, uint8_t channel, ads1299_input_mux_t mux)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (channel < 1 || channel > ADS1299_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;

    uint8_t reg = ADS1299_REG_CH1SET + (channel - 1);
    return ads1299_update_register_masked(dev, reg, ADS1299_CHSET_MUX_MASK, ADS1299_CHSET_MUX_VAL((uint8_t)mux));
}

esp_err_t ads1299_set_all_channels_mux(ads1299_t *dev, ads1299_input_mux_t mux)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (dev->rdatac_active) return ESP_ERR_INVALID_STATE;
    if (!dev->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t err = ESP_OK;
    for (uint8_t ch = 1; ch <= ADS1299_NUM_CHANNELS; ch++) {
        uint8_t reg = ADS1299_REG_CH1SET + (ch - 1);
        uint8_t cur = 0;
        err = ads1299_read_register(dev, reg, &cur);
        if (err != ESP_OK) break;
        uint8_t newv = (cur & ~ADS1299_CHSET_MUX_MASK) | ADS1299_CHSET_MUX_VAL((uint8_t)mux);
        err = ads1299_write_register(dev, reg, newv);
        if (err != ESP_OK) break;
    }

    xSemaphoreGive(dev->mutex);
    return err;
}

/* Channel power-down helper */
esp_err_t ads1299_set_channel_powerdown(ads1299_t *dev, uint8_t channel, bool powerdown)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (channel < 1 || channel > ADS1299_NUM_CHANNELS) return ESP_ERR_INVALID_ARG;

    uint8_t reg = ADS1299_REG_CH1SET + (channel - 1);
    uint8_t pdv = ADS1299_CHSET_PDN_VAL(powerdown);
    return ads1299_update_register_masked(dev, reg, ADS1299_CHSET_PDN_MASK, pdv);
}

esp_err_t ads1299_set_all_channels_powerdown(ads1299_t *dev, bool powerdown)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (dev->rdatac_active) return ESP_ERR_INVALID_STATE;
    if (!dev->mutex) return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;

    esp_err_t err = ESP_OK;
    uint8_t pdv = ADS1299_CHSET_PDN_VAL(powerdown);
    for (uint8_t ch = 1; ch <= ADS1299_NUM_CHANNELS; ch++) {
        uint8_t reg = ADS1299_REG_CH1SET + (ch - 1);
        uint8_t cur = 0;
        err = ads1299_read_register(dev, reg, &cur);
        if (err != ESP_OK) break;

        uint8_t newv = (cur & (uint8_t)~ADS1299_CHSET_PDN_MASK) | pdv;
        err = ads1299_write_register(dev, reg, newv);
        if (err != ESP_OK) break;
    }

    xSemaphoreGive(dev->mutex);
    return err;
}

/* SRB1 routing */
esp_err_t ads1299_set_srb1(ads1299_t *dev, bool on)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t mask = ADS1299_MISC1_SRB1_ON;
    uint8_t val = on ? ADS1299_MISC1_SRB1_ON : 0x00;
    return ads1299_update_register_masked(dev, ADS1299_REG_MISC1, mask, val);
}

/* Bias helpers */
esp_err_t ads1299_set_bias_enabled(ads1299_t *dev, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    uint8_t val = enable ? ADS1299_CONFIG3_BIAS_ON : ADS1299_CONFIG3_BIAS_OFF;
    uint8_t mask = ADS1299_CONFIG3_BIAS_MASK;
    return ads1299_update_register_masked(dev, ADS1299_REG_CONFIG3, mask, val);
}

esp_err_t ads1299_set_config4_loff_comp(ads1299_t *dev, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;

    return ads1299_update_register_masked(dev, ADS1299_REG_CONFIG4, ADS1299_CONFIG4_LOFF_COMP_MASK,
                                         enable ? ADS1299_CONFIG4_LOFF_COMP_ON : ADS1299_CONFIG4_LOFF_COMP_OFF);
}

/* ---------- Bias / Lead-off per-channel helpers ---------- */
static inline esp_err_t ads1299__validate_channel(uint8_t channel)
{
    return (channel >= 1 && channel <= ADS1299_NUM_CHANNELS) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t ads1299_set_bias_sense(ads1299_t *dev, uint8_t channel, bool positive, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (ads1299__validate_channel(channel) != ESP_OK) return ESP_ERR_INVALID_ARG;

    uint8_t reg = positive ? ADS1299_REG_BIAS_SENSP : ADS1299_REG_BIAS_SENSN;
    uint8_t mask = (uint8_t)(1u << (channel - 1));
    uint8_t val = enable ? mask : 0x00;
    return ads1299_update_register_masked(dev, reg, mask, val);
}

esp_err_t ads1299_set_all_bias_sense(ads1299_t *dev, bool positive, uint8_t channel_mask)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (dev->rdatac_active) return ESP_ERR_INVALID_STATE;
    if (!dev->mutex) return ESP_ERR_INVALID_STATE;

    uint8_t reg = positive ? ADS1299_REG_BIAS_SENSP : ADS1299_REG_BIAS_SENSN;

    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    /* Read current, replace masked bits, write back */
    uint8_t cur = 0;
    esp_err_t err = ads1299_read_register(dev, reg, &cur);
    if (err == ESP_OK) {
        uint8_t newv = (cur & (uint8_t)~channel_mask) | (channel_mask & channel_mask);
        /* newv sets the bits that channel_mask requests; if we wanted to allow clearing bits
         * selectively, caller should compute channel_mask accordingly. For clarity we write
         * the requested mask as-is. */
        err = ads1299_write_register(dev, reg, newv);
    }
    xSemaphoreGive(dev->mutex);
    return err;
}

esp_err_t ads1299_set_loff_sense(ads1299_t *dev, uint8_t channel, bool positive, bool enable)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (ads1299__validate_channel(channel) != ESP_OK) return ESP_ERR_INVALID_ARG;

    uint8_t reg = positive ? ADS1299_REG_LOFF_SENSP : ADS1299_REG_LOFF_SENSN;
    uint8_t mask = (uint8_t)(1u << (channel - 1));
    uint8_t val = enable ? mask : 0x00;
    return ads1299_update_register_masked(dev, reg, mask, val);
}

esp_err_t ads1299_set_all_loff_sense(ads1299_t *dev, bool positive, uint8_t channel_mask)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (dev->rdatac_active) return ESP_ERR_INVALID_STATE;
    if (!dev->mutex) return ESP_ERR_INVALID_STATE;

    uint8_t reg = positive ? ADS1299_REG_LOFF_SENSP : ADS1299_REG_LOFF_SENSN;

    if (xSemaphoreTake(dev->mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    uint8_t cur = 0;
    esp_err_t err = ads1299_read_register(dev, reg, &cur);
    if (err == ESP_OK) {
        uint8_t newv = (cur & (uint8_t)~channel_mask) | (channel_mask & channel_mask);
        err = ads1299_write_register(dev, reg, newv);
    }
    xSemaphoreGive(dev->mutex);
    return err;
}

esp_err_t ads1299_set_loff_flip(ads1299_t *dev, uint8_t channel, bool flip)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->initialized) return ESP_ERR_INVALID_STATE;
    if (ads1299__validate_channel(channel) != ESP_OK) return ESP_ERR_INVALID_ARG;

    uint8_t mask = (uint8_t)(1u << (channel - 1));
    uint8_t val = flip ? mask : 0x00;
    return ads1299_update_register_masked(dev, ADS1299_REG_LOFF_FLIP, mask, val);
}


uint32_t ads1299_sample_rate_to_hz(ads1299_sample_rate_t rate)
{
    switch (rate)
    {
        case ADS1299_DR_16kSPS: return 16000;
        case ADS1299_DR_8kSPS:  return 8000;
        case ADS1299_DR_4kSPS:  return 4000;
        case ADS1299_DR_2kSPS:  return 2000;
        case ADS1299_DR_1kSPS:  return 1000;
        case ADS1299_DR_500SPS: return 500;
        case ADS1299_DR_250SPS: return 250;
        default: return 0;
    }
}


int32_t ads1299_count_to_microvolts(int32_t signed_count, uint32_t vref_millivolts, ads1299_pga_gain_t gain) {

    switch (gain) {
        case ADS1299_PGA_GAIN_1: gain = 1; break;
        case ADS1299_PGA_GAIN_2: gain = 2; break;
        case ADS1299_PGA_GAIN_4: gain = 4; break;
        case ADS1299_PGA_GAIN_6: gain = 6; break;
        case ADS1299_PGA_GAIN_8: gain = 8; break;
        case ADS1299_PGA_GAIN_12: gain = 12; break;
        case ADS1299_PGA_GAIN_24: gain = 24; break;
        default:
            ESP_LOGE(TAG, "Invalid gain value: %d", gain);
            return 0; // or handle error as appropriate
    }
    // 1. Calculate the denominator: Gain * (2^23 - 1)
    // For 24-bit signed data, max positive range is 0x7FFFFF (8,388,607)
    int64_t denominator = (int64_t)gain * 8388607;

    // 2. Multiply first using 64-bit precision to prevent overflow, then divide
    // Microvolts = (Count * VREF_mV * 1000) / Denominator
    int64_t microvolts = ((int64_t)signed_count * vref_millivolts * 1000) / denominator;

    return (int32_t)microvolts;
}
