#include <cstdint>
#include <algorithm>
#include <string_view>
#include <span>
#include <string>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


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

/**
 * @brief Binary telemetry frame header
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /**< Synchronization bytes [0xAA, 0x55] */
    uint16_t length;        /**< Payload length in bytes */
    uint32_t chunk_seq;     /**< Monotonically increasing sequence number */
} telemetry_header_t;

#define CHANNEL_GAIN ADS1299_PGA_GAIN_24
#define SAMPLE_RATE ADS1299_DR_250SPS

static RingbufHandle_t s_telemetry_ringbuf = nullptr;
static uint32_t s_chunk_sequence = 0;


class WifiStation {
public:
    // Strongly typed status
    enum class Status {
        Connected,
        Failed,
        Connecting
    };

    WifiStation(std::string_view ssid, std::string_view password, int max_retries = 5)
        : max_retries_(max_retries), ssid_(ssid), password_(password)
    {
        event_group_ = xEventGroupCreate();
    }

    ~WifiStation() {
        unregister_handlers();
        if (event_group_) {
            vEventGroupDelete(event_group_);
        }
    }

    // Disable copy semantics for hardware resource managers
    WifiStation(const WifiStation&) = delete;
    WifiStation& operator=(const WifiStation&) = delete;

    Status connect() {
        ESP_ERROR_CHECK(esp_netif_init());

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Initialise Event handlres
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &WifiStation::event_handler_trampoline,
            this,
            &instance_any_id_));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &WifiStation::event_handler_trampoline,
            this,
            &instance_got_ip_));

        wifi_config_t wifi_config{};
        std::copy_n(ssid_.begin(), std::min(ssid_.size(), sizeof(wifi_config.sta.ssid) - 1), wifi_config.sta.ssid);
        std::copy_n(password_.begin(), std::min(password_.size(), sizeof(wifi_config.sta.password) - 1), wifi_config.sta.password);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

        EventBits_t wifi_connect_event_bits = xEventGroupWaitBits(
            event_group_,
            CONNECTED_BIT | FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        if (wifi_connect_event_bits & FAIL_BIT) {
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d retries", max_retries_);
            return Status::Failed;
        }

        return Status::Connected;
    }


private:
    static constexpr EventBits_t CONNECTED_BIT = BIT0;
    static constexpr EventBits_t FAIL_BIT = BIT1;
    static constexpr const char* TAG = "WifiStation";

    // Static C-compatible entry point for ESP-IDF C API
    static void event_handler_trampoline(void* arg, esp_event_base_t base, int32_t id, void* data) {
        // Safe cast back to C++ instance reference
        auto* self = static_cast<WifiStation*>(arg);
        self->handle_event(base, id, data);
    }

    // Member function handling actual event logic with direct access to private member variables
    void handle_event(esp_event_base_t base, int32_t id, void* data) {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
            if (retry_count_ < max_retries_) {
                esp_wifi_connect();
                retry_count_++;
                ESP_LOGI(TAG, "Retrying connection (%d/%d)...", retry_count_, max_retries_);
            } else {
                xEventGroupSetBits(event_group_, FAIL_BIT);
            }
        } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
            auto* event = static_cast<ip_event_got_ip_t*>(data);
            ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            retry_count_ = 0;
            xEventGroupSetBits(event_group_, CONNECTED_BIT);
        }
    }

    void unregister_handlers() {
        if (instance_any_id_) esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id_);
        if (instance_got_ip_) esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip_);
    }

    int max_retries_;
    int retry_count_{0};
    std::string_view ssid_;
    std::string_view password_;

    EventGroupHandle_t event_group_{nullptr};
    esp_event_handler_instance_t instance_any_id_{nullptr};
    esp_event_handler_instance_t instance_got_ip_{nullptr};
};

class Endpoint {
public:
    Endpoint(const std::string_view ip, uint16_t port) {
                std::memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_port = htons(port);

                // This line sets the IP address:
                inet_pton(AF_INET, ip.data(), &addr.sin_addr);
    };

private:
    friend class UdpSocket;
    struct sockaddr_in addr{};
};




class UdpSocket {
public:
    UdpSocket() noexcept {
        fd_m = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (fd_m >= 0) {
            int sndbuf = 8 * 1024;
            setsockopt(
                fd_m,
                SOL_SOCKET,
                SO_SNDBUF,
                &sndbuf,
                sizeof(sndbuf)
            );
        }
    }

    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    UdpSocket(UdpSocket&& other) noexcept = default;
    UdpSocket& operator=(UdpSocket&& other) noexcept = default;

    ~UdpSocket() {
        close(fd_m);
    }

    [[nodiscard]]
    bool is_open() const noexcept {
        return fd_m != -1;
    }

    [[nodiscard]] bool send_to(std::span<const std::byte> data, const Endpoint& destination) const noexcept
    {
        if (!is_open()) {
            ESP_LOGE(TAG, "Socket is not open");
            return false;
        }

        for (int attempt = 0; attempt < 5; ++attempt) {
            ssize_t sent_bytes = sendto(
                fd_m,
                data.data(),
                data.size(),
                0,
                reinterpret_cast<const sockaddr*>(&destination.addr),
                sizeof(destination.addr)
            );

            if (sent_bytes == static_cast<ssize_t>(data.size())) {
                return true;
            }

            if (sent_bytes < 0) {
                int err = errno;

                // LwIP returns ENOMEM (12) or ENOBUFS (105) when network buffers are temporarily exhausted
                if (err == ENOBUFS || err == ENOMEM) {
                    vTaskDelay(pdMS_TO_TICKS(10)); // Yield to allow LwIP task to flush TX queues
                    continue;
                }

                ESP_LOGE(
                    TAG,
                    "sendto() failed: errno=%d (%s)",
                    err,
                    strerror(err)
                );

                return false;
            }

            ESP_LOGE(
                TAG,
                "Partial UDP send: %zd/%zu bytes",
                sent_bytes,
                data.size()
            );

            return false;
        }

        ESP_LOGW(TAG, "UDP send failed after retries: buffer memory full");
        return false;
    }

private:
    int fd_m = -1;
    constexpr static const char* TAG = "UdpSocket";
};




constexpr static const char* TAG_TLM = "TelemetryTX";
/**
 * @brief Dedicated task to drain the ring buffer and transmit via native USB
 */
[[noreturn]] static void telemetry_tx_task(void *arg)
{

    size_t item_size;

    UdpSocket udp_socket;
    Endpoint endpoint{"192.168.1.57", CONFIG_EXAMPLE_PORT};


    for (;;) {
        void *item = xRingbufferReceive(s_telemetry_ringbuf, &item_size, portMAX_DELAY);

        if (item != nullptr) {
            // Bypass stdout and VFS completely. Write directly to the USB hardware FIFO.
            // This prevents the OS from corrupting binary 0x0A bytes into 0x0D 0x0A.
            // usb_serial_jtag_write_bytes((const char *)item, item_size, portMAX_DELAY);
            auto code = udp_socket.send_to({static_cast<const std::byte*>(item), item_size}, endpoint);
            if (!code) {
                ESP_LOGE(TAG_TLM, "Failed to send UDP packet");
            }
            vRingbufferReturnItem(s_telemetry_ringbuf, item);
        }
    }
}

void telemetry_init()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    WifiStation wifi_station(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_MAXIMUM_RETRY);
    wifi_station.connect();


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
    BaseType_t result = xRingbufferSendAcquire(s_telemetry_ringbuf, &frame_ptr, total_frame_size, pdMS_TO_TICKS(5));

    if (result != pdTRUE || frame_ptr == nullptr) {
        ESP_LOGW(TAG_TLM, "Ringbuffer full");
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



constexpr static const char* TAG = "Main";

extern "C" void app_main(void) {



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

    // Run your desired configs (gain, bias, shorting inputs to srb...)
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_MISC1, ADS1299_MISC1_SRB1_ON));


    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_CONFIG3, ADS1299_CONFIG3_BIAS_ON));
    ESP_ERROR_CHECK(ads1299_write_register(&dev1, ADS1299_REG_BIAS_SENSP, 0x01));

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

