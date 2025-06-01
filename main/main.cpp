#include <arpa/inet.h>
#include <netdb.h>
#include <string.h>
#include <sys/socket.h>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char* TAG = "EspCamera";

class Wifi
{
public:
    Wifi(const char* ssid, const char* pass, int maxRetry)
        : m_ssid{ssid}
        , m_pass{pass}
        , m_max_retry{maxRetry}
        , m_retry_num{0}
        , m_wifi_event_group{nullptr}
        , m_instance_any_id{nullptr}
        , m_instance_got_ip{nullptr}
        , m_wifi_connected_bit{BIT0}
        , m_wifi_fail_bit{BIT1}
    {
        m_wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            [](void* arg, esp_event_base_t base, int32_t id, void* data)
            { static_cast<Wifi*>(arg)->EventHandler(base, id, data); },
            this,
            &m_instance_any_id);

        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            [](void* arg, esp_event_base_t base, int32_t id, void* data)
            { static_cast<Wifi*>(arg)->EventHandler(base, id, data); },
            this,
            &m_instance_got_ip);

        wifi_config_t wifi_config = {};
        strlcpy((char*)wifi_config.sta.ssid, m_ssid, sizeof(wifi_config.sta.ssid));
        strlcpy((char*)wifi_config.sta.password, m_pass, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        EventBits_t bits = xEventGroupWaitBits(m_wifi_event_group, m_wifi_connected_bit | m_wifi_fail_bit, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & m_wifi_connected_bit)
        {
            ESP_LOGI(TAG, "Connected to AP: %s", m_ssid);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to connect: %s", m_ssid);
            abort();
        }
    }

private:
    void EventHandler(esp_event_base_t base, int32_t id, void* data)
    {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
        }
        else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (m_retry_num < m_max_retry)
            {
                esp_wifi_connect();
                m_retry_num++;
                ESP_LOGI(TAG, "Retry #%d", m_retry_num);
            }
            else
            {
                xEventGroupSetBits(m_wifi_event_group, m_wifi_fail_bit);
            }
        }
        else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        {
            m_retry_num = 0;
            xEventGroupSetBits(m_wifi_event_group, m_wifi_connected_bit);
        }
    }

    const char* m_ssid;
    const char* m_pass;

    int m_max_retry;
    int m_retry_num;

    EventGroupHandle_t           m_wifi_event_group;
    esp_event_handler_instance_t m_instance_any_id;
    esp_event_handler_instance_t m_instance_got_ip;

    const uint32_t m_wifi_connected_bit;
    const uint32_t m_wifi_fail_bit;
};

class FrameQueue
{
public:
    FrameQueue(int length)
    {
        m_queue = xQueueCreate(length, sizeof(camera_fb_t*));
        if (!m_queue)
        {
            ESP_LOGE(TAG, "Queue creation failed");
            abort();
        }
    }

    bool Push(camera_fb_t* frame, TickType_t timeout)
    {
        return xQueueSend(m_queue, &frame, timeout) == pdPASS;
    }

    bool Pop(camera_fb_t** frame, TickType_t timeout)
    {
        return xQueueReceive(m_queue, frame, timeout) == pdPASS;
    }

private:
    QueueHandle_t m_queue;
};

class Camera
{
public:
    Camera(FrameQueue& queue, uint32_t fps) : m_queue{queue}, m_delay_ms{1'000 / fps}
    {
        camera_config_t config{};
        config.pin_pwdn     = 32;
        config.pin_reset    = -1;
        config.pin_xclk     = 0;
        config.pin_sccb_sda = 26;
        config.pin_sccb_scl = 27;
        config.pin_d7       = 35;
        config.pin_d6       = 34;
        config.pin_d5       = 39;
        config.pin_d4       = 36;
        config.pin_d3       = 21;
        config.pin_d2       = 19;
        config.pin_d1       = 18;
        config.pin_d0       = 5;
        config.pin_vsync    = 25;
        config.pin_href     = 23;
        config.pin_pclk     = 22;
        config.xclk_freq_hz = 20'000'000;
        config.frame_size   = FRAMESIZE_SVGA;
        config.pixel_format = PIXFORMAT_JPEG;
        config.jpeg_quality = 15;
        config.fb_count     = 10;

        if (esp_camera_init(&config) != ESP_OK)
        {
            ESP_LOGE(TAG, "Camera init failed");
            abort();
        }

        xTaskCreate([](void* arg) { static_cast<Camera*>(arg)->CaptureTask(); }, "capture", 4'096, this, 5, nullptr);
    }

    void ReturnFrame(camera_fb_t* fb)
    {
        esp_camera_fb_return(fb);
    }

private:
    void CaptureTask()
    {
        while (true)
        {
            camera_fb_t* frame = esp_camera_fb_get();
            if (!frame)
            {
                vTaskDelay(pdMS_TO_TICKS(m_delay_ms));
                continue;
            }

            if (!m_queue.Push(frame, pdMS_TO_TICKS(10)))
            {
                ReturnFrame(frame);
                ESP_LOGW(TAG, "Frame dropped");
            }
        }
    }

    FrameQueue&    m_queue;
    const uint32_t m_delay_ms;
};

constexpr uint32_t kMaxPayloadSize = 1'400;
constexpr uint32_t kHeaderSize     = 10;

class FrameStreamer
{
public:
    FrameStreamer(const char* ip, uint16_t port, FrameQueue& queue, Camera& camera)
        : m_server_ip{ip}
        , m_server_port{port}
        , m_queue{queue}
        , m_camera{camera}
        , m_sock{-1}
        , m_frame_counter{0}
    {
        xTaskCreate([](void* arg) { static_cast<FrameStreamer*>(arg)->StreamTask(); }, "streamer", 4'096, this, 6, nullptr);
    }

private:
    void SetupUdpSocket()
    {
        m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (m_sock < 0)
        {
            ESP_LOGE(TAG, "UDP socket creation failed");
            return;
        }

        memset(&m_dest_addr, 0, sizeof(m_dest_addr));
        m_dest_addr.sin_family = AF_INET;
        m_dest_addr.sin_port   = htons(m_server_port);
        inet_pton(AF_INET, m_server_ip, &m_dest_addr.sin_addr.s_addr);

        ESP_LOGI(TAG, "UDP socket ready for %s:%d", m_server_ip, m_server_port);
    }

    void StreamTask()
    {
        SetupUdpSocket();

        while (true)
        {
            camera_fb_t* frame = nullptr;
            if (m_queue.Pop(&frame, pdMS_TO_TICKS(10)))
            {
                SendFrameUdp(frame);
                m_camera.ReturnFrame(frame);
            }
        }

        if (m_sock >= 0)
        {
            close(m_sock);
            m_sock = -1;
        }
    }

    void SendFrameUdp(camera_fb_t* frame)
    {
        m_frame_counter++;

        const size_t total_packets = (frame->len + kMaxPayloadSize - 1) / kMaxPayloadSize;
        if (total_packets > 65'535)
        {
            ESP_LOGE(TAG, "Frame too large");
            return;
        }

        for (size_t i = 0; i < total_packets; i++)
        {
            const size_t offset    = i * kMaxPayloadSize;
            const size_t data_size = (i == total_packets - 1) ? (frame->len - offset) : kMaxPayloadSize;

            *reinterpret_cast<uint32_t*>(m_buffer)     = m_frame_counter;
            *reinterpret_cast<uint16_t*>(m_buffer + 4) = i;
            *reinterpret_cast<uint16_t*>(m_buffer + 6) = total_packets;
            *reinterpret_cast<uint16_t*>(m_buffer + 8) = data_size;

            memcpy(m_buffer + kHeaderSize, frame->buf + offset, data_size);

            sendto(m_sock, m_buffer, kHeaderSize + data_size, 0, (struct sockaddr*)&m_dest_addr, sizeof(m_dest_addr));
        }
    }

    const char* m_server_ip;
    uint16_t    m_server_port;

    FrameQueue& m_queue;
    Camera&     m_camera;

    int                m_sock;
    struct sockaddr_in m_dest_addr;

    uint32_t m_frame_counter;

    uint8_t m_buffer[kHeaderSize + kMaxPayloadSize];
};

extern "C" void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    FrameQueue frameQueue(5);
    Camera     camera(frameQueue, 30);

    Wifi wifi(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_MAXIMUM_RETRY);

    FrameStreamer streamer(CONFIG_EXAMPLE_IPV4_ADDR, CONFIG_EXAMPLE_PORT, frameQueue, camera);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }
}
