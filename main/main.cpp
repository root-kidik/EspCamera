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

class WiFiManager
{
public:
    WiFiManager(const char* ssid, const char* pass, int max_retry) : ssid_(ssid), pass_(pass), max_retry_(max_retry)
    {
    }

    esp_err_t connect()
    {
        s_wifi_event_group_ = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            [](void* arg, esp_event_base_t base, int32_t id, void* data)
            { static_cast<WiFiManager*>(arg)->event_handler(base, id, data); },
            this,
            &instance_any_id_);

        esp_event_handler_instance_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            [](void* arg, esp_event_base_t base, int32_t id, void* data)
            { static_cast<WiFiManager*>(arg)->event_handler(base, id, data); },
            this,
            &instance_got_ip_);

        wifi_config_t wifi_config = {};
        strlcpy((char*)wifi_config.sta.ssid, ssid_, sizeof(wifi_config.sta.ssid));
        strlcpy((char*)wifi_config.sta.password, pass_, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group_, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "Connected to AP: %s", ssid_);
            return ESP_OK;
        }
        else
        {
            ESP_LOGE(TAG, "Failed to connect: %s", ssid_);
            return ESP_FAIL;
        }
    }

private:
    void event_handler(esp_event_base_t base, int32_t id, void* data)
    {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
        }
        else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (retry_num_ < max_retry_)
            {
                esp_wifi_connect();
                retry_num_++;
                ESP_LOGI(TAG, "Retry #%d", retry_num_);
            }
            else
            {
                xEventGroupSetBits(s_wifi_event_group_, WIFI_FAIL_BIT);
            }
        }
        else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
        {
            retry_num_ = 0;
            xEventGroupSetBits(s_wifi_event_group_, WIFI_CONNECTED_BIT);
        }
    }

    const char* ssid_;
    const char* pass_;
    int         max_retry_;

    int                          retry_num_          = 0;
    EventGroupHandle_t           s_wifi_event_group_ = nullptr;
    esp_event_handler_instance_t instance_any_id_    = nullptr;
    esp_event_handler_instance_t instance_got_ip_    = nullptr;

    static constexpr auto WIFI_CONNECTED_BIT = BIT0;
    static constexpr auto WIFI_FAIL_BIT      = BIT1;
};

class Camera
{
public:
    Camera()
    {
        config_.pin_pwdn     = 32;
        config_.pin_reset    = -1;
        config_.pin_xclk     = 0;
        config_.pin_sccb_sda = 26;
        config_.pin_sccb_scl = 27;
        config_.pin_d7       = 35;
        config_.pin_d6       = 34;
        config_.pin_d5       = 39;
        config_.pin_d4       = 36;
        config_.pin_d3       = 21;
        config_.pin_d2       = 19;
        config_.pin_d1       = 18;
        config_.pin_d0       = 5;
        config_.pin_vsync    = 25;
        config_.pin_href     = 23;
        config_.pin_pclk     = 22;
        config_.xclk_freq_hz = 20'000'000;
        config_.frame_size   = FRAMESIZE_SVGA;
        config_.pixel_format = PIXFORMAT_JPEG;
        config_.jpeg_quality = 15;
        config_.fb_count     = 10;
    }

    esp_err_t init()
    {
        return esp_camera_init(&config_);
    }

    camera_fb_t* capture_frame()
    {
        return esp_camera_fb_get();
    }

    void return_frame(camera_fb_t* fb)
    {
        esp_camera_fb_return(fb);
    }

private:
    camera_config_t config_ = {};
};

class FrameSender
{
public:
    FrameSender(const char* ip, uint16_t port, QueueHandle_t queue, Camera& camera)
        : server_ip_(ip)
        , server_port_(port)
        , frame_queue_(queue)
        , camera_(camera)
    {
    }

    void start()
    {
        xTaskCreate([](void* arg) { static_cast<FrameSender*>(arg)->run(); }, "sender", 4'096, this, 6, nullptr);
    }

private:
    void run()
    {
        setup_udp_socket();

        while (true)
        {
            camera_fb_t* frame = nullptr;
            if (xQueueReceive(frame_queue_, &frame, pdMS_TO_TICKS(10)) == pdPASS)
            {
                send_frame_udp(frame);
                camera_.return_frame(frame);
            }
        }

        close(sock_);
        sock_ = -1;
    }

    void setup_udp_socket()
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock_ < 0)
        {
            ESP_LOGE(TAG, "UDP socket creation failed");
            return;
        }

        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port   = htons(server_port_);
        inet_pton(AF_INET, server_ip_, &dest_addr_.sin_addr.s_addr);

        ESP_LOGI(TAG, "UDP socket ready for %s:%d", server_ip_, server_port_);
    }

    void send_frame_udp(camera_fb_t* frame)
    {
        const size_t max_payload = 1'400;
        const size_t header_size = 10;
        uint8_t      packet[header_size + max_payload];

        static uint32_t frame_counter = 0;
        frame_counter++;

        const size_t total_packets = (frame->len + max_payload - 1) / max_payload;
        if (total_packets > 65'535)
        {
            ESP_LOGE(TAG, "Frame too large");
            return;
        }

        for (size_t i = 0; i < total_packets; i++)
        {
            const size_t offset    = i * max_payload;
            const size_t data_size = (i == total_packets - 1) ? (frame->len - offset) : max_payload;

            *reinterpret_cast<uint32_t*>(packet)     = frame_counter;
            *reinterpret_cast<uint16_t*>(packet + 4) = i;
            *reinterpret_cast<uint16_t*>(packet + 6) = total_packets;
            *reinterpret_cast<uint16_t*>(packet + 8) = data_size;

            memcpy(packet + header_size, frame->buf + offset, data_size);

            sendto(sock_, packet, header_size + data_size, 0, (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
        }
    }

private:
    const char*        server_ip_;
    uint16_t           server_port_;
    QueueHandle_t      frame_queue_;
    Camera&            camera_;
    int                sock_ = -1;
    struct sockaddr_in dest_addr_;
};

class FrameCapture
{
public:
    FrameCapture(Camera& camera, QueueHandle_t queue, uint32_t fps)
        : camera_(camera)
        , frame_queue_(queue)
        , delay_ms_(1'000 / fps)
    {
    }

    void start()
    {
        xTaskCreate([](void* arg) { static_cast<FrameCapture*>(arg)->run(); }, "capture", 4'096, this, 5, nullptr);
    }

private:
    void run()
    {
        while (true)
        {
            camera_fb_t* frame = camera_.capture_frame();
            if (!frame)
            {
                vTaskDelay(pdMS_TO_TICKS(delay_ms_));
                continue;
            }

            if (xQueueSend(frame_queue_, &frame, pdMS_TO_TICKS(10)) != pdPASS)
            {
                camera_.return_frame(frame);
                ESP_LOGW(TAG, "Frame dropped");
            }
        }
    }

    Camera&        camera_;
    QueueHandle_t  frame_queue_;
    const uint32_t delay_ms_;
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

    Camera camera;
    if (camera.init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed");
        return;
    }

    WiFiManager wifi(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_MAXIMUM_RETRY);
    if (wifi.connect() != ESP_OK)
    {
        return;
    }

    QueueHandle_t frame_queue = xQueueCreate(5, sizeof(camera_fb_t*));
    if (!frame_queue)
    {
        ESP_LOGE(TAG, "Queue creation failed");
        return;
    }

    FrameCapture capture(camera, frame_queue, 30);
    capture.start();

    FrameSender sender(CONFIG_EXAMPLE_IPV4_ADDR, CONFIG_EXAMPLE_PORT, frame_queue, camera);
    sender.start();

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1'000));
    }
}
