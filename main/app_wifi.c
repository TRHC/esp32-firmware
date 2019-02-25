#include "app_main.h"
#include "app_mqtt.h"
#include <esp_http_server.h>

char* wifi_conf_ssid;
char* wifi_conf_pass;
static httpd_handle_t server = NULL;

static int getConnectionInfo() {
    nvs_handle wifi_nvs_handle;
    size_t req_size;
    nvs_open("wifiConf", NVS_READONLY, &wifi_nvs_handle);

    nvs_get_str(wifi_nvs_handle, "ssid", NULL, &req_size);
    wifi_conf_ssid = malloc(req_size);
    nvs_get_str(wifi_nvs_handle, "ssid", wifi_conf_ssid, &req_size);

    nvs_get_str(wifi_nvs_handle, "pass", NULL, &req_size);
    wifi_conf_pass = malloc(req_size);
    nvs_get_str(wifi_nvs_handle, "pass", wifi_conf_pass, &req_size);

    nvs_close(wifi_nvs_handle);

    if(strlen(wifi_conf_ssid) == 0) {
        return -1;
    }

	return 0;
}

static void saveConnectionInfo(const char* ssid, const char* pass) {
    nvs_handle wifi_nvs_handle;
    nvs_open("wifiConf", NVS_READWRITE, &wifi_nvs_handle);

    nvs_set_str(wifi_nvs_handle, "ssid", ssid);
    nvs_set_str(wifi_nvs_handle, "pass", pass);

    nvs_commit(wifi_nvs_handle);
    nvs_close(wifi_nvs_handle);
}

static esp_err_t http_index_html(httpd_req_t *req) {
    extern const unsigned char index_start[] asm("_binary_index_html_start");
    extern const unsigned char index_end[] asm("_binary_index_html_end");
    const size_t index_size = (index_end - index_start);

    httpd_resp_send_chunk(req, (const char *)index_start, index_size);
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

httpd_uri_t index_html = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_index_html,
    .user_ctx  = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_html);
        return server;
    }

    ESP_LOGE("http", "Error starting server!");
    return NULL;
}

esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI("wifi", "Got IP: %s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_network_event_group, BIT0);
        mqtt_app_start();
        start_webserver();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
                xEventGroupClearBits(s_network_event_group, BIT0);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI("wifi", "trying to reconnect to wifi %dth time", s_retry_num);
            } else {
                vTaskDelay(50000 / portTICK_PERIOD_MS);
                ESP_LOGI("wifi", "lazy attempt to reconnect");
                esp_wifi_connect();
            }
            ESP_LOGW("wifi", "connect to the AP fail\n");
            break;
        }
    default:
        break;
    }
    return ESP_OK;
}

void wifi_init_sta() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI("wifi", "wifi_init_sta finished.");
    ESP_LOGI("wifi", "connect to ap SSID:%s password:%s",
             CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
}