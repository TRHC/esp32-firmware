#include "app_main.h"
#include "app_mqtt.h"


esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI("wifi", "Got IP: %s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, BIT0);
        mqtt_app_start();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
                xEventGroupClearBits(s_wifi_event_group, BIT0);
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI("wifi", "retry to connect to the AP");
            }
            ESP_LOGI("wifi", "connect to the AP fail\n");
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