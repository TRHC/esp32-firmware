#include "app_main.h"

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("mqtt", "MQTT_EVENT_CONNECTED");
            xEventGroupSetBits(s_mqtt_event_group, BIT0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(s_mqtt_event_group, BIT0);
            ESP_LOGI("mqtt", "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE("mqtt", "MQTT_EVENT_ERROR");
            break;
        default:
            break;
    }
    return ESP_OK;
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = "shpag.ga",
        .client_id = CONFIG_MQTT_CLIENT_ID,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}