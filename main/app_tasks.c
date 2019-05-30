#include "trhc.h"

void mqtt_task(void * pvParameter) {
    char buf[100];
    EventBits_t uxBits;
    ESP_LOGI("mqtt", "Task registered");
    vTaskDelay(7000 / portTICK_PERIOD_MS);
    while(true) {
        uxBits = xEventGroupWaitBits(s_network_event_group, BIT0 | BIT1, false, true, portTICK_PERIOD_MS );
        if(uxBits & BIT1) {
            sprintf(buf, "%.2f", tc);
            if (s_temp_measured) {
                esp_mqtt_client_publish(mqtt_client, "esp32_iot/env_data", buf, 0, 2, 0);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void measure_task(void * pvParameter) {
    float thrc;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    while(true) {
        if(tc == -999.0) {
            s_temp_measured = false;
        } else {
            s_temp_measured = true;
        }

        xEventGroupSetBits(s_status_event_group, BIT0);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void display_task(void * pvParameter) {
    char buf[20], buf1[20];
    EventBits_t uxBits;


    while(true) {
        uxBits = xEventGroupWaitBits(s_status_event_group, BIT0, false, true, portTICK_PERIOD_MS );
        if (uxBits & BIT0) {
            xEventGroupClearBits(s_status_event_group, BIT0);

            uxBits = xEventGroupWaitBits(s_status_event_group, BIT1, false, true, portTICK_PERIOD_MS );
            if(uxBits & BIT1) {
                xEventGroupClearBits(s_status_event_group, BIT1);
                s_display_meas = !s_display_meas;
                i2c_lcd1602_clear(lcd2004);
            }

            display_status();
            if(s_display_meas && s_temp_measured) {
                display_env();
            } else if(s_display_meas) {
                display_error();
            } else {
                display_info();
            }

        }
    }
}