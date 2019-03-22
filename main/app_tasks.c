#include "app_main.h"
#include "app_display.h"


void mqtt_task(void * pvParameter) {
    char buf[100];
    EventBits_t uxBits;
    ESP_LOGI("mqtt", "Task registered");
    vTaskDelay(7000 / portTICK_PERIOD_MS);
    while(true) {
        uxBits = xEventGroupWaitBits(s_network_event_group, BIT0 | BIT1, false, true, portTICK_PERIOD_MS );
        if(uxBits & BIT1) {
            sprintf(buf, "%.2f,%.2f", tc, rh);
            if (s_temp_measured) {
                esp_mqtt_client_publish(mqtt_client, "esp32_iot/env_data", buf, 0, 2, 0);
            }
            if(s_ccs881_ready) {
                sprintf(buf, "%d,%d,%d", tvoc, eco2, baseline);
                esp_mqtt_client_publish(mqtt_client, "esp32_iot/air_quality", buf, 0, 2, 0);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void measure_task(void * pvParameter) {
    while(true) {
        tc = si7021_read_temperature();
        rh = si7021_read_humidity();
        if(tc == -999.0) {
            s_temp_measured = false;
        } else {
            s_temp_measured = true;
        }

        // ccs811_set_environmental_data(ccs811, tc, rh);
        s_ccs811_res = ccs811_get_results(ccs811, &tvoc, &eco2, 0, 0);
        baseline     = ccs811_get_baseline(ccs811);
        xEventGroupSetBits(s_status_event_group, BIT0);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void display_task(void * pvParameter) {
    char buf[20], buf1[20];
    EventBits_t uxBits;


    while(true) {
        // Retrive env. data and compensate CCS811
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
                if (s_ccs811_res) {
                    display_air();
                } else  {
                    ESP_LOGE("ccs811", "Unable to retrieve sensor data");
                } 
            } else if(s_display_meas) {
                display_error();
            } else {
                display_info();
            }

        }
    }
}

void ccs811_ready_task(void * pvParameter) {
    vTaskDelay(60000 * CONFIG_CCS811_READY_MIN / portTICK_PERIOD_MS);
    ESP_LOGW("ccs811", "Setting CCS811 sensor in ready-state");

    ccs811_set_baseline(ccs811, nvs_base);
    s_ccs881_ready = true;
    while(true) {
        vTaskDelay(60000 * CONFIG_CCS811_BASE_SAVE_PERIOD / portTICK_PERIOD_MS);
        nvs_open("storage", NVS_READWRITE, &my_handle);
        nvs_commit(my_handle);
        if (baseline != 0) {
            nvs_set_u16(my_handle, "nvs_base", baseline);
        }
        nvs_close(my_handle);
        ESP_LOGW("ccs811", "Successful save of BASELINE param");
    }
}
