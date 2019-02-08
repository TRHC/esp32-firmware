/**
 * @file app_main.c
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"
#include "rom/uart.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "nvs.h"


#include "smbus.h"
#include "i2c-lcd1602.h"
#include "si7021.h"
#include "ccs811.h"

#define TAG "app"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

// WiFi Configuration
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group; // Event group
const int WIFI_CONNECTED_BIT = BIT0;          // 1st bit of event group
static int s_retry_num = 0;

static i2c_lcd1602_info_t * lcd2004;
static ccs811_sensor_t* ccs811;

nvs_handle my_handle;
uint16_t nvs_base = 93000;

static EventGroupHandle_t s_mqtt_event_group; // Event group
esp_mqtt_client_handle_t mqtt_client;

uint8_t temp[8] = {
    0b00100,
    0b01010,
    0b01010,
    0b01110,
    0b01110,
    0b11111,
    0b11111,
    0b01110
};

uint8_t drop[8] = {
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10001,
    0b10001,
    0b01110,
};

uint8_t lobit[8] = {
    0b11111,
	0b10001,
	0b11111,
	0b00000,
	0b01110,
	0b10001,
	0b01010,
	0b00100
};

uint8_t hibit[8] = {
    0b11111,
	0b11111,
	0b11111,
	0b00000,
	0b01110,
	0b11111,
	0b01110,
	0b00100
};

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI("mqtt", "MQTT_EVENT_CONNECTED");
            xEventGroupSetBits(s_mqtt_event_group, BIT0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            xEventGroupClearBits(s_mqtt_event_group, BIT0);
            ESP_LOGI("mqtt", "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void) {
    s_mqtt_event_group = xEventGroupCreate();
    esp_mqtt_client_config_t mqtt_cfg = {
        .host = "mqtt.mydevices.com",
        .client_id = CONFIG_MQTT_CLIENT_ID,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD,
        .event_handle = mqtt_event_handler,
        // .user_context = (void *)your_context
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI("wifi", "Got IP: %s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        mqtt_app_start();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < CONFIG_WIFI_MAXIMUM_RETRY) {
                esp_wifi_connect();
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
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
    s_wifi_event_group = xEventGroupCreate();

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

static void i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

void display_measure_task(void * pvParameter) {
    char buf[100];
    char buf1[100];
    float tc, rh;
    uint16_t tvoc, eco2, base;

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    i2c_lcd1602_clear(lcd2004);
    i2c_lcd1602_move_cursor(lcd2004, 12, 3);
    i2c_lcd1602_write_string(lcd2004, "g.betsan");

    while(true) {
        // Retrive env. data and compensate CCS811
        tc = si7021_read_temperature();
        rh = si7021_read_humidity();
        ccs811_set_environmental_data(ccs811, tc, rh);

        i2c_lcd1602_move_cursor(lcd2004, 19, 0);
        if(xEventGroupGetBits(s_wifi_event_group)) {
            i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_3);
        } else {
            i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_2);
        }

        if(xEventGroupGetBits(s_mqtt_event_group)) {
            sprintf(buf1, "v1/%s/things/%s/data/0", CONFIG_MQTT_USERNAME, CONFIG_MQTT_CLIENT_ID);
            sprintf(buf, "temp,c=%.2f", tc);
            esp_mqtt_client_publish(mqtt_client, buf1, buf, 0, 2, 0);
            
            sprintf(buf1, "v1/%s/things/%s/data/1", CONFIG_MQTT_USERNAME, CONFIG_MQTT_CLIENT_ID);
            sprintf(buf, "rel_hum,p=%.2f", rh);
            esp_mqtt_client_publish(mqtt_client, buf1, buf, 0, 2, 0);
        } else {

        }

        sprintf(buf, "%.2f", rh);
        i2c_lcd1602_move_cursor(lcd2004, 0, 0);
        i2c_lcd1602_write_string(lcd2004, buf);
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_1);

        sprintf(buf, "%.2f", tc);
        i2c_lcd1602_move_cursor(lcd2004, 0, 1);
        i2c_lcd1602_write_string(lcd2004, buf);
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_0);

        if (ccs811_get_results(ccs811, &tvoc, &eco2, 0, 0)) {
            sprintf(buf, "TVOC: %u", tvoc);
            i2c_lcd1602_move_cursor(lcd2004, 0, 2);
            i2c_lcd1602_write_string(lcd2004, buf);

            sprintf(buf, "eCO2: %u", eco2);
            i2c_lcd1602_move_cursor(lcd2004, 0, 3);
            i2c_lcd1602_write_string(lcd2004, buf);

            base = ccs811_get_baseline(ccs811);
            sprintf(buf, "B: %u", base);
            i2c_lcd1602_move_cursor(lcd2004, 12, 2);
            i2c_lcd1602_write_string(lcd2004, buf);
        } else  {
            ESP_LOGE("ccs811", "Unable to retrieve sensor data");
        } 


        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void i2c_display_init() {
    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    smbus_init(smbus_info, I2C_MASTER_NUM, CONFIG_LCD_I2C_ADDRESS);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the LCD device with backlight off and turn it on
    lcd2004 = i2c_lcd1602_malloc();
    i2c_lcd1602_init(lcd2004, smbus_info, true, CONFIG_LCD_ROWS, 40, CONFIG_LCD_COLUMNS);

    if(lcd2004) {
        ESP_LOGI("lcd", "Successful init. on 0x%X", CONFIG_LCD_I2C_ADDRESS);
        i2c_lcd1602_set_backlight(lcd2004, true);
        i2c_lcd1602_move_cursor(lcd2004, 0, 1);
        i2c_lcd1602_write_string(lcd2004, "TRHC-Firmware ESP32;");
        i2c_lcd1602_move_cursor(lcd2004, 14, 3);
        i2c_lcd1602_write_string(lcd2004, "v0.0.1");
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_0, temp);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_1, drop);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_2, lobit);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_3, hibit);
    } else {
        ESP_LOGE("lcd", "Unable to init LCD");
    }
}

void i2c_ccs811_init() {
    ccs811 = ccs811_init_sensor(0, CCS811_I2C_ADDRESS_2);
    if(ccs811) {
        ESP_LOGI("ccs811", "Successful init. on 0x%X", CONFIG_CCS811_I2C_ADDRESS);
    } else {
        ESP_LOGE("ccs811", "Unable to init CCS811");
    }
}

void nvs_init() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("nvs", "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI("nvs", "Successful open of NVS");

        err = nvs_get_u16(my_handle, "nvs_base", &nvs_base);
        switch (err) {
            case ESP_OK:
                ESP_LOGI("nvs", "Successful reading of stored BASELINE param: %d", nvs_base);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW("nvs", "BASELINE param was not found in memory, writing default: %d", nvs_base);
                err = nvs_set_u16(my_handle, "nvs_base", nvs_base);
                if(err != ESP_OK) {
                    ESP_LOGE("nvs", "Error writing default BASELINE to memory");
                } else {
                    ESP_LOGI("nvs", "Successful writing of default BASELINE to memory");
                }
                
                break;
            default :
                ESP_LOGE("nvs", "Error (%s) reading!", esp_err_to_name(err));
        }

        // Write default or retrieved BASELINE
        

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        err = nvs_commit(my_handle);
        if(err != ESP_OK) {
            ESP_LOGE("nvs", "Error commiting NVS changes");
        } else {
            ESP_LOGI("nvs", "Successful commiting NVS changes");
        }
        // Close
        nvs_close(my_handle);
    }
}

void app_main() {
    ESP_LOGW(TAG, "TRHC-Monitor firmware start");
    nvs_init();
    i2c_master_init();

    // Initialize I2C Slaves
    i2c_display_init();
    i2c_ccs811_init();
    
    xTaskCreate(&display_measure_task, "display_measure_task", 2048, NULL, 5, NULL);
    wifi_init_sta();
}

