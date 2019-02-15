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
#include "driver/touch_pad.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "sdkconfig.h"
#include "rom/uart.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "smbus.h"
#include "i2c-lcd1602.h"
#include "si7021.h"
#include "ccs811.h"

#include "chars.h"

#define TAG "app"

// I2C Configuration
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

// Touch pad Configuration
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_THRESH_PERCENT  (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)
#define TOUCH_PAD_NUMBER      (3)

// WiFi Configuration
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group; // Event group

static i2c_lcd1602_info_t * lcd2004;
static ccs811_sensor_t* ccs811;
static float tc, rh;
static uint16_t tvoc, eco2, base;
static int s_retry_num = 0;
static bool s_pad_activated;
static bool s_display_meas = true;
static bool p_display_meas = true;
static bool s_ccs811_res;

TaskHandle_t xHandle;
nvs_handle my_handle;
uint16_t nvs_base = 4000;

static EventGroupHandle_t s_mqtt_event_group; // Event group
esp_mqtt_client_handle_t mqtt_client;

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

static void mqtt_app_start(void) {
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

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
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

void mqtt_task(void * pvParameter) {
    char buf[100];

    vTaskDelay(7000 / portTICK_PERIOD_MS);
    while(true) {
        if(xEventGroupGetBits(s_wifi_event_group)) {
            if(xEventGroupGetBits(s_mqtt_event_group)) {
                sprintf(buf, "%.2f,%.2f,%d,%d", tc, rh, tvoc, eco2);
                esp_mqtt_client_publish(mqtt_client, "esp32_iot/sense", buf, 0, 2, 0);
                //sprintf(buf, "%.2f,%.2f", tc, rh);
                //esp_mqtt_client_publish(mqtt_client, "esp32_iot/si7021", buf, 0, 2, 0);
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void display_status() {
    i2c_lcd1602_move_cursor(lcd2004, 17, 0);

    if(s_display_meas) {  
       i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_7); 
    } else {
       i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_6);
    }

    if(xEventGroupGetBits(s_mqtt_event_group)) {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_5);
    } else {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_4);
    }

    if(xEventGroupGetBits(s_wifi_event_group)) {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_3);
    } else {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_2);
    }
}

void display_env() {
    char buf[20];
    sprintf(buf, "RelH: %.2f%%", rh);
    i2c_lcd1602_move_cursor(lcd2004, 0, 0);
    i2c_lcd1602_write_string(lcd2004, buf);
    i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_1);

    sprintf(buf, "Temp: %.2fC", tc);
    i2c_lcd1602_move_cursor(lcd2004, 0, 1);
    i2c_lcd1602_write_string(lcd2004, buf);
    i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_0);
}

void display_air() {
    char buf[20];

    sprintf(buf, "TVOC: %uppb", tvoc);
    i2c_lcd1602_move_cursor(lcd2004, 0, 2);
    i2c_lcd1602_write_string(lcd2004, "    ");
    i2c_lcd1602_move_cursor(lcd2004, 0, 2);
    i2c_lcd1602_write_string(lcd2004, buf);

    sprintf(buf, "eCO2: %uppm", eco2);
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, "    ");
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, buf);
}

void display_info() {
    char buf[60];

    i2c_lcd1602_move_cursor(lcd2004, 14, 3);
    i2c_lcd1602_write_string(lcd2004, "v3.0.1");

    i2c_lcd1602_move_cursor(lcd2004, 12, 2);
    i2c_lcd1602_write_string(lcd2004, "g.betsan");

    sprintf(buf, "Mem: %dKB", esp_get_free_heap_size() / 1024);
    i2c_lcd1602_move_cursor(lcd2004, 0, 0);
    i2c_lcd1602_write_string(lcd2004, buf);
    //i2c_lcd1602_write_string(lcd2004, "Ri: T H _");
    
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    sprintf(buf, "Rev: %d", chip.revision);
    i2c_lcd1602_move_cursor(lcd2004, 0, 1);
    i2c_lcd1602_write_string(lcd2004, buf);
    //i2c_lcd1602_write_string(lcd2004, "Lo: T H q");

    sprintf(buf, "Base: %d", ccs811_get_baseline(ccs811));
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, "          ");
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, buf); 
}

void display_measure_task(void * pvParameter) {
    char buf[20], buf1[20];

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    i2c_lcd1602_clear(lcd2004);

    while(true) {
        // Retrive env. data and compensate CCS811
        tc = si7021_read_temperature();
        rh = si7021_read_humidity();
        ccs811_set_environmental_data(ccs811, tc, rh);
        s_ccs811_res = ccs811_get_results(ccs811, &tvoc, &eco2, 0, 0);

        if(s_pad_activated) {
            s_display_meas = !s_display_meas;
            i2c_lcd1602_clear(lcd2004);
        }

        display_status();
        if(s_display_meas) {
            display_env();
            if (s_ccs811_res) {
                display_air();
            } else  {
                ESP_LOGE("ccs811", "Unable to retrieve sensor data");
            } 
        } else {
            display_info();
        }

        s_pad_activated = false;
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
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_2, wifi_off);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_3, wifi_on);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_4, mqtt_off);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_5, mqtt_on);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_6, tp_on);
        i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_7, tp_off);
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

static void tp_rtc_intr(void * arg) {
    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();

    if ((pad_intr >> TOUCH_PAD_NUMBER) & 0x01) {
        s_pad_activated = true;
    }
}

void tp_init() {
    uint16_t touch_value;

    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PAD_NUMBER, TOUCH_THRESH_NO_USE);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);

    touch_pad_read_filtered(TOUCH_PAD_NUMBER, &touch_value);
    ESP_LOGI("tp", "touch pad val is %d", touch_value);
    ESP_ERROR_CHECK(touch_pad_set_thresh(TOUCH_PAD_NUMBER, touch_value * TOUCH_THRESH_PERCENT / 100));

    touch_pad_isr_register(tp_rtc_intr, NULL);
    touch_pad_intr_enable();

}

void app_main() {
    ESP_LOGW(TAG, "TRHC-Monitor firmware start");
    s_mqtt_event_group = xEventGroupCreate();
    s_wifi_event_group = xEventGroupCreate();

    nvs_init();
    tp_init();
    i2c_master_init();

    // Initialize I2C Slaves
    i2c_display_init();
    i2c_ccs811_init();
    
    xTaskCreate(&display_measure_task, "display_measure_task", 2048, NULL, 5, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2048, NULL, 4, NULL);
    wifi_init_sta();
}

