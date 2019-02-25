#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "esp_spiffs.h"
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
#include "app_mqtt.h"
#include "app_wifi.h"
#include "app_display.h"
#include "app_tasks.h"

#define TAG "app"

// I2C Configuration
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       80000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

EventGroupHandle_t s_network_event_group;
i2c_lcd1602_info_t * lcd2004;
ccs811_sensor_t* ccs811;
esp_mqtt_client_handle_t mqtt_client;
nvs_handle my_handle;

float tc, rh;
uint16_t tvoc, eco2, baseline;
uint16_t nvs_base = 4000;
uint8_t  s_retry_num = 0;
bool s_pad_activated;
bool s_display_meas = true;
bool s_ccs811_res;
bool s_ccs881_ready;

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

    if ((pad_intr >> CONFIG_TOUCH_INFO_NUMBER) & 0x01) {
        s_pad_activated = true;
    }
}

void tp_init() {
    uint16_t touch_value;

    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(CONFIG_TOUCH_INFO_NUMBER, 0);
    touch_pad_filter_start(CONFIG_TOUCH_FILTER_PERIOD);

    touch_pad_read_filtered(CONFIG_TOUCH_INFO_NUMBER, &touch_value);
    ESP_LOGI("tp", "touch pad val is %d", touch_value);
    ESP_ERROR_CHECK(touch_pad_set_thresh(CONFIG_TOUCH_INFO_NUMBER, touch_value * CONFIG_TOUCH_THRESH_PERCENT / 100));

    touch_pad_isr_register(tp_rtc_intr, NULL);
    touch_pad_intr_enable();

}

static esp_err_t init_spiffs(void) {
    ESP_LOGI("spiffs", "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,   // This decides the maximum number of files that can be created on the storage
      .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("spiffs", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("spiffs", "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE("spiffs", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("spiffs", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI("spiffs", "Partition size: total: %d, used: %d", total, used);
    return ESP_OK;
}

void app_main() {
    ESP_LOGW(TAG, "TRHC-Monitor firmware start");
    s_network_event_group = xEventGroupCreate();

    nvs_init();
    init_spiffs();
    tp_init();
    i2c_master_init();

    // Initialize I2C Slaves
    i2c_display_init();
    i2c_ccs811_init();
    
    xTaskCreate(&display_measure_task, "display_measure_task", 2048, NULL, 5, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2048, NULL, 4, NULL);
    wifi_init_sta();
    xTaskCreate(&ccs811_ready_task, "ccs811_ready_task", 1512, NULL, 5, NULL);
}

