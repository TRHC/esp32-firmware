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
#include "driver/ledc.h"
#include "iot_touchpad.h"
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

#include "trhc.h"

#define TAG "app"

// I2C Configuration
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

EventGroupHandle_t s_network_event_group;
EventGroupHandle_t s_status_event_group;
i2c_lcd1602_info_t * lcd2004;
esp_mqtt_client_handle_t mqtt_client;
nvs_handle my_handle;

float tc;
uint8_t s_retry_num ;

bool s_pad_activated;
bool s_display_meas = false;
bool s_ccs881_measured;
bool s_ccs881_ready;
bool s_temp_measured;

static void i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;  // GY-2561 provides 10kΩ pullups
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
        i2c_lcd1602_write_string(lcd2004, "Bi-Ice (c) 2019;");
        i2c_lcd1602_move_cursor(lcd2004, 14, 3);
        i2c_lcd1602_write_string(lcd2004, "v3.1.1");
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
        nvs_close(my_handle);
    }
}

static void tp_a_cb(tp_handle_t tp_dev) {
    xEventGroupSetBits(s_status_event_group, (BIT0 | BIT1));
}

static void tp_b_cb(tp_handle_t tp_dev) {
    gpio_set_level(12, 0);
}

static void tp_b_hold_cb(tp_handle_t tp_dev) {
    gpio_set_level(12, 1);
}

static void tp_init() {
    tp_handle_t a_tp_handle = iot_tp_create(CONFIG_TOUCH_A_NUMBER, 0.1);
    iot_tp_add_cb(a_tp_handle, TOUCHPAD_CB_TAP, tp_a_cb, NULL);

    tp_handle_t b_tp_handle = iot_tp_create(CONFIG_TOUCH_B_NUMBER, 0.1);
    iot_tp_add_cb(b_tp_handle, TOUCHPAD_CB_TAP, tp_b_cb, NULL);
    iot_tp_add_custom_cb(b_tp_handle, 1, tp_b_hold_cb, NULL);
    ESP_LOGI("touch", "Added TPs");
}

void gpio_init() {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SEL_12;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
    gpio_set_level(12, 0);
}

void buzzer_init() {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 1000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
        .timer_num = LEDC_TIMER_0           // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_conf = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 20,
        .gpio_num   = 21,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };

    ledc_channel_config(&ledc_conf);
    
    vTaskDelay(130/portTICK_RATE_MS);
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1500);
    vTaskDelay(350/portTICK_RATE_MS);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
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

    // Initialize I2C Slaves
    i2c_master_init();
    i2c_display_init();

    s_network_event_group = xEventGroupCreate();
    s_status_event_group  = xEventGroupCreate();
    xEventGroupSetBits(s_status_event_group, BIT1);

    // buzzer_init();
    nvs_init();
    init_spiffs();
    tp_init();
    gpio_init();
    wifi_init_sta();
    init_ds18b20();
    
    xTaskCreate(&display_task, "display_task", 2048, NULL, 5, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 2048, NULL, 4, NULL);
    xTaskCreate(&measure_task, "measure_task", 2048, NULL, 4, NULL);
}

