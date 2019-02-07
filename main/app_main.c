/**
 * @file app_main.c
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "rom/uart.h"
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

static i2c_lcd1602_info_t * lcd2004;
static ccs811_sensor_t* ccs811;

nvs_handle my_handle;
uint32_t nvs_base = 93000;

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

uint8_t drop[8] =
{
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10001,
    0b10001,
    0b01110,
};


static void i2c_master_init(void)
{
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
    char buf[20];
    float tc, rh;
    uint16_t tvoc, eco2;
    uint32_t base;

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    i2c_lcd1602_clear(lcd2004);
    i2c_lcd1602_move_cursor(lcd2004, 12, 3);
    i2c_lcd1602_write_string(lcd2004, "g.betsan");

    while(true) {
        tc = si7021_read_temperature();
        rh = si7021_read_humidity();
        ccs811_set_environmental_data(ccs811, tc, rh);

        i2c_lcd1602_move_cursor(lcd2004, 0, 3);
        sprintf(buf, "%.1f%%", uxTaskGetStackHighWaterMark( NULL ) / 40.96);
        i2c_lcd1602_write_string(lcd2004, buf);

        sprintf(buf, "%.2f", rh);
        i2c_lcd1602_move_cursor(lcd2004, 0, 0);
        i2c_lcd1602_write_string(lcd2004, buf);
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_1);

        sprintf(buf, "%.2f", tc);
        i2c_lcd1602_move_cursor(lcd2004, 0, 1);
        i2c_lcd1602_write_string(lcd2004, buf);
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_0);

        if (ccs811_get_results(ccs811, &tvoc, &eco2, 0, 0)) {
            sprintf(buf, "TVOC: %d", tvoc);
            i2c_lcd1602_move_cursor(lcd2004, 8, 0);
            i2c_lcd1602_write_string(lcd2004, buf);

            sprintf(buf, "eCO2: %d", eco2);
            i2c_lcd1602_move_cursor(lcd2004, 8, 1);
            i2c_lcd1602_write_string(lcd2004, buf);

            base = ccs811_get_baseline(ccs811);
            sprintf(buf, "Base: %d", base);
            i2c_lcd1602_move_cursor(lcd2004, 6, 2);
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

        err = nvs_get_u32(my_handle, "nvs_base", &nvs_base);
        switch (err) {
            case ESP_OK:
                ESP_LOGI("nvs", "Successful reading of stored BASELINE param: %d", nvs_base);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW("nvs", "BASELINE param was not found in memory, writing default: %d", nvs_base);
                err = nvs_set_u32(my_handle, "nvs_base", nvs_base);
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

void app_main()
{
    ESP_LOGW(TAG, "TRHC-Monitor firmware start");
    i2c_master_init();

    // Initialize I2C Slaves
    i2c_display_init();
    i2c_ccs811_init();
    nvs_init();
    
    xTaskCreate(&display_measure_task, "display_measure_task", 2048, NULL, 5, NULL);
}

