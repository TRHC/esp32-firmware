#ifndef TRHC_H
#define TRHC_H
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
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"


// Event groups
    EventGroupHandle_t s_network_event_group;
    EventGroupHandle_t s_status_event_group;

// Peripheral
    i2c_lcd1602_info_t * lcd2004;

// Handlers
    esp_mqtt_client_handle_t mqtt_client;
    nvs_handle my_handle;

// Variables
    float tc;
    uint8_t  s_retry_num;
    bool s_pad_activated;
    bool s_display_meas;
    bool s_temp_measured;


// Display
    void display_info();
    void display_env();
    void display_status();
    void display_error();

// MQTT
    void mqtt_app_start();
    void mqtt_task(void *);

// Chars
    uint8_t mqtt_on[8], mqtt_off[8], wifi_on[8], wifi_off[8], drop[8], 
            temp[8], tp_on[8], tp_off[8], bars[8][8];
    
    int nums[10][9];

// WiFi
    esp_err_t wifi_event_handler(void *, system_event_t *);
    void wifi_init_sta();

// Tasks
    void measure_task(void *);
    void watch_task(void *);
    void display_task(void *);
    void mqtt_task(void *);

// DS18B20
    esp_err_t init_ds18b20();
    float get_temp();
    DS18B20_Info ds18b20_info;
    char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];


#endif