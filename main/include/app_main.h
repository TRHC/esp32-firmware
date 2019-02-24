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


#ifndef APP_MAIN_H_   
#define APP_MAIN_H_

// WiFi Configuration
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t s_mqtt_event_group;
static i2c_lcd1602_info_t * lcd2004;
static ccs811_sensor_t* ccs811;
static esp_mqtt_client_handle_t mqtt_client;
static nvs_handle my_handle;

static float tc, rh;
static uint16_t tvoc, eco2, baseline;
static uint16_t nvs_base;
static uint8_t  s_retry_num;
static bool s_pad_activated;
static bool s_display_meas;
static bool s_ccs811_res;
static bool s_ccs881_ready;

#endif