#include "app_main.h"

void display_status() {
    i2c_lcd1602_move_cursor(lcd2004, 17, 0);

    if(s_display_meas) {  
       i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_7); 
    } else {
       i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_6);
    }

    // MQTT
    if(xEventGroupGetBits(s_network_event_group) & BIT1) {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_5);
    } else {
        i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_4);
    }

    // WiFi
    if(xEventGroupGetBits(s_network_event_group) & BIT0) {
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
    i2c_lcd1602_write_string(lcd2004, "       ");
    i2c_lcd1602_move_cursor(lcd2004, 0, 2);
    i2c_lcd1602_write_string(lcd2004, buf);

    sprintf(buf, "eCO2: %uppm", eco2);
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, "       ");
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

    sprintf(buf, "Base: %d", baseline);
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, "          ");
    i2c_lcd1602_move_cursor(lcd2004, 0, 3);
    i2c_lcd1602_write_string(lcd2004, buf); 
}