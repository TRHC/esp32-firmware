#include "trhc.h"

void int_to_bin_digit(uint32_t in, uint8_t* out)
{
    uint8_t count = 8;
    unsigned int mask = 1U << (count-1);
    int i;
    for (i = 0; i < count; i++) {
        out[i] = (in & mask) ? 1 : 0;
        in <<= 1;
    }
}

void write_custom(uint32_t ch) {
    uint8_t custom[8];
    int_to_bin_digit(ch, &custom);
    //ESP_LOGW("dbg", "%d", custom[0]);
    i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_0, custom);
    i2c_lcd1602_write_char(lcd2004, I2C_LCD1602_CHARACTER_CUSTOM_0);
}

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
    sprintf(buf, "%.1f", tc);
    i2c_lcd1602_move_cursor(lcd2004, 0, 1);
    write_custom(bigNums[0][0]);
    i2c_lcd1602_write_string(lcd2004, "Test:");
    //i2c_lcd1602_write_char(lcd2004, (char)bigNums[0][0]);
}

void display_error() {
    i2c_lcd1602_clear(lcd2004);
    i2c_lcd1602_move_cursor(lcd2004, 0, 0);
    i2c_lcd1602_write_string(lcd2004, "No sensors");
    i2c_lcd1602_move_cursor(lcd2004, 12, 2);
    i2c_lcd1602_write_string(lcd2004, "BI-Ice");
    i2c_lcd1602_move_cursor(lcd2004, 12, 3);
    i2c_lcd1602_write_string(lcd2004, "g.betsan");
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
}