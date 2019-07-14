#include "trhc.h"

void define_status() {
    i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_2, wifi_off);
    i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_3, wifi_on);
    i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_4, mqtt_off);
    i2c_lcd1602_define_char(lcd2004, I2C_LCD1602_INDEX_CUSTOM_5, mqtt_on);
}

void define_bigNums() {
    uint8_t i;
    for(i = 0; i < 8; i++) {
        i2c_lcd1602_define_char(lcd2004, i, bars[i]);
    }
}

void write_custom(const uint8_t ch, int col, int row) {
    i2c_lcd1602_move_cursor(lcd2004, col, row);
    if(ch == 8) {
        i2c_lcd1602_write_string(lcd2004, " "); 
    } else {
        i2c_lcd1602_write_char(lcd2004, ch); 
    }
}

void print_bignum(uint8_t num, int col, int row) {
    uint8_t i, ii, ccol, crow;
    
    if(num == 11) { // it is dot (.)
        i2c_lcd1602_move_cursor(lcd2004, col, 2);
        i2c_lcd1602_write_char(lcd2004, 7); 
        return;
    }

    for(i = 0; i < 9; i++) {
        ccol = i % 3;
        crow = i / 3;
        write_custom(nums[num][i], col + ccol, row + crow);
    }
}

void print_bigFloat(float num) {
    char buf[6];
    uint8_t i, col;
    col = 0;

    sprintf(buf, "%.2f", num);
    // ESP_LOGI("dbg", "DS18B20: %s", buf);
    for(i = 0; i <= 5; i++) {
        char ch = buf[i];
        if(ch == '-') {
            print_bignum(10, col, 0);
            col += 4;
        } else if(ch == '.') {
            print_bignum(11, col, 0);
            col += 2;
        } else if(ch <= 57 && ch >= 48) {
            print_bignum(ch - '0', col, 0);
            col += 4;
        }
    }
}

void display_status() {
    define_status();
    i2c_lcd1602_move_cursor(lcd2004, 17, 0);

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
    define_bigNums();
    print_bigFloat(tc);
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