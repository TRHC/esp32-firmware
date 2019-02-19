#include "esp_system.h"

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

uint8_t wifi_off[8] = {
    0b11111,
	0b10001,
	0b11111,
	0b00000,
	0b01110,
	0b10001,
	0b01010,
	0b00100
};

uint8_t wifi_on[8] = {
    0b11111,
	0b11111,
	0b11111,
	0b00000,
	0b01110,
	0b11111,
	0b01110,
	0b00100
};

uint8_t mqtt_on[8] = {
    0b11111,
	0b11111,
	0b11111,
	0b00000,
	0b01100,
	0b01010,
	0b01010,
	0b01100
};

uint8_t mqtt_off[8] = {
    0b11111,
	0b10001,
	0b11111,
	0b00000,
	0b01100,
	0b01010,
	0b01010,
	0b01100
};

uint8_t tp_on[8] = {
	0b11111,
	0b11111,
	0b11111,
	0b00000,
	0b01110,
	0b00100,
	0b00100,
	0b00100
};

uint8_t tp_off[8] = {
	0b11111,
	0b10001,
	0b11111,
	0b00000,
	0b01110,
	0b00100,
	0b00100,
	0b00100
};