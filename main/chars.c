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


uint8_t bars[8][8] = {
	{
		0b11100,
    	0b11110,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111
  	},
	{    
		0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b00000,
    	0b00000,
    	0b00000
	},
	{
		0b00000,
    	0b00000,
  		0b00000,
  		0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111
	},
	{
		0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b01111,
    	0b00111
	},
	{
		0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
   		0b11111,
		0b11111,
    	0b00000,
    	0b00000
	},
	{
		0b00000,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b00000,
    	0b00000
	},
	{
		0b00000,
    	0b11100,
    	0b11110,
    	0b11111,
    	0b11111,
    	0b11111,
   		0b11111,
    	0b11111
	},
	{
		0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111,
    	0b11111
	}
};

int nums[11][9] = {
	{ // 0
		7, 1, 0,
		7, 8, 7,
		3, 2, 7
	},
	{ // 1
		1, 0, 8,
		8, 7, 8,
		2, 7, 2
	},
	{ // 2
		1, 1, 0,
		2, 1, 1,
		7, 2, 2
	},
	{ // 3
		1, 1, 0,
		8, 5, 0,
		2, 2, 7
	},
	{ // 4
		7, 8, 7,
		3, 2, 7,
		8, 8, 7
	},
	{ // 5
		7, 1, 1,
		4, 5, 0,
		3, 2, 7
	},
	{ // 6
		7, 1, 1,
		7, 1, 6,
		3, 2, 7
	},
	{ // 7
		1, 1, 7,
		8, 2, 1,
		8, 7, 8
	},
	{ // 8
		7, 1, 0,
		7, 5, 7,
		3, 2, 7
	},
	{ // 9
		7, 1, 0,
		4, 5, 7,
		8, 8, 7
	},
	{ // -
		8, 8, 8,
		5, 5, 5,
		8, 8, 8
	}
};