#include "trhc.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

OneWireBus * owb;
DS18B20_Info * ds18b20_info;


esp_err_t init_ds18b20() {
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, CONFIG_OWB_PIN, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    OneWireBus_ROMCode rom_code;
    owb_status status = owb_read_rom(owb, &rom_code);
    if (status == OWB_STATUS_OK) {
        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
        printf("Single device %s present\n", rom_code_s);

        ds18b20_info = ds18b20_malloc();
        ds18b20_init_solo(ds18b20_info, owb);
        ds18b20_use_crc(ds18b20_info, true);
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_9_BIT);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        return ESP_OK;
    } else {
        printf("An error occurred reading ROM code: %d", status);
        return ESP_FAIL;
    }

    
}

float get_temp() {
    ds18b20_convert_and_read_temp(ds18b20_info, &tc); 
    return tc;
}
