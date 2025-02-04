#include <stdio.h>
#include "ads1115_adc.h"

#define TAG "ADS1115 Test"

void app_main(void)
{
    vTaskDelay(500);
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_5,
        .sda_io_num = GPIO_NUM_6,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_hdl;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_hdl));
    adc_ads1115 adc;
    ads1115_cfg adc_cfg =
        {
            .i2c_addr = 0x48,
            .int_pin = GPIO_NUM_7,
            .shunt_resistor = 10, // Ω
        };
    adc_ads1115_init(&adc, &adc_cfg);
    adc_ads1115_begin(&adc, bus_hdl);

    adc_ads1115_enable_drdy(&adc);
    adc_ads1115_set_mux(&adc, AIN0_diff_AIN1);
    adc_ads1115_set_pga(&adc, FS_4096mV);
    adc_ads1115_set_conv_mode(&adc, false);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "voltage: %f", adc_ads1115_readVolt(&adc));
    }
}