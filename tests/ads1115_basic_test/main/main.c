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
            .shunt_resistor = 10,

            .rate_cfg = rate_16sps,

            .channel_gains = {FS_256mV, FS_256mV, FS_256mV, FS_256mV},
            .channel_muxs = {AIN0_single_end, AIN1_single_end, AIN2_single_end, AIN3_single_end},
            .channel_mean_nums = {10, 10, 0, 0}, // no mean calculation for now

            .use_drdy_interrupt = true,    // whether to use drdy interrupt or just poll for conversion completion in continuous read mode; if true, int_pin must be set to a valid gpio pin
            .continuous_conversion = true, // whether adc should read continuously or only when activated
            .read_task_period_ms = 1000,   // period of continuous read task in ms if continuous adc read is deisabled
            .continuous_read_task_enabled = true,
        };
    adc_ads1115_init(&adc, &adc_cfg);
    adc_ads1115_begin(&adc, bus_hdl);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "voltage: %f", adc_ads1115_read_milliamps(&adc, 0));
        ESP_LOGI(TAG, "voltage: %f", adc_ads1115_read_milliamps(&adc, 1));
    }
}