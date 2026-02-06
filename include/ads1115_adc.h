#ifndef ADS1115_H
#define ADS1115_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_task.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "math.h"

#define ADS1115_MAX_NUM_CH 4 // max number of channels (if differential channels are used, this number is lower)

typedef enum
{
    AIN0_diff_AIN1,
    AIN0_diff_AIN3,
    AIN1_diff_AIN3,
    AIN2_diff_AIN3,
    AIN0_single_end,
    AIN1_single_end,
    AIN2_single_end,
    AIN3_single_end,
} ads1115_mux_cfg;

typedef enum
{
    FS_6144mV,
    FS_4096mV,
    FS_2048mV,
    FS_1024mV,
    FS_512mV,
    FS_256mV,
} ads1115_pga_cfg;

typedef enum
{
    rate_8sps,
    rate_16sps,
    rate_32sps,
    rate_64sps,
    rate_128sps,
    rate_250sps,
    rate_475sps,
    rate_860sps,
} ads1115_rate_cfg;

typedef enum
{
    assert_1_conv,
    assert_2_conv,
    assert_4_conv,
    comp_off,
} ads1115_comp_queue_cfg;

typedef struct
{
    i2c_master_dev_handle_t dev_hdl;

    // HW
    uint8_t i2c_addr;      // 0x48 to 0x4B
    gpio_num_t int_pin;    // interrupt input pin
    double shunt_resistor; // Ω

    // settings for all channels
    ads1115_rate_cfg rate_cfg;

    // channel specific settings
    ads1115_pga_cfg channel_gains[ADS1115_MAX_NUM_CH];
    ads1115_mux_cfg channel_muxs[ADS1115_MAX_NUM_CH];
    uint8_t channel_mean_nums[ADS1115_MAX_NUM_CH]; // number of means to calculate for each channel if continuous is active; if 0 channel is deactivated

    bool use_drdy_interrupt;           // whether to use drdy interrupt or just poll for conversion completion in continuous read mode; if true, int_pin must be set to a valid gpio pin
    bool continuous_conversion;        // whether adc should read continuously or only when activated
    int16_t read_task_period_ms;       // period of continuous read task in ms if continuous adc read is deisabled
    bool continuous_read_task_enabled; // whether continuous read task is enabled

    TaskHandle_t continuous_read_taskhdl; // task handle for continuous read task; if continuous_conversion is false, this is not used
    uint8_t act_channel_idx;              // current channel being read in continuous mode
    uint8_t mean_idx[ADS1115_MAX_NUM_CH];
    bool mean_valid[ADS1115_MAX_NUM_CH];
    uint8_t mean_num[ADS1115_MAX_NUM_CH]; // one sec for one mean val
    int16_t *mean_arr[ADS1115_MAX_NUM_CH];
    // mutex protects the values above
    SemaphoreHandle_t mean_arr_mutex[ADS1115_MAX_NUM_CH];
} adc_ads1115;

typedef struct
{
    // HW
    uint8_t i2c_addr;      // 0x48 to 0x4B
    gpio_num_t int_pin;    // interrupt input pin
    double shunt_resistor; // Ω

    // settings for all channels
    ads1115_rate_cfg rate_cfg;

    // channel specific settings
    ads1115_pga_cfg channel_gains[ADS1115_MAX_NUM_CH];
    ads1115_mux_cfg channel_muxs[ADS1115_MAX_NUM_CH];
    uint8_t channel_mean_nums[ADS1115_MAX_NUM_CH]; // number of means to calculate for each channel if continuous is active; if 0 channel is deactivated

    bool use_drdy_interrupt;           // whether to use drdy interrupt or just poll for conversion completion in continuous read mode; if true, int_pin must be set to a valid gpio pin
    bool continuous_conversion;        // whether adc should read continuously or only when activated
    int16_t read_task_period_ms;       // period of continuous read task in ms if continuous adc read is deisabled
    bool continuous_read_task_enabled; // whether continuous read task is enabled
} ads1115_cfg;

void adc_ads1115_init(adc_ads1115 *adc, ads1115_cfg *config);
esp_err_t adc_ads1115_begin(adc_ads1115 *adc, i2c_master_bus_handle_t bus_hdl);
esp_err_t adc_ads1115_read_raw(adc_ads1115 *adc, int8_t channel_idx, int16_t *result);

double adc_ads1115_scale_volt(adc_ads1115 *adc, uint8_t channel_idx, int16_t val);
double adc_ads1115_scale_milliamps(adc_ads1115 *adc, uint8_t channel_idx, int16_t val);
double adc_ads1115_read_volt(adc_ads1115 *adc, uint8_t channel_idx);
double adc_ads1115_read_milliamps(adc_ads1115 *adc, uint8_t channel_idx);

void adc_ads1115_set_mux(adc_ads1115 *adc, ads1115_mux_cfg new_cfg);
ads1115_mux_cfg adc_ads1115_get_mux(adc_ads1115 *adc);
void adc_ads1115_set_pga(adc_ads1115 *adc, ads1115_pga_cfg new_cfg);
ads1115_pga_cfg adc_ads1115_get_pga(adc_ads1115 *adc);
void adc_ads1115_set_conv_mode(adc_ads1115 *adc, bool continuous);
void adc_ads1115_set_rate(adc_ads1115 *adc, ads1115_rate_cfg new_cfg);
ads1115_rate_cfg adc_ads1115_get_rate(adc_ads1115 *adc);

void adc_ads1115_disable_alert_drdy(adc_ads1115 *adc);

void adc_ads1115_set_comp_mode(adc_ads1115 *adc, bool window_comp);
void adc_ads1115_set_comp_pol(adc_ads1115 *adc, bool polarity);
void adc_ads1115_set_comp_latch(adc_ads1115 *adc, bool latching);
void adc_ads1115_set_comp_queue(adc_ads1115 *adc, ads1115_comp_queue_cfg new_cfg);

esp_err_t adc_ads1115_set_thresh_lo(adc_ads1115 *adc, int16_t thresh);
esp_err_t adc_ads1115_set_thresh_hi(adc_ads1115 *adc, int16_t thresh);
esp_err_t adc_ads1115_enable_drdy(adc_ads1115 *adc);
#endif