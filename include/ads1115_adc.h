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
    uint8_t i2c_addr;   // 0x48 to 0x4B
    gpio_num_t int_pin; // interrupt input pin
    ads1115_pga_cfg gain;
    bool continuous_conv;
    bool autogain;
    TaskHandle_t task_to_notify;
    i2c_master_dev_handle_t dev_hdl;
    double shunt_resistor; // Ω
} adc_ads1115;

typedef struct
{
    uint8_t i2c_addr;
    gpio_num_t int_pin;
    double shunt_resistor; // Ω
} ads1115_cfg;

void adc_ads1115_init(adc_ads1115 *adc, ads1115_cfg *config);
void adc_ads1115_begin(adc_ads1115 *adc, i2c_master_bus_handle_t bus_hdl);

void adc_ads1115_start_single_conv(adc_ads1115 *adc);
bool adc_ads1115_read_busy(adc_ads1115 *adc);
void adc_ads1115_set_mux(adc_ads1115 *adc, ads1115_mux_cfg new_cfg);
ads1115_mux_cfg adc_ads1115_get_mux(adc_ads1115 *adc);
void adc_ads1115_set_pga(adc_ads1115 *adc, ads1115_pga_cfg new_cfg);
ads1115_pga_cfg adc_ads1115_get_pga(adc_ads1115 *adc);
void adc_ads1115_set_conv_mode(adc_ads1115 *adc, bool continuous);
void adc_ads1115_set_rate(adc_ads1115 *adc, ads1115_rate_cfg new_cfg);
ads1115_rate_cfg adc_ads1115_get_rate(adc_ads1115 *adc);
void adc_ads1115_set_comp_mode(adc_ads1115 *adc, bool window_comp);
void adc_ads1115_set_comp_pol(adc_ads1115 *adc, bool polarity);
void adc_ads1115_set_comp_latch(adc_ads1115 *adc, bool latching);
void adc_ads1115_set_comp_queue(adc_ads1115 *adc, ads1115_comp_queue_cfg new_cfg);
void adc_ads1115_disable_alert_drdy(adc_ads1115 *adc);
uint16_t adc_ads1115_read(adc_ads1115 *adc);
double adc_ads1115scaleVolt(adc_ads1115 *adc, int16_t val);
double adc_ads1115scaleAmps(adc_ads1115 *adc, int16_t val);
double adc_ads1115_readVolt(adc_ads1115 *adc);
double adc_ads1115_readAmps(adc_ads1115 *adc);
void adc_ads1115_register_task(adc_ads1115 *adc, TaskHandle_t task_to_notify, bool enable);
void adc_ads1115_autogain(adc_ads1115 *adc, bool enable);
esp_err_t adc_ads1115_set_thresh_lo(adc_ads1115 *adc, int16_t thresh);
esp_err_t adc_ads1115_set_thresh_hi(adc_ads1115 *adc, int16_t thresh);
esp_err_t adc_ads1115_enable_drdy(adc_ads1115 *adc);
double adc_ads1115scaleAmps(adc_ads1115 *adc, int16_t val);
#endif