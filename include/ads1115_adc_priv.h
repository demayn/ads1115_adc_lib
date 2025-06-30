#ifndef ADS1115_PRIV_H
#define ADS1115_PRIV_H

#include "ads1115_adc.h"

#define TAG "I2C_ADC_ADS1115"

#define PCF8563_NUM_REGS 4

#define I2C_MASTER_TIMEOUT_MS 10

// rtc registers
enum rtc_registers
{
    reg_conv_res,
    reg_config,
    reg_thresh_lo,
    reg_thresh_hi,
};

const double fs_ranges[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};

#define OP_STATUS_MASK 0x8000 // write one to start single conmversion(in power down), read 0: conv ongoing, 1: idle
#define MUX_MASK 0x7000 // input multiplexer config mask
#define MUX_SHIFT 12 // bitshift by this val generates the correct byte
#define PGA_MASK 0x0E00 // programmable gain amplifier mask
#define PGA_SHIFT 9 // bitshift by this val generates the correct byte
#define CONV_MODE_MASK 0x0100 // 0 for cont. conversion mode
#define RATE_MASK 0x00E0 // conversion rate mask
#define RATE_SHIFT 5 // bitshift by this val generates the correct byte
#define COMP_MODE_MASK 0x0010 // 0 for traditional comparator, 1 for window comparator (1 if signal in between thresholds)
#define COMP_POL_MASK 0x0008 // 0 for active LOW, 1 for active HIGH
#define COMP_LATCH_MASK 0x0004 // 0 non-latching; 1 lathcing (until data is read or SMBUS request)
#define COMP_QUEUE_MASK 0x0003 // determines after how many conversions exceeding the thresholds alert will be asserted; can also disable the output
#define DRDY_EN_MASK 0x8000 // set low thres to 0 and high thres to 1 to enable DRDY output

// private functions
static esp_err_t read_reg(adc_ads1115 *adc, uint8_t addr, uint16_t *returnval);
static esp_err_t write_reg(adc_ads1115 *adc, uint8_t addr, uint16_t val);
static esp_err_t set_reg_bit(adc_ads1115 *adc, uint8_t addr, uint16_t bitmask, bool en);
static esp_err_t adc_ads1115_read_raw(adc_ads1115 *adc, int16_t *result);
static void isr_handler(void *_task_handle);
#endif