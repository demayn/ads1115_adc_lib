#ifndef ADS1115_PRIV_H
#define ADS1115_PRIV_H

#include "ads1115_adc.h"

#define TAG "I2C_ADC_ADS1115"

#define I2C_MASTER_TIMEOUT_MS 10
#define ADS1115_SMPHR_TOUT_MS 100
#define ADS1115_SCL_SPEED_HZ 400000

#define ADS1115_NUM_REGS 4

// ads1115 adc registers
enum adc_registers
{
    reg_conv_res,
    reg_config,
    reg_thresh_lo,
    reg_thresh_hi,
};

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

uint8_t adc_ads1115_next_channel_idx(uint8_t current_channel);

void adc_ads1115_drdy_isr_handler(void *_cont_read_handle);
void adc_ads1115_cont_read_task(void *ads1115_v);

esp_err_t adc_ads1115_start_single_conv(adc_ads1115 *adc);
bool adc_ads1115_read_busy(adc_ads1115 *adc);

esp_err_t ads1115_read_conversion(adc_ads1115 *adc, uint8_t channel_idx, int16_t *result);
#endif