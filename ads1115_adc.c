#include "ads1115_adc_priv.h"

static esp_err_t read_reg(adc_ads1115 *adc, uint8_t addr, uint16_t *returnval)
{
    uint8_t buf[2];
    *returnval = 0;
    if (i2c_master_transmit_receive(adc->dev_hdl, &addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS) != ESP_OK)
        return ESP_ERR_TIMEOUT;
    *returnval = (buf[0] << 8) + buf[1];
    //ESP_LOGI(TAG, "r register val: %04x from addr %02x", *returnval, addr);
    return ESP_OK;
}
static esp_err_t read_reg_signed(adc_ads1115 *adc, uint8_t addr, int16_t *returnval)
{
    uint8_t buf[2];
    *returnval = 0;
    if (i2c_master_transmit_receive(adc->dev_hdl, &addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS) != ESP_OK)
        return ESP_ERR_TIMEOUT;
    *returnval = (buf[0] << 8) + buf[1];
    return ESP_OK;
}

static esp_err_t write_reg(adc_ads1115 *adc, uint8_t addr, uint16_t val)
{
    uint8_t buf[3];
    buf[0] = addr;
    buf[1] = val >> 8;
    buf[2] = val & 0xFF;
    //ESP_LOGI(TAG, "w register val: %02x%02x for addr %02x", buf[1], buf[2], addr);
    return (i2c_master_transmit(adc->dev_hdl, buf, 3, -1));
}

static esp_err_t set_reg_bit(adc_ads1115 *adc, uint8_t addr, uint16_t bitmask, bool en)
{
    uint16_t reg_val;
    esp_err_t rtrn;
    rtrn = read_reg(adc, addr, &reg_val);
    if (rtrn != ESP_OK)
        return rtrn;

    if (en)
        reg_val |= bitmask;
    else
        reg_val &= ~bitmask;
    write_reg(adc, addr, reg_val);
    if (rtrn != ESP_OK)
        return rtrn;
    return ESP_OK;
}

static void isr_handler(void *_task_handle)
{
    TaskHandle_t *task_handle = (TaskHandle_t *)_task_handle;
    BaseType_t buf;
    if (*task_handle != NULL)
        vTaskNotifyGiveFromISR(*task_handle, &buf);
}

/************************** Public Functions **************************/

void adc_ads1115_init(adc_ads1115 *adc, ads1115_cfg *config)
{
    adc->i2c_addr = config->i2c_addr;
    adc->int_pin = config->int_pin;
    adc->shunt_resistor = config->shunt_resistor;

    adc->autogain = false,
    adc->continuous_conv = false,
    adc->gain = FS_2048mV; // default
    ESP_LOGI(TAG, "adc init");
}

/// @brief begin rtc operation (setup)
/// @param rtc rtc handle
/// @param bus_hdl i2c bus handle
void adc_ads1115_begin(adc_ads1115 *adc, i2c_master_bus_handle_t bus_hdl)
{
    i2c_device_config_t adc_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = adc->i2c_addr,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_hdl, &adc_dev_cfg, &adc->dev_hdl));

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    adc->task_to_notify = xTaskGetCurrentTaskHandle();

    gpio_isr_handler_add(adc->int_pin, isr_handler, &adc->task_to_notify);

    gpio_config_t int_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = true,
        .pull_down_en = false,
        .pin_bit_mask = 1 << adc->int_pin,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&int_cfg);

    ESP_LOGI(TAG, "adc begin");
}

void adc_ads1115_start_single_conv(adc_ads1115 *adc)
{
    set_reg_bit(adc, reg_config, OP_STATUS_MASK, true);
}

bool adc_ads1115_read_busy(adc_ads1115 *adc)
{
    uint16_t buf = 0;
    read_reg(adc, reg_config, &buf);
    return ((buf & OP_STATUS_MASK) > 0);
}

void adc_ads1115_set_mux(adc_ads1115 *adc, ads1115_mux_cfg new_cfg)
{
    set_reg_bit(adc, reg_config, MUX_MASK, false);
    uint16_t reg_val = 0;
    read_reg(adc, reg_config, &reg_val);
    write_reg(adc, reg_config, reg_val | (new_cfg << MUX_SHIFT));
}

ads1115_mux_cfg adc_ads1115_get_mux(adc_ads1115 *adc)
{
    uint16_t returnval;
    read_reg(adc, reg_config, &returnval);
    return (returnval & MUX_MASK) >> MUX_SHIFT;
}

void adc_ads1115_set_pga(adc_ads1115 *adc, ads1115_pga_cfg new_cfg)
{
    adc->gain = new_cfg;
    set_reg_bit(adc, reg_config, PGA_MASK, false);
    uint16_t reg_val = 0;
    read_reg(adc, reg_config, &reg_val);
    write_reg(adc, reg_config, reg_val | (new_cfg << PGA_SHIFT));
}

ads1115_pga_cfg adc_ads1115_get_pga(adc_ads1115 *adc)
{
    uint16_t buf;
    read_reg(adc, reg_config, &buf);
    return ((buf & PGA_MASK) >> PGA_SHIFT);
}

void adc_ads1115_set_conv_mode(adc_ads1115 *adc, bool continuous)
{
    set_reg_bit(adc, reg_config, CONV_MODE_MASK, !continuous);
    adc->continuous_conv = continuous;
}

void adc_ads1115_set_rate(adc_ads1115 *adc, ads1115_rate_cfg new_cfg)
{
    ESP_LOGI(TAG, "rate");
    set_reg_bit(adc, reg_config, RATE_MASK, false);
    uint16_t reg_val = 0;
    read_reg(adc, reg_config, &reg_val);
    write_reg(adc, reg_config, reg_val | (new_cfg << RATE_SHIFT));
}

ads1115_rate_cfg adc_ads1115_get_rate(adc_ads1115 *adc)
{
    uint16_t buf;
    read_reg(adc, reg_config, &buf);
    return ((buf & RATE_MASK) >> RATE_SHIFT);
}

void adc_ads1115_set_comp_mode(adc_ads1115 *adc, bool window_comp)
{
    set_reg_bit(adc, reg_config, COMP_MODE_MASK, window_comp);
}

void adc_ads1115_set_comp_pol(adc_ads1115 *adc, bool polarity)
{
    set_reg_bit(adc, reg_config, COMP_POL_MASK, polarity);
}

void adc_ads1115_set_comp_latch(adc_ads1115 *adc, bool latching)
{
    set_reg_bit(adc, reg_config, COMP_LATCH_MASK, latching);
}

void adc_ads1115_set_comp_queue(adc_ads1115 *adc, ads1115_comp_queue_cfg new_cfg)
{

    set_reg_bit(adc, reg_config, COMP_QUEUE_MASK, false);
    uint16_t reg_val = 0;
    read_reg(adc, reg_config, &reg_val);
    write_reg(adc, reg_config, reg_val | (new_cfg << RATE_SHIFT));
    ESP_LOGI(TAG, "queue done");
}

void adc_ads1115_disable_alert_drdy(adc_ads1115 *adc)
{
    set_reg_bit(adc, reg_config, COMP_QUEUE_MASK, true);
}

esp_err_t adc_ads1115_read_raw(adc_ads1115 *adc, int16_t *result)
{
    esp_err_t err;
    if (!adc->continuous_conv)
    {
        err = set_reg_bit(adc, reg_config, OP_STATUS_MASK, true);
        if (err != ESP_OK)
            return err;

        // wait for notification from ISR
        if (ulTaskNotifyTake(pdTRUE, 100 / portTICK_PERIOD_MS) == 0)
        {
            ESP_LOGE(TAG, "ADC read timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    err = read_reg_signed(adc, reg_conv_res, result);
    if (err != ESP_OK)
        return err;

    return ESP_OK;
}

// TODO: single shot and continuous mode; check if new val available ; maybe make autogain only available if continuous mode without interrupt is enabled?
uint16_t adc_ads1115_read(adc_ads1115 *adc)
{
    int16_t val = 0;
    adc_ads1115_read_raw(adc, &val);
    //ESP_LOGI(TAG, "raw val %d", val);
    if (adc->autogain)
    {
        // increase until max gain or adc saturation)
        while ((val < (INT16_MAX - 100)) & (val > (INT16_MIN + 100)))
        {
            if (adc->gain < FS_256mV)
                adc_ads1115_set_pga(adc, adc->gain + 1);
            else
                break;
            read_reg_signed(adc, reg_conv_res, &val);
        }

        // decrease gain until saturation is gone
        while ((val >= (INT16_MAX - 100)) & (val <= (INT16_MIN + 100)))
        {
            if (adc->gain > FS_6144mV)
                adc_ads1115_set_pga(adc, adc->gain - 1);
            else
                break;
            read_reg_signed(adc, reg_conv_res, &val);
        }
    }
    return val;
}

double adc_ads1115scaleVolt(adc_ads1115 *adc, int16_t val)
{
    return ((double)val / INT16_MAX) * fs_ranges[adc->gain];
}

double adc_ads1115scaleAmps(adc_ads1115 *adc, int16_t val)
{
    return (adc_ads1115scaleVolt(adc, val) / adc->shunt_resistor);
}

double adc_ads1115_readVolt(adc_ads1115 *adc)
{
    return (adc_ads1115scaleVolt(adc, adc_ads1115_read(adc)));
}

double adc_ads1115_readAmps(adc_ads1115 *adc)
{
    return (adc_ads1115scaleAmps(adc, adc_ads1115_read(adc)));
}

void adc_ads1115_register_task(adc_ads1115 *adc, TaskHandle_t task_to_notify, bool enable)
{
}

void adc_ads1115_autogain(adc_ads1115 *adc, bool enable)
{
}

esp_err_t adc_ads1115_set_thresh_lo(adc_ads1115 *adc, int16_t thresh)
{
    return ESP_OK;
}

esp_err_t adc_ads1115_set_thresh_hi(adc_ads1115 *adc, int16_t thresh)
{
    return ESP_OK;
}

esp_err_t adc_ads1115_enable_drdy(adc_ads1115 *adc)
{
    ESP_LOGI(TAG, "drdy");
    // set hi thres MSB to 1 & lo thres MSB to 0 to enable DRDY/Comparator pin as DRDY
    esp_err_t err = set_reg_bit(adc, reg_thresh_hi, DRDY_EN_MASK, true);
    if (err != ESP_OK)
        return err;
    err = set_reg_bit(adc, reg_thresh_lo, DRDY_EN_MASK, false);
    if (err != ESP_OK)
        return err;
    adc_ads1115_set_comp_queue(adc, assert_1_conv); // not entirely sure if this is necessary
    return ESP_OK;
}

// TODO: function to set new task to notify and re-register ISR
