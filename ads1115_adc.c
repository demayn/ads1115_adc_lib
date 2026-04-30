#include "ads1115_adc_priv.h"

const double ads1115_fs_ranges[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
const double ads1115_sample_rates[] = {8, 16, 32, 64, 128, 250, 475, 860};

static esp_err_t read_reg(adc_ads1115 *adc, uint8_t addr, uint16_t *returnval)
{
    uint8_t buf[2];
    *returnval = 0;
    if (i2c_master_transmit_receive(adc->dev_hdl, &addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS) != ESP_OK)
        return ESP_ERR_TIMEOUT;
    *returnval = (buf[0] << 8) + buf[1];
    // ESP_LOGI(TAG, "r register val: %04x from addr %02x", *returnval, addr);
    return ESP_OK;
}

static esp_err_t read_reg_signed(adc_ads1115 *adc, uint8_t addr, int16_t *returnval)
{
    uint8_t buf[2];
    *returnval = 0;
    esp_err_t err = i2c_master_transmit_receive(adc->dev_hdl, &addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS);
    if (err != ESP_OK)
    {
        return err;
    }
    *returnval = (buf[0] << 8) + buf[1];
    return ESP_OK;
}

static esp_err_t write_reg(adc_ads1115 *adc, uint8_t addr, uint16_t val)
{
    uint8_t buf[3];
    buf[0] = addr;
    buf[1] = val >> 8;
    buf[2] = val & 0xFF;
    // ESP_LOGI(TAG, "w register val: %02x%02x for addr %02x", buf[1], buf[2], addr);
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

/// @brief returns the next channel (in a cyclic manner)
/// @param current_channel the current channel
/// @return the next channel after that (cyclic)
uint8_t adc_ads1115_next_channel_idx(uint8_t current_channel)
{
    uint8_t next_channel = 0;
    if (current_channel < (ADS1115_MAX_NUM_CH - 1))
        next_channel = current_channel + 1;
    return next_channel;
}

void adc_ads1115_drdy_isr_handler(void *_cont_read_handle)
{
    TaskHandle_t *cont_read_handle = (TaskHandle_t *)_cont_read_handle;
    if (*cont_read_handle != NULL)
    {
        BaseType_t idk;
        vTaskNotifyGiveFromISR(*cont_read_handle, &idk);
    }
    return;
}

void adc_ads1115_cont_read_task(void *ads1115_v)
{
    adc_ads1115 *adc = (adc_ads1115 *)ads1115_v;
    if (adc == NULL)
    {
        ESP_LOGE(TAG, "ADC task received NULL handle");
        vTaskDelete(NULL);
        return;
    }

    TickType_t delay = portMAX_DELAY;
    if (adc->read_task_period_ms > 0)
        delay = pdMS_TO_TICKS(adc->read_task_period_ms);

    while (1)
    {
        // set next channel if the current channel is inactive
        uint8_t next_channel_idx = adc->act_channel_idx;
        // try to find active channel
        for (int i = 0; i < ADS1115_MAX_NUM_CH && adc->mean_num[next_channel_idx] <= 0; i++)
            next_channel_idx = adc_ads1115_next_channel_idx(next_channel_idx);
        if (adc->act_channel_idx != next_channel_idx)
        {
            adc_ads1115_set_mux(adc, adc->channel_muxs[next_channel_idx]);
            adc_ads1115_set_pga(adc, adc->channel_gains[next_channel_idx]);
        }
        adc->act_channel_idx = next_channel_idx;

        if (adc->mean_num[next_channel_idx] != 0)
        {
            // get the adc val
            if (!adc->continuous_conversion) // if adc is NOT converting all the time, request a value
                if (adc_ads1115_start_single_conv(adc) != ESP_OK)
                {
                    ESP_LOGE(TAG, "failed to start single conversion");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }

            uint32_t notification = ulTaskNotifyTake(pdTRUE, delay); // wait for notification from isr or timeout for the read task period

            int16_t conv_result;
            if (notification || !adc->use_drdy_interrupt) // if we were notified or we are not using interrupts, read the adc value
            {
                esp_err_t err = ads1115_read_conversion(adc, adc->act_channel_idx, &conv_result);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "failed to read conversion %s", esp_err_to_name(err));
                    vTaskDelay(100/portTICK_PERIOD_MS);
                    continue;
                }
            }
            else
            {
                ESP_LOGW(TAG, "no notification received");
                vTaskDelay(20 / portTICK_PERIOD_MS);
                continue;
            }
            if (adc->mean_arr_mutex[adc->act_channel_idx] == NULL || adc->mean_arr[adc->act_channel_idx] == NULL)
            {
                ESP_LOGE(TAG, "Mean storage not initialized for channel %d", adc->act_channel_idx);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            if (xSemaphoreTake(adc->mean_arr_mutex[adc->act_channel_idx], ADS1115_SMPHR_TOUT_MS / portTICK_PERIOD_MS) != pdTRUE)
            {
                ESP_LOGW(TAG, "Semaphore timeout for channel %d", adc->act_channel_idx);
                vTaskDelay(ADS1115_SMPHR_TOUT_MS / portTICK_PERIOD_MS);
                continue;
            }

            adc->mean_arr[adc->act_channel_idx][adc->mean_idx[adc->act_channel_idx]] = conv_result;
            // ESP_LOGD(TAG, "CH %d, idx %d, val %d", adc->act_channel_idx, adc->mean_idx[adc->act_channel_idx], adc->mean_arr[adc->act_channel_idx][adc->mean_idx[adc->act_channel_idx]]);
            adc->mean_idx[adc->act_channel_idx]++;
            uint8_t old_channel_idx = adc->act_channel_idx;
            // ESP_LOGD(TAG, "CH %d, idx %d", adc->act_channel_idx, adc->mean_idx[adc->act_channel_idx]);

            // mean array for this channel is full, move on to next active channel
            if (adc->mean_idx[adc->act_channel_idx] >= adc->mean_num[adc->act_channel_idx])
            {
                adc->mean_idx[adc->act_channel_idx] = 0;
                adc->mean_valid[adc->act_channel_idx] = true;
                xSemaphoreGive(adc->mean_arr_mutex[old_channel_idx]);
                // move on to next channel
                uint8_t next_channel_idx = adc_ads1115_next_channel_idx(adc->act_channel_idx);

                // try to find active channel
                for (int i = 0; i < 3 && adc->mean_num[next_channel_idx] <= 0; i++)
                    next_channel_idx = adc_ads1115_next_channel_idx(next_channel_idx);
                if (adc->act_channel_idx != next_channel_idx) // switch (if there is more than one channel active)
                {
                    adc_ads1115_set_mux(adc, adc->channel_muxs[next_channel_idx]);
                    adc->act_channel_idx = next_channel_idx;
                    adc_ads1115_set_pga(adc, adc->channel_gains[next_channel_idx]);
                    vTaskDelay(100 / portTICK_PERIOD_MS); // to let ad settle;

                    ulTaskNotifyTake(pdTRUE, delay); // discard the first value after that
                }
            }
            else
                xSemaphoreGive(adc->mean_arr_mutex[old_channel_idx]);
        }
        else
        {
            vTaskDelay(ADS1115_SMPHR_TOUT_MS / portTICK_PERIOD_MS); // no active channels configured
        }
    }
}

esp_err_t adc_ads1115_start_single_conv(adc_ads1115 *adc)
{
    return set_reg_bit(adc, reg_config, OP_STATUS_MASK, true);
}

bool adc_ads1115_read_busy(adc_ads1115 *adc)
{
    uint16_t buf = 0;
    if (read_reg(adc, reg_config, &buf) != ESP_OK)
        return true; // if read fails, just assume it's busy to avoid reading wrong value
    return ((buf & OP_STATUS_MASK) > 0);
}

esp_err_t ads1115_read_conversion(adc_ads1115 *adc, uint8_t channel_idx, int16_t *result)
{
    uint8_t addr = reg_conv_res;
    return read_reg_signed(adc, addr, result);
}

/************************** Public Functions **************************/

void adc_ads1115_init(adc_ads1115 *adc, ads1115_cfg *config)
{
    // Copy hardware configuration
    adc->i2c_addr = config->i2c_addr;
    adc->int_pin = config->int_pin;
    adc->shunt_resistor = config->shunt_resistor;

    // Copy settings for all channels
    adc->rate_cfg = config->rate_cfg;

    // Copy channel-specific settings
    for (uint8_t i = 0; i < ADS1115_MAX_NUM_CH; i++)
    {
        adc->channel_gains[i] = config->channel_gains[i];
        adc->channel_muxs[i] = config->channel_muxs[i];
        adc->channel_mean_nums[i] = config->channel_mean_nums[i];
    }

    // Copy operation mode settings
    adc->use_drdy_interrupt = config->use_drdy_interrupt;
    adc->continuous_conversion = config->continuous_conversion;
    adc->read_task_period_ms = config->read_task_period_ms;
    adc->continuous_read_task_enabled = config->continuous_read_task_enabled;

    // Initialize device handle and task handle
    adc->dev_hdl = NULL;
    adc->continuous_read_taskhdl = NULL;

    // Initialize runtime variables
    adc->act_channel_idx = 0;
    for (uint8_t i = 0; i < ADS1115_MAX_NUM_CH; i++)
    {
        adc->mean_idx[i] = 0;
        adc->mean_valid[i] = false;
        adc->mean_num[i] = config->channel_mean_nums[i];
        adc->mean_arr[i] = NULL;
        adc->mean_arr_mutex[i] = NULL;
    }

    ESP_LOGI(TAG, "adc init");
}

/// @brief begin adc operation; this should be called from the task that also tries to read the adc, if the internal task is not used (ISR will notify the task calling begin)
/// @param adc adc handle
/// @param bus_hdl i2c bus handle
esp_err_t adc_ads1115_begin(adc_ads1115 *adc, i2c_master_bus_handle_t bus_hdl)
{
    i2c_device_config_t adc_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = adc->i2c_addr,
        .scl_speed_hz = 400000,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_hdl, &adc_dev_cfg, &adc->dev_hdl), TAG, "failed to add adc device to i2c bus");

    adc_ads1115_set_rate(adc, adc->rate_cfg);

    // setup values to channel zero(start)
    uint8_t act_channel = adc->act_channel_idx;
    adc_ads1115_set_mux(adc, adc->channel_muxs[act_channel]);
    adc_ads1115_set_pga(adc, adc->channel_gains[act_channel]);

    adc_ads1115_set_conv_mode(adc, adc->continuous_conversion);

    if (adc->continuous_read_task_enabled) // if there is a dedicated task for continuous reading
    {
        for (uint8_t i = 0; i < ADS1115_MAX_NUM_CH; i++)
        {
            if (adc->mean_num[i] > 0)
            {
                adc->mean_arr[i] = malloc(sizeof(int16_t) * adc->mean_num[i]);
                adc->mean_arr_mutex[i] = xSemaphoreCreateMutex();
                if (adc->mean_arr[i] == NULL || adc->mean_arr_mutex[i] == NULL)
                {
                    ESP_LOGE(TAG, "Failed to allocate mean storage for channel %u", (unsigned)i);
                    return ESP_ERR_NO_MEM;
                }
            }
        }
        adc->continuous_read_taskhdl = NULL;
        if (xTaskCreate(adc_ads1115_cont_read_task, "ads1115_cont", 4608, (void *)adc, tskIDLE_PRIORITY + 1, &adc->continuous_read_taskhdl) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create ads1115_cont_read_task");
            return ESP_ERR_NO_MEM;
        }
    }

    if (adc->use_drdy_interrupt)
    {
        TaskHandle_t *isr_task_to_notify = malloc(sizeof(TaskHandle_t));
        if (isr_task_to_notify == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate isr_task_to_notify");
            return ESP_ERR_NO_MEM;
        }
        *isr_task_to_notify = adc->continuous_read_taskhdl;

        if (!adc->continuous_read_task_enabled) // if there is no other task, just notify the current one
            *isr_task_to_notify = xTaskGetCurrentTaskHandle();

        // Install GPIO ISR service only if not already installed
        esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGE(TAG, "gpio isr install failed");
            return ret;
        }

        ESP_RETURN_ON_ERROR(gpio_isr_handler_add(adc->int_pin, adc_ads1115_drdy_isr_handler, (void *)isr_task_to_notify), TAG, "gpio isr handler add failed");
        gpio_config_t int_cfg = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = true,
            .pull_down_en = false,
            .pin_bit_mask = 1ULL << adc->int_pin,
            .intr_type = GPIO_INTR_POSEDGE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&int_cfg), TAG, "gpio config failed");
        adc_ads1115_enable_drdy(adc);
    }
    else
        ESP_LOGW(TAG, "DRDY interrupt not enabled, continuous read task will have to poll for conversion completion, which might cause higher latency and lower sample rate");

    ESP_LOGI(TAG, "adc begin");
    return ESP_OK;
}

/// @brief
/// @param adc
/// @param channel_idx specified channel idx to set pga and mux for OR -1 to use current config
/// @param result pointer to store the raw ADC result
/// @return
esp_err_t adc_ads1115_read_raw(adc_ads1115 *adc, int8_t channel_idx, int16_t *result)
{
    int32_t returnval = 0;
    if (adc->continuous_read_task_enabled) // if continuous readout, simply read mean
    {
        if (channel_idx < 0 || channel_idx > ADS1115_MAX_NUM_CH - 1)
        {
            ESP_LOGE(TAG, "channel %d invalid for read task mode", channel_idx);
            return ESP_ERR_INVALID_ARG;
        }
        if (adc->mean_num[channel_idx] == 0)
        {
            ESP_LOGE(TAG, "Mean value for channel %d inactive", channel_idx);
            return ESP_ERR_INVALID_ARG;
        }
        if (xSemaphoreTake(adc->mean_arr_mutex[channel_idx], ADS1115_SMPHR_TOUT_MS / portTICK_PERIOD_MS) != pdTRUE)
        {
            ESP_LOGE(TAG, "Semaphore take for channel %d timeout", channel_idx);
            return ESP_ERR_TIMEOUT;
        }
        if (!adc->mean_valid[channel_idx])
        {
            xSemaphoreGive(adc->mean_arr_mutex[channel_idx]);
            ESP_LOGE(TAG, "Mean value for channel %d not valid yet", channel_idx);
            return ESP_ERR_INVALID_STATE; // mean value not valid yet
        }
        for (int i = 0; i < adc->mean_num[channel_idx]; i++)
            returnval += adc->mean_arr[channel_idx][i];

        returnval /= adc->mean_num[channel_idx];
        xSemaphoreGive(adc->mean_arr_mutex[channel_idx]);
        *result = (int16_t)returnval;
    }
    else // manual readout, just read the adc value directly without using mean
    {
        // set mux and pga for the channel if specified
        if (channel_idx < ADS1115_MAX_NUM_CH && channel_idx >= 0)
        {
            if (adc->act_channel_idx != channel_idx)
            {
                adc->act_channel_idx = channel_idx;
                adc_ads1115_set_mux(adc, adc->channel_muxs[channel_idx]);
                adc_ads1115_set_pga(adc, adc->channel_gains[channel_idx]);
            }
            vTaskDelay(1); // small delay to allow mux and pga to settle
        }

        TickType_t drdy_tout = pdMS_TO_TICKS(1000 / ads1115_sample_rates[adc->rate_cfg] + 100); // calculate timeout based on sample rate plus 100ms margin

        if (!adc->continuous_conversion) // if adc is NOT converting all the time, request a value
            ESP_RETURN_ON_ERROR(adc_ads1115_start_single_conv(adc), TAG, "failed to start single conversion");

        if (!ulTaskNotifyTake(pdTRUE, drdy_tout)) // wait for drdy notification
        {
            if (adc->int_pin > GPIO_NUM_0) // if drdy interrupt is enabled, log warning if notification is not received within timeout, but try to read adc value anyway since it might still be available

                ESP_LOGW(TAG, "No DRDY notification received within timeout, trying to read adc value anyway");
        }

        return ads1115_read_conversion(adc, channel_idx, result);
    }

    return ESP_OK;
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
    adc->continuous_conversion = continuous;
}

void adc_ads1115_set_rate(adc_ads1115 *adc, ads1115_rate_cfg new_cfg)
{
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
    write_reg(adc, reg_config, reg_val | (new_cfg & COMP_QUEUE_MASK));
}

void adc_ads1115_disable_alert_drdy(adc_ads1115 *adc)
{
    set_reg_bit(adc, reg_config, COMP_QUEUE_MASK, true);
}

double adc_ads1115_scale_volt(adc_ads1115 *adc, uint8_t channel_idx, int16_t val)
{
    return ((double)val / INT16_MAX) * ads1115_fs_ranges[adc->channel_gains[channel_idx]];
}

double adc_ads1115_scale_milliamps(adc_ads1115 *adc, uint8_t channel_idx, int16_t val)
{
    return ((adc_ads1115_scale_volt(adc, channel_idx, val) / adc->shunt_resistor) * 1000);
}

double adc_ads1115_read_volt(adc_ads1115 *adc, uint8_t channel_idx)
{
    int16_t raw_val;
    if (adc_ads1115_read_raw(adc, channel_idx, &raw_val) != ESP_OK)
        return NAN;
    return (adc_ads1115_scale_volt(adc, channel_idx, raw_val));
}

double adc_ads1115_read_milliamps(adc_ads1115 *adc, uint8_t channel_idx)
{
    int16_t raw_val;
    if (adc_ads1115_read_raw(adc, channel_idx, &raw_val) != ESP_OK)
        return NAN;
    return (adc_ads1115_scale_milliamps(adc, channel_idx, raw_val));
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