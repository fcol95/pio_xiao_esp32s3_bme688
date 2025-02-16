#include "ambient_sense.h"

#include "driver/i2c.h" //< For BME688 I2C communication port
#include "esp_log.h"
#include "esp_rom_sys.h" //< For BME688 delay_us port

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme68x.h"

#define AMBIENT_SENSE_MEAS_LOOP_PERIOD_MS 250

#define BME688_I2C_ADDR 0x76
#define BME688_I2C_SPEED_HZ 400000

static const char *LOG_TAG = "ambient_sense";

static const i2c_device_config_t s_bme688_i2c_dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_7,
    .device_address = BME688_I2C_ADDR,
    .scl_speed_hz = BME688_I2C_SPEED_HZ,
    .scl_wait_us = 0,                 // 0 == Use the default reg value
    .flags.disable_ack_check = false, // False == Enable ACK check
};

static i2c_master_dev_handle_t s_bme688_i2c_dev_handle = NULL;

static void bme68x_delay_us(uint32_t period, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr,
                                             const uint8_t *reg_data,
                                             uint32_t length,
                                             void *intf_ptr);

esp_err_t ambient_sense_init(i2c_master_bus_handle_t i2c_bus_handle)
{
    if (i2c_bus_handle == NULL)
        return ESP_FAIL;

    esp_err_t i2c_ret = i2c_master_bus_add_device(i2c_bus_handle, &s_bme688_i2c_dev_config, &s_bme688_i2c_dev_handle);
    if (i2c_ret != ESP_OK || s_bme688_i2c_dev_handle == NULL)
    {
        ESP_LOGE(LOG_TAG, "I2C Master Adding BME688 Device Failed!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ambient_sense_task(void *pvParameter)
{
    struct bme68x_dev bme688_handle = {
        .intf = BME68X_I2C_INTF,
        // Port Functions and Pointer
        .intf_ptr = &s_bme688_i2c_dev_handle,
        .delay_us = bme68x_delay_us,
        .read = bme68x_i2c_read,
        .write = bme68x_i2c_write,
        .amb_temp = 25, // Ambient temperature in degrees Celsius
    };

    int8_t ret = bme68x_init(&bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x initialization failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x initialization succeeded");
    }

    struct bme68x_data data;
    uint8_t n_fields;

    // Set sensor configuration
    struct bme68x_conf conf = {
        .os_hum = BME68X_OS_16X,
        .os_pres = BME68X_OS_1X,
        .os_temp = BME68X_OS_2X,
        .filter = BME68X_FILTER_OFF,
        .odr = BME68X_ODR_NONE,
    };
    ret = bme68x_set_conf(&conf, &bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x configuration failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x configuration succeeded");
    }

    // Set heater configuration
    struct bme68x_heatr_conf heatr_conf = {
        .enable = BME68X_DISABLE,
        .heatr_temp = 320, // Target temperature in degree Celsius
        .heatr_dur = 150,  // Duration in milliseconds
    };
    ret = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x heater configuration failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x heater configuration succeeded");
    }

    while (1)
    {
        // Set sensor to forced mode
        ret = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme688_handle);
        if (ret != BME68X_OK)
        {
            ESP_LOGE(LOG_TAG, "BME68x setting operation mode failed");
            return;
        }

        // Wait for the measurement to complete
        vTaskDelay(pdMS_TO_TICKS(1 + (bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme688_handle) / 1000)));

        // Get sensor data
        ret = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme688_handle);
        if (ret == BME68X_OK && n_fields > 0)
        {
            ESP_LOGI(LOG_TAG,
                     "Temperature: %.1f°C, Pressure: %.1fhPa, Humidity: %.1f%%, Gas Resistance: %.2fMOhms.",
                     data.temperature,
                     data.pressure / 100.0,
                     data.humidity,
                     data.gas_resistance / 1e6);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
            return;
        }

        // Wait for before the next read
        vTaskDelay(pdMS_TO_TICKS(AMBIENT_SENSE_MEAS_LOOP_PERIOD_MS));
    }
}

// BME688 microseconds delay function implementation
static void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    // Use esp_rom_delay_us to delay for the specified period
    esp_rom_delay_us(period);
}

// BME688 I2C read function implementation
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C device handle
    i2c_master_dev_handle_t bme688_i2c_dev_handle = *(i2c_master_dev_handle_t *)intf_ptr;

    const int32_t i2c_write_timeout_ms = -1; // -1 == Wait forever
    esp_err_t i2c_ret = i2c_master_transmit_receive(bme688_i2c_dev_handle, &reg_addr, 1, reg_data, length, i2c_write_timeout_ms);

    // Return success or failure
    return (i2c_ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// BME688 I2C write function implementation
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C device handle
    i2c_master_dev_handle_t bme688_i2c_dev_handle = *(i2c_master_dev_handle_t *)intf_ptr;

    i2c_master_transmit_multi_buffer_info_t write_buffers_array[2] = {
        {
            .write_buffer = &reg_addr,
            .buffer_size = 1,
        },
        {
            .write_buffer = (uint8_t *)reg_data,
            .buffer_size = length,
        },
    };

    // TODO: Update to use i2c_master_register_event_callbacks and be asynchronous

    // Perform the I2C multi-buffer transmit
    const int32_t i2c_write_timeout_ms = -1; // -1 == Wait forever
    esp_err_t i2c_ret = i2c_master_multi_buffer_transmit(bme688_i2c_dev_handle, write_buffers_array, 2, i2c_write_timeout_ms);

    // Return success or failure
    return (i2c_ret == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}
