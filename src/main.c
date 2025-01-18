#include <stdio.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h" //< For LED toggling
#include "driver/i2c.h"  //< For BME688 I2C communication port

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_rom_sys.h" //< For BME688 delay_us port
#include "esp_log.h"

#include "bme68x.h"

static const char *TAG = "main";

#ifdef CONFIG_BLINK_GPIO
#define BLINK_GPIO (gpio_num_t) CONFIG_BLINK_GPIO
#else
#define BLINK_GPIO (gpio_num_t)21 // LED_BUILT_IN
#endif

void blink_task(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        // Blink off (output low)
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Blink on (output high)
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// BME688 microseconds delay function implementation
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    // Use esp_rom_delay_us to delay for the specified period
    esp_rom_delay_us(period);
}

// BME688 I2C read function implementation
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C address
    uint8_t i2c_addr = *(uint8_t *)intf_ptr;

    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return BME68X_E_COM_FAIL; // Communication failure
    }

    // Queue the write of the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Queue the read of the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1)
    {
        i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute the I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    // Return success or failure
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// BME688 I2C write function implementation
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C address
    uint8_t i2c_addr = *(uint8_t *)intf_ptr;

    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return BME68X_E_COM_FAIL; // Communication failure
    }

    // Queue the write of the register address and data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);
    i2c_master_stop(cmd);

    // Execute the I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    // Return success or failure
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

void bme688_task(void *pvParameter)
{
    uint8_t i2c_address = 0x76; // Replace with your sensor's I2C address

    struct bme68x_dev bme688_handle =
        {
            .intf = BME68X_I2C_INTF,
            // Port Functions and Pointer
            .intf_ptr = &i2c_address,
            .delay_us = bme68x_delay_us,
            .read = bme68x_i2c_read,
            .write = bme68x_i2c_write,
            .amb_temp = 25, // Ambient temperature in degrees Celsius
        };

    int8_t ret = bme68x_init(&bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(TAG, "BME68x initialization failed");
    }
    else
    {
        ESP_LOGI(TAG, "BME68x initialization succeeded");
    }

    struct bme68x_conf conf;
    struct bme68x_heatr_conf heatr_conf;
    struct bme68x_data data;
    uint8_t n_fields;

    // Set sensor configuration
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    bme68x_set_conf(&conf, &bme688_handle);

    // Set heater configuration
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 320; // Target temperature in degree Celsius
    heatr_conf.heatr_dur = 150;  // Duration in milliseconds
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme688_handle);

    while (1)
    {
        // Set sensor to forced mode
        bme68x_set_op_mode(BME68X_FORCED_MODE, &bme688_handle);

        // Wait for the measurement to complete
        vTaskDelay(pdMS_TO_TICKS(1 + (bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme688_handle) / 1000)));

        // Get sensor data
        ret = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme688_handle);
        if (ret == BME68X_OK && n_fields > 0)
        {
            ESP_LOGI(TAG, "Temperature: %.2f°C, Pressure: %.2fhPa, Humidity: %.2f%%, Gas Resistance: %.2fOhms.", data.temperature, data.pressure / 100.0, data.humidity, data.gas_resistance);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to get sensor data");
        }

        // Wait for 1 second before the next read
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_8, // XIAO ESP32S3 SDA GPIO
        .scl_io_num = GPIO_NUM_9, // XIAO ESP32S3 SCL GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // I2C clock speed (400 kHz)
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void print_board_info(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "  This is %s chip with %d CPU core(s), WiFi%s%s, ",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK)
    {
        ESP_LOGE(TAG, "Get flash size failed!");
        return;
    }

    ESP_LOGI(TAG, "%ldMB %s flash", flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "Minimum free heap size: %ld bytes", esp_get_minimum_free_heap_size());
}

void app_main()
{
    ESP_LOGI(TAG, "-- XIAO ESP32S3 BME688 Test --");

    print_board_info();

    ESP_LOGI(TAG, "Starting program...");

    // Drivers Init
    i2c_master_init();

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&bme688_task, "bme688_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}