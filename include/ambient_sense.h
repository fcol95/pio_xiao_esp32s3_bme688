#ifndef AMBIENT_SENSE__H__
#define AMBIENT_SENSE__H__

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t ambient_sense_init(i2c_master_bus_handle_t i2c_bus_handle);
void ambient_sense_task(void *pvParameter);

#endif // AMBIENT_SENSE__H__