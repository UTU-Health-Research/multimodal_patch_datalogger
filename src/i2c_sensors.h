// src/imu.h
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "data_types.h"
#include "config.h"
#include <string.h>

// Detected temperature sensor type
typedef enum {
    TEMP_SENSOR_NONE = 0,
    TEMP_SENSOR_TMP117,
    TEMP_SENSOR_MAX30205
} temp_sensor_type_t;

// Initialize and start the IMU task
void sensors_start(void);

// Get the latest IMU data - thread-safe access function
void sensors_get_latest_data(imu_sample_t *imu1_data, imu_sample_t *imu2_data, float *temperature);

// Exposed so other modules can check what was detected
temp_sensor_type_t sensors_get_temp_sensor_type(void);

// I2C functions
esp_err_t i2c_read_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_write_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len); 