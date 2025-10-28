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

// Initialize and start the IMU task
void imu_start(void);

// Get the latest IMU data - thread-safe access function
void imu_get_latest_data(imu_sample_t *imu1_data, imu_sample_t *imu2_data);

// I2C functions
esp_err_t i2c_read_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_write_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len); 