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