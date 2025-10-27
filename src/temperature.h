// src/temperature.h
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "data_types.h"
#include "config.h"
#include <string.h>

// Initialize and start the temperature task
void temp_start(void);

// Get the latest temperature data - thread-safe access function
void temp_get_latest_data(float *temp_data);