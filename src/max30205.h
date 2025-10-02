#pragma once
#include "esp_err.h"

/**
 * @brief Initialize and start temperature sensor monitoring
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t temp_start(void);

/**
 * @brief Get latest temperature reading
 * 
 * @return float Temperature in Celsius
 */
float temp_get_latest(void);