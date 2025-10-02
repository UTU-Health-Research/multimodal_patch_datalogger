// src/max30205.h
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "data_types.h"
#include "config.h"
#include <string.h>

// MAX30205 I2C Address
#define MAX30205_I2C_ADDR          0x90

// MAX30205 Register Addresses
#define MAX30205_TEMP_REG          0x00    // Temperature register
#define MAX30205_CONFIG_REG        0x01    // Configuration register
#define MAX30205_THYST_REG         0x02    // Hysteresis register
#define MAX30205_TOS_REG           0x03    // Over-temp shutdown threshold register

// Function prototypes
esp_err_t max30205_init(i2c_port_t i2c_port);
esp_err_t max30205_read_temp(i2c_port_t i2c_port, float *temperature);

// Task related functions
void temperature_start(void);
void temperature_get_latest_data(temperature_sample_t *temperature_data);

// Helper functions
esp_err_t max30205_read_register(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t max30205_write_register(i2c_port_t i2c_port, uint8_t reg_addr, uint8_t *data, size_t len);