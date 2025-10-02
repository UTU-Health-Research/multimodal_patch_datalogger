#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "config.h"

static const char *TAG = "TEMP";

// Latest temperature data - protected by mutex
static SemaphoreHandle_t s_temp_mutex = NULL;
static volatile float s_latest_temperature = 0.0f;

// Read temperature from MAX30205
static esp_err_t max30205_read_temp(float *temp) {
    if (!temp) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30205_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MAX30205_REG_TEMP, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30205_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(IMU_I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Convert to temperature (16-bit, MSB first)
        int16_t raw_temp = (data[0] << 8) | data[1];
        *temp = raw_temp * 0.00390625f;  // Resolution is 2^-8 = 0.00390625°C
    }
    
    return ret;
}

// Temperature sampling task
static void temp_task(void *pvParameters) {
    float temperature = 0.0f;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Try to initialize temperature sensor
    bool init_success = false;
    esp_err_t ret = i2c_write_register_with_timeout(IMU_I2C_PORT, MAX30205_ADDR, 
                                              MAX30205_REG_CONFIG, 0x00); // Continuous conversion mode
    init_success = (ret == ESP_OK);
    
    ESP_LOGI(TAG, "Temperature sensor init: %s", init_success ? "Success" : "Failed");
    
    // Main task loop
    while(1) {
        // Try to read temperature
        if (max30205_read_temp(&temperature) == ESP_OK) {
            // Update the global buffer with mutex protection
            xSemaphoreTake(s_temp_mutex, portMAX_DELAY);
            s_latest_temperature = temperature;
            xSemaphoreGive(s_temp_mutex);
            
            ESP_LOGD(TAG, "Temperature: %.2f°C", temperature);
        }
        
        // Delay until next sample time (can use a different period than IMU if needed)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100)); // 10Hz sampling rate
    }
}

// Initialize and start temperature monitoring
esp_err_t temp_start(void) {
    // Create mutex if not already created
    if (s_temp_mutex == NULL) {
        s_temp_mutex = xSemaphoreCreateMutex();
        if (s_temp_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create temperature mutex");
            return ESP_FAIL;
        }
    }
    
    // Create temperature sampling task
    BaseType_t task_created = xTaskCreatePinnedToCore(
        temp_task,
        "temp_task",
        2048,
        NULL,
        5,  // Priority (adjust as needed)
        NULL,
        0    // Run on Core 0 like IMU task
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create temperature task");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Get the latest temperature reading - thread-safe access function
float temp_get_latest(void) {
    float temperature = 0.0f;
    
    if (s_temp_mutex != NULL) {
        xSemaphoreTake(s_temp_mutex, portMAX_DELAY);
        temperature = s_latest_temperature;
        xSemaphoreGive(s_temp_mutex);
    }
    
    return temperature;
}