#include "temperature.h"
#include "imu.h" //for I2C write and read functions

static const char *TAG = "TEMP";

// Mutex to protect temperature buffer access
static SemaphoreHandle_t s_temp_mutex = NULL;

// Latest temperature reading - initialized to zero
static volatile float s_temp_latest = 0.0f;

// Initialize MAX30205 temperature sensor
esp_err_t max30205_init(void) {
    // Configure the sensor - set to default mode (comparator mode, active low, one-shot)
    uint8_t config = 0x00;
    esp_err_t ret = i2c_write_register_with_timeout(TEMP_I2C_PORT, MAX30205_ADDR, MAX30205_REG_CONFIG, &config, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MAX30205: %d", ret);
        return ret;
    }
    
    return ESP_OK;
}

// Read temperature from MAX30205
esp_err_t max30205_read_temp(float *temperature) {
    if (temperature == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t data[2];
    esp_err_t ret = i2c_read_register_with_timeout(TEMP_I2C_PORT, MAX30205_ADDR, MAX30205_REG_TEMP, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert the 16-bit temperature value to Celsius
    // MAX30205 reports temperature as a 16-bit value with the LSB = 0.00390625°C
    int16_t raw_temp = (data[0] << 8) | data[1];
    *temperature = raw_temp * 0.00390625f;
    
    return ESP_OK;
}

// Temperature sensor task
static void temp_task(void *pvParameters) {
    float temp;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Initialize the MAX30205 sensor
    esp_err_t init_result = max30205_init();
    if (init_result == ESP_OK) {
        ESP_LOGI(TAG, "MAX30205 temperature sensor initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to initialize MAX30205: %d", init_result);
    }
    
    // Main task loop
    while(1) {
        // Try to read temperature from the sensor
        esp_err_t ret = max30205_read_temp(&temp);
        
        // Update the global value with mutex protection
        xSemaphoreTake(s_temp_mutex, portMAX_DELAY);
        if (ret == ESP_OK) {
            s_temp_latest = temp;
            ESP_LOGD(TAG, "Temperature: %.2f°C", temp);
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "Temperature read error: %d", ret);
        }
        xSemaphoreGive(s_temp_mutex);
        
        // Delay until the next sample time
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TEMP_SAMPLE_PERIOD_MS));
    }
}

// Initialize and start the temperature task
void temp_start(void) {
    // Create mutex for temperature data
    s_temp_mutex = xSemaphoreCreateMutex();
    
    // Create temperature task on Core 0 with appropriate priority
    xTaskCreatePinnedToCore(temp_task, "temp_task", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
    
    ESP_LOGI(TAG, "Temperature monitoring task started");
}

// Get the latest temperature reading - thread-safe access function
void temp_get_latest(float *temperature) {
    if (temperature == NULL) return;
    
    xSemaphoreTake(s_temp_mutex, portMAX_DELAY);
    *temperature = s_temp_latest;
    xSemaphoreGive(s_temp_mutex);
}