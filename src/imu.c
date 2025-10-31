#include "imu.h"
#include "math.h"

static const char *TAG = "IMU";

// Mutex to protect IMU buffer access
static SemaphoreHandle_t s_imu_mutex = NULL;

// Latest IMU data - initialized to zeros
static volatile imu_sample_t s_imu1_latest = {0};
static volatile imu_sample_t s_imu2_latest = {0};

// Low-level I2C read with timeout
esp_err_t i2c_read_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Low-level I2C write with timeout
esp_err_t i2c_write_register_with_timeout(i2c_port_t i2c_port, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Initialize LSM6DSOX with timeout
static esp_err_t lsm6dsox_init_with_timeout(i2c_port_t i2c_port, uint8_t addr) {
    // Simple check if device is responsive
    uint8_t whoami = 0;
    esp_err_t ret;
    
    // Read WHO_AM_I register (should return 0x6C for LSM6DSOX)
    ret = i2c_read_register_with_timeout(i2c_port, addr, 0x0F, &whoami, 1);
    if (ret != ESP_OK) {
        return ret; // Device not responding
    }
    
    if (whoami != 0x6C) {
        ESP_LOGW(TAG, "Device at address 0x%02x has unexpected WHO_AM_I value: 0x%02x (expected 0x6C)", addr, whoami);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Device is responsive, proceed with minimal initialization
    // Configure accelerometer: 2g range, 104Hz
    uint8_t accel_config = 0x40; // 0x40 = 104Hz, 2g
    ret = i2c_write_register_with_timeout(i2c_port, addr, 0x10, &accel_config, 1);
    if (ret != ESP_OK) return ret;
    
    // Configure gyroscope: 125dps, 104Hz
    uint8_t gyro_config = 0x42; // 0x42 = 104Hz, 125dps
    ret = i2c_write_register_with_timeout(i2c_port, addr, 0x11, &gyro_config, 1);
    
    return ret;
}

// Read data from LSM6DSOX with timeout
static esp_err_t lsm6dsox_read_data_with_timeout(i2c_port_t i2c_port, uint8_t addr, imu_sample_t *data) {
    uint8_t buffer[12] = {0};
    esp_err_t ret;
    
    // Read gyroscope registers one by one
    for (int i = 0; i < 6; i++) {
        ret = i2c_read_register_with_timeout(i2c_port, addr, 0x22 + i, &buffer[i], 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyro reg 0x%02x: %d", 0x22 + i, ret);
            return ret;
        }
    }
    
    // Read accelerometer registers one by one
    for (int i = 0; i < 6; i++) {
        ret = i2c_read_register_with_timeout(i2c_port, addr, 0x28 + i, &buffer[6 + i], 1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accel reg 0x%02x: %d", 0x28 + i, ret);
            return ret;
        }
    }
    if (ret != ESP_OK) {
        return ret;
    }

    //esp_log_buffer_hex("IMU sample", &buffer, sizeof(buffer)); //printing data from sample

    // Parse data from buffer
    data->gyro_x = (float)((buffer[1] << 8) | buffer[0]) * SENSITIVITY_GYROSCOPE / 1000.0f;
    data->gyro_y = (float)((buffer[3] << 8) | buffer[2]) * SENSITIVITY_GYROSCOPE / 1000.0f;
    data->gyro_z = (float)((buffer[5] << 8) | buffer[4]) * SENSITIVITY_GYROSCOPE / 1000.0f;
    data->accel_x = (float)((buffer[7] << 8) | buffer[6]) * SENSITIVITY_ACCELEROMETER / 1000.0f;
    data->accel_y = (float)((buffer[9] << 8) | buffer[8]) * SENSITIVITY_ACCELEROMETER / 1000.0f;
    data->accel_z = (float)((buffer[11] << 8) | buffer[10]) * SENSITIVITY_ACCELEROMETER / 1000.0f;

    return ESP_OK;
}

// IMU task - runs on Core 0
static void imu_task(void *pvParameters) {
    imu_sample_t imu1_data = {0}, imu2_data = {0};
    TickType_t last_wake_time = xTaskGetTickCount();
    
    // Try to initialize IMU sensors
    bool init_success1 = (lsm6dsox_init_with_timeout(IMU_I2C_PORT, IMU1_ADDR) == ESP_OK);
    bool init_success2 = (lsm6dsox_init_with_timeout(IMU_I2C_PORT, IMU2_ADDR) == ESP_OK);
    
    ESP_LOGI(TAG, "IMU1 init: %s", init_success1 ? "Success" : "Failed");
    ESP_LOGI(TAG, "IMU2 init: %s", init_success2 ? "Success" : "Failed");
    
    // Main task loop
    while(1) {
        // Try to read from IMU1 - if it fails, keep using last values
        esp_err_t ret1 = lsm6dsox_read_data_with_timeout(IMU_I2C_PORT, IMU1_ADDR, &imu1_data);
        if (ret1 != ESP_OK && ret1 != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "IMU1 read error: %d", ret1); // Debug level logging
        }

        // Try to read from IMU2 - if it fails, keep using last values
        esp_err_t ret2 = lsm6dsox_read_data_with_timeout(IMU_I2C_PORT, IMU2_ADDR, &imu2_data);
        if (ret2 != ESP_OK && ret2 != ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "IMU2 read error: %d", ret2); // Debug level logging
        }
    
        // For debugging: read and log WHO_AM_I and CTRL registers
    // uint8_t whoami1 = 0;
    // uint8_t whoami2 = 0;
    // uint8_t regval = 0;
    // esp_err_t ret;
    
    // // Read WHO_AM_I register (should return 0x6C for LSM6DSOX)
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6A, 0x0F, &whoami1, 1);

    //  // Read WHO_AM_I register (should return 0x6C for LSM6DSOX)
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6B, 0x0F, &whoami2, 1);
 
    // // For logging both values together
    // ESP_LOGI(TAG, "WHO_AM_I values - IMU1: 0x%02x, IMU2: 0x%02x (expected 0x6C)", 
    //      whoami1, whoami2);

    //     // 5. Read back to verify settings
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6A, 0x10, &regval, 1);
    // ESP_LOGI(TAG, "CTRL1_XL = 0x%02x (expected 0x40)", regval);
    
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6A, 0x11, &regval, 1);
    // ESP_LOGI(TAG, "CTRL2_G = 0x%02x (expected 0x42)", regval);

    //     // 5. Read back to verify settings
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6B, 0x10, &regval, 1);
    // ESP_LOGI(TAG, "CTRL1_XL = 0x%02x (expected 0x40)", regval);
    
    // ret = i2c_read_register_with_timeout(IMU_I2C_PORT, 0x6B, 0x11, &regval, 1);
    // ESP_LOGI(TAG, "CTRL2_G = 0x%02x (expected 0x42)", regval);

        // Update the global buffer with mutex protection
        xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
        memcpy((void*)&s_imu1_latest, &imu1_data, sizeof(imu_sample_t));
        memcpy((void*)&s_imu2_latest, &imu2_data, sizeof(imu_sample_t));
        xSemaphoreGive(s_imu_mutex);
        
        // Delay until the next sample time
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS));
    }
}

// Initialize and start the IMU task
void imu_start(void) {
    // Create mutex for IMU data
    s_imu_mutex = xSemaphoreCreateMutex();
    
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IMU_SDA_PIN,
        .scl_io_num = IMU_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    
    esp_err_t ret = i2c_param_config(IMU_I2C_PORT, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = i2c_driver_install(IMU_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // Set I2C timeout to prevent hanging
    i2c_set_timeout(IMU_I2C_PORT, I2C_TIMEOUT_MS * 80000); // Convert ms to APB cycles
    
    // Create IMU task on Core 0 with lower priority than ADS task
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    
    ESP_LOGI(TAG, "IMU task started");
}

// Get the latest IMU data - thread-safe access function
void imu_get_latest_data(imu_sample_t *imu1_data, imu_sample_t *imu2_data) {
    if (!imu1_data || !imu2_data) return;
    
    xSemaphoreTake(s_imu_mutex, portMAX_DELAY);
    memcpy(imu1_data, (void*)&s_imu1_latest, sizeof(imu_sample_t));
    memcpy(imu2_data, (void*)&s_imu2_latest, sizeof(imu_sample_t));
    xSemaphoreGive(s_imu_mutex);
}