#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"

#include "driver/gpio.h"

#include "config.h"
#include "data_types.h"
#include "ads1298r.h"
#include "i2c_sensors.h"
#include "pipeline.h"
#include "sd_logger.h"
#include "ecg_inference.h"

// #include "serial_stream.h"          // commented out — not used for now

static const char *TAG = "app";

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "=== Application starting ===");

    // ---- Sensor queue: ADS pushes pkt_sample_t at 500 Hz ----
    QueueHandle_t sensor_q = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(pkt_sample_t));
    if (!sensor_q) {
        ESP_LOGE(TAG, "sensor_q create failed");
        return;
    }

    // ---- I2C sensors (IMU × 2 + temperature) ----
    sensors_start();

    // ---- ADS1298R acquisition ----
    ads1298r_start(sensor_q);

    // ---- Serial streaming (disabled to reduce load) ----
    // serial_stream_init(1024, 1024);
    // serial_stream_task_start();

    // ---- SD card logger ----
    QueueHandle_t sd_q = sd_logger_start();
    if (!sd_q) {
        ESP_LOGE(TAG, "SD logger failed — continuing without SD");
    } else {
        ESP_LOGI(TAG, "SD logger started");
    }

    // ---- ESP-DL ECG inference ----
    if (ecg_inference_init() != 0) {
        ESP_LOGE(TAG, "ECG inference init failed — predictions disabled");
    } else {
        ESP_LOGI(TAG, "ECG inference ready");
    }

    // ---- Pipeline: ties everything together ----
    pipeline_start(sensor_q, sd_q);

    ESP_LOGI(TAG, "System running: 500 Hz acq → 250 Hz logging, inference enabled");

    if (sd_q) {
        gpio_set_level(GREEN_LED_ANODE, 1);
        ESP_LOGI(TAG, "Recording to SD card");
    } else {
        ESP_LOGW(TAG, "Running without SD card");
    }
}