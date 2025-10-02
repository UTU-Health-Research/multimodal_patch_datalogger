// src/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "config.h"
#include "data_types.h"
#include "ads1298r.h"
#include "imu.h"
#include "max30205.h"
#include "pipeline.h"
#include "sd_logger.h"
#include "esp_random.h"

static const char *TAG = "app";


void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);  // Or DEBUG if needed
    ESP_LOGI("INIT", "Hello logger!");
    // Create the sensor queue (_sample_t from ADS at 500 Hz)
    QueueHandle_t sensor_q = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(pkt_sample_t));
    if (!sensor_q) { ESP_LOGE(TAG, "sensor_q create failed"); return; }

    imu_start();
    temp_start();  // Initialize temperature sensor

    // Start ADS producer (pushes into sensor_q)
    ads1298r_start(sensor_q);
    
    QueueHandle_t sd_q = sd_logger_start();
    if (!sd_q) { ESP_LOGE(TAG, "sd_q create failed"); return; }

    // Start pipeline (decimate to 250 Hz, batch and forward to sd_q)
    pipeline_start(sensor_q, sd_q);

    ESP_LOGI(TAG, "System running: 500 Hz in → 250 Hz out, batched to SD");

}
