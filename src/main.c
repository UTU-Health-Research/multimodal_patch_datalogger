// src/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "config.h"
#include "data_types.h"
#include "ads1298r.h"
#include "i2c_sensors.h"
#include "pipeline.h"
#include "sd_logger.h"
#include "esp_random.h"
//#include "serial_stream.h"
#include "driver/uart.h"    

static const char *TAG = "app";


void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);  // Or DEBUG if needed
    uart_set_baudrate(CONFIG_ESP_CONSOLE_UART_NUM, 500000);  // Increase baud rate

    ESP_LOGI("INIT", "Hello logger!");

    // Create the sensor queue (_sample_t from ADS at 500 Hz)
    QueueHandle_t sensor_q = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(pkt_sample_t));
    if (!sensor_q) { ESP_LOGE(TAG, "sensor_q create failed"); return; }

    sensors_start(); // Start sensors task
    //temp_start(); // Start temperature sensor task

    // Start ADS producer (pushes into sensor_q)
    ads1298r_start(sensor_q);

    
    QueueHandle_t sd_q = sd_logger_start();
    if (!sd_q) { ESP_LOGE(TAG, "sd_q create failed"); 
        return; }

    // Start pipeline (decimate to 250 Hz, batch and forward to sd_q)
    pipeline_start(sensor_q, sd_q);

    ESP_LOGI(TAG, "System running: 500 Hz in - 250 Hz out, batched to SD");
    gpio_set_level(GREEN_LED_ANODE, 1); // turn on green LED

    //serial_streamer_init();
    //serial_streamer_start();

        // while (1) {
        //     vTaskDelay(pdMS_TO_TICKS(1000));
        // }

}
