// src/main.c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "config.h"
#include "data_types.h"
#include "ads1298r.h"
#include "i2c_sensors.h"
#include "pipeline.h"
#include "sd_logger.h"
#include "esp_random.h"
#include "serial_stream.h"
#include "driver/uart.h"    

// CPU stats monitoring configuration
#define STATS_COLLECTION_TICKS 500  // 5 seconds at 100Hz
#define STATS_AUTO_INTERVAL_MS 1000  // Auto-display every minute, set to 0 to disable

// Stats markers for Python parser to detect
#define STATS_START_MARKER "\n<<<CPU_STATS_BEGIN>>>\n"
#define STATS_END_MARKER "<<<CPU_STATS_END>>>\n"

static const char *TAG = "app";

// Stats display function
static void display_real_time_stats(void)
{
    TaskStatus_t *pxTaskStatusArray;
    UBaseType_t uxArraySize;
    uint32_t ulTotalRunTime;
    
    // Get number of tasks
    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    
    if (pxTaskStatusArray != NULL) {
        // Generate raw stats
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        // Send start marker
        printf(STATS_START_MARKER);
        
        // Display header
        printf("CPU Stats (over 2.5s)\n");
        printf("Total Tasks: %d\n", uxArraySize);
        
        // Display each task with bar graph representation
        for (int x = 0; x < uxArraySize; x++) {
            // Calculate percentage
            uint32_t ulStatsPercentage = 0;
            if (ulTotalRunTime > 0) {
                ulStatsPercentage = (pxTaskStatusArray[x].ulRunTimeCounter * 100) / ulTotalRunTime;
            }
            
            // Display task name
            printf("%-16s [", pxTaskStatusArray[x].pcTaskName);
            
            // Generate bar visualization (shorter to minimize impact)
            for (int i = 0; i < 30; i++) {
                printf("%c", (i < (ulStatsPercentage * 30) / 100) ? '|' : ' ');
            }
            
            // Display percentage
            printf("] %3lu%%\n", ulStatsPercentage);
        }
        
        // Send end marker
        printf(STATS_END_MARKER);
        
        vPortFree(pxTaskStatusArray);
    }
}

//Stats monitor task
void stats_monitor_task(void *pvParameter)
{
    // Delay to let the system stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        #if STATS_AUTO_INTERVAL_MS > 0
        // Display stats
        vTaskDelay(pdMS_TO_TICKS(STATS_COLLECTION_TICKS));
        display_real_time_stats();
        
        // Wait for next interval
        vTaskDelay(pdMS_TO_TICKS(STATS_COLLECTION_TICKS));
        #else
        vTaskDelay(pdMS_TO_TICKS(10000));
        #endif
    }
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);  // Or DEBUG if needed
    //uart_set_baudrate (CONFIG_ESP_CONSOLE_UART_NUM, 500000); // Increase baud rate

    ESP_LOGI("INIT", "Hello logger!");

    // Create the sensor queue (_sample_t from ADS at 500 Hz)
    QueueHandle_t sensor_q = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(pkt_sample_t));
    if (!sensor_q) { ESP_LOGE(TAG, "sensor_q create failed"); return; }

    sensors_start(); // Start sensors task
    //temp_start(); // Start temperature sensor task

    // Start ADS producer (pushes into sensor_q)
    ads1298r_start(sensor_q);

    // Initialize the serial stream
    serial_stream_init(1024, 1024); //adjustable buffer sizes

    // Start the serial stream task (new)
    serial_stream_task_start();

    // Try to start SD logger, but continue even if it fails
    QueueHandle_t sd_q = sd_logger_start();
    if (!sd_q) { 
        ESP_LOGE(TAG, "SD logger failed to start, continuing without SD logging");
        // Continue execution instead of returning
    }

    // Start pipeline regardless of SD card status
    pipeline_start(sensor_q, sd_q);
    
    ESP_LOGI(TAG, "System running: 500 Hz in - 250 Hz out");
    if (sd_q) {
        ESP_LOGI(TAG, "Data batched to SD card");
        gpio_set_level(GREEN_LED_ANODE, 1); // turn on green LED
    } else {
        ESP_LOGI(TAG, "Running without SD card logging");
    }

    // Create the stats monitoring task
    // xTaskCreate(stats_monitor_task, "stats_monitor", 4096, NULL, 1, NULL);
}