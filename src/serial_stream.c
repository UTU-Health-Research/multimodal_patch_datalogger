#include "serial_stream.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "data_types.h"  // For pkt_sample_t

static const char* TAG = "SERIAL_STREAM";
static bool initialized = false;
static QueueHandle_t serial_queue = NULL;
static TaskHandle_t serial_task_handle = NULL;

esp_err_t serial_stream_init(size_t rx_buffer_size, size_t tx_buffer_size) {
    if (initialized) {
        ESP_LOGW(TAG, "USB Serial JTAG already initialized");
        return ESP_OK;
    }

    usb_serial_jtag_driver_config_t config = {
        .rx_buffer_size = rx_buffer_size,
        .tx_buffer_size = tx_buffer_size,
    };

    esp_err_t ret = usb_serial_jtag_driver_install(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize USB Serial JTAG: %d", ret);
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "USB Serial JTAG initialized successfully");
    return ESP_OK;
}

int serial_stream_send_data(const void* data, size_t len, uint32_t timeout_ms) {
    if (!initialized) {
        ESP_LOGE(TAG, "USB Serial JTAG not initialized");
        return -1;
    }

    if (data == NULL || len == 0) {
        ESP_LOGW(TAG, "Invalid data or length");
        return 0;
    }

    int ret = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(timeout_ms));
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send data: %d", ret);
    }
    
    return ret;
}

QueueHandle_t serial_stream_get_queue(void) {
    if (serial_queue == NULL) {
        // Create queue with appropriate size (adjust based on your needs)
        // This queue will hold pkt_sample_t items
        serial_queue = xQueueCreate(20, sizeof(pkt_sample_t));
        if (serial_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create serial stream queue");
        } else {
            ESP_LOGI(TAG, "Serial stream queue created successfully");
        }
    }
    return serial_queue;
}

static void serial_stream_task(void *pvParameters) {
    pkt_sample_t out_data;
    
    ESP_LOGI(TAG, "Serial stream task started");
    
    for (;;) {
        // Wait for data to become available from the queue
        if (xQueueReceive(serial_queue, &out_data, portMAX_DELAY)) {
            // Send over serial at lower priority without blocking pipeline
            int ret = usb_serial_jtag_write_bytes(&out_data, sizeof(out_data), pdMS_TO_TICKS(10));
            if (ret < 0) {
                ESP_LOGW(TAG, "Serial write failed: %d", ret);
            }
            // No task delay needed here - the task will naturally yield when waiting for queue
        }
    }
}

void serial_stream_task_start(void) {
    // Don't start the task if it's already running
    if (serial_task_handle != NULL) {
        ESP_LOGW(TAG, "Serial stream task already running");
        return;
    }
    
    // Create the queue if it doesn't exist
    if (serial_queue == NULL) {
        serial_queue = xQueueCreate(20, sizeof(pkt_sample_t));
        if (serial_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create serial stream queue");
            return;
        }
    }
    
    // Create task with LOWER priority than pipeline and SD card tasks
    BaseType_t res = xTaskCreatePinnedToCore(
        serial_stream_task,
        "serial_stream",
        2048,  // Stack size
        NULL,  // Parameters
        configMAX_PRIORITIES - 8,  // Lower priority
        &serial_task_handle,
        1  // Core ID
    );
    
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create serial stream task");
        serial_task_handle = NULL;
    } else {
        ESP_LOGI(TAG, "Serial stream task created successfully");
    }
}

bool serial_stream_is_connected(void) {
    // Use the official ESP-IDF API to check USB Serial/JTAG connection status
    return usb_serial_jtag_is_connected();
}