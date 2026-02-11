#ifndef SERIAL_STREAM_H
#define SERIAL_STREAM_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Initialize the USB Serial JTAG interface
 * 
 * @param rx_buffer_size Size of receive buffer
 * @param tx_buffer_size Size of transmit buffer
 * @return esp_err_t ESP_OK on success
 */
esp_err_t serial_stream_init(size_t rx_buffer_size, size_t tx_buffer_size);

/**
 * @brief Send data directly over USB Serial JTAG
 * 
 * @param data Pointer to data to send
 * @param len Length of data in bytes
 * @param timeout_ms Timeout in milliseconds
 * @return int Number of bytes sent or negative error code
 */
int serial_stream_send_data(const void* data, size_t len, uint32_t timeout_ms);

/**
 * @brief Get the serial stream queue handle
 * 
 * @return QueueHandle_t Handle to serial data queue
 */
QueueHandle_t serial_stream_get_queue(void);

/**
 * @brief Start the serial stream task
 */
void serial_stream_task_start(void);

/**
 * @brief Check if USB Serial JTAG is connected to a host
 * 
 * @return bool True if connected, false otherwise
 */
bool serial_stream_is_connected(void);

#endif /* SERIAL_STREAM_H */