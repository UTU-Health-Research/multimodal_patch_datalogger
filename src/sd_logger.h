// src/sd_logger.h
#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize and mount SD card, open output file, and start writer task.
// Returns the queue handle the writer listens on (for batch_buf_t).
QueueHandle_t sd_logger_start();

// Stop writer, close file, unmount SD (optional for simple demos).
void sd_logger_stop(void);

#ifdef __cplusplus
}
#endif