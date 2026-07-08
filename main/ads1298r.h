// src/ads1298r.h
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif


// Initialize pins, SPI3 device, DRDY ISR, and start producer task.
// Samples are pushed into out_queue at SENSOR_RATE_HZ.
void ads1298r_start(QueueHandle_t out_queue);

#ifdef __cplusplus
}
#endif