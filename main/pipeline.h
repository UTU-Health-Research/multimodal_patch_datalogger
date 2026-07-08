// src/pipeline.h
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Start pipeline task:
// - in_queue: ecg_sample_t from ADS at 500 Hz
// - out_queue: batch_buf_t destined for SD writer
void pipeline_start(QueueHandle_t in_queue, QueueHandle_t out_queue);
