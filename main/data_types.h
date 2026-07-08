// src/data_types.h
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "config.h"

#include <stdint.h>

// LSM6DSOX data structure
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_sample_t;

// Combined data structure - single integrated packet
typedef struct {
    int32_t timestamp;     // 4 bytes
    float ecg[8];          // ECG_CHANNELS * 4 bytes = 32 bytes
    imu_sample_t imu1;       // 24 bytes
    imu_sample_t imu2;       // 24 bytes
    float temp;              // temperature reading 4 bytes
} pkt_sample_t;            // Total: 88 bytes


// Batch descriptor passed to SD writer
typedef struct {
    uint8_t *data;  // pointer to contiguous bytes to write
    size_t   len;   // length in bytes
} batch_buf_t;