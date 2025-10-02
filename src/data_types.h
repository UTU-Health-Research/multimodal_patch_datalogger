// src/data_types.h
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "config.h"

#include <stdint.h>

// LSM6DSOX data structure
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_sample_t;

// Combined data structure - single integrated packet
typedef struct {
    int32_t timestamp;     // 4 bytes
    float ecg[8];          // ECG_CHANNELS * 4 bytes = 32 bytes
    imu_sample_t imu1;       // 12 bytes
    imu_sample_t imu2;       // 12 bytes
} pkt_sample_t;            // Total: 60 bytes


// Batch descriptor passed to SD writer
typedef struct {
    uint8_t *data;  // pointer to contiguous bytes to write
    size_t   len;   // length in bytes
} batch_buf_t;