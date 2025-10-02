// src/config.h
#pragma once
#include "driver/i2c.h"

// Logging format
#define LOG_BINARY        1        // 1=binary, 0=CSV
#define DECIMATE_BY       2        // 500 Hz in, 250 Hz out
#define DECIMATE_AVG      1        // 1=average pairs, 0=drop every other

// Data geometry
#define ECG_CHANNELS      8
#define FRAME_SIZE_BYTES  27       // ADS1298R: 3 status + 8*3 bytes
#define SENSOR_RATE_HZ    500
#define OUTPUT_RATE_HZ    (SENSOR_RATE_HZ / DECIMATE_BY)

// Batching and queues
#define SAMPLES_PER_BATCH 256      // at 250 Hz ≈ 1.024 s/batch
#define SENSOR_QUEUE_LEN  2048     // frames (500 Hz → ~4 s cushion)
#define SD_QUEUE_LEN      8        // batch pointers queued to writer
#define WRITE_CHUNK_BYTES (16*1024)
#define SYNC_INTERVAL_US  (1000000)

// Pins (adjust to your board)
#define SD_USE_SDMMC      0        // 0=SDSPI (SPI2), 1=SDMMC (4-bit)

#define SD_MOUNT_POINT    "/sdcard"
#define FILENAME_PREFIX   "DAT_"
#define FILE_EXTENSION     ".bin"

#if SD_USE_SDMMC
  #define SDMMC_SLOT_WIDTH 4
#else
  #define SD_MOSI   2
  #define SD_MISO   44
  #define SD_SCLK   43
  #define SD_CS     1
#endif

#define ECG_MOSI  11
#define ECG_MISO  13
#define ECG_SCLK  12
#define ECG_CS    10
#define DRDY_PIN  9
#define START_PIN 14
#define PWDN_PIN  3
#define RESET_PIN 21
#define GREEN_LED_ANODE 7


// IMU Configuration
#define IMU_I2C_PORT         I2C_NUM_0
#define IMU_SDA_PIN          4  // Adjust according to your wiring
#define IMU_SCL_PIN          5  // Adjust according to your wiring
#define IMU1_ADDR            0x6A // Default LSM6DSOX address
#define IMU2_ADDR            0x6B // Alternative address (if SDO/SA0 pin is high)
#define IMU_SAMPLE_PERIOD_MS 10  // 100Hz sampling
#define I2C_TIMEOUT_MS       20  // 20ms timeout for I2C operations
#define I2C_FREQ_HZ          400000 // 400KHz I2C clock frequency

