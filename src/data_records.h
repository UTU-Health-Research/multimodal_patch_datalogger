#pragma once

#include <stdint.h>

// ============================================================
// Unified binary file format
// ============================================================
//
// Layout:
//   [32-byte file header]
//   [record] [record] [record] ...
//
// Each record starts with a 1-byte type tag, followed by
// type-specific payload.  Three record types exist:
//
//   0x01  ECG + sensor sample        (250 Hz)
//   0x02  Vital signs                (1 Hz)
//   0x03  Disease prediction         (~0.061 Hz)
//
// ============================================================

#define FILE_MAGIC              0x48534456  // "VDSH" little-endian
#define FILE_FORMAT_VERSION     1

#define RECORD_TYPE_ECG         0x01
#define RECORD_TYPE_VITALS      0x02
#define RECORD_TYPE_PREDICTION  0x03

#define NUM_DISEASE_CLASSES     17

// Vitals generation rate: every 250 decimated samples = 1 second at 250 Hz
#define VITALS_INTERVAL_SAMPLES 250

// ---- File header (32 bytes, written once) ----
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t ecg_rate_hz;
    uint8_t  ecg_channels;
    uint8_t  num_imu;
    uint8_t  num_disease_classes;
    uint8_t  reserved[21];
} file_header_t;

_Static_assert(sizeof(file_header_t) == 32, "file_header_t must be 32 bytes");

// ---- Vital signs record ----
typedef struct __attribute__((packed)) {
    uint8_t  type;                  // RECORD_TYPE_VITALS
    uint32_t timestamp_ms;
    float    heart_rate_bpm;
    float    hrv_sdnn_ms;
    float    resp_rate_bpm;
    float    temperature_c;
} vitals_record_t;

_Static_assert(sizeof(vitals_record_t) == 21, "vitals_record_t must be 21 bytes");

// ---- Disease prediction record ----
typedef struct __attribute__((packed)) {
    uint8_t  type;                  // RECORD_TYPE_PREDICTION
    uint32_t timestamp_ms;
    uint32_t window_index;
    float    inference_ms;
    float    probs[NUM_DISEASE_CLASSES];
} prediction_record_t;

_Static_assert(sizeof(prediction_record_t) == 81, "prediction_record_t must be 81 bytes");

// ---- Internal transfer struct (not written to disk) ----
typedef struct {
    uint32_t timestamp_ms;
    uint32_t window_index;
    float    inference_ms;
    float    probs[NUM_DISEASE_CLASSES];
    int      valid;
} prediction_result_t;