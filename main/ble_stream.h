#pragma once

#include <stdint.h>
#include "data_records.h"   // for NUM_DISEASE_CLASSES

#define BLE_NUM_ECG_CHANNELS   8
#define BLE_ECG_BUNDLE_SIZE    10
#define BLE_NUM_PREDICTIONS    NUM_DISEASE_CLASSES  // 17

// Float mV → int16 scale: 1 LSB = 0.01 mV = 10 µV, range ±327.67 mV
#define BLE_ECG_SCALE          100.0f

// Update this to match your board's BLE status LED
#ifndef BLE_LED_PIN
#define BLE_LED_PIN            18
#endif

// ---- BLE packet structures (match receiver app) ----

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;                      // 4
    uint8_t  heart_rate;                        // 1
    uint8_t  hrv;                               // 1
    uint8_t  resp_rate;                         // 1
    uint8_t  padding;                           // 1
    int16_t  temperature;                       // 2  (°C × 100)
    float    predictions[BLE_NUM_PREDICTIONS];  // 68
} ble_vitals_packet_t;                          // total: 78 bytes

typedef struct __attribute__((packed)) {
    int16_t channels[BLE_NUM_ECG_CHANNELS];     // 16
} ble_ecg_slice_t;                              // bundle of 10 = 160 bytes

_Static_assert(sizeof(ble_vitals_packet_t) == 78, "vitals packet size mismatch");
_Static_assert(sizeof(ble_ecg_slice_t)     == 16, "ecg slice size mismatch");

// ---- API ----
void ble_stream_init(void);
void ble_stream_send_vitals(const ble_vitals_packet_t *pkt);
void ble_stream_send_ecg(const ble_ecg_slice_t bundle[BLE_ECG_BUNDLE_SIZE]);