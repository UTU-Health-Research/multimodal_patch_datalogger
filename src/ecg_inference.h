#pragma once

#include "data_types.h"
#include "data_records.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the ESP-DL model and allocate the sample window.
 * Call after SD card is mounted (for future use) and before pipeline_start.
 * Returns 0 on success, negative on error.
 */
int ecg_inference_init(void);

/**
 * Returns non-zero when the inference engine is ready to accept samples.
 */
int ecg_inference_is_ready(void);

/**
 * Feed one raw 500 Hz sample.  Internally downsamples to 250 Hz,
 * fills a 4096-sample window, and runs inference when the window
 * is complete.  This call may block for the duration of inference
 * (~100-500 ms every ~16 s).
 */
void ecg_inference_push_sample_500hz(const pkt_sample_t *sample);

/**
 * Check whether a new prediction is available.
 * If so, copies the result into *out, clears the internal flag,
 * and returns 1.  Otherwise returns 0.
 *
 * Safe to call from the same task that calls push_sample (no mutex needed).
 */
int ecg_inference_get_prediction(prediction_result_t *out);

#ifdef __cplusplus
}
#endif