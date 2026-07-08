#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include "dl_model_base.hpp"
#include "dl_tensor_base.hpp"

#include "ecg_inference.h"
#include "config.h"
#include "data_types.h"
#include "data_records.h"

static const char *TAG = "ecg_inference";

// ---- Embedded model binary (linked via CMakeLists.txt) ----
extern const uint8_t model_espdl[]     asm("_binary_model_espdl_start");
extern const uint8_t model_espdl_end[] asm("_binary_model_espdl_end");

// ---- Model parameters ----
#define MODEL_SAMPLES       4096
#define MODEL_LEADS         12
#define MODEL_CLASSES       NUM_DISEASE_CLASSES   // 17

#define DEVICE_INPUT_HZ     500
#define MODEL_INPUT_HZ      250
#define DOWNSAMPLE_FACTOR   2

#define ECG_INPUT_EXP       (-7)    // scale = 2^-7 = 1/128
#define LOGITS_OUTPUT_EXP   (-3)    // scale = 2^-3 = 0.125

// ---- ESP-DL objects ----
static dl::Model      *s_model          = nullptr;
static dl::TensorBase *s_ecg_tensor     = nullptr;
static dl::TensorBase *s_logits_tensor  = nullptr;

static int8_t *s_ecg_input     = nullptr;
static int8_t *s_logits_output = nullptr;

// ---- Sample window ----
static float *s_window     = nullptr;   // [MODEL_SAMPLES * MODEL_LEADS]
static int    s_window_pos = 0;
static int    s_ready      = 0;
static int    s_window_index = 0;

// ---- 500→250 Hz downsampler ----
static float s_downsample_accum[MODEL_LEADS] = {0};
static int   s_downsample_count = 0;

// ---- Latest prediction result (consumed by pipeline) ----
static prediction_result_t s_latest_prediction = {};

static const char *LABELS[MODEL_CLASSES] = {
    "426783006", "426177001", "164934002", "427393009", "713426002",
    "427084000", "59118001",  "164889003", "59931005",  "47665007",
    "445118002", "39732003",  "164890007", "164909002", "270492004",
    "251146004", "284470004"
};

// ---- Helpers ----

static inline float sigmoidf_safe(float x)
{
    if (x >  40.0f) return 1.0f;
    if (x < -40.0f) return 0.0f;
    return 1.0f / (1.0f + expf(-x));
}

static inline int8_t quantize_to_int8(float x_norm)
{
    int q = (int)lrintf(x_norm * 128.0f);   // 2^7
    if (q >  127) q =  127;
    if (q < -128) q = -128;
    return (int8_t)q;
}

// ---- 8-channel device → 12-lead ECG ----

static void map_device_channels_to_12lead(const pkt_sample_t *sample,
                                          float leads[MODEL_LEADS])
{
    float I  = sample->ecg[1];
    float II = sample->ecg[2];

    leads[0]  = I;
    leads[1]  = II;
    leads[2]  = II - I;                     // III
    leads[3]  = -0.5f * (I + II);           // aVR
    leads[4]  = I  - 0.5f * II;             // aVL
    leads[5]  = II - 0.5f * I;              // aVF
    leads[6]  = sample->ecg[3];             // V1
    leads[7]  = sample->ecg[4];             // V2
    leads[8]  = sample->ecg[5];             // V3
    leads[9]  = sample->ecg[6];             // V4
    leads[10] = sample->ecg[7];             // V5
    leads[11] = sample->ecg[7];             // V6 (duplicate V5, temporary)
}

// ---- Run model on current window ----

static void run_inference_on_current_window(int32_t timestamp_ms)
{
    if (!s_ready || !s_model || !s_ecg_input || !s_logits_output || !s_window)
        return;

    // ---- Per-lead 0-1 normalization ----
    float lead_min[MODEL_LEADS], lead_max[MODEL_LEADS];

    for (int l = 0; l < MODEL_LEADS; l++) {
        lead_min[l] = s_window[l];
        lead_max[l] = s_window[l];
    }
    for (int t = 0; t < MODEL_SAMPLES; t++) {
        for (int l = 0; l < MODEL_LEADS; l++) {
            float v = s_window[t * MODEL_LEADS + l];
            if (v < lead_min[l]) lead_min[l] = v;
            if (v > lead_max[l]) lead_max[l] = v;
        }
    }

    // ---- Quantize into model input tensor ----
    for (int t = 0; t < MODEL_SAMPLES; t++) {
        for (int l = 0; l < MODEL_LEADS; l++) {
            float raw   = s_window[t * MODEL_LEADS + l];
            float range = lead_max[l] - lead_min[l];
            float x_norm = 0.0f;

            if (range > 1e-8f)
                x_norm = (raw - lead_min[l]) / range;

            if (x_norm < 0.0f) x_norm = 0.0f;
            if (x_norm > 1.0f) x_norm = 1.0f;

            s_ecg_input[t * MODEL_LEADS + l] = quantize_to_int8(x_norm);
        }
    }

    // ---- Inference ----
    int64_t t0 = esp_timer_get_time();
    s_model->run();
    int64_t t1 = esp_timer_get_time();

    // ---- Decode outputs ----
    float probs[MODEL_CLASSES];
    int   max_idx  = 0;
    float max_prob = -1.0f;

    for (int i = 0; i < MODEL_CLASSES; i++) {
        float logit = (float)s_logits_output[i] * 0.125f;
        probs[i] = sigmoidf_safe(logit);
        if (probs[i] > max_prob) {
            max_prob = probs[i];
            max_idx  = i;
        }
    }

    float inference_ms = (float)(t1 - t0) / 1000.0f;

    ESP_LOGI(TAG,
             "window=%d  inference=%.1f ms  max=%s  prob=%.3f",
             s_window_index, inference_ms, LABELS[max_idx], max_prob);

    // ---- Store result for pipeline to pick up ----
    s_latest_prediction.timestamp_ms = (uint32_t)timestamp_ms;
    s_latest_prediction.window_index = (uint32_t)s_window_index;
    s_latest_prediction.inference_ms = inference_ms;
    memcpy(s_latest_prediction.probs, probs, sizeof(probs));
    s_latest_prediction.valid = 1;

    s_window_index++;
}

// ---- Push one 250 Hz sample into the window ----

static void push_250hz_to_window(const float leads[MODEL_LEADS],
                                 int32_t timestamp_ms)
{
    for (int l = 0; l < MODEL_LEADS; l++)
        s_window[s_window_pos * MODEL_LEADS + l] = leads[l];

    s_window_pos++;

    if (s_window_pos >= MODEL_SAMPLES) {
        run_inference_on_current_window(timestamp_ms);
        s_window_pos = 0;   // non-overlapping windows
    }
}

// ============================================================
// Public API
// ============================================================

int ecg_inference_init(void)
{
    ESP_LOGI(TAG, "Initializing ECG inference engine");

    uint32_t model_size = (uint32_t)(model_espdl_end - model_espdl);
    ESP_LOGI(TAG, "Embedded model size: %lu bytes", (unsigned long)model_size);

    s_model = new dl::Model((const char *)model_espdl);
    if (!s_model) {
        ESP_LOGE(TAG, "Failed to create ESP-DL model");
        return -1;
    }

    s_ecg_tensor    = s_model->get_input("ecg");
    s_logits_tensor = s_model->get_output("logits");

    if (!s_ecg_tensor || !s_logits_tensor) {
        ESP_LOGE(TAG, "Could not find model input/output tensors");
        return -2;
    }

    s_ecg_input    = s_ecg_tensor->get_element_ptr<int8_t>();
    s_logits_output = s_logits_tensor->get_element_ptr<int8_t>();

    if (!s_ecg_input || !s_logits_output) {
        ESP_LOGE(TAG, "Could not get tensor data pointers");
        return -3;
    }

    // Allocate floating-point window: 4096 × 12 × 4 bytes ≈ 192 KB
    size_t window_bytes = MODEL_SAMPLES * MODEL_LEADS * sizeof(float);

    s_window = (float *)heap_caps_malloc(window_bytes,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_window) {
        ESP_LOGW(TAG, "PSRAM alloc failed, falling back to internal heap");
        s_window = (float *)heap_caps_malloc(window_bytes, MALLOC_CAP_8BIT);
    }
    if (!s_window) {
        ESP_LOGE(TAG, "Could not allocate inference window (%u bytes)",
                 (unsigned)window_bytes);
        return -4;
    }

    memset(s_window, 0, window_bytes);
    memset(s_downsample_accum, 0, sizeof(s_downsample_accum));
    memset(&s_latest_prediction, 0, sizeof(s_latest_prediction));

    s_window_pos      = 0;
    s_window_index    = 0;
    s_downsample_count = 0;
    s_ready           = 1;

    ESP_LOGI(TAG,
             "Ready: device=%d Hz  model=%d Hz  window=%d samples (%.1f s)",
             DEVICE_INPUT_HZ, MODEL_INPUT_HZ, MODEL_SAMPLES,
             (float)MODEL_SAMPLES / (float)MODEL_INPUT_HZ);

    return 0;
}

int ecg_inference_is_ready(void)
{
    return s_ready;
}

void ecg_inference_push_sample_500hz(const pkt_sample_t *sample)
{
    if (!s_ready || !sample || !s_window)
        return;

    float leads_500hz[MODEL_LEADS];
    map_device_channels_to_12lead(sample, leads_500hz);

    for (int l = 0; l < MODEL_LEADS; l++)
        s_downsample_accum[l] += leads_500hz[l];

    s_downsample_count++;

    if (s_downsample_count >= DOWNSAMPLE_FACTOR) {
        float leads_250hz[MODEL_LEADS];
        for (int l = 0; l < MODEL_LEADS; l++) {
            leads_250hz[l] = s_downsample_accum[l] / (float)DOWNSAMPLE_FACTOR;
            s_downsample_accum[l] = 0.0f;
        }
        s_downsample_count = 0;

        push_250hz_to_window(leads_250hz, (int32_t)sample->timestamp);
    }
}

int ecg_inference_get_prediction(prediction_result_t *out)
{
    if (!out || !s_latest_prediction.valid)
        return 0;

    memcpy(out, &s_latest_prediction, sizeof(prediction_result_t));
    s_latest_prediction.valid = 0;
    return 1;
}