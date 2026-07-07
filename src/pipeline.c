#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "pipeline.h"
#include "config.h"
#include "data_types.h"
#include "data_records.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"

#include "ecg_inference.h"

static const char *TAG = "pipeline";

static QueueHandle_t s_inQ  = NULL;
static QueueHandle_t s_outQ = NULL;     // may be NULL if SD failed

// ------------------------------------------------------------
// Helper: generate a random float in [min, max]
// ------------------------------------------------------------
static inline float rand_float(float min, float max)
{
    return min + ((float)(esp_random() % 10000) / 10000.0f) * (max - min);
}

// ------------------------------------------------------------
// Helper: send a small record to SD queue (non-blocking).
// Allocates a heap copy; SD writer will free it after write.
// ------------------------------------------------------------
static void send_record_to_sd(const void *record, size_t len)
{
    if (!s_outQ) return;

    uint8_t *buf = (uint8_t *)malloc(len);
    if (!buf) {
        ESP_LOGW(TAG, "malloc failed for record (%u bytes)", (unsigned)len);
        return;
    }
    memcpy(buf, record, len);

    batch_buf_t bb = { .data = buf, .len = len };
    if (pdPASS != xQueueSend(s_outQ, &bb, 0)) {
        free(buf);
        ESP_LOGD(TAG, "SD queue full, dropped record");
    }
}

// ------------------------------------------------------------
// Pipeline task
// ------------------------------------------------------------
static void pipeline_task(void *arg)
{
    // Each ECG record on disk: 1 type byte + sizeof(pkt_sample_t)
    const size_t ecg_record_sz = 1 + sizeof(pkt_sample_t);

    uint8_t *bin = (uint8_t *)heap_caps_malloc(
        SAMPLES_PER_BATCH * ecg_record_sz, MALLOC_CAP_8BIT);

    if (!bin) {
        ESP_LOGE(TAG, "Failed to allocate initial batch buffer");
        vTaskDelete(NULL);
        return;
    }

    size_t filled = 0;

    pkt_sample_t accum;
    bool have_pending = false;

    uint32_t vitals_counter = 0;

    // ---- Write file header as first record ----
    if (s_outQ) {
        file_header_t hdr;
        memset(&hdr, 0, sizeof(hdr));
        hdr.magic              = FILE_MAGIC;
        hdr.version            = FILE_FORMAT_VERSION;
        hdr.ecg_rate_hz        = 250;
        hdr.ecg_channels       = ECG_CHANNELS;
        hdr.num_imu            = 2;
        hdr.num_disease_classes = NUM_DISEASE_CLASSES;

        send_record_to_sd(&hdr, sizeof(hdr));
        ESP_LOGI(TAG, "File header sent to SD (%u bytes)", (unsigned)sizeof(hdr));
    }

    // ---- Main loop ----
    for (;;) {
        pkt_sample_t s;
        if (!xQueueReceive(s_inQ, &s, portMAX_DELAY))
            continue;

        // ---- Feed 500 Hz sample to inference engine ----
        if (ecg_inference_is_ready()) {
            ecg_inference_push_sample_500hz(&s);
        }

        // ---- Check for completed prediction ----
        prediction_result_t pred;
        if (ecg_inference_get_prediction(&pred)) {
            prediction_record_t pr;
            pr.type         = RECORD_TYPE_PREDICTION;
            pr.timestamp_ms = pred.timestamp_ms;
            pr.window_index = pred.window_index;
            pr.inference_ms = pred.inference_ms;
            memcpy(pr.probs, pred.probs, sizeof(pr.probs));

            send_record_to_sd(&pr, sizeof(pr));
            ESP_LOGI(TAG, "Prediction record sent (window %lu)",
                     (unsigned long)pred.window_index);
        }

        // ---- Decimate 500 Hz → 250 Hz ----
        pkt_sample_t out;

#if DECIMATE_AVG
        if (!have_pending) {
            accum = s;
            have_pending = true;
            continue;
        }

        out = s;
        out.timestamp = s.timestamp;

        for (int i = 0; i < ECG_CHANNELS; i++)
            out.ecg[i] = 0.5f * (accum.ecg[i] + s.ecg[i]);

        out.imu1.accel_x = 0.5f * (accum.imu1.accel_x + s.imu1.accel_x);
        out.imu1.accel_y = 0.5f * (accum.imu1.accel_y + s.imu1.accel_y);
        out.imu1.accel_z = 0.5f * (accum.imu1.accel_z + s.imu1.accel_z);
        out.imu1.gyro_x  = 0.5f * (accum.imu1.gyro_x  + s.imu1.gyro_x);
        out.imu1.gyro_y  = 0.5f * (accum.imu1.gyro_y  + s.imu1.gyro_y);
        out.imu1.gyro_z  = 0.5f * (accum.imu1.gyro_z  + s.imu1.gyro_z);

        out.imu2.accel_x = 0.5f * (accum.imu2.accel_x + s.imu2.accel_x);
        out.imu2.accel_y = 0.5f * (accum.imu2.accel_y + s.imu2.accel_y);
        out.imu2.accel_z = 0.5f * (accum.imu2.accel_z + s.imu2.accel_z);
        out.imu2.gyro_x  = 0.5f * (accum.imu2.gyro_x  + s.imu2.gyro_x);
        out.imu2.gyro_y  = 0.5f * (accum.imu2.gyro_y  + s.imu2.gyro_y);
        out.imu2.gyro_z  = 0.5f * (accum.imu2.gyro_z  + s.imu2.gyro_z);

        out.temp = 0.5f * (accum.temp + s.temp);
        have_pending = false;
#else
        static bool skip = false;
        if (skip) { skip = false; continue; }
        skip = true;
        out = s;
#endif

        // ---- Write ECG record into batch buffer ----
        bin[filled * ecg_record_sz] = RECORD_TYPE_ECG;
        memcpy(&bin[filled * ecg_record_sz + 1], &out, sizeof(pkt_sample_t));
        filled++;

        // ---- Vital signs (1 Hz placeholder) ----
        vitals_counter++;
        if (vitals_counter >= VITALS_INTERVAL_SAMPLES) {
            vitals_record_t vr;
            vr.type            = RECORD_TYPE_VITALS;
            vr.timestamp_ms    = out.timestamp;
            vr.heart_rate_bpm  = rand_float(60.0f, 100.0f);
            vr.hrv_sdnn_ms     = rand_float(20.0f, 80.0f);
            vr.resp_rate_bpm   = rand_float(12.0f, 20.0f);
            vr.temperature_c   = out.temp;   // real temperature

            send_record_to_sd(&vr, sizeof(vr));
            vitals_counter = 0;
        }

        // ---- Dispatch batch when full ----
        if (filled >= SAMPLES_PER_BATCH) {
            batch_buf_t bb = {
                .data = bin,
                .len  = filled * ecg_record_sz
            };

            if (s_outQ) {
                if (pdPASS != xQueueSend(s_outQ, &bb, pdMS_TO_TICKS(10))) {
                    ESP_LOGW(TAG, "SD queue full, discarding batch");
                    free(bb.data);
                }
            } else {
                free(bb.data);
            }

            bin = (uint8_t *)heap_caps_malloc(
                SAMPLES_PER_BATCH * ecg_record_sz, MALLOC_CAP_8BIT);

            if (!bin) {
                ESP_LOGE(TAG, "Batch buffer alloc failed — task exiting");
                vTaskDelete(NULL);
                return;
            }

            filled = 0;
        }
    }
}

void pipeline_start(QueueHandle_t in_queue, QueueHandle_t out_queue)
{
    s_inQ  = in_queue;
    s_outQ = out_queue;

    xTaskCreatePinnedToCore(
        pipeline_task,
        "pipeline",
        8192,                       // increased for inference call chain
        NULL,
        configMAX_PRIORITIES - 6,
        NULL,
        1                           // Core 1
    );
}