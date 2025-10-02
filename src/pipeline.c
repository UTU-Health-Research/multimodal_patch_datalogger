// src/pipeline.c
#include <string.h>
#include "pipeline.h"
#include "config.h"
#include "data_types.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "pipeline";

static QueueHandle_t s_inQ = NULL;
static QueueHandle_t s_outQ = NULL;

static void pipeline_task(void *arg) {
    // Batch buffer holds SAMPLES_PER_BATCH ecg_sample_t in chosen format
#if LOG_BINARY
    const size_t sample_sz = sizeof(pkt_sample_t);
#else
    // For CSV batching, we build lines into a text buffer later; here we keep binary then format on flush.
    const size_t sample_sz = sizeof(pkt_sample_t);
#endif
    pkt_sample_t accum;     // for averaging pairs
    bool have_pending = false;

    // Allocate one binary batch workspace; on dispatch, allocate a fresh block to hand over.
    uint8_t *bin = (uint8_t *)heap_caps_malloc(SAMPLES_PER_BATCH * sample_sz, MALLOC_CAP_8BIT);
    size_t filled = 0;

    for (;;) {
        pkt_sample_t s;
        if (!xQueueReceive(s_inQ, &s, portMAX_DELAY)) continue;

        // Decimate from 500 Hz to 250 Hz
#if DECIMATE_AVG
        if (!have_pending) {
            accum = s;
            have_pending = true;
            continue;
        } else {
            // average accum and s
            pkt_sample_t out = s;
            out.timestamp = s.timestamp; // last timestamp
            for (int i = 0; i < ECG_CHANNELS; ++i) {
                out.ecg[i] = 0.5f * (accum.ecg[i] + s.ecg[i]);
            }

            // Average IMU1 data
            out.imu1.accel_x = (int16_t)(0.5f * (accum.imu1.accel_x + s.imu1.accel_x));
            out.imu1.accel_y = (int16_t)(0.5f * (accum.imu1.accel_y + s.imu1.accel_y));
            out.imu1.accel_z = (int16_t)(0.5f * (accum.imu1.accel_z + s.imu1.accel_z));
            out.imu1.gyro_x = (int16_t)(0.5f * (accum.imu1.gyro_x + s.imu1.gyro_x));
            out.imu1.gyro_y = (int16_t)(0.5f * (accum.imu1.gyro_y + s.imu1.gyro_y));
            out.imu1.gyro_z = (int16_t)(0.5f * (accum.imu1.gyro_z + s.imu1.gyro_z));
            
            // Average IMU2 data
            out.imu2.accel_x = (int16_t)(0.5f * (accum.imu2.accel_x + s.imu2.accel_x));
            out.imu2.accel_y = (int16_t)(0.5f * (accum.imu2.accel_y + s.imu2.accel_y));
            out.imu2.accel_z = (int16_t)(0.5f * (accum.imu2.accel_z + s.imu2.accel_z));
            out.imu2.gyro_x = (int16_t)(0.5f * (accum.imu2.gyro_x + s.imu2.gyro_x));
            out.imu2.gyro_y = (int16_t)(0.5f * (accum.imu2.gyro_y + s.imu2.gyro_y));
            out.imu2.gyro_z = (int16_t)(0.5f * (accum.imu2.gyro_z + s.imu2.gyro_z));

            // Average temperature data
            out.temperature = 0.5f * (accum.temperature + s.temperature);
            
            have_pending = false;

            memcpy(&bin[filled * sample_sz], &out, sample_sz);
            filled++;
        }
#else
        static bool skip = false;
        if (skip) { skip = false; continue; }
        skip = true;

        memcpy(&bin[filled * sample_sz], &s, sample_sz);
        filled++;
#endif

        // Dispatch when batch full
        if (filled >= SAMPLES_PER_BATCH) {
#if LOG_BINARY
            batch_buf_t bb = {
                .data = bin,
                .len = filled * sample_sz
            };
#else
            // Convert binary samples to CSV text in a new buffer
            // Rough estimate: each line ~ (10 + 8*12 + commas) < 140 bytes
            size_t text_cap = SAMPLES_PER_BATCH * 160;
            char *txt = (char *)heap_caps_malloc(text_cap, MALLOC_CAP_8BIT);
            size_t tfill = 0;
            for (size_t i = 0; i < filled; ++i) {
                ecg_sample_t *ps = (ecg_sample_t *)&bin[i * sample_sz];
                int n = snprintf(&txt[tfill], text_cap - tfill,
                                 "%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                                 ps->t_ms,
                                 ps->ch[0], ps->ch[1], ps->ch[2], ps->ch[3],
                                 ps->ch[4], ps->ch[5], ps->ch[6], ps->ch[7]);
                if (n < 0 || (size_t)n >= (text_cap - tfill)) break;
                tfill += (size_t)n;
            }
            batch_buf_t bb = { .data = (uint8_t*)txt, .len = tfill };
            // recycle bin for next batch (keep the same binary workspace)
#endif
            // Print first few bytes of the batch buffer for inspection
            //ESP_LOG_BUFFER_HEX(TAG, bb.data, bb.len < 32 ? bb.len : 32);
            // Send to SD writer
            xQueueSend(s_outQ, &bb, portMAX_DELAY);

            // Allocate a fresh binary buffer for next fill
            bin = (uint8_t *)heap_caps_malloc(SAMPLES_PER_BATCH * sample_sz, MALLOC_CAP_8BIT);
            filled = 0;
        }
    }
}

void pipeline_start(QueueHandle_t in_queue, QueueHandle_t out_queue) {
    s_inQ = in_queue;
    s_outQ = out_queue;
    xTaskCreatePinnedToCore(pipeline_task, "pipeline", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 1);
}
