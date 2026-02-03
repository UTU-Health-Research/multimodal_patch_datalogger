// src/pipeline.c
#include <string.h>
#include "pipeline.h"
#include "config.h"
#include "data_types.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "serial_stream.h"

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
        pkt_sample_t out;

        if (!xQueueReceive(s_inQ, &s, portMAX_DELAY)) continue;

        // Decimate from 500 Hz to 250 Hz
#if DECIMATE_AVG
        if (!have_pending) {
            accum = s;
            have_pending = true;
            continue;
        } else {
            // average accum and s
            out = s;
            out.timestamp = s.timestamp; // last timestamp
            for (int i = 0; i < ECG_CHANNELS; ++i) {
                out.ecg[i] = 0.5f * (accum.ecg[i] + s.ecg[i]);
            }

            // Average IMU1 data
            out.imu1.accel_x = 0.5f * (accum.imu1.accel_x + s.imu1.accel_x);
            out.imu1.accel_y = 0.5f * (accum.imu1.accel_y + s.imu1.accel_y);
            out.imu1.accel_z = 0.5f * (accum.imu1.accel_z + s.imu1.accel_z);
            out.imu1.gyro_x = 0.5f * (accum.imu1.gyro_x + s.imu1.gyro_x);
            out.imu1.gyro_y = 0.5f * (accum.imu1.gyro_y + s.imu1.gyro_y);
            out.imu1.gyro_z = 0.5f * (accum.imu1.gyro_z + s.imu1.gyro_z);

            // Average IMU2 data
            out.imu2.accel_x = 0.5f * (accum.imu2.accel_x + s.imu2.accel_x);
            out.imu2.accel_y = 0.5f * (accum.imu2.accel_y + s.imu2.accel_y);
            out.imu2.accel_z = 0.5f * (accum.imu2.accel_z + s.imu2.accel_z);
            out.imu2.gyro_x = 0.5f * (accum.imu2.gyro_x + s.imu2.gyro_x);
            out.imu2.gyro_y = 0.5f * (accum.imu2.gyro_y + s.imu2.gyro_y);
            out.imu2.gyro_z = 0.5f * (accum.imu2.gyro_z + s.imu2.gyro_z);

            // Average temperature data
            out.temp = 0.5f * (accum.temp + s.temp);

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
    static uint8_t sample_counter = 0;

/*
        SEND SELECTED DATA OVER SERIAL PORT FOR TRIAGE PURPOSES
*/

    // After processing each sample at 250Hz
    sample_counter++;
    if (sample_counter >= 3) {  // Every other sample (125Hz)
        // Create a buffer to hold the specific values
    float data_to_send[9];

    // Fill the buffer with the values you want to send
    data_to_send[0] = out.ecg[1];
    data_to_send[1] = out.ecg[2];
    data_to_send[2] = out.ecg[3];
    data_to_send[3] = out.ecg[7];
    data_to_send[4] = out.imu1.accel_z;
    data_to_send[5] = out.imu1.gyro_y;  
    data_to_send[6] = out.imu2.accel_z;
    data_to_send[7] = out.imu2.gyro_y;  
    data_to_send[8] = out.temp;

// Log the buffer as hexadecimal values
ESP_LOG_BUFFER_HEX("DATA", data_to_send, sizeof(data_to_send));
        //serial_streamer_send_data(&out, sizeof(out));
        vTaskDelay(1);
        sample_counter = 0;
    }    
    }
}

void pipeline_start(QueueHandle_t in_queue, QueueHandle_t out_queue) {
    s_inQ = in_queue;
    s_outQ = out_queue;
    xTaskCreatePinnedToCore(pipeline_task, "pipeline", 4096, NULL, configMAX_PRIORITIES - 6, NULL, 1);
}
