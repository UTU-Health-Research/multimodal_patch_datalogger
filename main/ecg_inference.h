// ecg_inference.h
#pragma once

#include "data_types.h"
#include "data_records.h"

#ifdef __cplusplus
extern "C" {
#endif

int  ecg_inference_init(void);
int  ecg_inference_is_ready(void);
void ecg_inference_push_sample_500hz(const pkt_sample_t *sample);
int  ecg_inference_get_prediction(prediction_result_t *out);

#ifdef __cplusplus
}
#endif