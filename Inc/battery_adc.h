// Core/Inc/battery_adc.h
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "pins_config.h"   // for VBAT_* thresholds

float Battery_ReadVoltage(uint16_t samples);

/* Estimate SoC (%) from resting voltage (rough LiFePO4 curve).
   Returns 0..100 (clamped). */
float Battery_SoC_FromVoltage(float vbat);

/* --- Simple voltage classifier --- */
typedef enum { VBAT_OK = 0, VBAT_WARN, VBAT_CUTOFF } vbat_state_t;

static inline vbat_state_t Battery_Classify(float vbat) {
    if (vbat <= VBAT_CUTOFF_LOAD_V) return VBAT_CUTOFF;
    if (vbat <  VBAT_WARN_REST_V)   return VBAT_WARN;
    return VBAT_OK;
}
#ifdef __cplusplus
}
#endif
