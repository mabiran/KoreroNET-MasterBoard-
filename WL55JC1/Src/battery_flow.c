#include "battery_flow.h"
#include "pins_config.h"

static float g_accum_mAh = 0.0f;        // mAh since last "full" mark (discharge positive)
static float g_capacity_mAh = BATTERY_NOMINAL_mAh;
static int   g_full_marked = 0;

void BatteryFlow_Reset(float initial_mAh) { g_accum_mAh = initial_mAh; }
void BatteryFlow_SetCapacity_mAh(float cap_mAh) { if (cap_mAh > 0) g_capacity_mAh = cap_mAh; }
float BatteryFlow_GetCapacity_mAh(void) { return g_capacity_mAh; }

void BatteryFlow_Update(float current_A, float dt_s) {
    // By convention here: discharge current is positive; charge is negative.
    // mAh += A * s * 1000 / 3600
    g_accum_mAh += current_A * dt_s * (1000.0f / 3600.0f);
    if (g_accum_mAh < 0.0f) g_accum_mAh = 0.0f;   // clamp (can't be "more than full")
}

float BatteryFlow_Get_mAh(void) { return g_accum_mAh; }

float BatteryFlow_SoC_Percent(void) {
    if (g_capacity_mAh <= 0.0f) return -1.0f;
    float used = g_accum_mAh;
    float soc  = 100.0f * (1.0f - (used / g_capacity_mAh));
    if (soc < 0.0f) soc = 0.0f; if (soc > 100.0f) soc = 100.0f;
    return soc;
}

void BatteryFlow_MarkFull(void) {
    g_accum_mAh = 0.0f;
    g_full_marked = 1;
}

int BatteryFlow_IsFullMarked(void) { return g_full_marked; }
