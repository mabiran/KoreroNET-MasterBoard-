#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* Integrator API */
void  BatteryFlow_Reset(float initial_mAh);     // sets accumulator to initial_mAh
void  BatteryFlow_Update(float current_A, float dt_s); // integrate A over dt_s
float BatteryFlow_Get_mAh(void);

/* Capacity + SoC helpers (coulomb counting) */
void  BatteryFlow_SetCapacity_mAh(float cap_mAh);   // default from pins_config
float BatteryFlow_GetCapacity_mAh(void);
float BatteryFlow_SoC_Percent(void);               // 0..100 based on mAh & capacity

/* Full-mark flag (set when charging @ ≥14.2 V for N seconds) */
void  BatteryFlow_MarkFull(void);                  // sets mAh=0 and marks full
int   BatteryFlow_IsFullMarked(void);              // non-zero if full flag set

#ifdef __cplusplus
}
#endif
