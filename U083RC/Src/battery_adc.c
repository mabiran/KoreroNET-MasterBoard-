#include "battery_adc.h"
#include "pins_config.h"

#include "stm32u0xx_hal.h"   /* or your device family header if different */
extern ADC_HandleTypeDef hadc1;  /* provided by CubeMX (MX_ADC1_Init) */

static inline uint16_t adc_read_once(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);             // 10 ms timeout
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

float Battery_ReadVoltage(uint16_t samples) {
    if (samples == 0) samples = 1;

    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; ++i) {
        sum += adc_read_once();
    }
    float raw = (float)sum / (float)samples;

    /* Convert raw ADC -> Vadc -> Vbattery */
    float v_adc = (raw / ADC_FULL_SCALE) * ADC_VREF_VOLT;
    float v_bat = v_adc * VBAT_DIVIDER_GAIN;  // undo the divider

    return v_bat;
}
