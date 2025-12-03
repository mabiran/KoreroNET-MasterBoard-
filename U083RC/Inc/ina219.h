#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include "stm32u0xx_hal.h"   // adjust family include if needed

// Default INA219 7-bit address (0x40..0x4F). HAL expects 8-bit left-shifted.
#define INA219_I2C_ADDR_7B_DEFAULT   (0x45)

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr_7b;        // 7-bit address (e.g., 0x45)
    float   r_shunt_ohm;    // e.g., 0.1f
} INA219_t;

// Init with your I2C, address, and shunt resistor value (ohms).
bool INA219_Init(INA219_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr_7b, float r_shunt_ohm);

// Read bus voltage in volts (battery/charger side).
bool INA219_ReadBusVoltage_V(INA219_t* dev, float* v_bus);

// Read shunt voltage in millivolts (signed).
bool INA219_ReadShunt_mV(INA219_t* dev, float* v_shunt_mV);

// Compute current in amperes from shunt voltage / R (signed).
// Positive/negative depends on shunt orientation. Flip sign if needed.
bool INA219_ReadCurrent_A(INA219_t* dev, float* i_A);

// Convenience: read both voltage and current at once.
bool INA219_ReadVI(INA219_t* dev, float* v_bus, float* i_A);

#ifdef __cplusplus
}
#endif
