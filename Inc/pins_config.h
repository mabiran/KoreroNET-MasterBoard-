#pragma once
#include "main.h"   // gives you *_Pin and *_GPIO_Port from Cube labels

/* --- ADC & divider parameters --- */
#define VBAT_RTOP_OHMS      (56000.0f)
#define VBAT_RBOT_OHMS      (14942.0f)
#define VBAT_DIVIDER_GAIN   ((VBAT_RTOP_OHMS + VBAT_RBOT_OHMS) / VBAT_RBOT_OHMS)  /* ~4.748 */
#define ADC_VREF_VOLT       (3.3f)
#define ADC_FULL_SCALE      (4095.0f)

/* --- LiFePO4 (4S) voltage thresholds (resting/light load) --- */
#define VBAT_MAX_CHARGE_V   (14.3f)
#define VBAT_FULL_REST_V    (13.4f)
#define VBAT_WARN_REST_V    (12.8f)
#define VBAT_CUTOFF_LOAD_V  (11.2f)

/* --- INA219 + battery --- */
#define INA219_I2C_ADDR_7B  (0x45)             /* change to 0x40 if needed */
#define INA219_ADDR         (INA219_I2C_ADDR_7B << 1)
#define INA_REG_CONFIG      (0x00)
#define INA_REG_SHUNT_V     (0x01)
#define INA_REG_BUS_V       (0x02)
#define SHUNT_OHMS          (0.1f)

/* --- Battery capacity & charge detection (moved from scattered #defines) --- */
#define BATTERY_NOMINAL_mAh          (12000.0f)  /* pack nominal capacity */
#define CHARGE_VOLTAGE_THRESHOLD_V   (14.1f)    /* voltage considered “full” */
#define CHARGE_NEGATIVE_CURRENT_A    (-0.05f)    /* charging if I <= this */
#define CHARGE_CONFIRM_MS            (5000u)     /* must hold for this long */


/* --- RPi wake timings --- */
#define RPI_WAKE_DELAY_MS   (60000u)
#define RPI_WAKE_PULSE_MS   (200u)

/* --- pin alias (map User Label “RPISwitch” in .ioc) --- */

#define RPI_WAKE_GPIO_Port   GPIOB
#define RPI_WAKE_Pin         GPIO_PIN_10
#define AM_CONFIG_Port   	 GPIOA    //Third wire (Black)
#define AM_CONFIG_Pin		 GPIO_PIN_8
#define AM_REC_Port    		 GPIOB		//Fourth wire (White)
#define AM_REC_Pin			 GPIO_PIN_3
