#include "ina219.h"

#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_V       0x01
#define INA219_REG_BUS_V         0x02
// We don’t use CURRENT/POWER regs to avoid calibration complexity.
// Current is derived from shunt voltage / Rshunt for simplicity.

static bool write16(INA219_t* d, uint8_t reg, uint16_t val) {
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Mem_Write(d->hi2c, (d->addr_7b << 1), reg, I2C_MEMADD_SIZE_8BIT,
                             buf, 2, 10) == HAL_OK;
}
static bool read16(INA219_t* d, uint8_t reg, uint16_t* out) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(d->hi2c, (d->addr_7b << 1), reg, I2C_MEMADD_SIZE_8BIT,
                         buf, 2, 10) != HAL_OK) return false;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool INA219_Init(INA219_t* dev, I2C_HandleTypeDef* hi2c, uint8_t addr_7b, float r_shunt_ohm) {
    if (!dev || !hi2c || r_shunt_ohm <= 0.0f) return false;
    dev->hi2c = hi2c;
    dev->addr_7b = addr_7b;
    dev->r_shunt_ohm = r_shunt_ohm;

    // Reset then set a sensible default config:
    // BRNG=32V, PG=±320mV, BADC=12-bit, SADC=12-bit, Mode=Shunt+Bus, Continuous
    // Datasheet default config template:
    // [15]RST=0 | BRNG=1(32V) | PG=11(320mV) | BADC=0011(12b) | SADC=0011(12b) | MODE=111
    uint16_t cfg = 0;
    // Optional soft reset: write RST bit = 1 (device resets to default)
    // write16(dev, INA219_REG_CONFIG, 0x8000);

    cfg = (1u << 13)             // BRNG=1 -> 32V bus range
        | (3u << 11)             // PG=3 -> ±320mV shunt range
        | (0x3u << 7)            // BADC=3 -> 12-bit
        | (0x3u << 3)            // SADC=3 -> 12-bit
        | (0x7u);                // MODE=7 -> Shunt+Bus, continuous
    return write16(dev, INA219_REG_CONFIG, cfg);
}

bool INA219_ReadBusVoltage_V(INA219_t* dev, float* v_bus) {
    if (!dev || !v_bus) return false;
    uint16_t raw;
    if (!read16(dev, INA219_REG_BUS_V, &raw)) return false;
    // Bus Voltage Register:
    // bits [15..3] = value, LSB = 4 mV, bits [2..0] flags
    raw >>= 3;
    *v_bus = (float)raw * 0.004f;  // 4 mV per LSB
    return true;
}

bool INA219_ReadShunt_mV(INA219_t* dev, float* v_shunt_mV) {
    if (!dev || !v_shunt_mV) return false;
    uint16_t raw;
    if (!read16(dev, INA219_REG_SHUNT_V, &raw)) return false;
    // Shunt Voltage Register: signed, LSB = 10 µV
    int16_t sraw = (int16_t)raw;
    *v_shunt_mV = (float)sraw * 0.01f;  // 10 µV -> 0.01 mV per LSB
    return true;
}

bool INA219_ReadCurrent_A(INA219_t* dev, float* i_A) {
    if (!dev || !i_A) return false;
    float mv;
    if (!INA219_ReadShunt_mV(dev, &mv)) return false;
    // I = V/R ; mv -> V: *1e-3
    *i_A = (mv * 1e-3f) / dev->r_shunt_ohm;
    return true;
}

bool INA219_ReadVI(INA219_t* dev, float* v_bus, float* i_A) {
    bool ok1 = INA219_ReadBusVoltage_V(dev, v_bus);
    bool ok2 = INA219_ReadCurrent_A(dev, i_A);
    return ok1 && ok2;
}
