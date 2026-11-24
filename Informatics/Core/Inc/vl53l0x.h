#ifndef VL53L0X_H
#define VL53L0X_H


#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"



#define VL53L0X_DEFAULT_ADDRESS_8BIT  0x52
#define VL53L0X_DEFAULT_ADDRESS_7BIT  0x29
#define VL53L0X_DEFAULT_TIMEOUT_MS    500




typedef enum {
    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
    VL53L0X_VCSEL_PERIOD_FINAL_RANGE
} vl53l0x_vcsel_period_type_t;

typedef struct {
    uint16_t raw_distance_mm;
    uint16_t signal_cnt;
    uint16_t ambient_cnt;
    uint16_t spad_cnt;
    uint8_t  range_status;
} vl53l0x_stats_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr_8bit;
    uint16_t io_timeout_ms;
    bool is_timeout;
    uint32_t timeout_start_ms;
    uint8_t stop_variable;
    uint32_t measurement_timing_budget_us;
} vl53l0x_t;

bool VL53L0X_Init(vl53l0x_t *dev, I2C_HandleTypeDef *hi2c);
void VL53L0X_SetAddress(vl53l0x_t *dev, uint8_t new_8bit_addr);
uint8_t VL53L0X_GetAddress(const vl53l0x_t *dev);

bool VL53L0X_SetSignalRateLimit(vl53l0x_t *dev, float limit_mcps);
float VL53L0X_GetSignalRateLimit(vl53l0x_t *dev);

bool VL53L0X_SetMeasurementTimingBudget(vl53l0x_t *dev, uint32_t budget_us);
uint32_t VL53L0X_GetMeasurementTimingBudget(vl53l0x_t *dev);

bool VL53L0X_SetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type, uint8_t period_pclks);
uint8_t VL53L0X_GetVcselPulsePeriod(vl53l0x_t *dev, vl53l0x_vcsel_period_type_t type);

void VL53L0X_StartContinuous(vl53l0x_t *dev, uint32_t period_ms);
void VL53L0X_StopContinuous(vl53l0x_t *dev);
uint16_t VL53L0X_ReadDistanceContinuous(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats);
uint16_t VL53L0X_ReadDistanceSingle(vl53l0x_t *dev, vl53l0x_stats_t *extra_stats);

void VL53L0X_SetTimeout(vl53l0x_t *dev, uint16_t timeout_ms);
uint16_t VL53L0X_GetTimeout(const vl53l0x_t *dev);
bool VL53L0X_TimeoutOccurred(vl53l0x_t *dev);

#endif /* VL53L0X_H */
