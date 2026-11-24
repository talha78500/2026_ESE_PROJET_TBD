#ifndef BSP_BQ27220_H
#define BSP_BQ27220_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"
#include <stdio.h>
/**
 * @file bq27220.h
 * @brief Couche BSP (Board Support Package) pour la jauge de batterie (Fuel Gauge) TI BQ27220.
 *        Gère les commandes I2C standards (Standard Commands) sur 16 bits (Little-Endian).
 */

#define BQ27220_I2C_ADDR_7BIT           0x55
#define BQ27220_I2C_ADDR_8BIT           (BQ27220_I2C_ADDR_7BIT << 1) // 0xAA

/* Commandes I2C standards (Standard Commands) du BQ27220 */
#define BQ27220_CMD_CONTROL             0x00
#define BQ27220_CMD_TEMP                0x06
#define BQ27220_CMD_VOLTAGE             0x08
#define BQ27220_CMD_BATTERY_STATUS      0x0A
#define BQ27220_CMD_CURRENT             0x0C
#define BQ27220_CMD_REMAINING_CAPACITY  0x10
#define BQ27220_CMD_FULL_CHARGE_CAP     0x12
#define BQ27220_CMD_AVG_CURRENT         0x14
#define BQ27220_CMD_STATE_OF_CHARGE     0x2C
#define BQ27220_CMD_STATE_OF_HEALTH     0x2E
#define BQ27220_CMD_CYCLE_COUNT         0x2A

/* Sous-commandes de contrôle Control() */
#define BQ27220_SUBCMD_CONTROL_STATUS   0x0000
#define BQ27220_SUBCMD_DEVICE_NUMBER    0x0001
#define BQ27220_SUBCMD_FW_VERSION       0x0002
#define BQ27220_SUBCMD_SOFT_RESET       0x0012

/* Structure de l'instance du BQ27220 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
} bq27220_t;

/* Initialisation */
bool bq27220_init(bq27220_t * const me, I2C_HandleTypeDef *hi2c);

/* Lecture des mesures analogiques directes */
bool bq27220_read_voltage_mv(bq27220_t * const me, uint16_t *voltage_mv);
bool bq27220_read_current_ma(bq27220_t * const me, int16_t *current_ma);
bool bq27220_read_avg_current_ma(bq27220_t * const me, int16_t *avg_current_ma);
bool bq27220_read_temperature_c(bq27220_t * const me, float *temp_celsius);

/* Lecture des grandeurs calculées par l'algorithme CEDV */
bool bq27220_read_soc_pct(bq27220_t * const me, uint16_t *soc_pct);
bool bq27220_read_soh_pct(bq27220_t * const me, uint16_t *soh_pct);
bool bq27220_read_remaining_capacity_mah(bq27220_t * const me, uint16_t *rem_cap_mah);
bool bq27220_read_full_capacity_mah(bq27220_t * const me, uint16_t *full_cap_mah);
bool bq27220_read_cycle_count(bq27220_t * const me, uint16_t *cycles);
bool bq27220_read_status(bq27220_t * const me, uint16_t *status);

/* Fonctions bas niveau */
bool bq27220_read_word(bq27220_t * const me, uint8_t cmd, uint16_t *val);
bool bq27220_write_subcmd(bq27220_t * const me, uint16_t subcmd);

#endif /* BSP_BQ27220_H */
