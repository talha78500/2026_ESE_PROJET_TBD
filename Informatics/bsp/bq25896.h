#ifndef BSP_BQ25896_H
#define BSP_BQ25896_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

/**
 * @file bq25896.h
 * @brief Couche BSP (Board Support Package) pour le chargeur / régulateur BQ25896.
 *        Seul ce module gère les registres et le bus I2C vers le BQ25896.
 */

#define BQ25896_I2C_ADDR_7BIT       0x6B
#define BQ25896_I2C_ADDR_8BIT       (BQ25896_I2C_ADDR_7BIT << 1) // 0xD6

/* Map des Registres BQ25896 */
#define BQ25896_REG_IINLIM          0x00 // Input Current Limit & EN_HIZ, EN_ILIM
#define BQ25896_REG_VINDPM_OS       0x01 // BHOT, BCOLD, VINDPM_OS
#define BQ25896_REG_ADC_CFG         0x02 // CONV_START, CONV_RATE, BOOST_FREQ, ICO_EN
#define BQ25896_REG_PWR_CFG         0x03 // WD_RST, OTG_CONFIG, CHG_CONFIG, SYS_MIN
#define BQ25896_REG_ICHG            0x04 // Fast charge current
#define BQ25896_REG_PRE_TERM        0x05 // Precharge & termination current
#define BQ25896_REG_VREG            0x06 // Charge voltage limit & BATLOWV
#define BQ25896_REG_TIMER_CFG       0x07 // WATCHDOG, EN_TIMER, CHG_TIMER
#define BQ25896_REG_IR_THERM        0x08 // IR Compensation & Thermal regulation
#define BQ25896_REG_BATFET_CFG      0x09 // BATFET control & Reset
#define BQ25896_REG_BOOST_CFG       0x0A // BOOSTV & BOOST_LIM
#define BQ25896_REG_SYS_STATUS      0x0B // VBUS_STAT, CHRG_STAT, PG_STAT, VSYS_STAT
#define BQ25896_REG_FAULT           0x0C // Faults (WATCHDOG, BOOST, CHRG, BAT, NTC)
#define BQ25896_REG_VINDPM          0x0D // VINDPM threshold
#define BQ25896_REG_BATV            0x0E // ADC VBAT
#define BQ25896_REG_SYSV            0x0F // ADC VSYS
#define BQ25896_REG_TSPCT           0x10 // ADC TS ratio
#define BQ25896_REG_VBUSV           0x11 // ADC VBUS
#define BQ25896_REG_ICHGR           0x12 // ADC Charge Current
#define BQ25896_REG_PART_REV        0x14 // Part number & Dev Revision

/* Énumérations de statut */
typedef enum {
    BQ25896_VBUS_NO_INPUT  = 0,
    BQ25896_VBUS_USB_SDP   = 1,
    BQ25896_VBUS_ADAPTER   = 2,
    BQ25896_VBUS_OTG_BOOST = 7
} bq25896_vbus_stat_t;

typedef enum {
    BQ25896_CHRG_DISABLED   = 0,
    BQ25896_CHRG_PRECHARGE  = 1,
    BQ25896_CHRG_FAST_CHG   = 2,
    BQ25896_CHRG_DONE       = 3
} bq25896_chrg_stat_t;

typedef enum {
    BQ25896_WDT_DISABLED = 0,
    BQ25896_WDT_40S      = 1,
    BQ25896_WDT_80S      = 2,
    BQ25896_WDT_160S     = 3
} bq25896_wdt_t;

/* Structures de données */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
} bq25896_t;

typedef struct {
    bq25896_vbus_stat_t vbus_stat;
    bq25896_chrg_stat_t chrg_stat;
    bool                power_good;
    bool                vsys_in_min_reg;
} bq25896_status_t;

typedef struct {
    bool    watchdog_fault;
    bool    boost_fault;
    uint8_t chrg_fault; // 00: Normal, 01: Input fault, 10: Thermal shutdown, 11: Safety timer
    bool    bat_ovp_fault;
    uint8_t ntc_fault;  // 000: Normal, 010: Warm, 011: Cool, 101: Cold, 110: Hot
    uint8_t raw;
} bq25896_faults_t;

/* Initialisation */
bool bq25896_init(bq25896_t * const me, I2C_HandleTypeDef *hi2c);

/* Contrôle Watchdog */
bool bq25896_set_watchdog(bq25896_t * const me, bq25896_wdt_t wdt);
bool bq25896_reset_watchdog_timer(bq25896_t * const me);

/* Contrôle Boost / OTG */
bool bq25896_enable_boost(bq25896_t * const me, bool enable);
bool bq25896_set_boost_voltage(bq25896_t * const me, uint16_t mv); // 4550 à 5510 mV
bool bq25896_set_boost_current_limit(bq25896_t * const me, uint8_t code); // 0=500mA .. 3=1400mA
bool bq25896_disable_boost_thermal_protection(bq25896_t * const me);

/* Contrôle Charge (Buck) */
bool bq25896_enable_charge(bq25896_t * const me, bool enable);
bool bq25896_set_charge_current(bq25896_t * const me, uint16_t current_ma); // pas 64mA, max 3008mA
bool bq25896_set_input_current_limit(bq25896_t * const me, uint16_t limit_ma); // 100 à 3250 mA

/* Monitoring & ADC */
bool bq25896_enable_adc(bq25896_t * const me, bool continuous);
bool bq25896_read_vbat_mv(bq25896_t * const me, uint16_t *vbat_mv);
bool bq25896_read_vbus_mv(bq25896_t * const me, uint16_t *vbus_mv);
bool bq25896_read_vsys_mv(bq25896_t * const me, uint16_t *vsys_mv);
bool bq25896_read_status(bq25896_t * const me, bq25896_status_t *status);
bool bq25896_read_faults(bq25896_t * const me, bq25896_faults_t *faults);

/* Contrôle Alimentation / Ship Mode */
bool bq25896_enter_ship_mode(bq25896_t * const me);

/* Lecture / Écriture générique */
bool bq25896_read_reg(bq25896_t * const me, uint8_t reg, uint8_t *val);
bool bq25896_write_reg(bq25896_t * const me, uint8_t reg, uint8_t val);
bool bq25896_update_bits(bq25896_t * const me, uint8_t reg, uint8_t mask, uint8_t val);

#endif /* BSP_BQ25896_H */
