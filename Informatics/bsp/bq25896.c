#include "bq25896.h"

bool bq25896_init(bq25896_t * const me, I2C_HandleTypeDef *hi2c)
{
    if (me == NULL || hi2c == NULL) {
        return false;
    }

    me->hi2c = hi2c;
    me->i2c_addr = BQ25896_I2C_ADDR_8BIT;

    // Test de présence sur le bus I2C
    if (HAL_I2C_IsDeviceReady(me->hi2c, me->i2c_addr, 3, 50) != HAL_OK) {
        return false;
    }

    // Vérification du Device Part Number dans REG14 (PN[2:0] doit être 000 pour BQ25896)
    uint8_t part_info = 0;
    if (!bq25896_read_reg(me, BQ25896_REG_PART_REV, &part_info)) {
        return false;
    }

    uint8_t pn = (part_info >> 3) & 0x07;
    if (pn != 0) { // 000b = bq25896
        return false;
    }

    return true;
}

bool bq25896_read_reg(bq25896_t * const me, uint8_t reg, uint8_t *val)
{
    if (me == NULL || me->hi2c == NULL || val == NULL) return false;
    return (HAL_I2C_Mem_Read(me->hi2c, me->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100) == HAL_OK);
}

bool bq25896_write_reg(bq25896_t * const me, uint8_t reg, uint8_t val)
{
    if (me == NULL || me->hi2c == NULL) return false;
    return (HAL_I2C_Mem_Write(me->hi2c, me->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

bool bq25896_update_bits(bq25896_t * const me, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t current = 0;
    if (!bq25896_read_reg(me, reg, &current)) {
        return false;
    }
    uint8_t updated = (current & ~mask) | (val & mask);
    if (updated == current) {
        return true; // Déjà à la bonne valeur
    }
    return bq25896_write_reg(me, reg, updated);
}

bool bq25896_set_watchdog(bq25896_t * const me, bq25896_wdt_t wdt)
{
    return bq25896_update_bits(me, BQ25896_REG_TIMER_CFG, 0x30, (uint8_t)((wdt & 0x03) << 4));
}

bool bq25896_reset_watchdog_timer(bq25896_t * const me)
{
    return bq25896_update_bits(me, BQ25896_REG_PWR_CFG, (1 << 6), (1 << 6));
}

bool bq25896_enable_boost(bq25896_t * const me, bool enable)
{
    uint8_t val = enable ? (1 << 5) : 0;
    return bq25896_update_bits(me, BQ25896_REG_PWR_CFG, (1 << 5), val);
}

bool bq25896_set_boost_voltage(bq25896_t * const me, uint16_t mv)
{
    if (mv < 4550) mv = 4550;
    if (mv > 5510) mv = 5510;

    uint8_t code = (uint8_t)((mv - 4550) / 64);
    if (code > 0x0F) code = 0x0F;

    return bq25896_update_bits(me, BQ25896_REG_BOOST_CFG, 0xF0, (uint8_t)(code << 4));
}

bool bq25896_set_boost_current_limit(bq25896_t * const me, uint8_t code)
{
    if (code > 0x06) code = 0x06; // 000: 0.5A, 001: 0.75A, 010: 1.2A, 011: 1.4A, 100: 1.65A, 101: 1.875A, 110: 2.15A
    return bq25896_update_bits(me, BQ25896_REG_BOOST_CFG, 0x07, code);
}

bool bq25896_disable_boost_thermal_protection(bq25896_t * const me)
{
    // REG01 bits 7:6 BHOT[1:0] = 11 (Disable boost mode thermal protection)
    return bq25896_update_bits(me, BQ25896_REG_VINDPM_OS, 0xC0, 0xC0);
}

bool bq25896_enable_charge(bq25896_t * const me, bool enable)
{
    uint8_t val = enable ? (1 << 4) : 0;
    return bq25896_update_bits(me, BQ25896_REG_PWR_CFG, (1 << 4), val);
}

bool bq25896_set_charge_current(bq25896_t * const me, uint16_t current_ma)
{
    if (current_ma > 3008) current_ma = 3008;
    uint8_t code = (uint8_t)(current_ma / 64);
    return bq25896_update_bits(me, BQ25896_REG_ICHG, 0x7F, code & 0x7F);
}

bool bq25896_set_input_current_limit(bq25896_t * const me, uint16_t limit_ma)
{
    if (limit_ma < 100) limit_ma = 100;
    if (limit_ma > 3250) limit_ma = 3250;
    uint8_t code = (uint8_t)((limit_ma - 100) / 50);
    return bq25896_update_bits(me, BQ25896_REG_IINLIM, 0x3F, code & 0x3F);
}

bool bq25896_enable_adc(bq25896_t * const me, bool continuous)
{
    uint8_t val = continuous ? (1 << 6) : 0;
    return bq25896_update_bits(me, BQ25896_REG_ADC_CFG, (1 << 6), val);
}

bool bq25896_read_vbat_mv(bq25896_t * const me, uint16_t *vbat_mv)
{
    if (vbat_mv == NULL) return false;
    uint8_t reg = 0;
    if (!bq25896_read_reg(me, BQ25896_REG_BATV, &reg)) return false;
    *vbat_mv = (uint16_t)(2304 + ((reg & 0x7F) * 20));
    return true;
}

bool bq25896_read_vbus_mv(bq25896_t * const me, uint16_t *vbus_mv)
{
    if (vbus_mv == NULL) return false;
    uint8_t reg = 0;
    if (!bq25896_read_reg(me, BQ25896_REG_VBUSV, &reg)) return false;
    *vbus_mv = (uint16_t)(2600 + ((reg & 0x7F) * 100));
    return true;
}

bool bq25896_read_vsys_mv(bq25896_t * const me, uint16_t *vsys_mv)
{
    if (vsys_mv == NULL) return false;
    uint8_t reg = 0;
    if (!bq25896_read_reg(me, BQ25896_REG_SYSV, &reg)) return false;
    *vsys_mv = (uint16_t)(2304 + ((reg & 0x7F) * 20));
    return true;
}

bool bq25896_read_status(bq25896_t * const me, bq25896_status_t *status)
{
    if (status == NULL) return false;
    uint8_t reg = 0;
    if (!bq25896_read_reg(me, BQ25896_REG_SYS_STATUS, &reg)) return false;

    status->vbus_stat       = (bq25896_vbus_stat_t)((reg >> 5) & 0x07);
    status->chrg_stat       = (bq25896_chrg_stat_t)((reg >> 3) & 0x03);
    status->power_good      = (bool)((reg >> 2) & 0x01);
    status->vsys_in_min_reg = (bool)(reg & 0x01);

    return true;
}

bool bq25896_read_faults(bq25896_t * const me, bq25896_faults_t *faults)
{
    if (faults == NULL) return false;
    uint8_t reg = 0;

    // Double lecture obligatoire de REG0C selon la datasheet (Section 9.2.9.3)
    // 1ere lecture lit le statut précédent, 2eme lecture lit le statut courant.
    bq25896_read_reg(me, BQ25896_REG_FAULT, &reg);
    if (!bq25896_read_reg(me, BQ25896_REG_FAULT, &reg)) return false;

    faults->raw            = reg;
    faults->watchdog_fault = (bool)((reg >> 7) & 0x01);
    faults->boost_fault    = (bool)((reg >> 6) & 0x01);
    faults->chrg_fault     = (uint8_t)((reg >> 4) & 0x03);
    faults->bat_ovp_fault  = (bool)((reg >> 3) & 0x01);
    faults->ntc_fault      = (uint8_t)(reg & 0x07);

    return true;
}

bool bq25896_enter_ship_mode(bq25896_t * const me)
{
    // REG09 : bit 5 = BATFET_DIS (1: force turn off), bit 3 = BATFET_DLY (0: immediate)
    uint8_t reg09 = 0;
    bq25896_read_reg(me, BQ25896_REG_BATFET_CFG, &reg09);
    reg09 &= ~(1 << 3); // BATFET_DLY = 0 (extinction immédiate)
    reg09 |= (1 << 5);  // BATFET_DIS = 1 (coupe le BATFET, bascule en Ship Mode)
    return bq25896_write_reg(me, BQ25896_REG_BATFET_CFG, reg09);
}
