#include "bq27220.h"

bool bq27220_init(bq27220_t * const me, I2C_HandleTypeDef *hi2c)
{
    if (me == NULL || hi2c == NULL) {
        return false;
    }

    me->hi2c = hi2c;
    me->i2c_addr = BQ27220_I2C_ADDR_8BIT;

    // Test de présence sur le bus I2C
    if (HAL_I2C_IsDeviceReady(me->hi2c, me->i2c_addr, 3, 50) != HAL_OK) {
        return false;
    }

    // Vérification de lecture d'une tension cohérente (> 500 mV)
    uint16_t v_mv = 0;
    if (!bq27220_read_voltage_mv(me, &v_mv)) {
        return false;
    }

    return true;
}

bool bq27220_read_word(bq27220_t * const me, uint8_t cmd, uint16_t *val)
{
    if (me == NULL || me->hi2c == NULL || val == NULL) {
        return false;
    }

    uint8_t buf[2] = {0};
    // Le BQ27220 renvoie 2 octets en Little-Endian (octet de poids faible en premier)
    if (HAL_I2C_Mem_Read(me->hi2c, me->i2c_addr, cmd, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) != HAL_OK) {
        return false;
    }

    *val = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    return true;
}

bool bq27220_write_subcmd(bq27220_t * const me, uint16_t subcmd)
{
    if (me == NULL || me->hi2c == NULL) {
        return false;
    }

    uint8_t buf[2];
    buf[0] = (uint8_t)(subcmd & 0xFF);        // LSB
    buf[1] = (uint8_t)((subcmd >> 8) & 0xFF); // MSB

    return (HAL_I2C_Mem_Write(me->hi2c, me->i2c_addr, BQ27220_CMD_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) == HAL_OK);
}

bool bq27220_read_voltage_mv(bq27220_t * const me, uint16_t *voltage_mv)
{
    return bq27220_read_word(me, BQ27220_CMD_VOLTAGE, voltage_mv);
}

bool bq27220_read_current_ma(bq27220_t * const me, int16_t *current_ma)
{
    if (current_ma == NULL) return false;
    uint16_t raw = 0;
    if (!bq27220_read_word(me, BQ27220_CMD_CURRENT, &raw)) {
        return false;
    }
    // Inversion logicielle pour compenser l'inversion SRP / SRN du PCB
    *current_ma = (int16_t)(((int16_t)raw));
    return true;
}

bool bq27220_read_avg_current_ma(bq27220_t * const me, int16_t *avg_current_ma)
{
    if (avg_current_ma == NULL) return false;
    uint16_t raw = 0;
    if (!bq27220_read_word(me, BQ27220_CMD_AVG_CURRENT, &raw)) {
        return false;
    }
    // Inversion logicielle pour compenser l'inversion SRP / SRN du PCB
    *avg_current_ma = (int16_t)(((int16_t)raw));
    return true;
}

bool bq27220_read_temperature_c(bq27220_t * const me, float *temp_celsius){
    if (temp_celsius == NULL) return false;
    uint16_t raw = 0;
    if (!bq27220_read_word(me, BQ27220_CMD_TEMP, &raw)) {
        return false;
    }
    printf("[DEBUG] Temp raw = 0x%04X (%u)\r\n", raw, raw); // <-- Ajoutez ceci
    *temp_celsius = ((float)raw / 10.0f) - 273.15f;
    return true;
}

bool bq27220_read_soc_pct(bq27220_t * const me, uint16_t *soc_pct)
{
    return bq27220_read_word(me, BQ27220_CMD_STATE_OF_CHARGE, soc_pct);
}

bool bq27220_read_soh_pct(bq27220_t * const me, uint16_t *soh_pct)
{
    return bq27220_read_word(me, BQ27220_CMD_STATE_OF_HEALTH, soh_pct);
}

bool bq27220_read_remaining_capacity_mah(bq27220_t * const me, uint16_t *rem_cap_mah)
{
    return bq27220_read_word(me, BQ27220_CMD_REMAINING_CAPACITY, rem_cap_mah);
}

bool bq27220_read_full_capacity_mah(bq27220_t * const me, uint16_t *full_cap_mah)
{
    return bq27220_read_word(me, BQ27220_CMD_FULL_CHARGE_CAP, full_cap_mah);
}

bool bq27220_read_cycle_count(bq27220_t * const me, uint16_t *cycles)
{
    return bq27220_read_word(me, BQ27220_CMD_CYCLE_COUNT, cycles);
}

bool bq27220_read_status(bq27220_t * const me, uint16_t *status)
{
    return bq27220_read_word(me, BQ27220_CMD_BATTERY_STATUS, status);
}
