#include "apds9960.h"

#define APDS9960_ADDR     (0x39U << 1)   /* adresse 7 bits décalée pour HAL */
#define I2C_TIMEOUT_MS    10U

/* Registres */
#define REG_ENABLE        0x80U
#define REG_ATIME         0x81U
#define REG_PPULSE        0x8EU
#define REG_CONTROL       0x8FU
#define REG_ID            0x92U
#define REG_STATUS        0x93U
#define REG_CDATAL        0x94U   /* puis CDATAH, RDATAL/H, GDATAL/H, BDATAL/H (8 octets) */
#define REG_PDATA         0x9CU

/* ENABLE */
#define ENABLE_PON        (1U << 0)   /* power on */
#define ENABLE_AEN        (1U << 1)   /* moteur couleur (ALS) */
#define ENABLE_PEN        (1U << 2)   /* moteur proximité */

/* STATUS */
#define STATUS_AVALID     (1U << 0)   /* mesure couleur prête */
#define STATUS_PVALID     (1U << 1)   /* mesure proximité prête */

/* ID : 0xAB sur la plupart des modules, 0x9C sur certaines révisions */
#define ID_A              0xABU
#define ID_B              0x9CU

/* Réglages.
 * ATIME = 0xDB -> (256-219) = 37 cycles x 2,78 ms = ~103 ms par mesure couleur
 *                 (valeur max d'un canal : 37 x 1024 = 37888).
 * PPULSE = 0x87 -> impulsions de 16 us, 8 impulsions.
 * CONTROL = 0x09 -> LED 100 mA (bits 7:6 = 00), PGAIN x4 (bits 3:2 = 10),
 *                   AGAIN x4 couleur (bits 1:0 = 01). */
#define ATIME_VALUE       0xDBU
#define PPULSE_VALUE      0x87U
#define CONTROL_VALUE     0x09U

static bool write_reg(apds9960_t * const me, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(me->hi2c, APDS9960_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1U, I2C_TIMEOUT_MS) == HAL_OK;
}

static bool read_regs(apds9960_t * const me, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(me->hi2c, APDS9960_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, I2C_TIMEOUT_MS) == HAL_OK;
}

bool apds9960_init(apds9960_t * const me, I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0U;

    me->hi2c = hi2c;

    if (!read_regs(me, REG_ID, &id, 1U)) {
        return false;
    }
    if ((id != ID_A) && (id != ID_B)) {
        return false;
    }

    bool ok = write_reg(me, REG_ENABLE, 0x00U);   /* tout éteint avant de configurer */
    ok = ok && write_reg(me, REG_ATIME,   ATIME_VALUE);
    ok = ok && write_reg(me, REG_PPULSE,  PPULSE_VALUE);
    ok = ok && write_reg(me, REG_CONTROL, CONTROL_VALUE);
    ok = ok && write_reg(me, REG_ENABLE,  ENABLE_PON | ENABLE_AEN | ENABLE_PEN);
    return ok;
}

bool apds9960_read_proximity(apds9960_t * const me, uint8_t *prox)
{
    uint8_t status = 0U;

    if (!read_regs(me, REG_STATUS, &status, 1U)) {
        return false;
    }
    if ((status & STATUS_PVALID) == 0U) {
        return false;
    }
    return read_regs(me, REG_PDATA, prox, 1U);
}

bool apds9960_read_rgbc(apds9960_t * const me, apds9960_rgbc_t *rgbc)
{
    uint8_t status = 0U;
    uint8_t raw[8];

    if (!read_regs(me, REG_STATUS, &status, 1U)) {
        return false;
    }
    if ((status & STATUS_AVALID) == 0U) {
        return false;
    }
    if (!read_regs(me, REG_CDATAL, raw, (uint16_t)sizeof raw)) {
        return false;
    }

    /* Octet bas d'abord (little-endian) : lire le bas verrouille le haut. */
    rgbc->clear = (uint16_t)(((uint16_t)raw[1] << 8) | raw[0]);
    rgbc->red   = (uint16_t)(((uint16_t)raw[3] << 8) | raw[2]);
    rgbc->green = (uint16_t)(((uint16_t)raw[5] << 8) | raw[4]);
    rgbc->blue  = (uint16_t)(((uint16_t)raw[7] << 8) | raw[6]);
    return true;
}
