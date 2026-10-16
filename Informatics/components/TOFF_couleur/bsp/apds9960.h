#ifndef APDS9960_H
#define APDS9960_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

/* Couche BSP : seul module qui parle I2C à l'APDS-9960.
 * L'app ne l'inclut jamais : main.c récupère les valeurs et les passe à l'app. */

typedef struct {
    I2C_HandleTypeDef *hi2c;
} apds9960_t;

typedef struct {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} apds9960_rgbc_t;

/* Renvoie false si le capteur ne répond pas ou si l'ID est faux. */
bool apds9960_init(apds9960_t * const me, I2C_HandleTypeDef *hi2c);

/* Non bloquantes (timeout I2C 10 ms) : renvoient false s'il n'y a pas de
 * nouvelle mesure prête, ou en cas d'erreur I2C. */
bool apds9960_read_proximity(apds9960_t * const me, uint8_t *prox);
bool apds9960_read_rgbc(apds9960_t * const me, apds9960_rgbc_t *rgbc);

#endif /* APDS9960_H */
