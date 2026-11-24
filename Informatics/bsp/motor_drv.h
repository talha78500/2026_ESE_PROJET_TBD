#ifndef BSP_MOTOR_DRV_H
#define BSP_MOTOR_DRV_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

/**
 * @file motor_drv.h
 * @brief Couche BSP pour un moteur DC (driver 2 entrees IN1/IN2) avec encodeur en quadrature.
 *        Seul ce module manipule les timers PWM / encodeur et la broche SLEEP du driver.
 *
 *        Logique de commande (mode IN/IN, ex: DRV8833) :
 *          - Avant   : PWM sur IN1, IN2 = 0
 *          - Arriere : IN1 = 0, PWM sur IN2
 *          - Roue libre (coast) : IN1 = 0, IN2 = 0
 *          - Frein   : IN1 = 1, IN2 = 1
 */

#define MOTOR_DRV_DUTY_MAX   1000   // Commande en pour-mille : -1000 .. +1000

typedef struct {
    TIM_HandleTypeDef *htim_pwm;    // Timer PWM (ex: &htim1)
    uint32_t           ch_in1;      // Canal PWM IN1 (ex: TIM_CHANNEL_1)
    uint32_t           ch_in2;      // Canal PWM IN2 (ex: TIM_CHANNEL_2)
    TIM_HandleTypeDef *htim_enc;    // Timer encodeur (ex: &htim2)
    bool               enc_32bit;   // true si le compteur est 32 bits (TIM2), false si 16 bits
    bool               invert_dir;  // Inverse le sens de rotation commande
    bool               invert_enc;  // Inverse le sens de comptage de l'encodeur
} motor_drv_cfg_t;

typedef struct {
    motor_drv_cfg_t cfg;
    uint32_t        last_count;     // Derniere valeur brute du compteur
    int32_t         position;       // Position cumulee en ticks (signee, sans debordement)
    int16_t         duty;           // Derniere commande appliquee (pour-mille)
} motor_drv_t;

/* Initialisation : demarre les PWM IN1/IN2 (rapport 0) et l'encodeur */
bool motor_drv_init(motor_drv_t * const me, const motor_drv_cfg_t *cfg);

/* Broche SLEEP commune aux drivers (true = driver actif) */
void motor_drv_sleep(bool awake);

/* Commandes */
void motor_drv_set_duty(motor_drv_t * const me, int16_t duty_permille); // -1000 .. +1000
void motor_drv_coast(motor_drv_t * const me);
void motor_drv_brake(motor_drv_t * const me);

/* Encodeur */
int32_t motor_drv_read_delta(motor_drv_t * const me);   // Ticks depuis le dernier appel (met aussi a jour position)
int32_t motor_drv_get_position(const motor_drv_t * const me);
void    motor_drv_reset_position(motor_drv_t * const me);

#endif /* BSP_MOTOR_DRV_H */
