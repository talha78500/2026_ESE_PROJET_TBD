#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/**
 * @file motor.h
 * @brief Couche Applicative (APP) pour les moteurs du robot.
 *        Orchestre les 2 moteurs via le BSP (motor_drv) : commande, mesure de vitesse et securite.
 */

/* Ticks encodeur par tour de l'arbre de sortie (apres reducteur), AFFECTE LE CALCUL EN RPM.
 * A ajuster selon la fiche technique du FIT0483 et le mode encodeur configure (TI1 seul = x2, TI1+TI2 = x4). */
#ifndef MOTOR_ENC_TICKS_PER_REV
#define MOTOR_ENC_TICKS_PER_REV   1000
#endif

/* Zone morte : rapport cyclique minimum (pour-mille) applique des qu'une commande non nulle est demandee.
 * 0 = desactive. A regler un peu en dessous du seuil de demarrage mesure (ex: 550). */
#ifndef MOTOR_MIN_DUTY
#define MOTOR_MIN_DUTY            550
#endif

/* Arret automatique si aucune commande recue depuis ce delai (ms). 0 = desactive. */
#ifndef MOTOR_CMD_TIMEOUT_MS
#define MOTOR_CMD_TIMEOUT_MS      0
#endif

typedef enum {
    MOTOR_1 = 0,
    MOTOR_2,
    MOTOR_COUNT
} motor_id_t;

typedef enum {
    MOTOR_STATE_UNINIT = 0,
    MOTOR_STATE_STOPPED,     // Roue libre
    MOTOR_STATE_BRAKING,     // Frein actif
    MOTOR_STATE_RUNNING
} motor_state_t;

typedef struct {
    motor_state_t state;
    int16_t       cmd_permille;     // Derniere commande (-1000 .. +1000)
    int32_t       position_ticks;   // Position cumulee
    int32_t       speed_tps;        // Vitesse en ticks/s (signee)
    int32_t       speed_rpm;        // Vitesse en tr/min (signee)
} motor_info_t;

/**
 * @brief Initialise les 2 moteurs et sort le driver de veille
 * @param htim_pwm  Timer PWM (ex: &htim1, CH1/CH2 = moteur 1, CH3/CH4 = moteur 2)
 * @param htim_enc1 Timer encodeur moteur 1 (ex: &htim2, 32 bits)
 * @param htim_enc2 Timer encodeur moteur 2 (ex: &htim3, 16 bits)
 * @return true si l'initialisation a reussi
 */
bool motor_init(TIM_HandleTypeDef *htim_pwm,
                TIM_HandleTypeDef *htim_enc1,
                TIM_HandleTypeDef *htim_enc2);

/** @brief Commande en pour-mille (-1000 .. +1000). Retourne false si non initialise. */
bool motor_set_speed(motor_id_t id, int16_t permille);

/** @brief Arret en roue libre */
void motor_stop(motor_id_t id);

/** @brief Arret avec frein */
void motor_brake(motor_id_t id);

/** @brief Arret immediat des 2 moteurs (roue libre) */
void motor_stop_all(void);

/** @brief Met le driver en veille (true = reveille, false = veille, moteurs coupes) */
void motor_enable(bool enable);

/** @brief Remet la position d'un moteur a zero */
void motor_reset_position(motor_id_t id);

/**
 * @brief Met a jour les mesures de vitesse / position et la securite (timeout).
 *        A appeler periodiquement (ex: toutes les 10 a 50 ms).
 */
void motor_update(void);

/** @brief Affiche un diagnostic sur la console UART (printf) */
void motor_print_diagnostics(void);

/** @brief Retourne les informations courantes d'un moteur (NULL si id invalide) */
const motor_info_t* motor_get_info(motor_id_t id);

#endif /* APP_MOTOR_H */
