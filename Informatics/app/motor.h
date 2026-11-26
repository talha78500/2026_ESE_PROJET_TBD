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

/* Ticks encodeur par tour de l'arbre de sortie (apres reducteur), AFFECTE LE CALCUL EN RPM ET DISTANCE.
 * FIT0483 : 7 paires de poles x reducteur 100:1 x 2 fronts (mode encodeur TIM_ENCODERMODE_TI1) = 1400 ticks/tour. */
#ifndef MOTOR_ENC_TICKS_PER_REV
#define MOTOR_ENC_TICKS_PER_REV   1400
#endif

/* Diametre des roues en mm (circonference = pi * 43 mm ~= 135.09 mm) */
#ifndef MOTOR_WHEEL_DIAMETER_MM
#define MOTOR_WHEEL_DIAMETER_MM   42.0f
#endif

/* Sens relatif du moteur 2 par rapport au moteur 1 pour avancer droit (-1 car montes face a face) */
#ifndef MOTOR_CHASSIS_M2_SIGN
#define MOTOR_CHASSIS_M2_SIGN     (-1)
#endif

/* Entraxe (voie) entre les deux roues en mm : distance centre-a-centre.
 * Calibre d'apres mesure reelle (150 mm donnait ~130 deg pour 90 deg demandes -> 150 * 90 / 130 ~= 104 mm). */
#ifndef MOTOR_TRACK_WIDTH_MM
#define MOTOR_TRACK_WIDTH_MM      104.0f
#endif

/* Convention de sens pour motor_turn_angle_deg() (+1 ou -1 selon le cote du Moteur 1/2) */
#ifndef MOTOR_TURN_DIR_SIGN
#define MOTOR_TURN_DIR_SIGN       1
#endif

/* Vitesse par defaut en pour-mille (1..1000) pour motor_move_distance_mm() si 0 est passe */
#ifndef MOTOR_MOVE_DEFAULT_SPEED
#define MOTOR_MOVE_DEFAULT_SPEED  500
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

/** @brief Convertit une distance en mm en ticks encodeur (roue de MOTOR_WHEEL_DIAMETER_MM) */
int32_t motor_mm_to_ticks(float mm);

/** @brief Convertit un nombre de ticks encodeur en distance en mm */
float motor_ticks_to_mm(int32_t ticks);

/** @brief Convertit un angle de rotation sur place (degres) en ticks par roue */
int32_t motor_deg_to_ticks(float deg);

/** @brief Convertit un nombre de ticks par roue en angle de rotation sur place (degres) */
float motor_ticks_to_deg(int32_t ticks);

/**
 * @brief Lance un deplacement rectiligne non bloquant d'une distance donnee en mm.
 *        Synchronise les deux roues via leurs encodeurs, ralentit en fin de course
 *        et freine activement a l'arrivee (traite dans motor_update()).
 * @param distance_mm    Distance en mm (> 0 pour avancer, < 0 pour reculer)
 * @param speed_permille Vitesse cible (1..1000, ou 0 pour MOTOR_MOVE_DEFAULT_SPEED)
 * @return false si les moteurs ne sont pas initialises ou sont en veille
 */
bool motor_move_distance_mm(int32_t distance_mm, uint16_t speed_permille);

/**
 * @brief Fait tourner le robot sur lui-meme selon un angle donne en degres.
 *        Les deux roues tournent en sens lineaire oppose sur l'entraxe MOTOR_TRACK_WIDTH_MM,
 *        avec synchronisation encodeur, rampe de deceleration et freinage actif.
 * @param angle_deg      Angle de rotation en degres (positif / negatif selon le sens)
 * @param speed_permille Vitesse cible (1..1000, ou 0 pour MOTOR_MOVE_DEFAULT_SPEED)
 * @return false si les moteurs ne sont pas initialises ou sont en veille
 */
bool motor_turn_angle_deg(int32_t angle_deg, uint16_t speed_permille);

/** @brief Indique si un deplacement en distance est actuellement en cours */
bool motor_is_moving(void);

#endif /* APP_MOTOR_H */
