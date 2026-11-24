#ifndef APP_POWER_H
#define APP_POWER_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/**
 * @file power.h
 * @brief Couche Applicative (APP) pour la gestion d'énergie et de l'alimentation du robot.
 *        Orchestre le composant BQ25896 (via le BSP) et gère les états d'alimentation.
 */

typedef enum {
    POWER_STATE_UNINIT = 0,
    POWER_STATE_STANDBY,
    POWER_STATE_BOOST_5V_ACTIVE,
    POWER_STATE_CHARGING,
    POWER_STATE_FAULT
} power_state_t;

typedef struct {
    power_state_t state;
    uint16_t      vbat_mv;        // Tension de batterie / alim mesurée par l'ADC
    uint16_t      vbus_mv;        // Tension VBUS mesurée par l'ADC
    uint16_t      vsys_mv;        // Tension Système (VSYS) mesurée par l'ADC
    bool          is_5v_active;   // Vrai si le boost 5V est bien enclenché
    bool          has_fault;      // Vrai si au moins un défaut est présent
    uint8_t       last_fault_raw; // Code brut du registre REG0C
} power_info_t;

/**
 * @brief Initialise le sous-système d'alimentation
 * @param hi2c Handle I2C relié au BQ25896 (ex: &hi2c3)
 * @return true si le BQ25896 répond et est valide, false sinon
 */
bool power_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Active la sortie 5V boostée (Mode OTG) depuis la batterie / alim labo
 *        Désactive la charge, coupe le watchdog, inhibe le défaut thermique TS et règle Vout = 5.0V.
 * @return true si la séquence I2C a réussi
 */
bool power_enable_5v_boost(void);

/**
 * @brief Active le mode recharge de batterie (Mode Buck) depuis une source VBUS
 * @param ichg_ma Courant de charge batterie en mA (ex: 1024, 2048)
 * @param iinlim_ma Limite de courant sur la source VBUS en mA (ex: 1500, 2000)
 * @return true si la séquence I2C a réussi
 */
bool power_enable_charge(uint16_t ichg_ma, uint16_t iinlim_ma);

/**
 * @brief Met à jour les mesures et vérifie l'état et les fautes du système d'alimentation
 *        À appeler périodiquement (ex: toutes les 1 à 3 secondes) ou après un changement d'état.
 */
void power_update(void);

/**
 * @brief Affiche un rapport complet de diagnostic de l'alimentation sur la console UART (printf)
 */
void power_print_diagnostics(void);

/**
 * @brief Retourne les informations courantes du sous-système d'alimentation
 */
const power_info_t* power_get_info(void);

#endif /* APP_POWER_H */
