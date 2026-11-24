#ifndef APP_BATTERY_H
#define APP_BATTERY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32g4xx_hal.h"

/**
 * @file battery.h
 * @brief Couche Applicative (APP) pour la gestion et le monitoring de la batterie via le BQ27220.
 */

typedef struct {
    bool     is_ready;            // Vrai si la jauge BQ27220 répond
    uint16_t voltage_mv;          // Tension mesurée aux bornes (mV)
    int16_t  current_ma;          // Courant instantané (mA, positif=charge, négatif=décharge)
    int16_t  avg_current_ma;      // Courant moyen lissé (mA)
    uint16_t soc_pct;             // État de charge / pourcentage restant (0 à 100 %)
    uint16_t soh_pct;             // État de santé de la batterie (State of Health %)
    uint16_t rem_capacity_mah;    // Capacité restante estimée (mAh)
    uint16_t full_capacity_mah;   // Capacité max pleine charge (mAh)
    float    temperature_c;       // Température mesurée (°C)
    uint16_t cycle_count;         // Nombre de cycles de charge
    bool     is_charging;         // Vrai si du courant entre dans la batterie
    bool     is_low_battery;      // Vrai si la batterie est critique (< 3300 mV ou < 15%)
} battery_info_t;

/**
 * @brief Initialise le pilote de la jauge de batterie BQ27220
 * @param hi2c Handle I2C (ex: &hi2c3)
 * @return true si la jauge répond et renvoie une mesure valide
 */
bool battery_init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Met à jour toutes les données télémétriques de la batterie
 *        Recommandé : appeler toutes les 1 à 2 secondes (éviter de poller > 2 Hz selon TI)
 */
void battery_update(void);

/**
 * @brief Affiche un rapport complet de la batterie sur la liaison série UART
 */
void battery_print_diagnostics(void);

/**
 * @brief Récupère les données télémétriques courantes
 */
const battery_info_t* battery_get_info(void);

void battery_print_current();

#endif /* APP_BATTERY_H */
