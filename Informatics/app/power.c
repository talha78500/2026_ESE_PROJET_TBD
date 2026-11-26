#include "power.h"
#include "bq25896.h"
#include "main.h"
#include "motor.h"
#include <stdio.h>

static bq25896_t    s_charger;
static power_info_t s_info = {
    .state          = POWER_STATE_UNINIT,
    .vbat_mv        = 0,
    .vbus_mv        = 0,
    .vsys_mv        = 0,
    .is_5v_active   = false,
    .has_fault      = false,
    .last_fault_raw = 0
};

bool power_init(I2C_HandleTypeDef *hi2c)
{
    // Configuration de la broche PB6 (CHARGEUR_QON) en entree avec Pull-Up
    // Le bouton SW2 relie QON a la masse (GND). Au repos, la ligne est tiree a 3.3V.
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = CHARGEUR_QON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CHARGEUR_QON_GPIO_Port, &GPIO_InitStruct);

    if (!bq25896_init(&s_charger, hi2c)) {
        s_info.state = POWER_STATE_FAULT;
        return false;
    }

    s_info.state = POWER_STATE_STANDBY;
    return true;
}

bool power_enable_5v_boost(void)
{
    if (s_info.state == POWER_STATE_UNINIT) {
        return false;
    }

    // 1. Désactiver le Watchdog (pour éviter le reset automatique au bout de 40s)
    if (!bq25896_set_watchdog(&s_charger, BQ25896_WDT_DISABLED)) {
        return false;
    }

    // 2. Désactiver la surveillance thermique du Boost (évite l'arrêt sur TS Fault si alim labo / pas de CTN)
//    if (!bq25896_disable_boost_thermal_protection(&s_charger)) {
//        return false;
//    }

    // 3. Régler la tension de sortie Boost à 4.998V (~5.0V)
    if (!bq25896_set_boost_voltage(&s_charger, 4998)) {
        return false;
    }

    // 4. Régler la limite de courant Boost (code 2 = 1.2A, bien adapté à l'alim labo 1A-2A)
    if (!bq25896_set_boost_current_limit(&s_charger, 0x02)) {
        return false;
    }

    // 5. Désactiver la charge de batterie
    if (!bq25896_enable_charge(&s_charger, false)) {
        return false;
    }

    // 6. Activer le Boost OTG
    if (!bq25896_enable_boost(&s_charger, true)) {
        return false;
    }

    // 7. Activer l'ADC interne en conversion continue pour pouvoir monitorer tensions et courants
    bq25896_enable_adc(&s_charger, true);

    s_info.state = POWER_STATE_BOOST_5V_ACTIVE;

    // Délai matériel recommandé de 30-50 ms pour le démarrage de l'oscillateur Boost
    HAL_Delay(50);

    // Première lecture de statut
    power_update();

    return true;
}

bool power_enable_charge(uint16_t ichg_ma, uint16_t iinlim_ma)
{
    if (s_info.state == POWER_STATE_UNINIT) {
        return false;
    }

    // 1. Couper le Boost OTG
    if (!bq25896_enable_boost(&s_charger, false)) {
        return false;
    }

    // 2. Désactiver le Watchdog ou le configurer selon le besoin
    if (!bq25896_set_watchdog(&s_charger, BQ25896_WDT_DISABLED)) {
        return false;
    }

    // 3. Définir la limite de courant d'entrée VBUS
    if (!bq25896_set_input_current_limit(&s_charger, iinlim_ma)) {
        return false;
    }

    // 4. Définir le courant de charge batterie
    if (!bq25896_set_charge_current(&s_charger, ichg_ma)) {
        return false;
    }

    // 5. Activer la charge
    if (!bq25896_enable_charge(&s_charger, true)) {
        return false;
    }

    bq25896_enable_adc(&s_charger, true);

    s_info.state = POWER_STATE_CHARGING;
    power_update();

    return true;
}

void power_update(void)
{
    if (s_info.state == POWER_STATE_UNINIT) {
        return;
    }

    bq25896_status_t status;
    bq25896_faults_t faults;

    // Lecture du statut
    if (bq25896_read_status(&s_charger, &status)) {
        s_info.is_5v_active = (status.vbus_stat == BQ25896_VBUS_OTG_BOOST);
    }

    // Lecture des fautes
    if (bq25896_read_faults(&s_charger, &faults)) {
        s_info.has_fault = (faults.raw != 0);
        s_info.last_fault_raw = faults.raw;

        if (faults.boost_fault || faults.bat_ovp_fault) {
            s_info.state = POWER_STATE_FAULT;
        }
    }

    // Mesures ADC
    bq25896_read_vbat_mv(&s_charger, &s_info.vbat_mv);
    bq25896_read_vbus_mv(&s_charger, &s_info.vbus_mv);
    bq25896_read_vsys_mv(&s_charger, &s_info.vsys_mv);
}

void power_print_diagnostics(void)
{
    power_update();

    bq25896_status_t status;
    bq25896_faults_t faults;
    bq25896_read_status(&s_charger, &status);
    bq25896_read_faults(&s_charger, &faults);

    printf("\r\n========================================\r\n");
    printf("       [APP_POWER] DIAGNOSTIC ENERGIE   \r\n");
    printf("========================================\r\n");
    
    const char *state_str[] = {"UNINIT", "STANDBY", "BOOST 5V ACTIF", "CHARGING", "FAULT"};
    printf("Etat sous-systeme : %s\r\n", state_str[s_info.state]);

    printf("Sortie 5V Boost   : %s\r\n", s_info.is_5v_active ? "ACTIVE (5V present sur VBUS)" : "INACTIVE");
    if (!s_info.is_5v_active && s_info.state == POWER_STATE_BOOST_5V_ACTIVE) {
        printf("  -> AVERTISSEMENT : Verifiez que la broche physique OTG (pin 8) est au 3.3V !\r\n");
    }

    printf("Tension Alim/BAT  : %u mV\r\n", s_info.vbat_mv);
    printf("Tension SYS       : %u mV\r\n", s_info.vsys_mv);
    printf("Tension VBUS      : %u mV\r\n", s_info.vbus_mv);

    if (s_info.has_fault) {
        printf("\r\n[ALERTE] Fautes detectees (0x%02X) :\r\n", s_info.last_fault_raw);
        if (faults.watchdog_fault) printf("  - Watchdog Fault\r\n");
        if (faults.boost_fault)    printf("  - Boost Fault (VBUS surcharge, VBUS OVP ou BAT < 2.9V)\r\n");
        if (faults.chrg_fault)     printf("  - Charge Fault (code %d)\r\n", faults.chrg_fault);
        if (faults.bat_ovp_fault)  printf("  - Battery OVP Fault\r\n");
        if (faults.ntc_fault)      printf("  - NTC (Temperature) Fault (code %d)\r\n", faults.ntc_fault);
    } else {
        printf("Fautes materielles: Aucune (OK)\r\n");
    }
    printf("========================================\r\n\r\n");
}

const power_info_t* power_get_info(void)
{
    return &s_info;
}

bool power_is_button_pressed(void)
{
    // SW2 connecte QON (PB6) a GND -> actif au niveau bas (GPIO_PIN_RESET)
    return (HAL_GPIO_ReadPin(CHARGEUR_QON_GPIO_Port, CHARGEUR_QON_Pin) == GPIO_PIN_RESET);
}

void power_shutdown(void)
{
    printf("\r\n[POWER] Extinction du robot en cours...\r\n");
    printf("[POWER] Arret des moteurs et passage en Ship Mode (BATFET_DIS = 1)\r\n");

    // 1. Arret securise de tous les moteurs
    motor_stop_all();

    // 2. Laisser le temps a l'UART de transmettre les messages
    HAL_Delay(100);

    // 3. Ordre I2C au BQ25896 d'ouvrir le BATFET pour couper SYS
    if (!bq25896_enter_ship_mode(&s_charger)) {
        printf("[POWER] ERREUR : Echec de l'envoi de la commande I2C d'extinction !\r\n");
        return;
    }

    // 4. Attente de la coupure d'alimentation
    // Si alimente par batterie seule : la coupure est immediate (quelques millisecondes).
    // Si alimente par USB / VBUS : le robot restera alimente jusqu'au debranchement du cable USB.
    printf("[POWER] Alimentation coupee (ou en attente du debranchement USB)...\r\n");
    while (1) {
        __WFI();
    }
}

void power_button_process(void)
{
    static uint32_t s_press_start_tick = 0;
    static bool     s_was_pressed      = false;

    // Periode de grace de 1.5s apres le demarrage :
    // Evite que le temps d'appui necessaire a l'allumage physique (~1s)
    // ne declenche accidentellement une extinction immediate au boot.
    if (HAL_GetTick() < 1500) {
        s_was_pressed = false;
        return;
    }

    bool pressed = power_is_button_pressed();

    if (pressed) {
        if (!s_was_pressed) {
            s_was_pressed      = true;
            s_press_start_tick = HAL_GetTick();
        } else {
            uint32_t hold_time = HAL_GetTick() - s_press_start_tick;
            // Appui long maintenu pendant au moins 2000 ms (2 secondes)
            if (hold_time >= 2000) {
                printf("\r\n[POWER] Appui long detecte sur SW2 (QON) : %lu ms\r\n", (unsigned long)hold_time);
                power_shutdown();
            }
        }
    } else {
        s_was_pressed = false;
    }
}
