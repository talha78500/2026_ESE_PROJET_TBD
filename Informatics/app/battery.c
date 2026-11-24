#include "battery.h"
#include "bq27220.h"
#include <stdio.h>

static bq27220_t      s_gauge;
static battery_info_t s_battery = {0};

bool battery_init(I2C_HandleTypeDef *hi2c)
{
    if (!bq27220_init(&s_gauge, hi2c)) {
        s_battery.is_ready = false;
        return false;
    }

    s_battery.is_ready = true;
    battery_update();
    return true;
}

void battery_update(void)
{
    if (!s_battery.is_ready) {
        return;
    }

    // Lecture des grandeurs physiques
    bq27220_read_voltage_mv(&s_gauge, &s_battery.voltage_mv);
    bq27220_read_current_ma(&s_gauge, &s_battery.current_ma);
    bq27220_read_avg_current_ma(&s_gauge, &s_battery.avg_current_ma);
    bq27220_read_temperature_c(&s_gauge, &s_battery.temperature_c);

    // Lecture des calculs CEDV (SOC, capacités, santé)
    bq27220_read_soc_pct(&s_gauge, &s_battery.soc_pct);
    bq27220_read_soh_pct(&s_gauge, &s_battery.soh_pct);
    bq27220_read_remaining_capacity_mah(&s_gauge, &s_battery.rem_capacity_mah);
    bq27220_read_full_capacity_mah(&s_gauge, &s_battery.full_capacity_mah);
    bq27220_read_cycle_count(&s_gauge, &s_battery.cycle_count);

    // Déductions d'état
    s_battery.is_charging = (s_battery.current_ma > 10); // > 10 mA pour éliminer le bruit d'offset
    s_battery.is_low_battery = (s_battery.voltage_mv < 3300) || (s_battery.soc_pct < 15);
}

void battery_print_diagnostics(void)
{
    battery_update();

    printf("\r\n========================================\r\n");
    printf("     [APP_BATTERY] JAUGE BQ27220        \r\n");
    printf("========================================\r\n");

    if (!s_battery.is_ready) {
        printf("Etat Jauge      : ERREUR / NON REPONDUE\r\n");
        printf("========================================\r\n\r\n");
        return;
    }

    printf("Tension cellule : %u mV (%.3f V)\r\n", s_battery.voltage_mv, s_battery.voltage_mv / 1000.0f);
    printf("Courant instant : %d mA (%s)\r\n", s_battery.current_ma, 
           s_battery.current_ma > 0 ? "En charge" : (s_battery.current_ma < 0 ? "En decharge" : "Repos"));
    printf("Courant moyen   : %d mA\r\n", s_battery.avg_current_ma);
    printf("Niveau (SOC)    : %u %%\r\n", s_battery.soc_pct);
    printf("Sante (SOH)     : %u %%\r\n", s_battery.soh_pct);
    printf("Capacite rest.  : %u / %u mAh\r\n", s_battery.rem_capacity_mah, s_battery.full_capacity_mah);
    printf("Temperature     : %.1f °C\r\n", s_battery.temperature_c);
    printf("Cycles batterie : %u\r\n", s_battery.cycle_count);
    
    if (s_battery.is_low_battery) {
        printf("[ATTENTION] BATTERIE FAIBLE (< 3.3V ou < 15%%) !\r\n");
    }
    printf("========================================\r\n\r\n");
}

const battery_info_t* battery_get_info(void)
{
    return &s_battery;
}


