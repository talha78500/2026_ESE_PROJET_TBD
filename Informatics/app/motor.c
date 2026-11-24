#include "motor.h"
#include "motor_drv.h"
#include <stdio.h>

static motor_drv_t  s_drv[MOTOR_COUNT];
static motor_info_t s_info[MOTOR_COUNT];
static uint32_t     s_last_update_ms;
static uint32_t     s_last_cmd_ms[MOTOR_COUNT];
static bool         s_enabled;

static bool id_valid(motor_id_t id)
{
    return ((unsigned)id < MOTOR_COUNT) && (s_info[id].state != MOTOR_STATE_UNINIT);
}

bool motor_init(TIM_HandleTypeDef *htim_pwm,
                TIM_HandleTypeDef *htim_enc1,
                TIM_HandleTypeDef *htim_enc2)
{
    // Moteur 1 : TIM1 CH1/CH2 (AIN1/AIN2), encodeur TIM2 (32 bits)
    const motor_drv_cfg_t cfg1 = {
        .htim_pwm   = htim_pwm,
        .ch_in1     = TIM_CHANNEL_1,
        .ch_in2     = TIM_CHANNEL_2,
        .htim_enc   = htim_enc1,
        .enc_32bit  = true,
        .invert_dir = false,
        .invert_enc = false
    };

    // Moteur 2 : TIM1 CH3/CH4 (BIN1/BIN2), encodeur TIM3 (16 bits)
    const motor_drv_cfg_t cfg2 = {
        .htim_pwm   = htim_pwm,
        .ch_in1     = TIM_CHANNEL_3,
        .ch_in2     = TIM_CHANNEL_4,
        .htim_enc   = htim_enc2,
        .enc_32bit  = false,
        .invert_dir = false,
        .invert_enc = false
    };

    // Driver en veille pendant la configuration
    motor_drv_sleep(false);

    if (!motor_drv_init(&s_drv[MOTOR_1], &cfg1)) return false;
    if (!motor_drv_init(&s_drv[MOTOR_2], &cfg2)) return false;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        s_info[i] = (motor_info_t){ .state = MOTOR_STATE_STOPPED };
        s_last_cmd_ms[i] = HAL_GetTick();
    }
    s_last_update_ms = HAL_GetTick();

    motor_enable(true);
    return true;
}

void motor_enable(bool enable)
{
    s_enabled = enable;
    if (!enable) {
        motor_stop_all();
    }
    motor_drv_sleep(enable);
}

bool motor_set_speed(motor_id_t id, int16_t permille)
{
    if (!id_valid(id) || !s_enabled) return false;

    if (permille > MOTOR_DRV_DUTY_MAX)  permille = MOTOR_DRV_DUTY_MAX;
    if (permille < -MOTOR_DRV_DUTY_MAX) permille = -MOTOR_DRV_DUTY_MAX;

#if MOTOR_MIN_DUTY > 0
    // Compensation de zone morte : remappe 1..1000 vers MIN..1000 (0 reste 0)
    if (permille > 0) {
        permille = (int16_t)(MOTOR_MIN_DUTY +
                   ((int32_t)permille * (MOTOR_DRV_DUTY_MAX - MOTOR_MIN_DUTY)) / MOTOR_DRV_DUTY_MAX);
    } else if (permille < 0) {
        permille = (int16_t)(-MOTOR_MIN_DUTY +
                   ((int32_t)permille * (MOTOR_DRV_DUTY_MAX - MOTOR_MIN_DUTY)) / MOTOR_DRV_DUTY_MAX);
    }
#endif

    motor_drv_set_duty(&s_drv[id], permille);
    s_info[id].cmd_permille = permille;
    s_info[id].state = (permille != 0) ? MOTOR_STATE_RUNNING : MOTOR_STATE_STOPPED;
    s_last_cmd_ms[id] = HAL_GetTick();
    return true;
}

void motor_stop(motor_id_t id)
{
    if (!id_valid(id)) return;
    motor_drv_coast(&s_drv[id]);
    s_info[id].cmd_permille = 0;
    s_info[id].state = MOTOR_STATE_STOPPED;
}

void motor_brake(motor_id_t id)
{
    if (!id_valid(id)) return;
    motor_drv_brake(&s_drv[id]);
    s_info[id].cmd_permille = 0;
    s_info[id].state = MOTOR_STATE_BRAKING;
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_stop((motor_id_t)i);
    }
}

void motor_reset_position(motor_id_t id)
{
    if (!id_valid(id)) return;
    motor_drv_reset_position(&s_drv[id]);
    s_info[id].position_ticks = 0;
}

void motor_update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_last_update_ms;
    if (dt == 0) return;
    s_last_update_ms = now;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!id_valid((motor_id_t)i)) continue;

        // Mesure : delta de ticks sur dt ms
        int32_t delta = motor_drv_read_delta(&s_drv[i]);
        s_info[i].position_ticks = motor_drv_get_position(&s_drv[i]);
        s_info[i].speed_tps = (int32_t)(((int64_t)delta * 1000) / (int32_t)dt);
        s_info[i].speed_rpm = (int32_t)(((int64_t)s_info[i].speed_tps * 60) / MOTOR_ENC_TICKS_PER_REV);

        // Securite : arret si aucune commande recente
#if MOTOR_CMD_TIMEOUT_MS > 0
        if (s_info[i].state == MOTOR_STATE_RUNNING &&
            (now - s_last_cmd_ms[i]) > MOTOR_CMD_TIMEOUT_MS) {
            motor_stop((motor_id_t)i);
        }
#endif
    }
}

void motor_print_diagnostics(void)
{
    motor_update();

    static const char *state_str[] = {"UNINIT", "ARRET", "FREIN", "EN MARCHE"};

    printf("\r\n========================================\r\n");
    printf("       [APP_MOTOR] DIAGNOSTIC MOTEURS   \r\n");
    printf("========================================\r\n");
    printf("Driver            : %s\r\n", s_enabled ? "ACTIF" : "EN VEILLE");

    for (int i = 0; i < MOTOR_COUNT; i++) {
        const motor_info_t *m = &s_info[i];
        printf("Moteur %d : %s\r\n", i + 1, state_str[m->state]);
        printf("  Commande : %d/1000\r\n", m->cmd_permille);
        printf("  Position : %ld ticks\r\n", (long)m->position_ticks);
        printf("  Vitesse  : %ld ticks/s (%ld tr/min)\r\n", (long)m->speed_tps, (long)m->speed_rpm);
    }
    printf("========================================\r\n\r\n");
}

const motor_info_t* motor_get_info(motor_id_t id)
{
    if ((unsigned)id >= MOTOR_COUNT) return NULL;
    return &s_info[id];
}
