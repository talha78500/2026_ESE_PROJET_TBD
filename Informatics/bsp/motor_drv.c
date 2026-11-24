#include "motor_drv.h"
#include "main.h"   // MOTOR_SLEEP_Pin / MOTOR_SLEEP_GPIO_Port

static void set_compare(const motor_drv_t *me, uint32_t channel, uint32_t value)
{
    __HAL_TIM_SET_COMPARE(me->cfg.htim_pwm, channel, value);
}

bool motor_drv_init(motor_drv_t * const me, const motor_drv_cfg_t *cfg)
{
    if (me == NULL || cfg == NULL || cfg->htim_pwm == NULL || cfg->htim_enc == NULL) {
        return false;
    }

    me->cfg      = *cfg;
    me->position = 0;
    me->duty     = 0;

    // PWM : rapport cyclique a 0 avant le demarrage
    set_compare(me, me->cfg.ch_in1, 0);
    set_compare(me, me->cfg.ch_in2, 0);

    if (HAL_TIM_PWM_Start(me->cfg.htim_pwm, me->cfg.ch_in1) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(me->cfg.htim_pwm, me->cfg.ch_in2) != HAL_OK) return false;

    // Encodeur
    if (HAL_TIM_Encoder_Start(me->cfg.htim_enc, TIM_CHANNEL_ALL) != HAL_OK) return false;
    me->last_count = __HAL_TIM_GET_COUNTER(me->cfg.htim_enc);

    return true;
}

void motor_drv_sleep(bool awake)
{
    // Logique active a l'etat haut (voir fiche technique du driver, inverser si besoin)
    HAL_GPIO_WritePin(MOTOR_SLEEP_GPIO_Port, MOTOR_SLEEP_Pin,
                      awake ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_drv_set_duty(motor_drv_t * const me, int16_t duty_permille)
{
    if (me == NULL) return;

    if (duty_permille >  MOTOR_DRV_DUTY_MAX) duty_permille =  MOTOR_DRV_DUTY_MAX;
    if (duty_permille < -MOTOR_DRV_DUTY_MAX) duty_permille = -MOTOR_DRV_DUTY_MAX;

    me->duty = duty_permille;

    int32_t cmd = me->cfg.invert_dir ? -duty_permille : duty_permille;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(me->cfg.htim_pwm);
    uint32_t mag = (uint32_t)((cmd >= 0) ? cmd : -cmd);
    uint32_t duty;

    if (mag >= MOTOR_DRV_DUTY_MAX) {
        duty = arr + 1U;                                   // 100 % reel (pas de glitch bas)
    } else {
        duty = (uint32_t)(((uint64_t)mag * (arr + 1U)) / MOTOR_DRV_DUTY_MAX);
    }

    if (cmd > 0) {
        set_compare(me, me->cfg.ch_in1, duty);
        set_compare(me, me->cfg.ch_in2, 0);
    } else if (cmd < 0) {
        set_compare(me, me->cfg.ch_in1, 0);
        set_compare(me, me->cfg.ch_in2, duty);
    } else {
        set_compare(me, me->cfg.ch_in1, 0);
        set_compare(me, me->cfg.ch_in2, 0);
    }
}

void motor_drv_coast(motor_drv_t * const me)
{
    if (me == NULL) return;
    me->duty = 0;
    set_compare(me, me->cfg.ch_in1, 0);
    set_compare(me, me->cfg.ch_in2, 0);
}

void motor_drv_brake(motor_drv_t * const me)
{
    if (me == NULL) return;
    me->duty = 0;
    uint32_t full = __HAL_TIM_GET_AUTORELOAD(me->cfg.htim_pwm) + 1U;
    set_compare(me, me->cfg.ch_in1, full);
    set_compare(me, me->cfg.ch_in2, full);
}

int32_t motor_drv_read_delta(motor_drv_t * const me)
{
    if (me == NULL) return 0;

    uint32_t now = __HAL_TIM_GET_COUNTER(me->cfg.htim_enc);
    int32_t  delta;

    // La soustraction non signee gere le debordement du compteur
    if (me->cfg.enc_32bit) {
        delta = (int32_t)(now - me->last_count);
    } else {
        delta = (int32_t)(int16_t)(uint16_t)(now - me->last_count);
    }
    me->last_count = now;

    if (me->cfg.invert_enc) delta = -delta;

    me->position += delta;
    return delta;
}

int32_t motor_drv_get_position(const motor_drv_t * const me)
{
    return (me != NULL) ? me->position : 0;
}

void motor_drv_reset_position(motor_drv_t * const me)
{
    if (me == NULL) return;
    me->position   = 0;
    me->last_count = __HAL_TIM_GET_COUNTER(me->cfg.htim_enc);
}
