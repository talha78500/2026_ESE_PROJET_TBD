#include "bsp_lidar_motor.h"

#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim1;

int bsp_lidar_motor_start(void)
{
    bsp_lidar_motor_set_duty(0);
    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_OK ? 0 : -1;
}

void bsp_lidar_motor_set_duty(uint16_t duty_permille)
{
    uint32_t period_ticks = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U;
    uint32_t pulse_ticks = (period_ticks * duty_permille + 500U) / 1000U;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_ticks);
}
