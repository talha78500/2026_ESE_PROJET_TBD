#include "bsp_servo.h"

extern TIM_HandleTypeDef htim4;

void BSP_Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
}

void BSP_Servo_SetAngle(uint8_t angle)
{
    // Convertir l'angle (0 à 180°) en valeur de rapport cyclique (Compare Register)
    // 0° -> ~1ms d'impulsion, 180° -> ~2ms d'impulsion
    // Exemple théorique pour une période de 20ms :
    uint32_t pulse = 500 + (angle * (2500 - 500) / 180);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
}
