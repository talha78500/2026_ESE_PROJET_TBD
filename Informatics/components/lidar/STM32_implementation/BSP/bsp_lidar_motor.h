#ifndef BSP_LIDAR_MOTOR_H
#define BSP_LIDAR_MOTOR_H

#include <stdint.h>

/* Starts PWM on PA8 at zero duty. Call after TIM1 initialization. */
int bsp_lidar_motor_start(void);
/* Duty in thousandths of a period, from 0 to 1000. */
void bsp_lidar_motor_set_duty(uint16_t duty_permille);

#endif
