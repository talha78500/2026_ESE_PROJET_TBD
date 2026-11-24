#include "app_test.h"
#include "bsp_servo.h"

void APP_Servo_RunTest(void)
{
    BSP_Servo_Init();
    while (1)
    {
        BSP_Servo_SetAngle(0);
        HAL_Delay(1000);
        BSP_Servo_SetAngle(90);
        HAL_Delay(1000);
        BSP_Servo_SetAngle(180);
        HAL_Delay(1000);
    }
}
