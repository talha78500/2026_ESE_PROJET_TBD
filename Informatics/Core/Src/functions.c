#include "main.h"
#include <string.h>
#include <stdio.h>
#include "stm32g4xx_hal_gpio.h"
#define i2c_adress_tof 0x52

extern I2C_HandleTypeDef hi2c1;

static uint8_t tof_id;
static HAL_StatusTypeDef status;
static char msg[50];

const char *I2C_getBack(void)
{
    status = HAL_I2C_IsDeviceReady(&hi2c1, i2c_adress_tof, 3, 100);

    if (status == HAL_OK)
    {
        HAL_I2C_Mem_Read(&hi2c1, i2c_adress_tof, 0xC0, I2C_MEMADD_SIZE_8BIT, &tof_id, 1, 100);

        if (tof_id == 0xEE)
        {
            sprintf(msg, "Capteur OK, ID = 0x%02X\r\n", tof_id);
            HAL_GPIO_WritePin(LD2_Pin_GPIO_Port, LD2_Pin_Pin, GPIO_PIN_SET);
        }
        else
        {
            sprintf(msg, "ID incorrect: 0x%02X (attendu 0xEE)\r\n", tof_id);
        }
    }
    else
    {
        sprintf(msg, "Capteur non detecte sur le bus I2C\r\n");
    }

    return msg;
}
