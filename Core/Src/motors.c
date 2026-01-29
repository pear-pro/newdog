//多个电机
#include "motors.h"

Motor_HandleTypeDef motors[MOTOR_NUM];

/* 8 个电机共用一个串口 */
void Motors_Init(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        /* MotorID = 0 ~ 7 */
        Motor_Init(&motors[i], huart, i );
    }
}

/* 依次发送 8 帧 */
HAL_StatusTypeDef Motors_SendAll(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (Motor_SendCmd(&motors[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
//				else
//				{
//					HAL_Delay(1);
//				}
    }
	
    return HAL_OK;
}

