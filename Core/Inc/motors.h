//多个电机
#ifndef __MOTORS_H
#define __MOTORS_H

#include "motor.h"

#define MOTOR_NUM 8

extern Motor_HandleTypeDef motors[MOTOR_NUM];

void Motors_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motors_SendAll(void);

#endif

