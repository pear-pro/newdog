#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------- ??? ---------------- */
#define PI                      3.1415926535f
#define MOTOR_MAX_ID            14
#define MOTOR_DEC_RATIO         6.33f
#define MOTOR_CMD_LEN           17
#define MOTOR_WORK_MODE_FOC     1

/* ---------------- ???? ---------------- */
typedef struct
{
    uint8_t              MotorID;
    UART_HandleTypeDef*  huart;

    // 物理量的浮点形式
    float Tau_ff;       
    float Omega_des;     // rad/s
    float Theta_des;     // rad
    float Kp;
    float Kw;

   
    uint8_t TxData[MOTOR_CMD_LEN];

    // 报文的16进制形式
    int16_t Tau_set;
    int16_t Omega_set;
    int32_t Theta_set;
    int16_t Kp_set;
    int16_t Kw_set;

} Motor_HandleTypeDef;

/* ---------------- API ---------------- */
HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor,
                             UART_HandleTypeDef *huart,
                             uint8_t motor_id);

void Motor_SendCmd(Motor_HandleTypeDef *hmotor);
void Motor_SendCmd_AllAngle();
void Motor_InitBias();

#endif /* __MOTOR_H */
