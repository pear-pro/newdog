#include "motor.h"
#include "crc_ccitt.h"

//单个电机
static HAL_StatusTypeDef Motor_PackCmd(Motor_HandleTypeDef *hmotor);


HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor,
                             UART_HandleTypeDef *huart,
                             uint8_t motor_id)
{
    if (!hmotor || !huart || motor_id > MOTOR_MAX_ID)
        return HAL_ERROR;

    hmotor->huart   = huart;
    hmotor->MotorID = motor_id;

    hmotor->Tau_ff = 0.0f;
    hmotor->Omega_des = 0.0f;
    hmotor->Theta_des = 0.0f;
    hmotor->Kp = 0.0f;
    hmotor->Kw = 0.0f;

    return HAL_OK;
}


HAL_StatusTypeDef Motor_SendCmd(Motor_HandleTypeDef *hmotor)
{
    if (Motor_PackCmd(hmotor) != HAL_OK)
        return HAL_ERROR;

    return HAL_UART_Transmit(
        hmotor->huart,
        hmotor->TxData,
        MOTOR_CMD_LEN,
        100
    );
}


static HAL_StatusTypeDef Motor_PackCmd(Motor_HandleTypeDef *hmotor)
{
    if (!hmotor) return HAL_ERROR;

   
    hmotor->Tau_set   = (int16_t)(hmotor->Tau_ff * 256.0f);
    hmotor->Omega_set = (int16_t)(
        (hmotor->Omega_des * MOTOR_DEC_RATIO / (2.0f * PI)) * 256.0f
    );
    hmotor->Theta_set = (int32_t)(
        (hmotor->Theta_des * MOTOR_DEC_RATIO * 32768.0f) / (2.0f * PI)
    );
    hmotor->Kp_set    = (int16_t)(hmotor->Kp * 1280.0f);
    hmotor->Kw_set    = (int16_t)(hmotor->Kw * 1280.0f);

    uint8_t *tx = hmotor->TxData;

    tx[0] = 0xFE;
    tx[1] = 0xEE;

    tx[2] = (hmotor->MotorID & 0x0F) |
            ((MOTOR_WORK_MODE_FOC & 0x07) << 4);

    tx[3]  = hmotor->Tau_set;
    tx[4]  = hmotor->Tau_set >> 8;

    tx[5]  = hmotor->Omega_set;
    tx[6]  = hmotor->Omega_set >> 8;

    tx[7]  = hmotor->Theta_set;
    tx[8]  = hmotor->Theta_set >> 8;
    tx[9]  = hmotor->Theta_set >> 16;
    tx[10] = hmotor->Theta_set >> 24;

    tx[11] = hmotor->Kp_set;
    tx[12] = hmotor->Kp_set >> 8;

    tx[13] = hmotor->Kw_set;
    tx[14] = hmotor->Kw_set >> 8;

    unitree_crc_complete(tx);

    return HAL_OK;
}
