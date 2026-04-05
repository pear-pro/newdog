#include "motor.h"
#include "usart.h"
#include "crc_ccitt.h"
#include "global_var.h"

float motor1_bias = 5.80f;
float motor2_bias = 3.53f;
float motor3_bias = 3.57f;
float motor4_bias = 0.19f;
float motor5_bias = 4.40f;
float motor6_bias = 3.78f;
float motor7_bias = 0.10f;
float motor8_bias = -1.09f;

/* ---------------- ?????? ---------------- */
static HAL_StatusTypeDef Motor_PackCmd(Motor_HandleTypeDef *hmotor);

/* ---------------- ??? ---------------- */
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

void Motor_InitBias()
{
    uint8_t RxArray[16] = {0};

    // 恢复零力矩模式
    uint8_t InitArray[] = {                         
        0xFE, 0xEE, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};    
    unitree_crc_complete(InitArray);
    HAL_UART_Transmit(&huart6, InitArray, 17, 100);
    HAL_Delay(2000);

    // 逐个获取偏置值
    float* motor_bias_ptrs[8] = {
        &motor1_bias, &motor2_bias, &motor3_bias, &motor4_bias,
        &motor5_bias, &motor6_bias, &motor7_bias, &motor8_bias
    };


    for (int i = 0; i < 8; i++) {
        int motor_id = i + 1;
        
        uint8_t InitArray[] = {                         
            0xFE, 0xEE, motor_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        
        unitree_crc_complete(InitArray);
        HAL_UART_Transmit(&huart6, InitArray, 17, 100);
        
        if (HAL_UART_Receive(&huart6, RxArray, 16, 100) == HAL_OK) {
            int32_t raw_value = (int32_t)(RxArray[10] << 24|RxArray[9] << 16|RxArray[8] << 8|RxArray[7]);
            float bias_value = (float)raw_value * 32768.0f * 2.0f * PI;
            
            // 数据有效性检查
            if (bias_value >= -6.28f && bias_value <= 6.28f) {
                *(motor_bias_ptrs[i]) = bias_value;
            } 
        } 
        HAL_Delay(10);
    }
}


/* ---------------- ????(??) ---------------- */
void Motor_SendCmd(Motor_HandleTypeDef *hmotor)
{
    if (Motor_PackCmd(hmotor) != HAL_OK){
        //
    };

    RS485_SendFrame_Blocking(&huart6,hmotor->TxData);
}

/* ---------------- ????(??) ---------------- */
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

/*
hposition1 : hmotor1(α) , hmotor2(β)
hposition2 : hmotor3(α) , hmotor4(β)
hposition3 : hmotor5(α) , hmotor6(β)
hposition4 : hmotor7(α) , hmotor8(β)
*/
void Motor_SendCmd_AllAngle(){
    //hmotor1.Theta_des = motor1_bias / 6.33f + 6.28 * 20.0f / 360.0f;
    hmotor1.Theta_des = motor1_bias / 6.33f + 6.28 * hposition1.alpha / 360.0f;
    Motor_SendCmd(&hmotor1);
    HAL_Delay(1);
	//hmotor2.Theta_des = motor2_bias / 6.33f + 6.28 * 20.0f / 360.0f;
    hmotor2.Theta_des = motor2_bias / 6.33f + 6.28 * hposition1.beta / 360.0f;
    Motor_SendCmd(&hmotor2);
    HAL_Delay(1);
    hmotor3.Theta_des = motor3_bias / 6.33f + 6.28 * hposition2.alpha / 360.0f;
    Motor_SendCmd(&hmotor3);
    HAL_Delay(1);
    hmotor4.Theta_des = motor4_bias / 6.33f + 6.28 * hposition2.beta / 360.0f;
    Motor_SendCmd(&hmotor4);
    HAL_Delay(1);
    hmotor5.Theta_des = motor5_bias / 6.33f + 6.28 * -hposition3.alpha / 360.0f;
    Motor_SendCmd(&hmotor5);
    HAL_Delay(1);
    hmotor6.Theta_des = motor6_bias / 6.33f + 6.28 * -hposition3.beta / 360.0f;
    Motor_SendCmd(&hmotor6);
    HAL_Delay(1);
    hmotor7.Theta_des = motor7_bias / 6.33f + 6.28 * -hposition4.alpha / 360.0f;
    Motor_SendCmd(&hmotor7);
    HAL_Delay(1);
    hmotor8.Theta_des = motor8_bias / 6.33f + 6.28 * -hposition4.beta / 360.0f;
    Motor_SendCmd(&hmotor8);
	HAL_Delay(1);

}

