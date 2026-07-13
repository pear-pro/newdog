/*
 * motor.c
 */

#include "motor.h"
#include "usart.h"
#include "crc_ccitt.h"
#include "global_var.h"
#include "imu.h"
#include "tim.h"
#include "gait.h"
#include "string.h"
#include "kinematic.h"
#include "motor_feedback.h"


#define TRANSNIT_DELAY 500
 
float motor1_bias = 0.086f;
float motor2_bias = 0.171f;
float motor3_bias = 0.701f;
float motor4_bias = 0.268f;
float motor5_bias = 0.416f;
float motor6_bias = 0.699f;
float motor7_bias = 0.514f;
float motor8_bias = 0.383f;
float motor10_bias = 1.1595f;// 云台偏置   //  motor10_bias=   2*PI*motor_fb[10].theta/6.33/6.28


float flip_offset = 0.0f; // 3.165f 狗腿翻身对应的电机反转角度

static HAL_StatusTypeDef Motor_PackCmd(Motor_HandleTypeDef *hmotor);


HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor, UART_HandleTypeDef *huart, uint8_t motor_id)
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

//  初始化电机结构体,角度环控制只需要初始化kp和kw
void init_motor_parameters(void)
{
    Motor_Init(&hmotor1, &huart6, 1);
	hmotor1.Tau_ff = -swing_tau_ff;
    hmotor1.Kp = swing_Kp;
    hmotor1.Kw = Expect_kw;	
    Motor_Init(&hmotor2, &huart6, 2);
	hmotor2.Tau_ff = -swing_tau_ff;
    hmotor2.Kp = swing_Kp;
    hmotor2.Kw = Expect_kw;
    Motor_Init(&hmotor3, &huart6, 3);
    hmotor3.Tau_ff = -swing_tau_ff;
    hmotor3.Kp = swing_Kp;
    hmotor3.Kw = Expect_kw;	
    Motor_Init(&hmotor4, &huart6, 4);
    hmotor4.Tau_ff = -swing_tau_ff;
    hmotor4.Kp = swing_Kp;
    hmotor4.Kw = Expect_kw;    
    Motor_Init(&hmotor5, &huart6, 5);
    hmotor5.Tau_ff = swing_tau_ff;
    hmotor5.Kp = swing_Kp;
    hmotor5.Kw = Expect_kw;	
    Motor_Init(&hmotor6, &huart6, 6);
    hmotor6.Tau_ff = swing_tau_ff;
    hmotor6.Kp = swing_Kp;
    hmotor6.Kw = Expect_kw;    
    Motor_Init(&hmotor7, &huart6, 7);
    hmotor7.Tau_ff = swing_tau_ff;
    hmotor7.Kp = swing_Kp;
    hmotor7.Kw = Expect_kw;	
    Motor_Init(&hmotor8, &huart6, 8);
    hmotor8.Tau_ff = swing_tau_ff;
    hmotor8.Kp = swing_Kp;
    hmotor8.Kw = Expect_kw;  
    Motor_Init(&hmotor10, &huart6, 10);
    hmotor10.Tau_ff = swing_tau_ff;
    hmotor10.Kp = swing_Kp;
    hmotor10.Kw = Expect_kw;  
}

// 电机id重映射
void remap_motor_ids(void)
{
	hmotor1.MotorID = 5;
    hmotor2.MotorID = 4;
    hmotor3.MotorID = 6;
    hmotor4.MotorID = 7;
    hmotor5.MotorID = 9;
    hmotor6.MotorID = 8;
    hmotor7.MotorID = 2;
    hmotor8.MotorID = 3;
}

// 电机通信测试，使其匀速扫过一定范围的角度
void MotorTest_Sweep(int id, float step)
{
    Motor_HandleTypeDef* motor = NULL;
    float bias = 0;
    int is_positive = 1;  // 1:正角度电机, 0:负角度电机
    
    // 根据ID配置电机参数
    switch(id)
    {
        case 1: motor = &hmotor1; bias = motor1_bias; is_positive = 1; break;
        case 2: motor = &hmotor2; bias = motor2_bias; is_positive = 1; break;
        case 3: motor = &hmotor3; bias = motor3_bias; is_positive = 1; break;
        case 4: motor = &hmotor4; bias = motor4_bias; is_positive = 1; break;
        case 5: motor = &hmotor5; bias = motor5_bias; is_positive = 0; break;
        case 6: motor = &hmotor6; bias = motor6_bias; is_positive = 0; break;
        case 7: motor = &hmotor7; bias = motor7_bias; is_positive = 0; break;
        case 8: motor = &hmotor8; bias = motor8_bias; is_positive = 0; break;
        default: return;
    }
    
    // 正角度电机 (1-4)
    if(is_positive)
    {
        for(float angle = 20.0f; angle <= 110.0f; angle += step)
        {
            motor->Theta_des = bias  + 6.28f * angle / 360.0f;
            Motor_SendCmd(motor);
            HAL_Delay(1);
        }
        for(float angle = 110.0f; angle >= 20.0f; angle -= step)
        {
            motor->Theta_des = bias  + 6.28f * angle / 360.0f;
            Motor_SendCmd(motor);
            HAL_Delay(1);
        }
    }
    // 负角度电机 (5-8)
    else
    {
        for(float angle = -20.0f; angle >= -100.0f; angle -= step)
        {
            motor->Theta_des = bias  + 6.28f * angle / 360.0f;
            Motor_SendCmd(motor);
            HAL_Delay(1);
        }
        for(float angle = -100.0f; angle <= -20.0f; angle += step)
        {
            motor->Theta_des = bias  + 6.28f * angle / 360.0f;
            Motor_SendCmd(motor);
            HAL_Delay(1);
        }
    }
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

    // 获取偏置值指针
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

        #define MAX_RETRY 3
        uint8_t success = 0;

        for (int retry = 0; retry < MAX_RETRY; retry++)
        {
            HAL_UART_Transmit(&huart6, InitArray, 17, 100);

            if (HAL_UART_Receive(&huart6, RxArray, 16, 100) == HAL_OK)
            {
                // 校验包头
                if (RxArray[0] == 0xFD && RxArray[1] == 0xEE)
                {
                    success = 1;

                    int32_t raw_value = (int32_t)(
                        ((uint32_t)RxArray[7]) |
                        ((uint32_t)RxArray[8] << 8) |
                        ((uint32_t)RxArray[9] << 16) |
                        ((uint32_t)RxArray[10] << 24)
                    );            
                    float bias_value = (float)raw_value / 32768.0f * 2.0f * PI;
                    
                    // 数据有效性检查
                    // if (bias_value >= -6.28f && bias_value <= 6.28f) {
                    //     *(motor_bias_ptrs[i]) = bias_value;
                    // } 
                    *(motor_bias_ptrs[i]) = bias_value;

                    break;
                }
            }

            HAL_Delay(2);
        }

        // if (!success)
        // {
        //     // 这里你可以：
        //     // 1. 标记这个电机通信失败
        //     // 2. 或者继续下一个电机
        //     return;
        // }
    }
}

// 让电机放松
void motor_release()
{
//	uint8_t InitArray[] = { 0xFE, 0xEE, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x9A};	
//	unitree_crc_complete(InitArray);
//	HAL_UART_Transmit(&huart6, InitArray, 17, 1000);

    hmotor1.Tau_ff = 0.0f;
    hmotor1.Kp = 0.0f;
    hmotor1.Kw = 0.0f;	
    hmotor2.Tau_ff = 0.0f;
    hmotor2.Kp = 0.0f;
    hmotor2.Kw = 0.0f;	
    hmotor3.Tau_ff = 0.0f;
    hmotor3.Kp = 0.0f;
    hmotor3.Kw = 0.0f;	
    hmotor4.Tau_ff = 0.0f;
    hmotor4.Kp = 0.0f;
    hmotor4.Kw = 0.0f;
    hmotor5.Tau_ff = 0.0f;
    hmotor5.Kp = 0.0f;
    hmotor5.Kw = 0.0f;	
    hmotor6.Tau_ff = 0.0f;
    hmotor6.Kp = 0.0f;
    hmotor6.Kw = 0.0f;	
    hmotor7.Tau_ff = 0.0f;
    hmotor7.Kp = 0.0f;
    hmotor7.Kw = 0.0f;	
    hmotor8.Tau_ff = 0.0f;
    hmotor8.Kp = 0.0f;
    hmotor8.Kw = 0.0f;
//	  hmotor10.Tau_ff = 0.0f;
//    hmotor10.Kp = 0.0f;
//    hmotor10.Kw = 0.0f;
//	
  //  inverseKinematic_All();
    Motor_SendCmd_AllAngle();  

}


void Motor_SendCmd(Motor_HandleTypeDef *hmotor)
{
//    if (Motor_PackCmd(hmotor) != HAL_OK){
//        //
//    };
	Motor_PackCmd(hmotor);

	HAL_UART_Transmit(&huart6, hmotor->TxData, 17, 1000);

	Motor_Feedback_Process();    
  Motor_Feedback_TimeoutTask();
    //RS485_SendFrame_Blocking(&huart6,hmotor->TxData);
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

// 给云台的宇树电机发送信号
void gimbal_send_unitree(float angel){

//    hmotor10.Theta_des = motor10_bias / 6.33f + 6.28f * angel / 360.0f;
	hmotor10.Theta_des = motor10_bias  + 6.28f * angel / 360.0f;
	Motor_SendCmd(&hmotor10);
	MY_delay_us(TRANSNIT_DELAY);
	  Motor_Feedback_Process();    
    Motor_Feedback_TimeoutTask();
 		

}

/*
给腿部的宇树电机发送信号
hposition1 : hmotor1(α) , hmotor2(β)
hposition2 : hmotor3(α) , hmotor4(β)
hposition3 : hmotor5(α) , hmotor6(β)
hposition4 : hmotor7(α) , hmotor8(β)
*/
void Motor_SendCmd_AllAngle()
{
//    hmotor1.Theta_des = motor1_bias / 6.33f + 6.28 * 0.0f / 360.0f;
//    hmotor2.Theta_des = motor2_bias / 6.33f + 6.28 * 0.0f / 360.0f;
//    hmotor3.Theta_des = motor3_bias / 6.33f + 6.28 * 0.0f / 360.0f;
//    hmotor4.Theta_des = motor4_bias / 6.33f + 6.28f * 0.0f / 360.0f;
//    hmotor5.Theta_des = motor5_bias / 6.33f + 6.28f * 0.0f / 360.0f;
//    hmotor6.Theta_des = motor6_bias / 6.33f + 6.28f * 0.0f / 360.0f;
//    hmotor7.Theta_des = motor7_bias / 6.33f + 6.28f * 0.0f / 360.0f;
//    hmotor8.Theta_des = motor8_bias / 6.33f + 6.28f * 0.0f / 360.0f;
   
	hmotor1.Theta_des = motor1_bias + 6.28f * -hposition1.alpha / 360.0f - flip_offset;
	hmotor2.Theta_des = motor2_bias + 6.28f * -hposition1.beta / 360.0f + flip_offset;
	hmotor3.Theta_des = motor3_bias + 6.28f * hposition2.alpha / 360.0f + flip_offset;
	hmotor4.Theta_des = motor4_bias + 6.28f * hposition2.beta / 360.0f - flip_offset;
	hmotor5.Theta_des = motor5_bias + 6.28f * -hposition3.alpha / 360.0f - flip_offset;
	hmotor6.Theta_des = motor6_bias + 6.28f * -hposition3.beta / 360.0f + flip_offset;
	hmotor7.Theta_des = motor7_bias + 6.28f * hposition4.alpha / 360.0f + flip_offset;
	hmotor8.Theta_des = motor8_bias + 6.28f * hposition4.beta / 360.0f - flip_offset;
	
	Motor_SendCmd(&hmotor1);
	MY_delay_us(TRANSNIT_DELAY);	
	Motor_SendCmd(&hmotor3);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor8);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor2);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor7);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor6);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor4);
	MY_delay_us(TRANSNIT_DELAY);
	Motor_SendCmd(&hmotor5);
	MY_delay_us(TRANSNIT_DELAY);
	
	Motor_Feedback_Process();    
    Motor_Feedback_TimeoutTask();	
}

// ---------4310控制代码开始------------

void motor_4310_init(){

}


void gimbal_send_4310(float angel){

}



// ---------4310控制代码结束------------

