#ifndef __MOTOR_FEEDBACK_H
#define __MOTOR_FEEDBACK_H


#include <stdint.h>

#define MOTOR_NUM        8
#define MOTOR_FB_LEN     16

extern volatile uint8_t motor_timeout_flag;

typedef struct
{
    uint8_t  id;
    uint8_t  status;

    int16_t  tau_raw;
    int16_t  omega_raw;
    int32_t  theta_raw;

    float    tau;     // N·m
    float    omega;   // rad/s
    float    theta;   // rad

    int8_t   temp;    // ℃
    uint8_t  error;   // MERROR
    uint16_t force;   // 0–4095

    uint8_t  online;
} Motor_Feedback_t;

/* 全局反馈数组 */
extern Motor_Feedback_t motor_fb[MOTOR_NUM];

/* 接口 */
void Motor_Feedback_Init(void);
void Motor_Feedback_RxByte(uint8_t byte);
void Motor_Feedback_TimeoutTask(void);
void Motor_Feedback_Process(void);//


#endif


