#ifndef PWM_APP_H
#define PWM_APP_H

#include "main.h"

/* 舵机信号标准定义 (1us/tick) */
#define PWM_SERVO_HIGH  25000 // 25ms 吸合（高电平吸合）
#define PWM_SERVO_LOW   2500   // 2.5ms 关闭（高电平关闭）

typedef enum {
    PWM_IDLE = 0, // 待机
    PWM_IN   = 1, // 吸取 (B路)
    PWM_OUT  = 2  // 释放 (C路)
} PWM_State_e;

void PWM_Init(void);
void PWM_Set(PWM_State_e mode);

#endif
