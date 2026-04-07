#ifndef PWM_APP_H
#define PWM_APP_H

#include "main.h"

/* 伺服信号标准定义 (1us/tick) */
#define PWM_SERVO_HIGH  25000 // 25ms 脉宽：逻辑“开”
#define PWM_SERVO_LOW   2500   // 2.5ms 脉宽：逻辑“关”

typedef enum {
    PWM_IDLE = 0, // 待机
    PWM_IN   = 1, // 吸取 (B路开)
    PWM_OUT  = 2  // 释放 (C路开)
} PWM_State_e;

void PWM_Init(void);
void PWM_Set(PWM_State_e mode);

#endif
