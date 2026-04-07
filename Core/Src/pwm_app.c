#include "pwm_app.h"
#include "tim.h"

void PWM_Init(void) {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    PWM_Set(PWM_IDLE);
}

/*
只需要写
PWM_Set(PWM_IN);  or PWM_Set(PWM_OUT);
只要写PWM_Set(PWM_OUT);就会产生吸力，主函数有注释，后续可以根据需要在遥控器的不同状态下调用PWM_Set函数来控制吸取和释放动作。
PWM_Set(PWM_OUT);
就能控制舵机的吸取和释放了，PWM_IDLE 是待机状态，两个通道都输出低电平，舵机保持在初始位置不动。

*/

void PWM_Set(PWM_State_e mode) {
    switch (mode) {
        case PWM_IN:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_HIGH);     // C吸: 开启  
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_LOW);      // B放: 关闭
            break;
        case PWM_OUT:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_LOW);       // C吸: 关闭
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_HIGH);      // B放: 开启
            break;
        case PWM_IDLE:
        default:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_LOW);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_LOW);
            break;
    }
}
