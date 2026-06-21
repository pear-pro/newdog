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
只要写PWM_Set(PWM_OUT);就会吸合电磁铁给箱子充磁，请注释掉，您可以根据需要在程序的不同状态下调用PWM_Set函数来控制吸盘吸取和释放动作。
PWM_Set(PWM_OUT);
就能控制电磁铁吸取和释放了，PWM_IDLE 是待机状态（输出通道均低电平），此时吸盘在初始位置不动。

*/

void PWM_Set(PWM_State_e mode) {
    switch (mode) {
        case PWM_IN:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_HIGH);     // C路: 吸合
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_LOW);      // B路: 关闭
            break;
        case PWM_OUT:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_LOW);       // C路: 关闭
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_HIGH);      // B路: 吸合
            break;
        case PWM_IDLE:
        default:
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_SERVO_LOW);
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_SERVO_LOW);
            break;
    }
}
