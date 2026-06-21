 #ifndef ROBOT_ARM_CONTROL_H
 #define ROBOT_ARM_CONTROL_H
 
 #include "main.h"
 #include "motor_4310.h"
 #include "tim.h"
 #include <math.h>
 
 #define PI 3.1415926535f
 
 #define L1 14.83f//底座
 #define L2 46.2f//小臂（留疑问）
 #define L3 19.799f//大臂，
 #define offset2 -50.0f//大臂偏置角
 #define offset3 -20.0f//小臂偏置角
 
 extern motor_info_t damiao[4];
 extern TIM_HandleTypeDef htim8;

void Set_DM_Motor(uint8_t id, float angle_deg);
uint8_t Arm_Move_To(float x, float y, float z);
void Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps);
void Set_Servo_Angle_TIM8(uint32_t Channel, float angle);

 #endif
 
 
 