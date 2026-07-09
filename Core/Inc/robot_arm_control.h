#ifndef ROBOT_ARM_CONTROL_H
 #define ROBOT_ARM_CONTROL_H

 #include "main.h"
 #include "motor_4310.h"
 #include "tim.h"
 #include <math.h>

 #define PI 3.1415926535f

 #define L1 14.83f//底座
 #define L2 35.0f//小臂（留疑问）
 #define L3 19.799f//大臂
 #define offset2 -10.0f//大臂偏置角
 #define offset3 (110.0f / 2)//小臂偏置角
 #define offset1  80.0f//云台偏置角

 extern float currentTheta1;

 extern motor_info_t damiao[4];
 extern TIM_HandleTypeDef htim8;

void Set_DM_Motor(uint8_t id, float angle_deg);
uint8_t Arm_Move_To_Start(float x, float y, float z);
uint8_t Arm_Move_To(float x, float y, float z);
void Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps);
void Set_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Set_Camera_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Set_Sucker_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Arm_Forward_Kinematics(float *x, float *y, float *z);
void Arm_Circle_YZ(float y_center, float z_center, float radius, uint16_t num_points);

 #endif
