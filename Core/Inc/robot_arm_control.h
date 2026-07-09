#ifndef ROBOT_ARM_CONTROL_H
 #define ROBOT_ARM_CONTROL_H
 
 #include "main.h"
 #include "motor_4310.h"
 #include "tim.h"
 #include <math.h>
 
 #define PI 3.1415926535f
 
 #define L1 14.83f//??
 #define L2 35.0f//??(???)
 #define L3 19.799f//??
// #define offset2 -20.0f//?????
// #define offset3 (-90.0f / 2)//?????
 #define offset2 0.0f
 #define offset3 0.0f
 #define offset1  80.0f
 
 extern motor_info_t damiao[4];
 extern TIM_HandleTypeDef htim8;
 extern float currentTheta1;

void Set_DM_Motor(uint8_t id, float angle_deg);
//uint8_t Arm_Move_To_Start(float x, float y, float z);
//uint8_t Arm_Move_To(float x, float y, float z,float *theta1, float *theta2,  float *theta3, float *servo_angle);
void  Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps);
void Set_Camera_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Set_Sucker_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Arm_Base_Move(float targetX, float targetY, uint16_t steps);

 #endif
 