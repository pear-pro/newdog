// global_vars.h
#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

// 包含结构体定义的头文件
#include "motor.h"
#include "gait.h"

// 外部声明
extern Motor_HandleTypeDef hmotor1;
extern Motor_HandleTypeDef hmotor2;
extern Motor_HandleTypeDef hmotor3;
extern Motor_HandleTypeDef hmotor4;
extern Motor_HandleTypeDef hmotor5;
extern Motor_HandleTypeDef hmotor6;
extern Motor_HandleTypeDef hmotor7;
extern Motor_HandleTypeDef hmotor8;

extern Position_HandleTypeDef hposition1;
extern Position_HandleTypeDef hposition2;
extern Position_HandleTypeDef hposition3;
extern Position_HandleTypeDef hposition4;

#define motor1_bias 5.15f
#define motor2_bias 3.44f
#define motor3_bias 3.90f 
#define motor4_bias 1.03f
#define motor5_bias 4.40f
#define motor6_bias 3.78f
#define motor7_bias 0.12f
#define motor8_bias 4.95f

extern int move_state; // 运动状态
extern float height;      // 支撑高度
extern float step_height; // 摆动高度
extern float stride;      // 步幅
#endif // GLOBAL_VARS_H
