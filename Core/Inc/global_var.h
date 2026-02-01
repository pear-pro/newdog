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

#define motor1_bias 0.0f
#define motor2_bias 0.0f
#define motor3_bias 0.0f // 目前阶段还没用到后面的电机
#define motor4_bias 3.70f
#define motor5_bias 5.80f
#define motor6_bias 0.0f
#define motor7_bias 0.0f
#define motor8_bias 0.0f

#endif // GLOBAL_VARS_H
