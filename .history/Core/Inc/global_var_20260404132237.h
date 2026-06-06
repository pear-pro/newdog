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

extern motor1_bias = 5.80f;
extern motor2_bias = 3.53f;
extern motor3_bias = 3.57f;
extern motor4_bias = 0.19f;
extern motor5_bias = 4.40f;
extern motor6_bias = 3.78f;
extern motor7_bias = 0.10f;
extern motor8_bias = -1.09f;

extern move_state; // 运动状态
extern height;      // 支撑高度
extern step_height; // 摆动高度
extern stride;      // 步幅
#endif // GLOBAL_VARS_H
