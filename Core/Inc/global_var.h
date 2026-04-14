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
extern Motor_HandleTypeDef hmotor9;

extern Position_HandleTypeDef hposition1;
extern Position_HandleTypeDef hposition2;
extern Position_HandleTypeDef hposition3;
extern Position_HandleTypeDef hposition4;

extern float motor1_bias;
extern float motor2_bias;
extern float motor3_bias;
extern float motor4_bias;
extern float motor5_bias;
extern float motor6_bias;
extern float motor7_bias;
extern float motor8_bias;

extern float Expect_Kp;
extern float EXpect_kw;
extern float Expect_Tau_ff;

// 遥控参数
#define rc_x_max 40.0f
#define rc_y_max 20.0f

extern int move_state; // 运动状态
extern float height;      // 支撑高度
extern float step_height; // 摆动高度
extern float stride;      // 步幅
extern float rc_x;
extern float rc_y;

#endif // GLOBAL_VARS_H
