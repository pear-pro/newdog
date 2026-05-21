// global_vars.h
#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

// 包含结构体定义的头文件
#include "motor.h"
#include "gait.h"
#include "ht_10a_remote_control.h"


// 外部声明
extern Motor_HandleTypeDef hmotor1;
extern Motor_HandleTypeDef hmotor2;
extern Motor_HandleTypeDef hmotor3;
extern Motor_HandleTypeDef hmotor4;
extern Motor_HandleTypeDef hmotor5;
extern Motor_HandleTypeDef hmotor6;
extern Motor_HandleTypeDef hmotor7;
extern Motor_HandleTypeDef hmotor8;
extern Motor_HandleTypeDef hmotor10;

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
extern float motor10_bias;



// 遥控参数
extern Remote_Control_struct rcData;


#endif // GLOBAL_VARS_H
