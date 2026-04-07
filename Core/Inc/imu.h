/**
 * @file    imu.h
 * @brief   维特智能 HWT901B 姿态传感器驱动头文件
 */
#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

#include "main.h"
#include "can.h"
/* * ====================================================================
 * 编译期常量折叠宏 (极速转换系数)
 * 警告：建议不要在中断（IMU_CAN_RXCALLback）中使用浮点除法。通过乘以预先计算好的常数，
 * 可将 FPU 指令周期由十几个时钟周期优化至 1 个时钟周期。
 * ====================================================================
 */
#define IMU_ACC_RATIO   (16.0f / 32768.0f)     // 量程 16g
#define IMU_GYRO_RATIO  (2000.0f / 32768.0f)   // 量程 2000°/s
#define IMU_ANGLE_RATIO (180.0f / 32768.0f)    // 量程 180°

extern float body_roll;
extern float body_pitch;
extern float body_yaw;

extern float stab_roll;
extern float kp_roll;

/**
 * @brief 核心姿态与环境数据结构体
 */
typedef struct
{
    /* 三轴加速度 (单位: g) */
    float AccX;
    float AccY;
    float AccZ;
    
    /* 三轴角速度 (单位: °/s) */
    float GyroX;
    float GyroY;
    float GyroZ;
    
    /* 三轴欧拉角 (单位: °) */
    float Roll;
    float Pitch;
    float Yaw;
    
    /* 诊断信息: 帧到达计数器 */
    uint32_t FrameCount;
    /* 诊断信息: 最后一次成功刷新数据的时间戳 (ms) */
    uint32_t last_update_time;

} IMU_Info_t;

/* 对外暴露的全局 IMU 实体 (必须带有 volatile 关键字) */
extern volatile IMU_Info_t IMU_rx_data;

/* 中断层级专用的极速解析函数 */
void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan);
#endif /* __IMU_H */
