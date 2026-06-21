/**
 * @file    imu.h
 * @brief   维特智能 HWT901B 姿态传感器模块头文件
 */
#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

#include "main.h"
#include "can.h"
/* ====================================================================
 * 传感器常量定义 (量纲转换系数)
 * 警告：建议不要在中断(IMU_CAN_RXCALLback)中使用浮点除法；
 * 通过预计算好的常数可将 FPU 指令减少数十倍，编译优化后约 1 个时钟周期。
 * ====================================================================
 */
#define IMU_ACC_RATIO   (16.0f / 32768.0f)     // 量程 16g
#define IMU_GYRO_RATIO  (2000.0f / 32768.0f)   // 量程 2000°/s
#define IMU_ANGLE_RATIO (180.0f / 32768.0f)    // 量程 180°

extern volatile float body_roll;
extern volatile float body_pitch;
extern volatile float body_yaw;
extern float prev_body_roll;

extern volatile float VeloY;

extern float stab_roll;
extern float kp_roll;
extern float kd_roll;

extern volatile float GyroX;
extern volatile float GyroY;
extern volatile float GyroZ;

extern volatile float AccX;
extern volatile float AccY;
extern volatile float AccZ;
/**
 * @brief 传感器姿态与元信息数据结构体
 */
typedef struct
{
    /* 加速度计 (单位: g) */
    float AccX;
    float AccY;
    float AccZ;

    /* 陀螺仪 (单位: °/s) */
    float GyroX;
    float GyroY;
    float GyroZ;

    /* 欧拉角 (单位: °) */
    float Roll;
    float Pitch;
    float Yaw;

    /* 元信息: 帧计数统计 */
    uint32_t FrameCount;
    /* 元信息: 最后一次成功刷新数据的时间戳 (ms) */
    uint32_t last_update_time;

} IMU_Info_t;

/* 全局暴露的 IMU 实例 (请注意 volatile 关键字) */
extern volatile IMU_Info_t IMU_rx_data;
static void IMU_App_Update(void);
/* 中断层级专用的加速回调函数 */
void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan);
#endif /* __IMU_H */
