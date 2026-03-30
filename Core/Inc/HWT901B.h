/**
 * @file    hwt901B.h
 * @brief   HWT901B 陀螺仪 CAN 总线驱动 (终极解析版)
 * @note    适用波特率: 250kbps | 节点ID: 0x50 | 帧头: 0x55
 */

#ifndef __HWT901B_H
#define __HWT901B_H

#include "main.h"
#include "can.h"

/**
 * @brief 陀螺仪核心数据结构体
 * @note  包含三轴加速度(g)、三轴角速度(°/s)和三轴欧拉角(°)
 */
typedef struct
{
    /* 欧拉角 (单位: 度 °) */
    float Roll;   // 横滚角
    float Pitch;  // 俯仰角
    float Yaw;    // 偏航角

    /* 角速度 (单位: 度/秒 °/s) */
    float Wx;
    float Wy;
    float Wz;

    /* 加速度 (单位: g, 1g = 9.8m/s^2) */
    float Ax;
    float Ay;
    float Az;

    /* 状态指示：记录收到了多少个有效帧，可用于心跳监控 */
    uint32_t FrameCount;

} HWT901B_Data_t;

/* ---------------- 外部全局变量声明 ---------------- */
/* 声明给 main.c 等其他文件读取姿态数据使用 */
extern volatile HWT901B_Data_t HWT901B_Data;

/* ---------------- API 接口函数声明 ---------------- */

/**
 * @brief  初始化 HWT901B 硬件过滤器 (精准过滤 0x50)
 * @param  hcan: CAN句柄指针
 */
void HWT901B_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  HWT901B 核心数据解析回调函数 (放置于 HAL_CAN_RxFifo0MsgPendingCallback 中)
 * @param  hcan: CAN句柄指针
 */
void HWT901B_CAN_RxCallback(CAN_HandleTypeDef *hcan);


#endif /* __HWT901B_H */
