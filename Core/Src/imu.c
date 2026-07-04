
/**
 * @file    IMU.c
 * @brief   维特智能 HWT901B 姿态传感器 CAN 协议栈
 * @note    基于旧版本 coreimu，实际使用 0x50 标识符 + 0x55 帧头透传协议封装
 */
#include "IMU.h"
#include "gpio.h" // GPIO探针引脚定义
#include "main.h" // 用于获取系统时钟 HAL_GetTick()
#include <math.h>

/* 强制声明为 volatile，防止编译器优化导致取值异常。
 * 确保应用层算法每次读取的都是中断刚刚写入 RAM 的最新数据。
 */

volatile IMU_Info_t IMU_rx_data = {0};

// ---------- Y轴速度 ---------
volatile float VeloY=0.0f;

// ---- 水平平衡PID参数 ---
float stab_roll = 0.0f;
float kp_roll = 0.01f;
float kd_roll = 0.001f;

// ---- 姿态角 ---------------
volatile float body_roll = 0.0f;
volatile float body_pitch = 0.0f;
volatile float body_yaw = 0.0f;
float prev_body_roll = 0.0f;

// ---- 角速度 ---------------
volatile float GyroX=0.0f;
volatile float GyroY=0.0f;
volatile float GyroZ=0.0f;

// ---- 加速度 ---------------
volatile float AccX=0.0f;
volatile float AccY=0.0f;
volatile float AccZ=0.0f;

/**
 * @brief IMU 数据接收完成后的应用层处理回调
 * @details
 *   - 从接收缓冲区 (IMU_rx_data) 复制到应用层全局变量
 *   - 在中断上下文中以最高优先级执行
 *   - 用于后续滤波、姿态融合等实时算法
 */
static void IMU_App_Update(void)
{
    /* 将最新接收的欧拉角复制到应用层全局变量，加 isfinite 检查防止 NaN/inf 传播 */
    if (isfinite(IMU_rx_data.Roll))  body_roll  = IMU_rx_data.Roll;
    if (isfinite(IMU_rx_data.Pitch)) body_pitch = IMU_rx_data.Pitch - 90.0f;
    if (isfinite(IMU_rx_data.Yaw))   body_yaw   = IMU_rx_data.Yaw;

    if (isfinite(IMU_rx_data.GyroX)) GyroX = IMU_rx_data.GyroX;
    if (isfinite(IMU_rx_data.GyroY)) GyroY = IMU_rx_data.GyroY;
    if (isfinite(IMU_rx_data.GyroZ)) GyroZ = IMU_rx_data.GyroZ;

    if (isfinite(IMU_rx_data.AccX)) AccX = IMU_rx_data.AccX;
    if (isfinite(IMU_rx_data.AccY)) AccY = IMU_rx_data.AccY;
    if (isfinite(IMU_rx_data.AccZ)) AccZ = IMU_rx_data.AccZ;

    /* 以下为实时计算预留位置（如 PID 控制、卡尔曼滤波等），
     * 确保所有数值运算都在高频率中断中完成 */

    /* 待实现的实时计算
     * 例如：横滚稳定度的 PID 控制（PID计算还未完成，先放个占位）
     * kp_roll = body_roll * 0.6f + ...
     */
    // stab_roll = body_roll * kp_roll + ...
}


/**
 * @brief  HWT901B CAN 协议栈超快速的接收回调
 * @param  hcan: CAN 句柄指针
 * @note   该函数运行在极高频的 CAN RX0 中断服务程序中，严禁在此处使用耗时的 printf。
 */

void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    int16_t raw_x, raw_y, raw_z;

    if (hcan->Instance == CAN2)
    {

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
                /* 维特标准协议校验：帧头必须为 0x55 */
              if ( rx_header.IDE == CAN_ID_STD && rx_data[0] == 0x55 && rx_header.StdId == 0x50)
                {
                    /* 刷新全局时间戳和帧计数器，为上层应用/离线分析提供时间基准 */
                    IMU_rx_data.last_update_time = HAL_GetTick();
                    IMU_rx_data.FrameCount++;

                   /* 按帧类型分发处理 */
                 switch (rx_data[1])
                  {                /* 使用预计算的除法倒数，大幅压榨 FPU 开销 */
                        case 0x51: /* 加速度帧 (Ax, Ay, Az) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);


                            // 量程 16g，原始值为 16 位有符号整数，范围 -32768~32767
                            IMU_rx_data.AccX = (float)raw_x * IMU_ACC_RATIO;
                            IMU_rx_data.AccY = (float)raw_y * IMU_ACC_RATIO;
                            IMU_rx_data.AccZ = (float)raw_z * IMU_ACC_RATIO;
                            break;

                        case 0x52: /* 角速度帧 (Wx, Wy, Wz) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);

                            // 量程 2000°/s
                            IMU_rx_data.GyroX = (float)raw_x * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroY = (float)raw_y * IMU_GYRO_RATIO;
                            IMU_rx_data.GyroZ = (float)raw_z * IMU_GYRO_RATIO;
                            break;

                        case 0x53: /* 欧拉角帧 (Roll, Pitch, Yaw) */
                            raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                            raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                            raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);

                            // 量程 180°
                            IMU_rx_data.Roll  = (float)raw_x * IMU_ANGLE_RATIO;
                            IMU_rx_data.Pitch = (float)raw_y * IMU_ANGLE_RATIO;
                            IMU_rx_data.Yaw   = (float)raw_z * IMU_ANGLE_RATIO;

                            /* 角度帧更新后立即刷新应用层变量 */
                            IMU_App_Update();
                            break;

                        default:
                            /* 未支持帧类型 (如磁场等)，目前系统对此无需求，直接丢弃
                               如需扩展，请在此添加处理代码
                            */
                            break;
                    }

                    /* ==============================================================
                     * 调试用 LED 翻转：每收到 20 帧翻转一次电平
                     * 用途：肉眼观察 GPIO_PIN_14(PF14) 的闪烁频率，验证 IMU CAN 通信是否正常
                     * 原理：每收到 20 帧消息翻转一次，正常通信时约 50Hz 的翻转频率
                     * ============================================================== */
                    if (IMU_rx_data.FrameCount >= 20)
                    {
                        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
                        IMU_rx_data.FrameCount = 0;
                    }

               }
           }

      }
}
