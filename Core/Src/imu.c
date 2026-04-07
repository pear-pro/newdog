
/**
 * @file    IMU.c
 * @brief   维特智能 HWT901B 姿态传感器 CAN 协议解析
 * @note    已适配 coreimu 工程实际使用的 0x50 标识符 + 0x55 帧头透传协议封装
 */
#include "IMU.h"
#include "gpio.h" // 引入探针引脚定义
#include "main.h" // 引入获取系统时间的 HAL_GetTick() 

/* * 强制声明为 volatile，防御编译器激进优化陷阱。
 * 确保应用层算法每次读取到的都是中断刚刚写入 RAM 的鲜活数据。
 */

volatile IMU_Info_t IMU_rx_data = {0};


float body_roll = 0.0f; 
float body_pitch = 0.0f; 
float body_yaw = 0.0f;

float stab_roll = 0.0f; 
float kp_roll = 0.0f;
/**
 * @brief IMU 数据接收完成后的应用层处理回调
 * @details
 *   - 从接收缓冲区 (IMU_rx_data) 更新应用层全局变量
 *   - 在中断上下文中高优先级执行
 *   - 可在此添加滤波、补偿、融合等实时算法
 */
static void IMU_App_Update(void)
{
    /* 将最新接收的欧拉角复制到应用层全局变量 */
    body_roll = IMU_rx_data.Roll;
    body_pitch = IMU_rx_data.Pitch;
    body_yaw = IMU_rx_data.Yaw;
    
    /* 可在此添加实时计算
     * 例如：基于稳定余度的 PID 调整，后续PID计算还没完成，先放个占位
     * kp_roll = body_roll * 0.6f + ...
     */
    // stab_roll = body_roll * kp_roll + ...
}


/**
 * @brief  HWT901B CAN 协议超高速报文解析函数
 * @param  can_id: CAN 标准帧 ID (已在上一层路由校验，此处作为占位，保留未来扩展)
 * @param  rx_data:   8 字节的 CAN 数据域指针
 * @note   该函数运行在极高频的 CAN RX0 中断上下文中，严禁在此添加延时或 printf！
 */

void IMU_CAN_RXCALLback(CAN_HandleTypeDef *hcan)
{

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
     int16_t raw_x, raw_y, raw_z;
    
    if (hcan->Instance == CAN1)
    {
       
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {                       
                /* 维特标准协议二次校验：帧头必须为 0x55 */           
              if ( rx_header.IDE == CAN_ID_STD && rx_data[0] == 0x55 && rx_header.StdId == 0x50)
                {
                    /* 刷新全局时间戳与帧计数，为上层死机/离线看门狗提供判定依据 */
                    IMU_rx_data.last_update_time = HAL_GetTick();
                    IMU_rx_data.FrameCount++;

                   /* 解析不同报文类型 */
                 switch (rx_data[1])
                  {                /* 使用编译期折叠乘法代替运行时除法，压榨 FPU 性能 */
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
                            break;

                        default:
                            /* 其他报文 (如磁场等) 目前系统无需关注，直接放行
                            后续若有需求可再添加处理
                            */
                            break;
                    }
                    
                    /* 数据完成后，立即更新应用层变量 */
                    /* 将 IMU_rx_data 接收到的数据赋值给应用层变量，用于后续控制计算 */
                    IMU_App_Update();
                            
                    /* ==============================================================
                     * 探针低开销翻转：计数到 20 帧反转一次引脚
                     * 用途：肉眼观察 GPIO_PIN_14（PF14）的闪烁频率，评估 IMU CAN 通信健康度
                     * 原理：每收到 20 帧消息翻转一次，健康通信时约 50Hz 翻转频率
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

