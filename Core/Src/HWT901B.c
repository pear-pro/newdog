/**
 * @file    HWT901B.c
 * @brief   HWT901B 陀螺仪 CAN 总线驱动 (工业级最终交付版)
 * @note    适用波特率: 250kbps | 节点ID: 0x50
 * 包含精确硬件过滤与维特官方协议解包算法，内嵌高内聚物理探针
 */

#include "HWT901B.h"
#include "gpio.h" // 引入以便使用其他GPIO定义（如有）

/* 全局陀螺仪数据结构体实体（强制声明为 volatile，防御编译器寄存器优化陷阱） */
volatile HWT901B_Data_t HWT901B_Data = {0};

/**
 * @brief  初始化 HWT901B 硬件过滤器及探针
 */
void HWT901B_Init(CAN_HandleTypeDef *hcan)
{
    /* ==========================================================
     * 终极防御机制：强行在模块内部初始化 PF14 探针
     * 彻底断绝被 STM32CubeMX 覆盖或 gpio.c 遗漏导致引脚未上电的隐患
     * ========================================================== */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
    
    /* 初始灭灯 (大疆 A 板绿灯为低电平点亮，此处强制置高灭灯) */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
    /* ========================================================== */

    CAN_FilterTypeDef can_filter_st = {0};

    /* 启动严苛的硬件级过滤，只放行 ID 为 0x50 的标准帧 */
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    
    can_filter_st.FilterIdHigh = (0x50 << 5); 
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = (0x7FF << 5); 
    can_filter_st.FilterMaskIdLow = 0x0000;
    
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_st.SlaveStartFilterBank = 14; 

    if (HAL_CAN_ConfigFilter(hcan, &can_filter_st) != HAL_OK) {
        Error_Handler();
    }
    
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  HWT901B 核心数据解析回调函数 (由 CAN 接收中断触发)
 */
void HWT901B_CAN_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    int16_t raw_x, raw_y, raw_z;
    
    /* 中断内部分频器，用于物理探针显示 */
    static uint16_t led_prescaler = 0; 

    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            /* 二次软件校验：标准帧 && ID=0x50 && 帧头=0x55 */
            if (rx_header.IDE == CAN_ID_STD && rx_header.StdId == 0x50 && rx_data[0] == 0x55)
            {
                switch (rx_data[1])
                {
                    case 0x51: /* 解析加速度 (Ax, Ay, Az) */
                        raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                        raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                        raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                        HWT901B_Data.Ax = (float)raw_x / 32768.0f * 16.0f;
                        HWT901B_Data.Ay = (float)raw_y / 32768.0f * 16.0f;
                        HWT901B_Data.Az = (float)raw_z / 32768.0f * 16.0f;
                        break;

                    case 0x52: /* 解析角速度 (Wx, Wy, Wz) */
                        raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                        raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                        raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                        HWT901B_Data.Wx = (float)raw_x / 32768.0f * 2000.0f;
                        HWT901B_Data.Wy = (float)raw_y / 32768.0f * 2000.0f;
                        HWT901B_Data.Wz = (float)raw_z / 32768.0f * 2000.0f;
                        break;

                    case 0x53: /* 解析欧拉角 (Roll, Pitch, Yaw) */
                        raw_x = (int16_t)((rx_data[3] << 8) | rx_data[2]);
                        raw_y = (int16_t)((rx_data[5] << 8) | rx_data[4]);
                        raw_z = (int16_t)((rx_data[7] << 8) | rx_data[6]);
                        HWT901B_Data.Roll  = (float)raw_x / 32768.0f * 180.0f;
                        HWT901B_Data.Pitch = (float)raw_y / 32768.0f * 180.0f;
                        HWT901B_Data.Yaw   = (float)raw_z / 32768.0f * 180.0f;
                        break;

                    default:
                        break;
                }
                
                /* 心跳帧计数 */
                HWT901B_Data.FrameCount++;
                
                /* ==============================================================
                 * 探针翻转逻辑：压缩计数到 20 帧一闪，确保视觉可见度最大化
                 * ============================================================== */
                led_prescaler++;
                if (led_prescaler >= 20)
                {
                    HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
                    led_prescaler = 0;
                }
            }
        }
    }
}