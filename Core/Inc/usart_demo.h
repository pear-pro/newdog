/**
 ******************************************************************************
 * @file    usart_demo.h
 * @brief   UART8 串口通信模块 - 树莓派速度+摄像头指令接收
 ******************************************************************************
 *
 * 【协议格式】14 字节定长帧，收发统一格式
 *
 *   +--------+--------+--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+--------+
 *   | Byte0  | Byte1  | Byte2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  | 10  | 11  | 12  |  13    |
 *   +--------+--------+--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+--------+
 *   | 0x55   | 0xAA   | CTRL   |V_HIGH|V_LOW|W_HIGH|W_LOW|X_HIGH|X_LOW|Y_HIGH|Y_LOW|Z_HIGH|Z_LOW| CKSUM  |
 *   +--------+--------+--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+-----+--------+
 *
 *   - 帧头：0x55 + 0xAA（双字节同步，无需转义）
 *   - CTRL_MODE：控制指令类型（0x01=速度+摄像头）
 *   - V：目标线速度，16 位大端序，缩放因子 /10000
 *   - W：目标角速度，16 位大端序，缩放因子 /10000
 *   - X/Y/Z：摄像头识别物体三轴坐标，16 位大端序，缩放因子 /100（单位：米）
 *   - 校验和：(Byte2 ~ Byte12) 之和的低 8 位，位于 Byte13
 *
 * 【指令报文】树莓派 → STM32
 *   树莓派下发目标线速度 V、角速度 W 及摄像头 X/Y/Z 坐标
 *
 ******************************************************************************
 */

#ifndef USART_DEMO_H
#define USART_DEMO_H

#include "main.h"

/* ========================================================================== */
/*                             环形缓冲区配置                                    */
/* ========================================================================== */

#define USART_DEMO_RX_BUF_SIZE    256u   /* DMA 接收缓冲区大小，必须是 2 的幂 */
#define USART_DEMO_RX_BUF_MASK    (USART_DEMO_RX_BUF_SIZE - 1u)


/* ========================================================================== */
/*                            协议常量定义                                      */
/* ========================================================================== */

/* --- 帧头（双字节同步，无需转义） --- */
#define FRAME_HEADER_1      0x55u   /* 帧头第 1 字节 */
#define FRAME_HEADER_2      0xAAu   /* 帧头第 2 字节 */

/* --- 帧结构 --- */
#define FRAME_LEN           14u     /* 帧总长度（速度+摄像头） */
#define FRAME_DATA_START    2u      /* 数据起始偏移（跳过帧头） */

/* --- 字节位置索引 --- */
#define POS_HEAD1           0u      /* Byte0:  帧头 0x55 */
#define POS_HEAD2           1u      /* Byte1:  帧头 0xAA */
#define POS_CTRL            2u      /* Byte2:  控制指令类型 */
#define POS_V_HIGH          3u      /* Byte3:  线速度高字节 */
#define POS_V_LOW           4u      /* Byte4:  线速度低字节 */
#define POS_W_HIGH          5u      /* Byte5:  角速度高字节 */
#define POS_W_LOW           6u      /* Byte6:  角速度低字节 */
#define POS_CAM_X_HIGH      7u      /* Byte7:  摄像头X轴高字节 */
#define POS_CAM_X_LOW       8u      /* Byte8:  摄像头X轴低字节 */
#define POS_CAM_Y_HIGH      9u      /* Byte9:  摄像头Y轴高字节 */
#define POS_CAM_Y_LOW       10u     /* Byte10: 摄像头Y轴低字节 */
#define POS_CAM_Z_HIGH      11u     /* Byte11: 摄像头Z轴高字节 */
#define POS_CAM_Z_LOW       12u     /* Byte12: 摄像头Z轴低字节 */
#define POS_CKSUM           13u     /* Byte13: 校验和 */

/* --- 控制指令类型 --- */
#define CTRL_SPEED          0x01u   /* 速度+摄像头模式 */

/* --- 发送超时 --- */
#define TX_TIMEOUT_MS       10u     /* 发送超时（ms） */


/* ========================================================================== */
/*                           协议解析器状态机                                    */
/* ========================================================================== */

/**
 * @brief 解析器状态枚举
 *
 *   PARSE_SYNC_1 : 等待第 1 帧头 0x55
 *   PARSE_SYNC_2 : 等待第 2 帧头 0xAA
 *   PARSE_COLLECT: 收集后续 12 字节（CTRL + V + W + X + Y + Z + CKSUM）
 */
typedef enum {
    PARSE_SYNC_1 = 0,   /* 等待 0x55 */
    PARSE_SYNC_2,        /* 等待 0xAA */
    PARSE_COLLECT        /* 收集数据 */
} ParseState_TypeDef;

/**
 * @brief 解析器上下文
 */
typedef struct {
    ParseState_TypeDef state;       /* 当前解析状态 */
    uint8_t  buf[FRAME_LEN];       /* 完整帧缓冲区（含帧头） */
    uint8_t  idx;                   /* 当前写入位置（从 2 开始） */
} Parser_TypeDef;

/**
 * @brief 解析结果
 */
typedef struct {
    uint8_t  ctrl_mode;     /* 控制指令类型 */
    int16_t  velocity_v;    /* 目标线速度（有符号 16 位） */
    int16_t  velocity_w;    /* 目标角速度（有符号 16 位） */
    int16_t  camera_x;      /* 摄像头X轴坐标（有符号 16 位，/100 = 米） */
    int16_t  camera_y;      /* 摄像头Y轴坐标（有符号 16 位，/100 = 米） */
    int16_t  camera_z;      /* 摄像头Z轴坐标（有符号 16 位，/100 = 米） */
} CmdFrame_TypeDef;


/* ========================================================================== */
/*                             公共 API 接口                                    */
/* ========================================================================== */

void UART8_Demo_Init(void);
void UART8_Demo_Process(void);
void UART8_Demo_SendResponse(uint8_t ctrl_mode, int16_t v, int16_t w,
                             int16_t cam_x, int16_t cam_y, int16_t cam_z);

extern volatile uint8_t uart8_idle_flag;
extern volatile uint8_t uart8_walk_request;
extern float front_speed;
extern float turn_omega;

/* 摄像头目标坐标（单位：米，原始值/100） */
extern volatile int16_t camera_obj_x;
extern volatile int16_t camera_obj_y;
extern volatile int16_t camera_obj_z;
extern volatile uint8_t camera_data_fresh;  /* 1=有新摄像头数据 */

#endif /* USART_DEMO_H */
