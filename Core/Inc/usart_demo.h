/**
 ******************************************************************************
 * @file    usart_demo.h
 * @brief   UART8 串口通信模块 - 树莓派速度+摄像头指令接收
 ******************************************************************************
 *
 * 【协议格式】14 字节定长帧，收发统一格式
 *
 *   +------+------+------+------+------+------+------+------+------+------+------+------+------+------+
 *   | Byte0| Byte1| Byte2| Byte3| Byte4| Byte5| Byte6| Byte7| Byte8| Byte9|Byte10|Byte11|Byte12|Byte13|
 *   +------+------+------+------+------+------+------+------+------+------+------+------+------+------+
 *   | 0x55 | 0xAA | CTRL | V_H  | V_L  | W_H  | W_L  | X_H  | X_L  | Y_H  | Y_L  | Z_H  | Z_L  |CKSUM |
 *   +------+------+------+------+------+------+------+------+------+------+------+------+------+------+
 *
 *   - 帧头：0x55 + 0xAA（双字节同步，无需转义）
 *   - CTRL：控制指令类型（0x01=速度+摄像头下发）
 *   - V：目标线速度，16 位大端序，缩放 /10000
 *   - W：目标角速度，16 位大端序，缩放 /10000
 *   - X/Y/Z：摄像头目标坐标，16 位大端序，缩放 /100（单位：米）
 *   - 校验和：(Byte2 + ... + Byte12) & 0xFF，位于 Byte13
 *
 * 【指令报文】树莓派 → STM32
 *   树莓派下发目标线速度 V、角速度 W 和摄像头坐标 X/Y/Z
 *
 * 【应答报文】STM32 → 树莓派
 *   STM32 收到指令后回复应答帧
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
#define FRAME_LEN           14u     /* 帧总长度 */
#define FRAME_DATA_START    2u      /* 数据起始偏移（跳过帧头） */

/* --- 字节位置索引 --- */
#define POS_HEAD1           0u      /* Byte0:  帧头 0x55 */
#define POS_HEAD2           1u      /* Byte1:  帧头 0xAA */
#define POS_CTRL            2u      /* Byte2:  控制指令类型 */
#define POS_V_HIGH          3u      /* Byte3:  线速度高字节 */
#define POS_V_LOW           4u      /* Byte4:  线速度低字节 */
#define POS_W_HIGH          5u      /* Byte5:  角速度高字节 */
#define POS_W_LOW           6u      /* Byte6:  角速度低字节 */
#define POS_CAM_X_HIGH      7u      /* Byte7:  摄像头X高字节 */
#define POS_CAM_X_LOW       8u      /* Byte8:  摄像头X低字节 */
#define POS_CAM_Y_HIGH      9u      /* Byte9:  摄像头Y高字节 */
#define POS_CAM_Y_LOW      10u      /* Byte10: 摄像头Y低字节 */
#define POS_CAM_Z_HIGH     11u      /* Byte11: 摄像头Z高字节 */
#define POS_CAM_Z_LOW      12u      /* Byte12: 摄像头Z低字节 */
#define POS_CKSUM          13u      /* Byte13: 校验和 */

/* --- 控制指令类型 --- */
#define CTRL_SPEED          0x01u   /* 速度+摄像头下发模式 */

/* --- 发送超时 --- */
#define TX_TIMEOUT_MS       10u     /* 发送超时（ms） */


/* ========================================================================== */
/*                           协议解析器状态机                                    */
/* ========================================================================== */

/**
 * @brief 解析器状态枚举
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
    int16_t  camera_x;      /* 摄像头X坐标 */
    int16_t  camera_y;      /* 摄像头Y坐标 */
    int16_t  camera_z;      /* 摄像头Z坐标 */
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

/* 摄像头目标坐标（单位：米） */
extern float camera_obj_x;
extern float camera_obj_y;
extern float camera_obj_z;
extern volatile uint8_t camera_data_fresh;

#endif /* USART_DEMO_H */
