/**
 ******************************************************************************
 * @file    usart_demo.c
 * @brief   UART8 串口通信模块 - 树莓派速度指令接收
 ******************************************************************************
 *
 * 【协议格式】8 字节定长帧，收发统一
 *
 *   [0x55] [0xAA] [CTRL] [V_H] [V_L] [W_H] [W_L] [CKSUM]
 *
 *   - 双字节帧头 0x55+0xAA 
 *   - 校验和 = (Byte2 + Byte3 + Byte4 + Byte5 + Byte6) & 0xFF
 *   - 速度值大端序（高字节在前）
 *
 * 【数据流】
 *
 *   RPi 下发速度指令 → DMA 循环接收 → IDLE 中断 → 主循环解析 → 应用速度 → 回复应答
 *
 * 【状态机】
 *
 *   PARSE_SYNC_1: 等待 0x55
 *   PARSE_SYNC_2: 等待 0xAA
 *   PARSE_COLLECT: 收集后续 6 字节 → 校验 → 分发
 *
 ******************************************************************************
 */

#include "usart_demo.h"
#include "usart.h"
#include "global_var.h"
#include <string.h>

/* rcData 定义在 ht_10a_remote_control.c，通过 global_var.h 获取类型定义 */
extern Remote_Control_struct rcData;

/* ========================================================================== */
/*                           模块内部变量                                       */
/* ========================================================================== */

/** DMA 接收缓冲区（硬件自动写入） */
static uint8_t dma_rx_buf[USART_DEMO_RX_BUF_SIZE];

/** 环形缓冲区尾指针（软件读取位置） */
static volatile uint16_t rx_tail = 0;

/** IDLE 中断标志（ISR 置 1，主循环清 0） */
volatile uint8_t uart8_idle_flag = 0;

/** 协议解析器实例 */
static Parser_TypeDef parser;

/** 超时保护：最后一次收到有效帧的时间戳 */
static uint32_t last_rx_tick = 0;

/** 树莓派速度指令到达标志：1=有新速度数据，请求进入行走状态；0=无请求 ，也就是中间变量，决定是否传入*/
volatile uint8_t uart8_walk_request = 0;

/* ---- 【调试变量】Watch 窗口查看以下变量即可判断通信状态 ---- */
volatile uint8_t  dbg_last_frame[8] = {0};   // 最近一帧原始字节（每次收到新帧会覆盖）
volatile uint8_t  dbg_frame_ok_count = 0;    // 校验通过的帧计数
volatile uint8_t  dbg_frame_err_count = 0;   // 校验失败的帧计数
volatile int16_t  dbg_last_v = 0;            // 最近一次解析到的线速度
volatile int16_t  dbg_last_w = 0;            // 最近一次解析到的角速度

/* ---- 【抓帧】Watch 窗口操作：dbg_freeze=1 冻结，=0 解冻 ---- */
volatile uint8_t  dbg_freeze = 1;            // 1=冻结快照，不再更新；0=正常跟踪（默认=1，自动抓首帧）
volatile uint8_t  dbg_frozen_frame[8] = {0}; // 被冻结的那帧数据
volatile int16_t  dbg_frozen_v = 0;          // 冻结时的线速度
volatile int16_t  dbg_frozen_w = 0;          // 冻结时的角速度
volatile uint32_t dbg_frozen_tick = 0;       // 冻结时的时间戳


/* ========================================================================== */
/*                     DMA 回调空操作                                           */
/* ========================================================================== */

/**
 * @brief  DMA 传输完成回调 - 空实现
 * HAL 默认在 DMA 完成时关闭 UART 接收，循环模式下需覆盖为空操作。
 */
static void DMA_NoOpCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
}


/* ========================================================================== */
/*                        环形缓冲区读取函数                                    */
/* ========================================================================== */

static uint16_t RingBuf_Available(void)
{
    uint16_t head = USART_DEMO_RX_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(huart8.hdmarx);
    return (head - rx_tail) & USART_DEMO_RX_BUF_MASK;   // 可用字节数
}

static uint8_t RingBuf_ReadByte(void)
{
    uint8_t byte = dma_rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & USART_DEMO_RX_BUF_MASK;
    return byte;
}


/* ========================================================================== */
/*                           工具函数                                           */
/* ========================================================================== */

/**
 * @brief  计算 8 位校验和
 * @param  buf: 完整帧缓冲区（8 字节）
 * @return (Byte2 + Byte3 + Byte4 + Byte5 + Byte6) & 0xFF
 */
static uint8_t CalcChecksum8(const uint8_t *buf)
{
    return (uint8_t)((buf[POS_CTRL] + buf[POS_V_HIGH] + buf[POS_V_LOW]
                    + buf[POS_W_HIGH] + buf[POS_W_LOW]) & 0xFFu);
}

/**
 * @brief  重置解析器到初始状态
 */
static void Parser_Reset(void)
{
    memset(&parser, 0, sizeof(parser));
}


/* ========================================================================== */
/*                      协议解析器 - 状态机实现                                  */
/* ========================================================================== */

/**
 * @brief  向解析器喂入一个字节
 *
 * 双字节帧头同步：先找 0x55，再找 0xAA，然后收集 6 字节数据。
 * 不需要转义机制（帧头 0x55+0xAA 双字节组合在正常数据中几乎不会出现）。
 *
 * @param  byte: 从缓冲区读取的下一个字节
 * @return 1 = 收到完整一帧, 0 = 还在收集中
 */
static uint8_t Parser_FeedByte(uint8_t byte)
{
    switch (parser.state) {

    case PARSE_SYNC_1:
        if (byte == FRAME_HEADER_1) {
            parser.state = PARSE_SYNC_2;
        }
        break;

    case PARSE_SYNC_2:
        if (byte == FRAME_HEADER_2) {
            parser.buf[POS_HEAD1] = FRAME_HEADER_1;
            parser.buf[POS_HEAD2] = FRAME_HEADER_2;
            parser.idx = FRAME_DATA_START;  /* 从 Byte2 开始收集 */
            parser.state = PARSE_COLLECT;
        } else if (byte == FRAME_HEADER_1) {
            /* 连续收到 0x55，保持在 SYNC_2 等待 0xAA */
        } else {
            parser.state = PARSE_SYNC_1;
        }
        break;

    case PARSE_COLLECT:
        if (byte == FRAME_HEADER_1) {
            /* 收集过程中遇到 0x55，可能是新帧头，重新同步 */
            parser.buf[0] = FRAME_HEADER_1;
            parser.idx = FRAME_DATA_START;
            parser.state = PARSE_SYNC_2;
        } else {
            parser.buf[parser.idx++] = byte;

            if (parser.idx >= FRAME_LEN) {
                return 1u;  /* 帧已完整 */
            }
        }
        break;
    }
    return 0u;
}

/**
 * @brief  验证帧校验和
 * @return 1 = 校验通过, 0 = 失败
 */
static uint8_t Parser_ValidateFrame(const uint8_t *buf)
{
    uint8_t cksum = CalcChecksum8(buf);
    return (buf[POS_CKSUM] == cksum) ? 1u : 0u;
}


/* ========================================================================== */
/*                          命令分发器                                          */
/* ========================================================================== */

/**
 * @brief  根据控制指令类型执行动作
 *
 * 当前支持：
 *   CTRL_SPEED (0x01): 速度下发模式，注入 rcData 控制行走
 */
static void DispatchCommand(const CmdFrame_TypeDef *cmd)
{
    switch (cmd->ctrl_mode) {
    case CTRL_SPEED:
        rcData.R_y  = (float)cmd->velocity_v / 10000.0f;  // 线速度 → 前进步幅 (±1.0)
        rcData.R_x  = (float)cmd->velocity_w / 10000.0f;  // 角速度 → 转向差速 (±1.0)
       //   rcData.sw5  = 0x0320;   // 触发行走状态
       //  rcData.sw7  = 0x0320;   // temp_state=1 → motion_Mix()
        uart8_walk_request = 1;   // 请求进入行走状态，如果你觉得不好
        last_rx_tick = HAL_GetTick();
        break;

    default:
        break;
    }
}


/* ========================================================================== */
/*                            公共 API 实现                                     */
/* ========================================================================== */

void UART8_Demo_Init(void)
{
    Parser_Reset();

    HAL_UART_Receive_DMA(&huart8, dma_rx_buf, USART_DEMO_RX_BUF_SIZE);

    huart8.hdmarx->XferCpltCallback     = DMA_NoOpCallback;//覆盖默认回调，保持 DMA 循环接收不中断
    huart8.hdmarx->XferHalfCpltCallback = DMA_NoOpCallback;

    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);//使能 UART8 的 IDLE 中断
}

/**
 * @brief  主循环处理函数（非阻塞）
 *
 * 流程：检查 IDLE 标志 → 逐字节读取 → 双字节帧头同步 → 收集 8 字节 → 校验 → 分发
 */
void UART8_Demo_Process(void)
{
    if (uart8_idle_flag) {
        uart8_idle_flag = 0;

        while (RingBuf_Available() > 0) {
            uint8_t byte = RingBuf_ReadByte();

            if (Parser_FeedByte(byte)) {  // 收到完整一帧
                memcpy((void*)dbg_last_frame, parser.buf, FRAME_LEN);  // 【调试】保存原始帧
                if (Parser_ValidateFrame(parser.buf)) {
                    CmdFrame_TypeDef cmd;
                    cmd.ctrl_mode  = parser.buf[POS_CTRL];
                    cmd.velocity_v = (int16_t)(((uint16_t)parser.buf[POS_V_HIGH] << 8)
                                            |   (uint16_t)parser.buf[POS_V_LOW]);
                    cmd.velocity_w = (int16_t)(((uint16_t)parser.buf[POS_W_HIGH] << 8)
                                            |   (uint16_t)parser.buf[POS_W_LOW]);
                    dbg_last_v = cmd.velocity_v;  // 【调试】记录解析结果
                    dbg_last_w = cmd.velocity_w;
                    dbg_frame_ok_count++;          // 【调试】成功帧计数
                    if (!dbg_freeze) {             // 【抓帧】未冻结时更新快照
                        memcpy((void*)dbg_frozen_frame, parser.buf, FRAME_LEN);
                        dbg_frozen_v   = cmd.velocity_v;
                        dbg_frozen_w   = cmd.velocity_w;
                        dbg_frozen_tick = HAL_GetTick();
                    }
                    DispatchCommand(&cmd);//把参数传递给分发器，执行对应的动作
                } else {
                    dbg_frame_err_count++;         // 【调试】校验失败帧计数
                }
                Parser_Reset();
            }
        }
    }

    // 超时保护：1000ms 无数据 → 回到站立（暂时禁用）
//    if (last_rx_tick != 0 && (HAL_GetTick() - last_rx_tick > 1000)) {
//        rcData.R_x = 0.0f;
//        rcData.R_y = 0.0f;
//        rcData.sw5 = 0x0000;  // temp_state=2 → 站立
//        last_rx_tick = 0;     // 防止重复触发
//    }
}
//预留的发送函数接口，当前未使用
/**
 * @brief  发送应答帧给树莓派
 *
 * 帧格式：[0x55] [0xAA] [CTRL] [V_H] [V_L] [W_H] [W_L] [CKSUM]
 *
 * @param  ctrl_mode: 控制指令类型（原样回传）
 * @param  v        : 目标线速度（大端序发送）
 * @param  w        : 目标角速度（大端序发送）
 */
void UART8_Demo_SendResponse(uint8_t ctrl_mode, int16_t v, int16_t w)
{
    uint8_t frame[FRAME_LEN];

    frame[POS_HEAD1]  = FRAME_HEADER_1;                         /* 0x55 */
    frame[POS_HEAD2]  = FRAME_HEADER_2;                         /* 0xAA */
    frame[POS_CTRL]   = ctrl_mode;                              /* 控制模式 */
    frame[POS_V_HIGH] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu); /* 线速度高字节 */
    frame[POS_V_LOW]  = (uint8_t)((uint16_t)v & 0xFFu);        /* 线速度低字节 */
    frame[POS_W_HIGH] = (uint8_t)(((uint16_t)w >> 8) & 0xFFu); /* 角速度高字节 */
    frame[POS_W_LOW]  = (uint8_t)((uint16_t)w & 0xFFu);        /* 角速度低字节 */
    frame[POS_CKSUM]  = CalcChecksum8(frame);                   /* 校验和 */

    HAL_UART_Transmit(&huart8, frame, FRAME_LEN, TX_TIMEOUT_MS);
}
