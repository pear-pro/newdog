/**
 ******************************************************************************
 * @file    usart_demo.c
 * @brief   UART8 串口通信模块 - 树莓派速度指令接收（稳定恢复版）
 ******************************************************************************
 *
 * 协议：8 字节定长帧
 *   [0] 0x55
 *   [1] 0xAA
 *   [2] CTRL
 *   [3] V_H
 *   [4] V_L
 *   [5] W_H
 *   [6] W_L
 *   [7] CHECKSUM = (Byte2 + Byte3 + Byte4 + Byte5 + Byte6) & 0xFF
 *
 * 本版重点：
 *   1. 不依赖 IDLE 才解析，只要 DMA 缓冲区有字节就解析。
 *   2. 数据区遇到 0x55 不重新同步，避免合法数据导致丢帧。
 *   3. 自动检测 UART 错误 / DMA 接收关闭 / RxState 异常，并自动重启 UART8 DMA 接收。
 *   4. 中间停发、重新发，不需要重新打开串口，也不需要 MCU 复位。
 ******************************************************************************
 */

#include "usart_demo.h"
#include "usart.h"
#include "global_var.h"
#include <string.h>

/* rcData 定义在 ht_10a_remote_control.c，通过 global_var.h 获取类型定义 */
extern Remote_Control_struct rcData;

/* ========================================================================== */
/*                           可调参数                                           */
/* ========================================================================== */

/* 调通后如果想“断线自动停狗”，改成 1；当前先关闭，避免影响测试 */
#define UART8_ENABLE_LOST_TIMEOUT    0
#define UART8_LOST_TIMEOUT_MS        300u

/* 给运动层的速度限幅，保持原工程逻辑：[-1, 1] */
#define CMD_SPEED_LIMIT              1.0f

/* 是否在重启接收时强制把 DMA 模式设为 Circular。建议保持 1。 */
#define UART8_FORCE_DMA_CIRCULAR     1

/* ========================================================================== */
/*                           模块内部变量                                       */
/* ========================================================================== */

/** DMA 接收缓冲区：硬件 DMA 自动写入 */
static uint8_t dma_rx_buf[USART_DEMO_RX_BUF_SIZE];

/** 软件读取位置：指向下一个要读取的 DMA 缓冲区位置 */
static volatile uint16_t rx_tail = 0;

/** IDLE 中断标志：ISR 可置 1；本版本不依赖它解析，只保留用于调试 */
volatile uint8_t uart8_idle_flag = 0;

/** 协议解析器实例 */
static Parser_TypeDef parser;

/** 最后一次收到有效帧的时间戳 */
static uint32_t last_rx_tick = 0;

/** 树莓派速度指令到达标志：1=有新速度数据，请求进入行走状态 */
volatile uint8_t uart8_walk_request = 0;

/* ------------------------- 原有调试变量：保留名字 -------------------------- */
volatile uint8_t  dbg_last_frame[8] = {0};
volatile uint8_t  dbg_frame_ok_count = 0;       /* 8bit，会回绕；兼容原 Watch */
volatile uint8_t  dbg_frame_err_count = 0;      /* 8bit，会回绕；兼容原 Watch */
volatile int16_t  dbg_last_v = 0;
volatile int16_t  dbg_last_w = 0;

/* 默认不冻结，方便观察实时帧。需要抓某一帧时，在 Watch 手动改为 1。 */
volatile uint8_t  dbg_freeze = 0;
volatile uint8_t  dbg_frozen_frame[8] = {0};
volatile int16_t  dbg_frozen_v = 0;
volatile int16_t  dbg_frozen_w = 0;
volatile uint32_t dbg_frozen_tick = 0;

/* ------------------------- 新增调试变量：建议加入 Watch --------------------- */
volatile uint32_t dbg_process_count = 0;        /* UART8_Demo_Process() 调用次数 */
volatile uint32_t dbg_byte_count = 0;           /* 从 DMA 环形缓冲区取出的字节总数 */
volatile uint32_t dbg_frame_ok_total = 0;       /* 校验通过帧总数，32bit 不易回绕 */
volatile uint32_t dbg_frame_err_total = 0;      /* 校验失败帧总数，32bit 不易回绕 */
volatile uint32_t dbg_bad_ctrl_count = 0;       /* CTRL 不支持次数 */
volatile uint32_t dbg_uart_idle_count = 0;      /* IDLE 标志被主循环看到的次数 */
volatile uint32_t dbg_dma_head = 0;             /* DMA 当前写入位置 */
volatile uint32_t dbg_dma_tail = 0;             /* 软件读取位置 */
volatile uint32_t dbg_dma_avail = 0;            /* DMA 缓冲区可读字节数 */
volatile uint32_t dbg_dma_ndtr = 0;             /* DMA 剩余传输计数 */
volatile uint32_t dbg_dma_start_ret = 0;        /* HAL_UART_Receive_DMA 返回值 */
volatile uint32_t dbg_parser_state = 0;         /* 当前解析状态 */
volatile uint32_t dbg_parser_idx = 0;           /* 当前收集索引 */
volatile uint32_t dbg_uart_recover_count = 0;   /* 自动重启 UART8 DMA 接收次数 */
volatile uint32_t dbg_uart_error_code = 0;      /* 最近一次 UART 错误码快照 */
volatile uint32_t dbg_uart_rx_state = 0;        /* 最近一次 RxState 快照 */
volatile uint32_t dbg_uart_cr3 = 0;             /* 最近一次 CR3 快照 */
volatile uint32_t dbg_uart_recover_reason = 0;  /* 1=Init,2=Error,3=DMAR off,4=RxState异常 */

/* 最后一次校验失败的帧，便于看是校验错还是错位 */
volatile uint8_t  dbg_err_frame[8] = {0};

/* 分发给运动层的速度 */
float front_speed = 0.0f;
float turn_omega  = 0.0f;

/* ========================================================================== */
/*                           内部函数声明                                       */
/* ========================================================================== */

static void Parser_Reset(void);
static void UART8_Demo_RestartRx(uint32_t reason);

/* ========================================================================== */
/*                           内部工具函数                                       */
/* ========================================================================== */

/**
 * @brief DMA 完成回调空操作。
 */
static void DMA_NoOpCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
}

/**
 * @brief 重启 UART8 DMA 接收。
 * @param reason 调试用原因：1=Init,2=Error,3=DMAR off,4=RxState异常
 */
static void UART8_Demo_RestartRx(uint32_t reason)
{
    dbg_uart_recover_count++;
    dbg_uart_recover_reason = reason;
    dbg_uart_error_code = huart8.ErrorCode;
    dbg_uart_rx_state = huart8.RxState;
    dbg_uart_cr3 = huart8.Instance->CR3;

    /* 先停 DMA，防止旧状态残留 */
    HAL_UART_DMAStop(&huart8);

    /* 清 UART 常见错误标志：过载、帧错误、噪声、校验、空闲 */
    __HAL_UART_CLEAR_OREFLAG(&huart8);
    __HAL_UART_CLEAR_FEFLAG(&huart8);
    __HAL_UART_CLEAR_NEFLAG(&huart8);
    __HAL_UART_CLEAR_PEFLAG(&huart8);
    __HAL_UART_CLEAR_IDLEFLAG(&huart8);

    /* 恢复 HAL 状态，避免 Receive_DMA 返回 BUSY */
    huart8.ErrorCode = HAL_UART_ERROR_NONE;
    huart8.RxState = HAL_UART_STATE_READY;
    huart8.Lock = HAL_UNLOCKED;

    rx_tail = 0;
    Parser_Reset();
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

#if UART8_FORCE_DMA_CIRCULAR
    if (huart8.hdmarx != NULL) {
        huart8.hdmarx->Init.Mode = DMA_CIRCULAR;
        HAL_DMA_Init(huart8.hdmarx);
    }
#endif

    dbg_dma_start_ret = (uint32_t)HAL_UART_Receive_DMA(&huart8,
                                                       dma_rx_buf,
                                                       USART_DEMO_RX_BUF_SIZE);

    if (huart8.hdmarx != NULL) {
        huart8.hdmarx->XferCpltCallback     = DMA_NoOpCallback;
        huart8.hdmarx->XferHalfCpltCallback = DMA_NoOpCallback;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart8);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    dbg_uart_error_code = huart8.ErrorCode;
    dbg_uart_rx_state = huart8.RxState;
    dbg_uart_cr3 = huart8.Instance->CR3;
}

/**
 * @brief 获取 DMA 当前写入位置。
 * @return 0 ~ USART_DEMO_RX_BUF_SIZE-1
 */
static uint16_t RingBuf_GetHead(void)
{
    uint16_t ndtr;
    uint16_t head;

    if (huart8.hdmarx == NULL) {
        dbg_dma_ndtr = 0xFFFFFFFFu;
        return 0;
    }

    ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart8.hdmarx);
    if (ndtr > USART_DEMO_RX_BUF_SIZE) {
        ndtr = USART_DEMO_RX_BUF_SIZE;
    }

    head = (uint16_t)(USART_DEMO_RX_BUF_SIZE - ndtr);
    if (head >= USART_DEMO_RX_BUF_SIZE) {
        head = 0;
    }

    dbg_dma_ndtr = ndtr;
    dbg_dma_head = head;
    dbg_dma_tail = rx_tail;

    return head;
}

/**
 * @brief 计算 DMA 环形缓冲区当前可读字节数。
 *        通用写法，不要求 USART_DEMO_RX_BUF_SIZE 必须是 2 的整数次方。
 */
static uint16_t RingBuf_Available(void)
{
    uint16_t head = RingBuf_GetHead();
    uint16_t available;

    if (head >= rx_tail) {
        available = (uint16_t)(head - rx_tail);
    } else {
        available = (uint16_t)(USART_DEMO_RX_BUF_SIZE - rx_tail + head);
    }

    dbg_dma_avail = available;
    return available;
}

/**
 * @brief 从 DMA 环形缓冲区读取 1 字节。
 */
static uint8_t RingBuf_ReadByte(void)
{
    uint8_t byte = dma_rx_buf[rx_tail];

    rx_tail++;
    if (rx_tail >= USART_DEMO_RX_BUF_SIZE) {
        rx_tail = 0;
    }

    dbg_dma_tail = rx_tail;
    dbg_byte_count++;

    return byte;
}

/**
 * @brief 计算 8 位累加校验。
 */
static uint8_t CalcChecksum8(const uint8_t *buf)
{
    uint16_t sum = 0;

    sum += buf[POS_CTRL];
    sum += buf[POS_V_HIGH];
    sum += buf[POS_V_LOW];
    sum += buf[POS_W_HIGH];
    sum += buf[POS_W_LOW];

    return (uint8_t)(sum & 0xFFu);
}

/**
 * @brief 重置解析器。
 */
static void Parser_Reset(void)
{
    memset(&parser, 0, sizeof(parser));
    parser.state = PARSE_SYNC_1;
    parser.idx = 0;

    dbg_parser_state = parser.state;
    dbg_parser_idx = parser.idx;
}

/* ========================================================================== */
/*                          协议解析状态机                                      */
/* ========================================================================== */

/**
 * @brief 输入 1 字节，尝试组出完整 8 字节帧。
 * @return 1=完整帧已收齐；0=还未收齐。
 *
 * 注意：PARSE_COLLECT 阶段不能因为遇到 0x55 就重新同步。
 *      因为 V_H/V_L/W_H/W_L/CHECKSUM 都可能等于 0x55。
 */
static uint8_t Parser_FeedByte(uint8_t byte)
{
    switch (parser.state) {

    case PARSE_SYNC_1:
        if (byte == FRAME_HEADER_1) {
            parser.buf[POS_HEAD1] = FRAME_HEADER_1;
            parser.state = PARSE_SYNC_2;
        }
        break;

    case PARSE_SYNC_2:
        if (byte == FRAME_HEADER_2) {
            parser.buf[POS_HEAD2] = FRAME_HEADER_2;
            parser.idx = FRAME_DATA_START;
            parser.state = PARSE_COLLECT;
        } else if (byte == FRAME_HEADER_1) {
            /* 连续 0x55：继续等待 0xAA */
            parser.buf[POS_HEAD1] = FRAME_HEADER_1;
            parser.state = PARSE_SYNC_2;
        } else {
            Parser_Reset();
        }
        break;

    case PARSE_COLLECT:
        parser.buf[parser.idx++] = byte;

        if (parser.idx >= FRAME_LEN) {
            dbg_parser_state = parser.state;
            dbg_parser_idx = parser.idx;
            return 1u;
        }
        break;

    default:
        Parser_Reset();
        break;
    }

    dbg_parser_state = parser.state;
    dbg_parser_idx = parser.idx;
    return 0u;
}

/**
 * @brief 验证完整帧校验和。
 */
static uint8_t Parser_ValidateFrame(const uint8_t *buf)
{
    return (buf[POS_CKSUM] == CalcChecksum8(buf)) ? 1u : 0u;
}

/* ========================================================================== */
/*                          命令分发                                            */
/* ========================================================================== */

static float LimitFloat(float x, float min_value, float max_value)
{
    if (x > max_value) return max_value;
    if (x < min_value) return min_value;
    return x;
}

static void DispatchCommand(const CmdFrame_TypeDef *cmd)
{
    switch (cmd->ctrl_mode) {

    case CTRL_SPEED:
        /* 上位机协议：v/w = 实际物理量 * 1000。这里恢复成浮点。 */
        front_speed = (float)cmd->velocity_v / 1000.0f;
        turn_omega  = (float)cmd->velocity_w / 1000.0f;

        /* 保持原工程逻辑：给后级运动层的输入限制在 [-1, 1]。 */
        front_speed = LimitFloat(front_speed, -CMD_SPEED_LIMIT, CMD_SPEED_LIMIT);
        turn_omega  = LimitFloat(turn_omega,  -CMD_SPEED_LIMIT, CMD_SPEED_LIMIT);

        /* 兼容原工程：如果后级运动控制仍读取遥控结构体，就同步写入 rcData。
         * 如果你们后级已经直接读取 front_speed / turn_omega，这两行也不会影响调试。
         */
        rcData.R_y = front_speed;
        rcData.R_x = turn_omega;

        uart8_walk_request = 1;
        last_rx_tick = HAL_GetTick();
        break;

    default:
        dbg_bad_ctrl_count++;
        break;
    }
}

/* ========================================================================== */
/*                            公共 API                                          */
/* ========================================================================== */

void UART8_Demo_Init(void)
{
    /* 初始化时清零关键调试计数 */
    dbg_process_count = 0;
    dbg_byte_count = 0;
    dbg_frame_ok_total = 0;
    dbg_frame_err_total = 0;
    dbg_bad_ctrl_count = 0;
    dbg_uart_idle_count = 0;
    dbg_dma_head = 0;
    dbg_dma_tail = 0;
    dbg_dma_avail = 0;
    dbg_dma_ndtr = 0;
    dbg_dma_start_ret = 0;
    dbg_uart_recover_count = 0;
    dbg_uart_error_code = 0;
    dbg_uart_rx_state = 0;
    dbg_uart_cr3 = 0;
    dbg_uart_recover_reason = 0;

    UART8_Demo_RestartRx(1u);
}

/**
 * @brief 主循环处理函数。必须在 while(1) 或高频任务中持续调用。
 *
 * 关键点：
 *   - 不再使用 “if (uart8_idle_flag) 才解析” 的写法。
 *   - 只要 DMA 环形缓冲区存在新字节，就逐字节送入协议状态机。
 *   - 如果 UART 错误或 DMA 接收被 HAL 关闭，自动重启接收。
 */
void UART8_Demo_Process(void)
{
    dbg_process_count++;

    dbg_uart_error_code = huart8.ErrorCode;
    dbg_uart_rx_state = huart8.RxState;
    dbg_uart_cr3 = huart8.Instance->CR3;

    /* 情况 1：UART 出错，例如 ORE/FE/NE/PE */
    if (huart8.ErrorCode != HAL_UART_ERROR_NONE) {
        UART8_Demo_RestartRx(2u);
        return;
    }

    /* 情况 2：DMA 接收位被关了，说明 HAL 或错误流程把 DMA RX 关掉了 */
    if ((huart8.Instance->CR3 & USART_CR3_DMAR) == 0u) {
        UART8_Demo_RestartRx(3u);
        return;
    }

    /* 情况 3：DMA 接收正常时，RxState 应该是 BUSY_RX */
    if (huart8.RxState != HAL_UART_STATE_BUSY_RX) {
        UART8_Demo_RestartRx(4u);
        return;
    }

    if (uart8_idle_flag) {
        uart8_idle_flag = 0;
        dbg_uart_idle_count++;
    }

    while (RingBuf_Available() > 0u) {
        uint8_t byte = RingBuf_ReadByte();

        if (Parser_FeedByte(byte)) {
            memcpy((void *)dbg_last_frame, parser.buf, FRAME_LEN);

            if (Parser_ValidateFrame(parser.buf)) {
                CmdFrame_TypeDef cmd;

                cmd.ctrl_mode = parser.buf[POS_CTRL];
                cmd.velocity_v = (int16_t)(((uint16_t)parser.buf[POS_V_HIGH] << 8)
                                         |  ((uint16_t)parser.buf[POS_V_LOW]));
                cmd.velocity_w = (int16_t)(((uint16_t)parser.buf[POS_W_HIGH] << 8)
                                         |  ((uint16_t)parser.buf[POS_W_LOW]));

                dbg_last_v = cmd.velocity_v;
                dbg_last_w = cmd.velocity_w;

                dbg_frame_ok_count++;
                dbg_frame_ok_total++;

                if (!dbg_freeze) {
                    memcpy((void *)dbg_frozen_frame, parser.buf, FRAME_LEN);
                    dbg_frozen_v = cmd.velocity_v;
                    dbg_frozen_w = cmd.velocity_w;
                    dbg_frozen_tick = HAL_GetTick();
                }

                DispatchCommand(&cmd);
            } else {
                memcpy((void *)dbg_err_frame, parser.buf, FRAME_LEN);
                dbg_frame_err_count++;
                dbg_frame_err_total++;
            }

            Parser_Reset();
        }
    }

#if UART8_ENABLE_LOST_TIMEOUT
    if ((last_rx_tick != 0u) && ((HAL_GetTick() - last_rx_tick) > UART8_LOST_TIMEOUT_MS)) {
        front_speed = 0.0f;
        turn_omega = 0.0f;
        uart8_walk_request = 0;
        last_rx_tick = 0u;
    }
#endif
}

/**
 * @brief 发送应答帧给树莓派。当前可选使用。
 */
void UART8_Demo_SendResponse(uint8_t ctrl_mode, int16_t v, int16_t w)
{
    uint8_t frame[FRAME_LEN];

    frame[POS_HEAD1]  = FRAME_HEADER_1;
    frame[POS_HEAD2]  = FRAME_HEADER_2;
    frame[POS_CTRL]   = ctrl_mode;
    frame[POS_V_HIGH] = (uint8_t)(((uint16_t)v >> 8) & 0xFFu);
    frame[POS_V_LOW]  = (uint8_t)((uint16_t)v & 0xFFu);
    frame[POS_W_HIGH] = (uint8_t)(((uint16_t)w >> 8) & 0xFFu);
    frame[POS_W_LOW]  = (uint8_t)((uint16_t)w & 0xFFu);
    frame[POS_CKSUM]  = CalcChecksum8(frame);

    HAL_UART_Transmit(&huart8, frame, FRAME_LEN, TX_TIMEOUT_MS);
}
