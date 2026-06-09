/**
 ******************************************************************************
 * @file    usart_demo.c
 * @brief   UART8 串口通信模块 - 树莓派速度指令接收（稳定恢复版）
 ******************************************************************************
 *
 * ============================== 数据流架构 ==============================
 *
 *   树莓派 (115200, 8N1)
 *       │ 8 字节帧: [0x55][0xAA][CTRL][V_H][V_L][W_H][W_L][CKSUM]
 *       ▼
 *   UART8 PE0(RX) ← DMA1_Stream6, Circular 模式
 *       │ DMA 硬件自动写入，无需 CPU 干预
 *       ▼
 *   dma_rx_buf[256] 环形缓冲区
 *       │ head = 256 - __HAL_DMA_GET_COUNTER()  (硬件写入位置)
 *       │ tail = rx_tail                            (软件读取位置)
 *       ▼
 *   RingBuf_ReadByte() → Parser_FeedByte()  三状态机逐字节解析
 *       │
 *       ├─ PARSE_SYNC_1:  等 0x55
 *       ├─ PARSE_SYNC_2:  等 0xAA → 进入收集
 *       └─ PARSE_COLLECT: 收 6 字节 → 帧完整 → 校验
 *           │
 *           ▼
 *   DispatchCommand()
 *       │ velocity_v / 1000.0f → front_speed  (clamp [-1, 1])
 *       │ velocity_w / 1000.0f → turn_omega   (clamp [-1, 1])
 *       ▼
 *   motion_Mix() 步态执行
 *
 * ============================== 协议格式 ================================
 *
 *   [0x55] [0xAA] [CTRL] [V_H] [V_L] [W_H] [W_L] [CKSUM]
 *    Byte0   Byte1  Byte2  Byte3  Byte4  Byte5  Byte6  Byte7
 *
 *   - CTRL:    控制模式，0x01=速度下发(CTRL_SPEED)
 *   - V_H/V_L: 线速度 velocity_v, int16 大端序
 *   - W_H/W_L: 角速度 velocity_w, int16 大端序
 *   - CKSUM:   (Byte2+Byte3+Byte4+Byte5+Byte6) & 0xFF
 *
 *   例：55 AA 01 03 E8 00 00 EC
 *       velocity_v=0x03E8=1000, velocity_w=0
 *       校验：(0x01+0x03+0xE8+0x00+0x00)&0xFF = 0xEC ✓
 *
 * ============================== V2 vs V1 主要改动 =======================
 *
 *   1. 解析触发：V1 依赖 uart8_idle_flag 才解析；V2 轮询缓冲区，有数据就处理。
 *   2. 0x55 误判：V1 在 PARSE_COLLECT 遇到 0x55 就重同步（bug！因为数据区
 *      velocity_v 低字节可能是 0x55）；V2 直接收，不重同步。
 *   3. ORE 恢复：V1 无恢复机制，UART 溢出后永久卡死；V2 每次 Process() 检查
 *      3 种异常（ErrorCode / CR3_DMAR / RxState）并自动 RestartRx()。
 *   4. 环形缓冲区：V1 用位掩码（要求 BUF_SIZE 是 2 的幂）；V2 用通用取模。
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

/* 1000ms 无数据 → 自动站立（RPi 模式） */
#define UART8_ENABLE_LOST_TIMEOUT    1
#define UART8_LOST_TIMEOUT_MS        1000u

/* 给运动层的速度限幅，保持原工程逻辑：[-1, 1] */
#define CMD_SPEED_LIMIT              1.0f

/* 速度映射：velocity_v / VELOCITY_SCALE → front_speed ∈ [-1, 1]
   V_PHYSICAL_MAX=0.6m/s, VELOCITY_SCALE=1000×0.6=600
   例：600→1.0, 300→0.5, 60→0.1 */
#define VELOCITY_SCALE               600.0f

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
 * @brief 重启 UART8 DMA 接收 — 整个模块故障恢复的核心函数。
 *
 * 为什么需要这个函数？
 *   STM32F4 的 UART 一旦发生 ORE（Overrun Error，溢出错误），硬件会永久
 *   停止接收数据。HAL 库不会自动清除 ORE 也不会自动重启 DMA。必须手动：
 *   1) 清硬件错误标志；2) 恢复 HAL 状态机；3) 重启 DMA。
 *
 * 调用时机：
 *   UART8_Demo_Process() 每次执行时，如果检测到以下任一异常就会调用：
 *     - huart8.ErrorCode != HAL_UART_ERROR_NONE  (硬件出错)
 *     - CR3.DMAR == 0                            (DMA 接收被关闭)
 *     - huart8.RxState != HAL_UART_STATE_BUSY_RX  (HAL 状态异常)
 *
 * 执行步骤（顺序不能乱）：
 *
 *   [1] 记录故障现场 → 保存 ErrorCode/RxState/CR3，供 Watch 窗口查看
 *   [2] HAL_UART_DMAStop() → 停掉旧的 DMA 流，释放 HAL 内部锁
 *   [3] 清 UART 硬件错误标志 → ORE(Overrun) / FE(Framing) / NE(Noise) / PE(Parity)
 *       ★ ORE 不清 = UART 硬件永远罢工
 *   [4] 恢复 HAL 状态机 → ErrorCode=NONE, RxState=READY, Lock=UNLOCKED
 *       ★ HAL_UART_Receive_DMA() 内部检查这些状态，不恢复返回 HAL_BUSY
 *   [5] 清空软件上下文 → rx_tail 归零 + 解析器重置 + 缓冲区清空
 *       ★ 旧缓冲区里的半帧/错位帧必须丢弃，否则帧头永远找不到
 *   [6] 强制 DMA Circular 模式 → UART 出错时 HAL 可能偷偷把 DMA 改回 Normal
 *   [7] HAL_UART_Receive_DMA() → 重新启动 Circular DMA 接收
 *   [8] 重新使能 IDLE 中断 → 辅助判断通信空闲
 *   [9] 二次记录状态 → 确认恢复后一切正常
 *
 * @param reason 恢复原因：1=Init初始化, 2=UART硬件错误(ORE等), 3=CR3.DMAR被关, 4=RxState异常
 */
static void UART8_Demo_RestartRx(uint32_t reason)
{
    /* ---- [1] 记录故障现场，供 Watch 窗口诊断 ---- */
    dbg_uart_recover_count++;
    dbg_uart_recover_reason = reason;
    dbg_uart_error_code = huart8.ErrorCode;
    dbg_uart_rx_state = huart8.RxState;
    dbg_uart_cr3 = huart8.Instance->CR3;

    /* ---- [2] 停掉旧 DMA，释放 HAL 锁 ---- */
    HAL_UART_DMAStop(&huart8);

    /* ---- [3] 清 UART 硬件错误标志（ORD-Flag 通过读 SR+DR 清除方式，HAL 宏内部处理） ---- */
    __HAL_UART_CLEAR_OREFLAG(&huart8);   /* Overrun Error — 这是最常见的卡死原因       */
    __HAL_UART_CLEAR_FEFLAG(&huart8);   /* Framing Error — 波特率不匹配或接线不良        */
    __HAL_UART_CLEAR_NEFLAG(&huart8);   /* Noise Error  — 线路噪声                       */
    __HAL_UART_CLEAR_PEFLAG(&huart8);   /* Parity Error — 校验位不匹配                    */
    __HAL_UART_CLEAR_IDLEFLAG(&huart8); /* IDLE Flag    — 为重新使能 IDLE 中断做准备      */

    /* ---- [4] 恢复 HAL 状态机，否则 Receive_DMA() 返回 HAL_BUSY ---- */
    huart8.ErrorCode = HAL_UART_ERROR_NONE;
    huart8.RxState = HAL_UART_STATE_READY;
    huart8.Lock = HAL_UNLOCKED;

    /* ---- [5] 清空软件状态（旧数据 = 垃圾数据，必须丢弃） ---- */
    rx_tail = 0;
    Parser_Reset();
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    /* ---- [6] 强制 DMA Circular（HAL 在异常时可能改回 Normal） ---- */
#if UART8_FORCE_DMA_CIRCULAR
    if (huart8.hdmarx != NULL) {
        huart8.hdmarx->Init.Mode = DMA_CIRCULAR;
        HAL_DMA_Init(huart8.hdmarx);
    }
#endif

    /* ---- [7] 重新启动 Circular DMA 接收 ---- */
    dbg_dma_start_ret = (uint32_t)HAL_UART_Receive_DMA(&huart8,
                                                       dma_rx_buf,
                                                       USART_DEMO_RX_BUF_SIZE);

    /* ---- [8] 覆盖 HAL 默认的回调（保持 DMA 不被意外关闭） ---- */
    if (huart8.hdmarx != NULL) {
        huart8.hdmarx->XferCpltCallback     = DMA_NoOpCallback;
        huart8.hdmarx->XferHalfCpltCallback = DMA_NoOpCallback;
    }

    /* ---- [9] 重新使能 IDLE 中断 ---- */
    __HAL_UART_CLEAR_IDLEFLAG(&huart8);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);

    /* ---- 记录恢复后状态，确认正常 ---- */
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
 * @brief 三状态帧解析器：逐字节输入，组出完整 8 字节帧。
 *
 * ============================== 状态转移图 ================================
 *
 *                     +---- 0x55 ----+
 *                     v              |
 *   PARSE_SYNC_1 ──0x55──▶ PARSE_SYNC_2 ──0xAA──▶ PARSE_COLLECT ──收满6字节──▶ return 1
 *       ▲                      │                      │
 *       └──── 非 0x55 ─────────┘                      │
 *       └──── 非 0x55/0xAA ───────────────────────────┘
 *
 * ============================== 关键设计 ==================================
 *
 *   ★ V2 改动：PARSE_COLLECT 阶段不再检测 0x55。
 *     V1 在 COLLECT 阶段遇到 0x55 会跳回 SYNC_2，误以为遇到了新帧头。
 *     但是 velocity_v 和 velocity_w 的任意字节都可能 == 0x55！
 *     例如 velocity_v=85 → V_L=0x55，这个合法帧在 V1 中会被误判丢弃。
 *     V2 在 COLLECT 阶段直接收 6 字节，不检测帧头，靠校验和过滤错位帧。
 *
 *   ★ 丢字节恢复：如果 DMA 丢了一个字节导致帧错位，校验和必然失败 →
 *     Parser_Reset() 回到 SYNC_1 → 等下一个真正的 0x55+0xAA。
 *
 * @param byte 从环形缓冲区读出的下一个字节
 * @return 1=完整一帧已收齐（8字节都在 parser.buf 中），0=还在收集中
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
        /* 速度映射：velocity_v / VELOCITY_SCALE → front_speed
           VELOCITY_SCALE = 300.0，即 300(0.3m/s) 映射到 1.0(全步幅) */
        front_speed = (float)cmd->velocity_v / VELOCITY_SCALE;
        turn_omega  = (float)cmd->velocity_w / VELOCITY_SCALE;

        /* 保持原工程逻辑：给后级运动层的输入限制在 [-1, 1]。 */
        front_speed = LimitFloat(front_speed, -CMD_SPEED_LIMIT, CMD_SPEED_LIMIT);
        turn_omega  = LimitFloat(turn_omega,  -CMD_SPEED_LIMIT, CMD_SPEED_LIMIT);

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
 * @brief 主循环处理函数（必须在 while(1) 中持续调用，约 220Hz）。
 *
 * ============================== 执行流程 ==================================
 *
 *   [健康检查] 每次循环先检查 UART 硬件 / DMA / HAL 状态是否正常：
 *     检查 1: huart8.ErrorCode != HAL_UART_ERROR_NONE
 *             ↓ 最常见的是 ORE（Overrun Error），UART 收到新字节时 DR 未读走
 *     检查 2: CR3.DMAR == 0
 *             ↓ DMA 接收使能位被 HAL/错误流程关掉了
 *     检查 3: huart8.RxState != HAL_UART_STATE_BUSY_RX
 *             ↓ HAL 内部状态机异常（可能变 ERROR 或 READY）
 *     ★ 任一检查失败 → RestartRx() 自动恢复，然后本周期 return
 *
 *   [正常路径] 通过健康检查后：
 *     1. 记录 IDLE 标志（仅用于调试计数）
 *     2. 轮询环形缓冲区：RingBuf_Available() > 0 → RingBuf_ReadByte()
 *     3. Parser_FeedByte() → 帧完整 → Parser_ValidateFrame()
 *     4. 校验通过 → DispatchCommand() → front_speed/turn_omega 更新
 *
 * ============================== 关键设计决策 ==============================
 *
 *   Q: 为什么不依赖 IDLE 中断才解析？
 *   A: IDLE 中断在”持续发送”场景下本来就不一定每帧都触发。如果依赖 IDLE，
 *      数据会在 DMA 缓冲区里堆积，最终溢出触发 ORE。轮询方式响应更快。
 *
 *   Q: Recover 后为什么要 return（不继续解析当前已有的数据）？
 *   A: RestartRx() 执行了 memset+Parser_Reset，清空了所有上下文。
 *      恢复后 DMA 从头开始接收，下一周期再处理。
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

    /* ── 记录 IDLE 事件（仅用于调试统计，不影响数据解析） ── */
    if (uart8_idle_flag) {
        uart8_idle_flag = 0;
        dbg_uart_idle_count++;
    }

    /* ══════════════════════════════════════════════════════════════════
     *  主数据解析循环
     *
     *  流程：DMA 缓冲区 → 逐字节 → 状态机组帧 → 校验 → 分发
     *
     *  为什么不用 "if (uart8_idle_flag) 才处理" ？
     *    持续高速发送时 IDLE 中断不会每帧都触发。如果依赖 IDLE，
     *    数据会在 DMA 缓冲区中堆积，最终触发 ORE 导致硬件卡死。
     *    轮询方式（RingBuf_Available > 0）响应更快且不丢数据。
     *
     *  为什么 PARSE_COLLECT 阶段不检测 0x55 ？
     *    int16 速度值的低字节和高字节都可能等于 0x55。如果中途
     *    遇到 0x55 就跳回 SYNC_2，包含 0x55 字节的合法帧会被
     *    永久丢弃。V2 改为直接收集全部 6 字节，靠校验和过滤错位帧。
     * ══════════════════════════════════════════════════════════════════ */
    while (RingBuf_Available() > 0u) {
        uint8_t byte = RingBuf_ReadByte();

        if (Parser_FeedByte(byte)) {                             /* 状态机收满 8 字节 → 完整帧，return 1; */
            memcpy((void *)dbg_last_frame, parser.buf, FRAME_LEN); // 记录最后一帧，便于调试观察

            if (Parser_ValidateFrame(parser.buf)) {              /* 校验和通过 → 有效帧 */
                CmdFrame_TypeDef cmd;                           

                /* 大端序拼装 int16: V_H<<8 | V_L */
                cmd.ctrl_mode = parser.buf[POS_CTRL];
                cmd.velocity_v = (int16_t)(((uint16_t)parser.buf[POS_V_HIGH] << 8)
                                         |  ((uint16_t)parser.buf[POS_V_LOW]));
                cmd.velocity_w = (int16_t)(((uint16_t)parser.buf[POS_W_HIGH] << 8)
                                         |  ((uint16_t)parser.buf[POS_W_LOW]));

                /* 更新调试快照 */
                dbg_last_v = cmd.velocity_v;
                dbg_last_w = cmd.velocity_w;
                dbg_frame_ok_count++;
                dbg_frame_ok_total++;

                if (!dbg_freeze) {                               /* 未冻结时持续更新抓帧快照 */
                    memcpy((void *)dbg_frozen_frame, parser.buf, FRAME_LEN);
                    dbg_frozen_v = cmd.velocity_v;
                    dbg_frozen_w = cmd.velocity_w;
                    dbg_frozen_tick = HAL_GetTick();
                }

                DispatchCommand(&cmd);                           /* → front_speed / turn_omega */
            } else {                                             /* 校验和失败 → 帧错位或数据损坏 */
                memcpy((void *)dbg_err_frame, parser.buf, FRAME_LEN);
                dbg_frame_err_count++;
                dbg_frame_err_total++;
            }

            Parser_Reset();                                      /* 收完一帧（无论对错）→ 回 SYNC_1 等下一帧 */
        }
    }

    /* ══════════════════════════════════════════════════════════════════
     *  通信超时保护
     *
     *  场景：算法发送最后一帧 (0,0) 后停止发送 → 期望狗站立
     *
     *  超时后执行的动作（DispatchCommand 的反向操作）：
     *    front_speed = 0       ← 线速度归零
     *    turn_omega  = 0       ← 角速度归零
     *    uart8_walk_request=0  ← 通知 main.c 切换至 case 12 站立
     *    last_rx_tick = 0      ← 防止每周期重复触发
     *
     *  main.c 侧配合逻辑（第 279 行）：
     *    temp_state = uart8_walk_request ? 2 : 12;
     *    → 有数据=行走(case2)，超时=站立(case12)
     * ══════════════════════════════════════════════════════════════════ */
#if UART8_ENABLE_LOST_TIMEOUT
    /* last_rx_tick != 0 ：上电后至少收到过一帧数据，防止上电 500ms 误触发 */
    /* HAL_GetTick() - last_rx_tick > 500 ：距离最后一帧已超过超时阈值             */
    if ((last_rx_tick != 0u) && ((HAL_GetTick() - last_rx_tick) > UART8_LOST_TIMEOUT_MS))
    {
        front_speed = 0.0f;          /* 线速度归零 */
        turn_omega  = 0.0f;          /* 角速度归零 */
        uart8_walk_request = 0;      /* → main.c 第 279 行: temp_state = 0?2:12 = 12 (站立) */
        last_rx_tick = 0u;           /* 清空时间戳，防止每周期重复进入此分支 */
    }
#endif
}

/**
 * @brief 发送应答帧给树莓派。当前可选使用。暂时没有用到，后续可以根据需要调用
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
