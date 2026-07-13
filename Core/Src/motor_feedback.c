#include "stm32f4xx_hal.h"
#include "motor_feedback.h"
#include "crc_ccitt.h"
#include <string.h>
#include <math.h>
#include "usart.h"

extern UART_HandleTypeDef huart6;
//extern uint8_t usart6_rx_byte;
volatile uint8_t motor_timeout_flag = 0;

#define pi 3.1415926f

Motor_Feedback_t motor_fb[MOTOR_NUM];



/* -------- 环形缓冲区 -------- */
#define RX_BUF_SIZE 256

static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;  // volatile: ISR写，主循环读
static volatile uint16_t rx_tail = 0;  // volatile: 主循环写，ISR读

#define DMA_BUF_SIZE 256   // DMA硬件缓冲区（原来64，扩大到256匹配usart_demo）
static uint8_t dma_buf[DMA_BUF_SIZE];
static uint16_t last_dma_cnt = 0;  // 记录上次DMA位置

/* -------- 状态机 -------- */
static uint8_t  frame[MOTOR_FB_LEN];
static uint8_t  idx = 0;
static uint8_t  state = 0;

/* -------- 离线计数器（文件作用域，Motor_ParseFrame 需要访问） -------- */
static uint32_t offline_cnt[MOTOR_NUM] = {0};

/* -------- 私有函数 -------- */
static void Motor_ParseFrame(uint8_t *f);



void Motor_Feedback_Init(void)
{
    memset(motor_fb, 0, sizeof(motor_fb));
	 rx_head = rx_tail = 0;
    idx = 0;
    state = 0;
	last_dma_cnt = 0;
	HAL_UART_Receive_DMA(&huart6, dma_buf, DMA_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	

}


// USART6 DMA 接收重启（参考 usart_demo.c UART8_Demo_RestartRx）
// 完整执行 16 步恢复序列，确保从任何 UART 错误中恢复
void USART6_RestartRxDMA(void)
{
    // 1. 停掉旧的 DMA 接收
    HAL_UART_DMAStop(&huart6);

    // 2. 清除所有 UART 错误标志
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);

    // 3. 恢复 HAL 状态
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    huart6.RxState = HAL_UART_STATE_READY;
    huart6.Lock = HAL_UNLOCKED;

    // 4. 清空环形缓冲区和解析器状态
    rx_head = 0;
    rx_tail = 0;
    last_dma_cnt = 0;
    state = 0;
    idx = 0;

    // 5. 重置所有电机离线状态
    memset(offline_cnt, 0, sizeof(offline_cnt));
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_fb[i].online = 0;
    }

    // 6. 清空 DMA 缓冲区
    memset(dma_buf, 0, DMA_BUF_SIZE);

    // 7. 强制 DMA 为 Circular 模式（防止被 HAL 错误流程改回 Normal）
    HAL_DMA_Init(huart6.hdmarx);

    // 8. 重新启动 DMA 接收
    HAL_UART_Receive_DMA(&huart6, dma_buf, DMA_BUF_SIZE);

    // 9. 重新使能 IDLE 中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}



void DMA_CopyToRingBuf(void)
{
    // 获取DMA剩余计数
    uint16_t curr_cnt = __HAL_DMA_GET_COUNTER(huart6.hdmarx);
    // 计算新接收的字节数
    uint16_t new_len = 0;

    if (curr_cnt <= last_dma_cnt)
        new_len = last_dma_cnt - curr_cnt;
    else
        new_len = DMA_BUF_SIZE - curr_cnt + last_dma_cnt;

    // 拷贝数据到环形缓冲（带溢出保护）
    for (uint16_t i = 0; i < new_len; i++)
    {
        uint16_t next_head = (rx_head + 1) % RX_BUF_SIZE;
        if (next_head == rx_tail) break;  // 缓冲满，丢弃剩余数据
        uint16_t dma_idx = (last_dma_cnt + i) % DMA_BUF_SIZE;
        rx_buf[rx_head] = dma_buf[dma_idx];
        rx_head = next_head;
    }

    last_dma_cnt = curr_cnt;
}




void Motor_Feedback_TimeoutTask(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (motor_fb[i].online)
        {
            offline_cnt[i]++;
            if (offline_cnt[i] > 100)   // 100ms
            {
                motor_fb[i].online = 0;
                offline_cnt[i] = 0;
            }
        }
    }
}


void Motor_Feedback_Process(void)
{
    // UART 错误自动检测和恢复（参考 usart_demo.c）
    // 情况 1：UART 硬件出错（ORE/FE/NE/PE）
    if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
        USART6_RestartRxDMA();
        return;
    }
    // 情况 2：DMA 接收位被 HAL/错误流程关闭
    if ((huart6.Instance->CR3 & USART_CR3_DMAR) == 0u) {
        USART6_RestartRxDMA();
        return;
    }
    // 情况 3：RxState 不是 BUSY_RX（HAL 状态机异常）
    if (huart6.RxState != HAL_UART_STATE_BUSY_RX) {
        USART6_RestartRxDMA();
        return;
    }

    while (rx_tail != rx_head)
    {
        uint8_t b = rx_buf[rx_tail++]; 
        if (rx_tail >= RX_BUF_SIZE) 
            rx_tail = 0;

        switch (state)
        {
        case 0: // HEAD1
            if (b == 0xFD)
            {
                frame[0] = b; 
                idx = 1;
                state = 1;
            }
            break;

        case 1: // HEAD2
            if (b == 0xEE)
            {
                frame[1] = b;
                idx = 2;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // BODY
            frame[idx++] = b;
            if (idx >= MOTOR_FB_LEN)
            {
                Motor_ParseFrame(frame);
                state = 0;
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}

/* -------- 真�?�的协�??解析 -------- */
//motor_fb[0] -> ID = 0 的电�?           motor_fb[1] -> ID = 1 的电�?
static void Motor_ParseFrame(uint8_t *f)
{
    /* CRC 校验 0~13 */
    if (!CRC16_CCITT_Check(f, 14))
        return;

    uint8_t mode = f[2];
    uint8_t id = mode & 0x0F;
    uint8_t status = (mode >> 4) & 0x07;

    if ( id >= MOTOR_NUM)
        return;

    Motor_Feedback_t *fb = &motor_fb[id];

    fb->id     = id;
    fb->status = status;

    fb->tau_raw   = (int16_t)(f[3] | (f[4] << 8));
    fb->omega_raw = (int16_t)(f[5] | (f[6] << 8));
    fb->theta_raw = (int32_t)(f[7] | (f[8] << 8) |
                              (f[9] << 16) | (f[10] << 24));

    fb->tau   = fb->tau_raw / 256.0f;
    fb->omega = fb->omega_raw / 256.0f * 2.0f * pi;
    fb->theta = fb->theta_raw / 32768.0f * 2.0f * pi;

    fb->temp  = (int8_t)f[11];
    fb->error = f[12] & 0x07;

    fb->force = ((f[12] >> 3) & 0x1F) | ((uint16_t)f[13] << 5);

    fb->online = 1;
    offline_cnt[id] = 0;  // 喂狗：收到有效帧，重置离线计数器
}
