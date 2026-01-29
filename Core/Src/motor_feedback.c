#include "stm32f4xx_hal.h"
#include "motor_feedback.h"
#include "crc_ccitt.h"
#include <string.h>
#include <math.h>
#include "usart.h"

extern UART_HandleTypeDef huart6;
extern uint8_t usart6_rx_byte;
volatile uint8_t motor_timeout_flag = 0;

#define pi 3.1415926f

Motor_Feedback_t motor_fb[MOTOR_NUM];



/* -------- 鐜?褰㈢紦鍐插尯 -------- */
#define RX_BUF_SIZE 128

static uint8_t rx_buf[RX_BUF_SIZE];
static uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

/* -------- 鐘舵�佹満 -------- */
static uint8_t  frame[MOTOR_FB_LEN];
static uint8_t  idx = 0;
static uint8_t  state = 0;

/* -------- 绉佹湁鍑芥暟 -------- */
static void Motor_ParseFrame(uint8_t *f);

void Motor_Feedback_Init(void)
{
    memset(motor_fb, 0, sizeof(motor_fb));
	 rx_head = rx_tail = 0;
    idx = 0;
    state = 0;
	HAL_UART_Receive_IT(&huart6, &usart6_rx_byte, 1);

}

void Motor_Feedback_RxByte(uint8_t byte)
{
    rx_buf[rx_head++] = byte;
    if (rx_head >= RX_BUF_SIZE)
        rx_head = 0;
}



void Motor_Feedback_TimeoutTask(void)
{
    static uint32_t offline_cnt[MOTOR_NUM] = {0};

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
    while (rx_tail != rx_head)// 缓冲区有未处理字节时循环
    {
        uint8_t b = rx_buf[rx_tail++]; // 取出一个字节
        if (rx_tail >= RX_BUF_SIZE) //指针循环
            rx_tail = 0;

        switch (state)
        {
        case 0: // HEAD1
            if (b == 0xFD)
            {
                frame[0] = b; // 帧头错误，重置状态
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

/* -------- 鐪熸?ｇ殑鍗忚??瑙ｆ瀽 -------- */
//motor_fb[0] -> ID = 0 鐨勭數鏈?           motor_fb[1] -> ID = 1 鐨勭數鏈?
static void Motor_ParseFrame(uint8_t *f)
{
    /* CRC 鏍￠獙 0~13 */
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
}

