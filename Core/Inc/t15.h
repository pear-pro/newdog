#ifndef __T15_H__
#define __T15_H__

#include "main.h"
#include "bsp_rc.h"

#define SBUS_BUFLEN 25//SBUS协议数据缓存25bit（定义SBUS协议一帧数据的长度，用于存储）
#define SBUS_HUART       huart1//SBUS协议使用的UART1

#define SBUS_RX_BUF_NUM 50u//DMA双缓冲区
#define RC_FRAME_LENGTH 25u//SBUS协议数据缓存25bit（定义遥控器一帧数据的有效长度，用于解析）

#define SBUS_CH_VALUE_MIN 192
#define SBUS_CH_VALUE_MID 992
#define SBUS_CH_VALUE_MAX 1792
#define SBUS_CH_VALUE_OFFSET 992

#define SWITCH_SBUS_CH_VALUE_MIN -800
#define SWITCH_SBUS_CH_VALUE_MID 0
#define SWITCH_SBUS_CH_VALUE_MAX 800

#define POS_UP 1//定义开关位置UP
#define POS_MID 3//定义开关位置MID
#define POS_DOWN 2//定义开关位置DOWN

#define DEADZONE 100.0f//定义死区阈值（绝对值范围），用于过滤小范围值

#define KEY_NONE 0X00//定义虚拟按键初始状态
#define KEY_SWA_DOWN 0X01//定义虚拟按键SWITCH_DOWN
#define KEY_SWA_MID 0X02//定义虚拟按键SWITCH_MID
#define KEY_SWA_UP 0X04//定义虚拟按键SWITCH_UP
#define KEY_SWB_DOWN 0X08//定义虚拟按键SWITCH_B_DOWN
#define KEY_SWB_MID 0X10//定义虚拟按键SWITCH_B_MID
#define KEY_SWB_UP 0X20//定义虚拟按键SWITCH_B_UP
#define KEY_SWC_DOWN 0X40//定义虚拟按键SWITCH_C_DOWN
#define KEY_SWC_MID 0X80//定义虚拟按键SWITCH_C_MID
#define KEY_SWC_UP 0X100//定义虚拟按键SWITCH_C_UP
#define KEY_SWD_DOWN 0X200//定义虚拟按键SWITCH_D_DOWN
#define KEY_SWD_MID 0X400//定义虚拟按键SWITCH_D_MID
#define KEY_SWD_UP 0X800//定义虚拟按键SWITCH_D_UP
#define KEY_SE_DOWN 0X1000//定义虚拟按键SE_DOWN
#define KEY_SE_UP 0X2000//定义虚拟按键SE_UP
#define KEY_SF_PRESSED 0X4000//定义虚拟按键SF_PRESSED

typedef struct 
{
    int16_t ch[10];//通道数据

    // 按键状态记录
    uint16_t last_swa_state;        // SWA上次状态
    uint16_t last_swb_state;        // SWB上次状态
    uint16_t last_swc_state;        // SWC上次状态
    uint16_t last_swd_state;        // SWD上次状态
    uint16_t last_se_state;        // SE上次状态
    uint16_t last_sf_state;        // SF上次状态

    uint16_t key_flag;              // 当前按键标志

}SBUS_ctrl_t;

extern void sbus_remote_control_init(void);

extern const SBUS_ctrl_t *get_sbus_remote_control_point(void);

extern uint8_t   sbus_buffer[SBUS_BUFLEN];
void USART1_IRQHandlerCallBack(void);

static uint8_t detect_switch_position(int16_t value);
static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl);

#endif