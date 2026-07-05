 #ifndef HT_10A_REMOTE_CONTROL_H
 #define HT_10A_REMOTE_CONTROL_H

 #include "main.h"
 #include "bsp_rc.h"

 #define SBUS_BUFLEN 25//SBUS协议数据缓冲区25字节，即SBUS协议一帧数据的长度，用于存储
 #define SBUS_HUART       huart1//SBUS协议使用的UART1

 #define SBUS_RX_BUF_NUM 50u//DMA双缓冲大小
 #define RC_FRAME_LENGTH 25u//SBUS协议数据缓冲区25字节，即遥控器一帧数据的有效长度，用于解析

 #define SBUS_CH_VALUE_MIN 192
 #define SBUS_CH_VALUE_MID 992
 #define SBUS_CH_VALUE_MAX 1792
 #define SBUS_CH_VALUE_OFFSET 992

 #define SWITCH_SBUS_CH_VALUE_MIN -800
 #define SWITCH_SBUS_CH_VALUE_MID 0
 #define SWITCH_SBUS_CH_VALUE_MAX 800

 #define POS_UP 1//拨杆开关位置UP
 #define POS_MID 3//拨杆开关位置MID
 #define POS_DOWN 2//拨杆开关位置DOWN

 #define DEADZONE 150.0f//摇杆死区阈值，小于此值的范围视为无操作，用于过滤小范围值

 #define KEY_NONE 0X00//虚拟按键初始状态
 #define KEY_SWA_DOWN 0X01//虚拟按键SWITCH_DOWN
 #define KEY_SWA_MID 0X02//虚拟按键SWITCH_MID
 #define KEY_SWA_UP 0X04//虚拟按键SWITCH_UP
 #define KEY_SWB_DOWN 0X08//虚拟按键SWITCH_B_DOWN
 #define KEY_SWB_UP 0X10//虚拟按键SWITCH_B_UP
 #define KEY_SWC_DOWN 0X20//虚拟按键SWITCH_C_DOWN
 #define KEY_SWC_UP 0X40//虚拟按键SWITCH_C_UP
 #define KEY_SWD_DOWN 0X80//虚拟按键SWITCH_D_DOWN
 #define KEY_SWD_MID 0X100//虚拟按键SWITCH_D_MID
 #define KEY_SWD_UP 0X200//虚拟按键SWITCH_D_UP

 typedef struct
 {
     int16_t ch[8];//通道数据

     // 开关状态记录
     uint16_t last_swa_state;        // SWA上次状态
     uint16_t last_swb_state;        // SWB上次状态
     uint16_t last_swc_state;        // SWC上次状态
     uint16_t last_swd_state;        // SWD上次状态
     uint16_t key_flag;              // 当前按键标志

 }SBUS_ctrl_t;

 typedef struct
 {
	float R_x;
	float R_y;
	float L_x;
	float L_y;
	uint16_t sw5;
	uint16_t sw6;
	uint16_t sw7;
	uint16_t sw8;

 }Remote_Control_struct;

 extern void sbus_remote_control_init(void);

 extern const SBUS_ctrl_t *get_sbus_remote_control_point(void);

 extern uint8_t   sbus_buffer[SBUS_BUFLEN];
 void USART1_IRQHandlerCallBack(void);

 static uint8_t detect_switch_position(int16_t value);
 static void virtual_key_update(SBUS_ctrl_t *sbus_ctrl);

 #endif 
 
 // ---



