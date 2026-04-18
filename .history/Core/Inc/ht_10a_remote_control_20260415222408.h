 #ifndef HT_10A_REMOTE_CONTROL_H
 #define HT_10A_REMOTE_CONTROL_H

 #include "main.h"
 #include "bsp_rc.h"

 #define SBUS_BUFLEN 25//SBUSЭ�����ݻ���25bit������SBUSЭ��һ֡���ݵĳ��ȣ����ڴ洢��
 #define SBUS_HUART       huart1//SBUSЭ��ʹ�õ�UART1

 #define SBUS_RX_BUF_NUM 50u//DMA˫������
 #define RC_FRAME_LENGTH 25u//SBUSЭ�����ݻ���25bit������ң����һ֡���ݵ���Ч���ȣ����ڽ�����

 #define SBUS_CH_VALUE_MIN 192
 #define SBUS_CH_VALUE_MID 992
 #define SBUS_CH_VALUE_MAX 1792
 #define SBUS_CH_VALUE_OFFSET 992

 #define SWITCH_SBUS_CH_VALUE_MIN -800
 #define SWITCH_SBUS_CH_VALUE_MID 0
 #define SWITCH_SBUS_CH_VALUE_MAX 800

 #define POS_UP 1//���忪��λ��UP
 #define POS_MID 3//���忪��λ��MID
 #define POS_DOWN 2//���忪��λ��DOWN

 #define DEADZONE 150.0f//����������ֵ������ֵ��Χ�������ڹ���С��Χֵ

 #define KEY_NONE 0X00//�������ⰴ����ʼ״̬
 #define KEY_SWA_DOWN 0X01//�������ⰴ��SWITCH_DOWN
 #define KEY_SWA_MID 0X02//�������ⰴ��SWITCH_MID
 #define KEY_SWA_UP 0X04//�������ⰴ��SWITCH_UP
 #define KEY_SWB_DOWN 0X08//�������ⰴ��SWITCH_B_DOWN
 #define KEY_SWB_UP 0X10//�������ⰴ��SWITCH_B_UP
 #define KEY_SWC_DOWN 0X20//�������ⰴ��SWITCH_C_DOWN
 #define KEY_SWC_UP 0X40//�������ⰴ��SWITCH_C_UP
 #define KEY_SWD_DOWN 0X80//�������ⰴ��SWITCH_D_DOWN
 #define KEY_SWD_MID 0X100//�������ⰴ��SWITCH_D_MID
 #define KEY_SWD_UP 0X200//�������ⰴ��SWITCH_D_UP

 typedef struct 
 {
     int16_t ch[8];//ͨ������

     // ����״̬��¼
     uint16_t last_swa_state;        // SWA�ϴ�״̬
     uint16_t last_swb_state;        // SWB�ϴ�״̬
     uint16_t last_swc_state;        // SWC�ϴ�״̬
     uint16_t last_swd_state;        // SWD�ϴ�״̬
     uint16_t key_flag;              // ��ǰ������־

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

 