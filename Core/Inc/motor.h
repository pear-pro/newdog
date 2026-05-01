#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "stm32f4xx_hal_can.h"

/* ---------------- ??? ---------------- */
#define PI                      3.14159f
#define MOTOR_MAX_ID            14
#define MOTOR_DEC_RATIO         6.33f
#define MOTOR_CMD_LEN           17
#define MOTOR_WORK_MODE_FOC     1

extern float flip_offset;

/* ---------------- ???? ---------------- */
typedef struct
{
    uint8_t              MotorID;
    UART_HandleTypeDef*  huart;

    // 物理量的�?点形�?
    float Tau_ff;       
    float Omega_des;     // rad/s
    float Theta_des;     // rad
    float Kp;
    float Kw;

    // 报文�?16进制形式
    int16_t Tau_set;
    int16_t Omega_set;
    int32_t Theta_set;
    int16_t Kp_set;
    int16_t Kw_set;
    
    uint8_t TxData[MOTOR_CMD_LEN];

    // 接收
    int16_t tau_fbk_raw;
    int16_t omega_fbk_raw;
    int32_t theta_fbk_raw;

    float tau_fbk;
    float omega_fbk;
    float theta_fbk;

} Motor_HandleTypeDef;

/*        DM        */
typedef struct{
	int16_t Voltage;//电压值
	uint16_t Angle;//机械角度
	int16_t Speed;//转速
	int16_t Torque;//实际扭矩
	uint8_t Temp;//温度
	
}RxMsg_t;

typedef struct{
	pid_t 				Speed_pid;
	pid_t 				Angel_pid;
	
	//控制角度的参数
	uint16_t			FirstEntre;
	double			Target;//目标角度
	uint16_t 			lastRead;//上一次读取值
	uint16_t 			currentRead;//当前读取值
	uint16_t 			Zero;//上电后的第一个位置做为零点
	int32_t 			totalAngle;//总角度
	float				encoderAngle;//经过处理的电机角度
	int16_t				Current;//输出电流
	float				out;//输出电压

	float				angle;//目标角度
	float				speed;//目标速度
	float            	KP;
	float            	KD;
	float            	tor;
	uint32_t         	ID     ;//电机id
	
	RxMsg_t 			Rxmsg;
}motor_info_t;


/* ---------------- API ---------------- */
HAL_StatusTypeDef Motor_Init(Motor_HandleTypeDef *hmotor, UART_HandleTypeDef *huart, uint8_t motor_id);
void Motor_SendCmd(Motor_HandleTypeDef *hmotor);
void Motor_SendCmd_AllAngle(void);
void Motor_InitBias(void);
void MotorTest_Sweep(int id, float step);
void motor_release(void);
void remap_motor_ids(void);
void init_motor_parameters(void);
void Set_dm(CAN_HandleTypeDef* hcan,int16_t ID);
void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID);
void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID);
void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID);
void inilize_dm(void);

#endif /* __MOTOR_H */
