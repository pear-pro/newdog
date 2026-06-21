#ifndef __MOTOR_4310_H
#define __MOTOR_4310_H
#include "stm32f4xx.h"
#include <stdint.h>
#include "main.h"
#include "can.h"
#include "math_utils.h"

typedef struct{
	int16_t Voltage;//电压值
	float Angle;//机械角度
	float Speed;//转速
	int16_t Torque;//实际扭矩
	uint8_t Temp;//温度
	uint16_t ERR;
}RxMsg_t;

typedef struct{
	//控制角度的参数
	uint16_t			FirstEntre;
	double			Target;//目标角度
	float 			lastRead;//上一次读取值
	float 			currentRead;//当前读取值
	float 			Zero;//上电的第一个位置作为零点
	float 			totalAngle;//总角度
	float				encoderAngle;//编码器计算的单圈角度
	int16_t				Current;//当前电流
	float				out;//输出电压

	float               target_angle;
	float               target_speed;
	float				angle;//目标角度
	float				speed;//目标速度
	float            	KP;
	float            	KD;
	float            	tor;
	uint32_t         	ID     ;//电机id

	RxMsg_t 			Rxmsg;
}motor_info_t;

void Set_dm_zeropoint(CAN_HandleTypeDef* hcan,uint16_t CAN_ID);
void Set_dm_enable(CAN_HandleTypeDef* hcan,uint8_t ID);
void Set_dm_disable(CAN_HandleTypeDef* hcan,uint8_t ID);
void Set_dm_speed(CAN_HandleTypeDef* hcan,int16_t ID,float speed);
void Set_dm_pos(CAN_HandleTypeDef* hcan,uint16_t ID,float pos,float vel);
void Set_dm_mit(CAN_HandleTypeDef* hcan,int16_t ID);
void dm_motor_fbdata(motor_info_t *motor, uint8_t *rx_data);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float float_to_uint(float x, float x_min, float x_max, int bits);
#endif // __MOTOR_4310_H
