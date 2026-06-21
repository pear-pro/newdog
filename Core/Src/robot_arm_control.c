#include "robot_arm_control.h"
#include "stm32f4xx.h"
#include "can.h"
#include "motor.h"

/*
说明：
机械臂的相关代码写在这个包里，涉及电机发送指令的代码写motor.c.h里面
*/

extern CAN_HandleTypeDef hcan1;

#define CLAMP_COS(val) ((val) > 1.0f ? 1.0f : ((val) < -1.0f ? -1.0f : (val)))

void Set_DM_Motor(uint8_t id, float angle_deg) {
	uint8_t array_idx=id;
	float pos_rad = angle_deg * PI / 180.0f;
	damiao[array_idx].angle=pos_rad;

	damiao[array_idx].KP=5.0f;
	damiao[array_idx].KD = 0.9f;
  damiao[array_idx].speed = 5.0f;
	damiao[array_idx].tor = 0.0f;
	
	Set_dm_mit(&hcan1,array_idx);
}

uint8_t Arm_Move_To(float x, float y, float z)
{
	//1. 计算底座角度（旋转角）
	float theta1=atan2f(y,x)*180.0f/PI;
	
	float z_new = z - 12.8;
	
	//2. 几何计算
	float s=sqrtf(x*x+y*y);     //末端在地面上的投影距离
	float h= z_new -L1;               //末端相对于大臂旋转轴的高度差
	float dist=sqrtf(s*s+h*h);  //目标点到大臂旋转轴的直线距离

	// 检查目标点是否超出范围（工作空间检查）
	if(dist>(L2+L3)||dist<fabsf(L2-L3)) return 0;

	//3.余弦定理求解
	float cos_alpha=(L2*L2+L3*L3-dist*dist)/(2.0f *L2*L3);
	cos_alpha=CLAMP_COS(cos_alpha);
	float alpha=acosf(cos_alpha)*180.0f/PI;//三角形内角
	
	float phi=atan2f(h,s)*180.0f/PI;//连线仰角
	
	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2 * L3 * dist);
	if( cos_beta > 1.0f) cos_beta = 1.0f;
	if( cos_beta < -1.0f) cos_beta = -1.0f;
	float beta = acosf(cos_beta) * 180.0f / PI;

	// 4. 计算最终角度（加上 90 度偏移，使 90 度成为几何中位）
	float theta2,theta3;
	// 姿态:默认肘上
	theta2 = 90.0f - (phi + beta) + offset2; 
	theta3 = -(180.0f - alpha) + offset3;
	
  if(theta2>60.5f||theta2<-73.5f) return 0;
  if(theta3>131.5f||theta3<-143.0f) return 0;

	// 5.执行移动
	float gimbal_deg = theta1 + 10.0f; 
  gimbal_send_unitree(gimbal_deg);
	
	Set_DM_Motor(1, theta2);//大臂
	Set_DM_Motor(0, 2*theta3);//小臂
	
	Set_Servo_Angle_TIM8(TIM_CHANNEL_3,175.0f+theta3);
}

void Set_Servo_Angle_TIM8(uint32_t Channel, float angle) {
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 270.0f) angle = 270.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 270.0f) * 2000);
	
	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}


