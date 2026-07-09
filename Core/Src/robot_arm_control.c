#include "robot_arm_control.h"
#include "stm32f4xx.h"
#include "can.h"
#include "motor.h"
#include "motor_feedback.h"

/*
说明：
机械臂的相关代码写在这个包里，涉及电机发送指令的代码写motor.c.h里面
*/

extern CAN_HandleTypeDef hcan1;
extern Motor_HandleTypeDef hmotor10;

#define CLAMP_COS(val) ((val) > 1.0f ? 1.0f : ((val) < -1.0f ? -1.0f : (val)))

// Arm_Move_Smooth 内部状态变量
float currentTheta1 = 0.0f;   // 底座旋转角（extern in header）
static float currentTheta2 = 0.0f;   // 大臂关节角
static float currentTheta3 = 0.0f;   // 小臂关节角
static float currentX = 30.0f;
static float currentY = 0.0f;
static float currentZ = 25.0f;

void Set_DM_Motor(uint8_t id, float angle_deg) {
	uint8_t array_idx=id;
	float pos_rad = angle_deg * PI / 180.0f;
	damiao[array_idx].angle=pos_rad;

	damiao[array_idx].KP=15.0f; // 原为30，因大臂电机堵转发热降回15
	damiao[array_idx].KD = 0.9f;
  damiao[array_idx].speed = 5.0f;
	damiao[array_idx].tor = 0.0f;

	Set_dm_mit(&hcan1,array_idx);
}

uint8_t Arm_Move_To_Start(float x, float y, float z)
{
	//1. 计算底座角度（旋转角）
	float theta1=atan2f(y,x)*180.0f/PI;

	float z_new = z-12.8 ;

	//2. 几何计算
	float s=sqrtf(x*x+y*y);     //末端在地面上的投影距离
	float h= z_new -L1;               //末端相对于大臂旋转轴的高度差
	float dist=sqrtf(s*s+h*h);  //目标点到大臂旋转轴的直线距离

	// 检查目标点是否超出范围（工作空间检查）
//	if(dist>(L2+L3)||dist<fabsf(L2-L3)) return 0;
	// 检查目标点是否超出物理工作空间（考虑到浮点误差，可适当留一点裕度）
	if (dist > (L2 + L3 - 0.01f) || dist < (fabsf(L2 - L3) + 0.01f))
	{
    return 0;
	}

	//3.余弦定理求解
	float cos_alpha=(L2*L2+L3*L3-dist*dist)/(2.0f *L2*L3);
	cos_alpha=CLAMP_COS(cos_alpha);
	float alpha=acosf(cos_alpha)*180.0f/PI;//三角形内角

	float phi=atan2f(h,s)*180.0f/PI;//连线仰角

	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2.0 * L3 * dist);
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
	float gimbal_deg = theta1 + 150.0f;
  gimbal_send_unitree(gimbal_deg);

	Set_DM_Motor(1, theta2);//大臂
	Set_DM_Motor(0, 2*theta3);//小臂

	Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 35);
}

uint8_t Arm_Move_To(float x, float y, float z)
{
	//1. 计算底座角度（旋转角）
	float theta1=atan2f(y,x)*180.0f/PI;

	float z_new = z -12.8 ;

	//2. 几何计算
	float s=sqrtf(x*x+y*y);     //末端在地面上的投影距离
	float h= z_new -L1;               //末端相对于大臂旋转轴的高度差
	float dist=sqrtf(s*s+h*h);  //目标点到大臂旋转轴的直线距离

	// 检查目标点是否超出范围（工作空间检查）
//	if(dist>(L2+L3)||dist<fabsf(L2-L3)) return 0;
	// 检查目标点是否超出物理工作空间（考虑到浮点误差，可适当留一点裕度）
	if (dist > (L2 + L3 - 0.01f) || dist < (fabsf(L2 - L3) + 0.01f))
	{
    return 0;
	}

	//3.余弦定理求解
	float cos_alpha=(L2*L2+L3*L3-dist*dist)/(2.0f *L2*L3);
	cos_alpha=CLAMP_COS(cos_alpha);
	float alpha=acosf(cos_alpha)*180.0f/PI;//三角形内角

	float phi=atan2f(h,s)*180.0f/PI;//连线仰角

	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2.0 * L3 * dist);
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
	float gimbal_deg = theta1 + 150.0f;
  gimbal_send_unitree(gimbal_deg);

	Set_DM_Motor(1, theta2);//大臂
	Set_DM_Motor(0, 2*theta3);//小臂

	Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 170.0f - (alpha - 90 + phi + beta));
}

//舵机 - 相机（180度范围）
void Set_Camera_Servo_Angle_TIM8(uint32_t Channel, float angle)
{
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 180.0f) angle = 180.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 180.0f) * 2000);

	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}

//舵机 - 吸盘（270度范围）
void Set_Sucker_Servo_Angle_TIM8(uint32_t Channel, float angle)
{
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 270.0f) angle = 270.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 270.0f) * 2000);

	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}

//步长
void Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps)
{
	// theta1模式控制静态变量
	static float last_targetX = 0.0f;
	static float last_targetY = 0.0f;
	static float last_targetZ = 0.0f;
	static uint8_t theta1_step_en = 0;  // 1=步进模式 0=直接发目标角度
	const float pos_change_thresh = 0.01f;

	//1. 计算底座角度（旋转角）
	float targetTheta1=atan2f(targetY,targetX)*180.0f/PI;

	// 检测坐标变化，切换工作模式
	// 使用 OR（||）：任意坐标变化就进入步进模式
	// 原来用 AND（&&）会导致X固定时（如画圆）不触发步进
	if(fabsf(targetX - last_targetX) > pos_change_thresh || fabsf(targetY - last_targetY) > pos_change_thresh || fabsf(targetZ - last_targetZ) > pos_change_thresh)
	{
		last_targetX = targetX;
		last_targetY = targetY;
		last_targetZ = targetZ;
		theta1_step_en = 1;
	}
	else
	{
		theta1_step_en = 0;
	}

	float z_new = targetZ -12.8f ;

	//2. 几何计算
	float s=sqrtf(targetX*targetX+targetY*targetY);
	float h= z_new -L1;
	float dist=sqrtf(s*s+h*h);

	if (dist > (L2 + L3 - 0.01f) || dist < (fabsf(L2 - L3) + 0.01f))
	{
    return ;
	}

	//3.余弦定理求解
	float cos_alpha=(L2*L2+L3*L3-dist*dist)/(2.0f *L2*L3);
	cos_alpha=CLAMP_COS(cos_alpha);
	float alpha=acosf(cos_alpha)*180.0f/PI;

	float phi=atan2f(h,s)*180.0f/PI;

	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2.0f * L3 * dist);
	if( cos_beta > 1.0f) cos_beta = 1.0f;
	if( cos_beta < -1.0f) cos_beta = -1.0f;
	float beta = acosf(cos_beta) * 180.0f / PI;

	// 4. 计算最终角度
	float targetTheta2 = 90.0f - (phi + beta) + offset2;
	float targetTheta3 = -(180.0f - alpha) + offset3;

  if(targetTheta2>60.5f||targetTheta2<-73.5f) return;
  if(targetTheta3>131.5f||targetTheta3<-143.0f) return;

	float Servo = 170.0f - (alpha - 90.0f + phi + beta);

	Motor_Feedback_Process();
	Motor_Feedback_TimeoutTask();

	currentTheta1 = motor_fb[10].theta/39.7524f*360.0f + offset1;

	float controlThetal = currentTheta1;

	if(theta1_step_en == 0)
	{
		// XY未改变：直接发送目标角度
		float gimbal_deg = targetTheta1 + offset1;
		hmotor10.Kp = 0.2f;
		hmotor10.Kw = 0.01f;
		gimbal_send_unitree(gimbal_deg);

		Set_DM_Motor(1, targetTheta2);
		Set_DM_Motor(0, 2 * targetTheta3);
	}
	else
	{
		// XY改变：步进模式
		float deltaTheta1 = (targetTheta1 - currentTheta1) / steps;
		float deltaTheta2 = (targetTheta2 - currentTheta2) / steps;
		float deltaTheta3 = (targetTheta3 - currentTheta3) / steps;

		for (uint16_t i = 0; i < steps; i++)
		{
			controlThetal += deltaTheta1;
			float gimbal_deg = controlThetal + offset1;
			hmotor10.Kp=0.2f;
			hmotor10.Kw=0.01f;
			gimbal_send_unitree(gimbal_deg);

			currentTheta2 += deltaTheta2;
			currentTheta3 += deltaTheta3;

			Set_DM_Motor(1, currentTheta2);
			Set_DM_Motor(0, 2 * currentTheta3);

			HAL_Delay(10);
		}
		theta1_step_en=0;
	}

	// 最终精准到位
	Set_DM_Motor(1, currentTheta2);
	Set_DM_Motor(0, 2 * currentTheta3);

	// 更新坐标记录
	currentX = targetX;
	currentY = targetY;
	currentZ = targetZ;
}

//舵机
void Set_Servo_Angle_TIM8(uint32_t Channel, float angle) {
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 270.0f) angle = 270.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 270.0f) * 2000);

	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}

void Arm_Forward_Kinematics(float *x, float *y, float *z)
{
	float	theta1 =  motor_fb[10].theta/39.7524f*360.0f - 100.0f;
	float theta2 = damiao[1].Rxmsg.Angle * 180.0f/PI;
	float theta3 = (damiao[0].Rxmsg.Angle * 180.0f / PI) / 2.0f;

	float alpha = 180.0f + theta3 - offset3;
	float pb = 90.0f - theta2 + offset2; // phi和beta的和

	float alpha_rad = alpha * PI / 180.0f;
	float dist = sqrtf (L2*L2 + L3*L3 - 2.0f*L2*L3*cosf(alpha_rad));

	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2.0f * L3 * dist);
	cos_beta = CLAMP_COS(cos_beta);
	float beta_rad = acosf (cos_beta );
	float beta = beta_rad * 180.0f/PI;

	float phi = pb - beta ;
	float phi_rad = phi *PI /180.0f;

	float s = dist * cosf(phi_rad);  // 腕关节水平投影距离
	float h = dist * sinf(phi_rad);  // 腕关节相对肩关节的高度差

	float z_new = h + L1;             // 腕关节z坐标
	float end_z = z_new + 12.8f;      // 夹爪末端z坐标

	float theta1_rad = theta1 * PI / 180.0f;
	float end_x = s * cosf(theta1_rad);
	float end_y = s * sinf(theta1_rad);

	// 输出结果
	*x = end_x/100.0f;
	*y = end_y/100.0f;
	*z = end_z/100.0f;
}

// Y-Z 平面圆形轨迹运动
// 功能：让机械臂末端在Y-Z平面上画圆形
// 原理：固定X值，在圆上等间隔采样N个点，依次调用Arm_Move_Smooth移动到每个点
//
// 参数说明：
//   y_center, z_center : 圆心坐标（cm）
//   radius             : 圆半径（cm），建议不超过10，避免超出工作空间
//   num_points         : 采样点数，越多轨迹越平滑，但耗时越长
//
// 时间计算：
//   每个点调用Arm_Move_Smooth(x, y, z, steps)
//   当坐标变化时进入步进模式，每步HAL_Delay(10ms)
//   每点耗时 = steps * 10ms
//   总耗时 = num_points * steps * 10ms
//   例：50个点，每点5步 = 50 * 5 * 10ms = 2500ms ≈ 2.5秒一圈
//
// 使用示例：
//   Arm_Circle_YZ(25.0f, 35.0f, 5.0f, 50);  // 圆心(25,35)，半径5，50个点
//
// 注意事项：
//   1. 圆心和半径要确保所有点都在工作空间内
//   2. 大臂电机堵转时不要调用，先检查电机状态
//   3. 如果电机运动不平滑，增大steps或num_points
void Arm_Circle_YZ(float y_center, float z_center, float radius, uint16_t num_points)
{
    float x_fixed = 0.0f;  // 固定X=0，theta1=90度，末端在Y-Z平面运动

    for (uint16_t i = 0; i < num_points; i++)
    {
        // 参数方程：圆上第i个点的角度
        float t = 2.0f * PI * (float)i / (float)num_points;

        // 计算圆上第i个点的Y和Z坐标
        float y = y_center + radius * cosf(t);
        float z = z_center + radius * sinf(t);

        // 调用步进函数移动到该点
        // steps=5：每个目标点分5步到达，每步10ms，共50ms
        // 这样相邻两个目标点之间有50ms间隔，不会发送过快
        Arm_Move_Smooth(x_fixed, y, z, 5);
    }
}
