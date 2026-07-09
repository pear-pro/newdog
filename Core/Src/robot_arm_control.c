#include "robot_arm_control.h"
#include "stm32f4xx.h"
#include "can.h"
#include "motor.h"
#include "motor_4310.h"
#include "motor_feedback.h"
#include "global_var.h"

/*
说明：
机械臂的相关代码写在这个包里，涉及电机发送指令的代码写motor.c.h里面
*/

extern CAN_HandleTypeDef hcan1;
extern motor_info_t damiao[4];

#define CLAMP_COS(val) ((val) > 1.0f ? 1.0f : ((val) < -1.0f ? -1.0f : (val)))

// 闭环控制参数
#define CL_POS_TOLERANCE   0.5f   // 到位容差（度）
#define CL_STEP_TIMEOUT_MS 80     // 单步超时（ms）

// 云台反馈转换：电机轴弧度 → 云台角度（度）
// 发送: Theta_des = motor3_bias/6.33 + 2π*angle/360
// 反推: angle = (theta - motor3_bias/6.33) * 360 / 2π
#define GIMBAL_FB_TO_DEG(theta)  (((theta) - motor3_bias / 6.33f) * 360.0f / 6.28318f)

// 记录当前各关节角度
float currentTheta1 = 0.0f;   // 底座旋转角
static float currentTheta2 = 0.0f;   // 大臂关节角
static float currentTheta3 = 0.0f;   // 小臂关节角
//static float currentServo = 170.0f;   // 末端舵机角度

// 保留末端坐标记录，方便外部调用
static float currentX = 0.0f;
static float currentY = 0.0f;
static float currentZ = L1 + L2 + L3 +12.8f;

void Set_DM_Motor(uint8_t id, float angle_deg) {
	uint8_t array_idx=id;
	float pos_rad = angle_deg * PI / 180.0f;
	damiao[array_idx].angle=pos_rad;

	damiao[array_idx].KP=30.0f;
	damiao[array_idx].KD = 0.9f;
  damiao[array_idx].speed = 5.0f;
	damiao[array_idx].tor = 0.0f;
	
	Set_dm_mit(&hcan1,array_idx);
}

//舵机
void Set_Camera_Servo_Angle_TIM8(uint32_t Channel, float angle) 
{
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 180.0f) angle = 180.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 180.0f) * 2000);
	
	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}

void Set_Sucker_Servo_Angle_TIM8(uint32_t Channel, float angle) 
{
	if (angle < 0.0f) angle = 0.0f;
	if (angle > 270.0f) angle = 270.0f;

	uint32_t ccr_val = (uint32_t)(500 + (angle / 270.0f) * 2000);
	
	__HAL_TIM_SET_COMPARE(&htim8, Channel, ccr_val);
}

//步长
//uint8_t Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps)
//{
//	// ========== 新增：theta1模式控制静态变量 ==========
//	static float last_targetX = 0.0f;
//	static float last_targetY = 0.0f;
//	static float last_targetZ = 0.0f;
//	static uint8_t theta1_step_en = 0;  // 1=步进模式 0=直接发目标角度,保持静止
//	const float pos_change_thresh = 0.01f; // 坐标变化阈值，防浮点抖动误触发

//	if (steps == 0) return 1;  // 步数为零无意义

//	//1. 计算底座角度（旋转角）
//	float targetTheta1=atan2f(targetY,targetX)*180.0f/PI;
//	
//	// ========== 新增：检测XYZ变化，切换theta1工作模式 ==========
////	if(fabsf(targetX - last_targetX) > pos_change_thresh && fabsf(targetY - last_targetY) > pos_change_thresh &&fabsf(targetZ - last_targetZ) > pos_change_thresh)
////	{
////		last_targetX = targetX;
////		last_targetY = targetY;
////		last_targetZ = targetZ;
////		theta1_step_en = 1;  // XY改变，启用步进模式，逐步走到目标
////	}
////	else
////	{
////		theta1_step_en = 0;  // XY不变，直接发送目标角度，不步进
////	}
//	
//	float z_new = targetZ -12.8f ;
//	
//	//2. 几何计算
//	float s=sqrtf(targetX*targetX+targetY*targetY);     //末端在地面上的投影距离
//	float h= z_new -L1;               //末端相对于大臂旋转轴的高度差
//	float dist=sqrtf(s*s+h*h);  //目标点到大臂旋转轴的直线距离

//	// 工作空间钳位：沿同方向缩放到最近的可达点，不拒绝运动
//	{
//		const float dist_max = L2 + L3 - 0.01f;
//		const float dist_min = fabsf(L2 - L3) + 0.01f;

//		if (dist < 0.001f)      // 目标点在肩关节正上方，无法确定方向
//		{
//			return 1;
//		}
//		else if (dist > dist_max)
//		{
//			float scale = dist_max / dist;
//			s *= scale;
//			h *= scale;
//			dist = dist_max;
//		}
//		else if (dist < dist_min)
//		{
//			float scale = dist_min / dist;
//			s *= scale;
//			h *= scale;
//			dist = dist_min;
//		}
//	}

//	//3.余弦定理求解
//	float cos_alpha=(L2*L2+L3*L3-dist*dist)/(2.0f *L2*L3);
//	cos_alpha=CLAMP_COS(cos_alpha);
//	float alpha=acosf(cos_alpha)*180.0f/PI;//三角形内角
//	
//	float phi=atan2f(h,s)*180.0f/PI;//连线仰角
//	
//	float cos_beta = (L3 * L3 + dist * dist - L2 * L2) / (2.0f * L3 * dist);
//	if( cos_beta > 1.0f) cos_beta = 1.0f;
//	if( cos_beta < -1.0f) cos_beta = -1.0f;
//	float beta = acosf(cos_beta) * 180.0f / PI;

//	// 4. 计算最终角度（加上 90 度偏移，使 90 度成为几何中位）
//	// 姿态:默认肘上
//	float targetTheta2 = 90.0f - (phi + beta) + offset2;
//	float targetTheta3 = -(180.0f - alpha) + offset3;
//	
//		// 关节角钳位：超出机械限位时保留在极限值，不拒绝运动
//		if (targetTheta2 > 90.0f)  targetTheta2 = 90.0f;
//		if (targetTheta2 < -90.0f) targetTheta2 = -90.0f;
//		if (targetTheta3 > -45.0f)  targetTheta3 = -45.0f;
//		if (targetTheta3 < -180.0f) targetTheta3 = -180.0f;
//		if (targetTheta1 > 90.0f)   targetTheta1 = 90.0f;
//		if (targetTheta1 < -90.0f)  targetTheta1 = -90.0f;

//	float Servo = 170.0f - (alpha - 90.0f + phi + beta);
//	
//	// ===== 开环步进（待闭环验证通过后再升级）=====

//	// 计算每个关节每步的角度增量
//	float deltaTheta1 = (targetTheta1 + offset1 - currentTheta1) / steps;
//	float deltaTheta2 = (targetTheta2 - currentTheta2) / steps;
//	float deltaTheta3 = (targetTheta3 - currentTheta3) / steps;

//	hmotor3.Kp = 0.2f;
//	hmotor3.Kw = 0.01f;

//	float step_gimbal = currentTheta1;
//	float step_theta2 = currentTheta2;
//	float step_theta3 = currentTheta3;

//	for (uint16_t i = 0; i < steps; i++)
//	{
//		step_gimbal += deltaTheta1;
//		step_theta2 += deltaTheta2;
//		step_theta3 += deltaTheta3;

//		gimbal_send_unitree(step_gimbal);
//		Set_DM_Motor(1, step_theta2);
//		Set_DM_Motor(0, 2 * step_theta3);

//		HAL_Delay(10);
//	}

//	// 最终精准到位
//	gimbal_send_unitree(targetTheta1 + offset1);
//	Set_DM_Motor(1, targetTheta2);
//	Set_DM_Motor(0, 2 * targetTheta3);

//	// 更新全局角度记录
//	currentTheta1 = targetTheta1 + offset1;
//	currentTheta2 = targetTheta2;
//	currentTheta3 = targetTheta3;
//		
////			Set_Servo_Angle_TIM8(TIM_CHANNEL_3, Servo);//最终舵机
//	
//			// 更新坐标记录
////			currentX = targetX;
////			currentY = targetY;
////			currentZ = targetZ;
//		return 0;  // 成功
//}
/**
 * @brief  控制大臂、小臂运动到目标 Z 坐标（底座已事先由 Arm_Base_Move 固定）
 * @param  targetX, targetY: 用于计算水平距离 s（必须与底座当前朝向一致）
 * @param  targetZ: 末端高度（cm）
 * @param  steps: 插值步数
 */
void Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps)
{
    // ===== 静态变量：检测 XYZ 变化以决定是否步进 =====
    static float lastX = 0.0f, lastY = 0.0f, lastZ = 0.0f;
    static uint8_t need_step = 0;
    const float eps = 0.01f;

    if (fabsf(targetX - lastX) > eps || fabsf(targetY - lastY) > eps || fabsf(targetZ - lastZ) > eps) {
        need_step = 1;
        lastX = targetX; lastY = targetY; lastZ = targetZ;
    } else {
        need_step = 0;
    }

    // ===== 几何逆解（仅大臂、小臂） =====
    float z_new = targetZ - 12.8f;
    float s = sqrtf(targetX * targetX + targetY * targetY);
    float h = z_new - L1;
    float dist = sqrtf(s * s + h * h);

    // 工作空间钳位：同方向缩放 s,h，保持 phi 方向不变
    {
        const float dist_max = L2 + L3 - 0.01f;
        const float dist_min = fabsf(L2 - L3) + 0.01f;
        if (dist < 0.001f) return;  // 肩关节正上方，无法定向
        if (dist > dist_max) { float scale = dist_max / dist; s *= scale; h *= scale; dist = dist_max; }
        else if (dist < dist_min) { float scale = dist_min / dist; s *= scale; h *= scale; dist = dist_min; }
    }

    float cos_alpha = CLAMP_COS((L2*L2 + L3*L3 - dist*dist) / (2*L2*L3));
    float alpha = acosf(cos_alpha) * 180.0f / PI;
    float phi = atan2f(h, s) * 180.0f / PI;
    float cos_beta = CLAMP_COS((L3*L3 + dist*dist - L2*L2) / (2*L3*dist));
    float beta = acosf(cos_beta) * 180.0f / PI;

    float targetTheta2 = 90.0f - (phi + beta) + offset2;
    float targetTheta3 = -(180.0f - alpha) + offset3;

    // 关节角钳位：超出机械限位时保留在极限值
    if (targetTheta2 > 90.0f)  targetTheta2 = 90.0f;
    if (targetTheta2 < -90.0f) targetTheta2 = -90.0f;
//    if (targetTheta3 > -45.0f)  targetTheta3 = -45.0f;
//    if (targetTheta3 < -180.0f) targetTheta3 = -180.0f;

//    // ===== 执行运动 =====
    if (need_step) {
        // 步进：从上次记录的当前角度插值到目标
        float delta2 = (targetTheta2 - currentTheta2) / steps;
        float delta3 = (targetTheta3 - currentTheta3) / steps;
        for (uint16_t i = 0; i < steps; i++) {
            currentTheta2 += delta2;
            currentTheta3 += delta3;
            Set_DM_Motor(1, currentTheta2);
            Set_DM_Motor(0, 2 * currentTheta3);
            HAL_Delay(10);
        }
    } else {
        // 坐标未变化：直接发目标角度（保持状态）
        Set_DM_Motor(1, targetTheta2);
        Set_DM_Motor(0, 2 * targetTheta3);
        // 关键：更新当前目标值，避免下次步进起点错误
        currentTheta2 = targetTheta2;
        currentTheta3 = targetTheta3;
    }

    // 最终精准到位（用目标值消除浮点累积误差）
    Set_DM_Motor(1, targetTheta2);
    Set_DM_Motor(0, 2 * targetTheta3);

    // 更新全局坐标记录（可选）
    currentX = targetX; currentY = targetY; currentZ = targetZ;
}

/**
 * @brief  控制底座旋转到目标 XY 对应的角度
 * @param  targetX, targetY: 末端目标水平坐标（单位：cm）
 * @param  steps: 步进插值的步数（如果 XY 变化了）
 * @note   内部用静态变量记录上一次目标，只有当 XY 变化时才执行运动，
 *         否则立即返回（不重复发送指令，节省通信且避免抖动）。
 */
void Arm_Base_Move(float targetX, float targetY, uint16_t steps)
{
    static float last_base_X = 0.0f;
    static float last_base_Y = 0.0f;
    static uint8_t initialized = 0;  // 首次调用标志
    const float eps = 0.01f;

    // 首次调用：直接到位建立起点，不依赖反馈（避免读到 0 导致跳变）
    if (!initialized) {
        initialized = 1;
        last_base_X = targetX;
        last_base_Y = targetY;
        float t1 = atan2f(targetY, targetX) * 180.0f / PI;
        if (t1 > 90.0f) t1 = 90.0f; if (t1 < -90.0f) t1 = -90.0f;
        currentTheta1 = t1 + offset1;
        return;  // 本次不发指令，下次调用正常步进
    }
	

    if (fabsf(targetX - last_base_X) < eps && fabsf(targetY - last_base_Y) < eps) {
        return;
    }
    last_base_X = targetX;
    last_base_Y = targetY;

    // 1. 几何底座角
    float targetTheta1 = atan2f(targetY, targetX) * 180.0f / PI;

    // 2. 钳位
    if (targetTheta1 > 90.0f)  targetTheta1 = 90.0f;
    if (targetTheta1 < -90.0f) targetTheta1 = -90.0f;

    // 3. 统一坐标系：云台电机角度 = θ1 + offset1
    float targetGimbal = targetTheta1 + offset1;

    // 4. 起点用全局记录值
    float startGimbal = currentTheta1;
    float delta = (targetGimbal - startGimbal) / steps;
    float ctrl = startGimbal;

    hmotor3.Kp = 0.2f;
    hmotor3.Kw = 0.01f;

    for (uint16_t i = 0; i < steps; i++) {
        ctrl += delta;
        gimbal_send_unitree(ctrl);   // 不再 +offset1，ctrl 本身已含
        HAL_Delay(10);
    }
    gimbal_send_unitree(targetGimbal);
    currentTheta1 = targetGimbal;
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
