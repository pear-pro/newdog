#include "robot_arm_control.h"
#include "stm32f4xx.h"
#include "can.h"
#include "motor.h"
#include "motor_4310.h"
#include "motor_feedback.h"
#include "global_var.h"
#include <math.h>

/*
说明：
机械臂的相关代码写在这个包里，涉及电机发送指令的代码写motor.c.h里面
*/

extern CAN_HandleTypeDef hcan1;
extern motor_info_t damiao[4];

#define CLAMP_COS(val) ((val) > 1.0f ? 1.0f : ((val) < -1.0f ? -1.0f : (val)))

// 记录当前各关节角度
float currentTheta1 = 0.0f;   // 底座旋转角
static float currentTheta2 = 0.0f;   // 大臂关节角
static float currentTheta3 = 0.0f;   // 小臂关节角
//static float currentServo = 170.0f;   // 末端舵机角度
float targettheta1 =0.0f;
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


void Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps)
{
    // ===== 静态变量：检测 XYZ 变化以决定是否步进 =====
    static float lastX = 0.0f, lastY = 0.0f, lastZ = 0.0f;

    const float eps = 0.01f;
	// 直接放在你的函数内部使用，输入变量：L1、L2、L3、R、Z（均为 double 类型）
double delta_z = targetZ - L1;
double R= sqrt(targetX*targetX+targetY*targetY);
double D = sqrt(R * R + delta_z * delta_z);
//int has_solution = 1;
double theta1, theta2;
const double eps_work = 1e-6;
	// 判断是否超出工作空间
if (D > L2 + L3 + eps_work || D < fabs(L2 - L3) - eps_work) {
    return;
}
 uint8_t need_step = 0;
 if (fabsf(targetX - lastX) > eps || fabsf(targetY - lastY) > eps || fabsf(targetZ - lastZ) > eps) {
        need_step = 1;
//      lastX = targetX;
//			lastY = targetY;
//			lastZ = targetZ;
    } else {
        need_step = 0;
    }
		    // 计算关节角 theta2

	
    double cos_theta2 = (L2*L2 + L3*L3 - D*D) / (2 * L2 * L3);
    if (cos_theta2 >  1.0) cos_theta2 =  1.0;
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
    theta2 = acos(cos_theta2);
	  float ture_theta2 = PI-theta2;
		if((ture_theta2)<=0){
		ture_theta2=0.0f;
		}
if(delta_z >=0){
    // 计算关节角 theta1（肘部向上姿态）
 	 double alpha = atan2(delta_z, R);
    double cos_gamma = (L2*L2 + D*D - L3*L3) / (2 * L2 * D);
    if (cos_gamma >  1.0) cos_gamma =  1.0;
    if (cos_gamma < -1.0) cos_gamma = -1.0;
    double gamma = acos(cos_gamma);

    theta1 = PI/2.0f-alpha - gamma;
	}
else{
    delta_z = L1 - targetZ;
	double alpha = atan2(R, delta_z);
	double cos_gamma = (L2*L2 + D*D - L3*L3) / (2 * L2 * D);
    if (cos_gamma >  1.0) cos_gamma =  1.0;
    if (cos_gamma < -1.0) cos_gamma = -1.0;
    double gamma = acos(cos_gamma);
    theta1 = PI-alpha - gamma;
	  
}
double targetTheta2 = theta1/PI*180.0f;
double targetTheta3 = -(ture_theta2)*2.0f/PI*180.0f;//负号是电机方向


    // ===== 执行运动 =====
    if (need_step) {
        // 步进：从上次记录的当前角度插值到目标
        float delta2 = (targetTheta2 - currentTheta2) / steps;
        float delta3 = (targetTheta3 - currentTheta3) / steps;
        for (uint16_t i = 0; i < steps; i++) {
            currentTheta2 += delta2;
            currentTheta3 += delta3;
            Set_DM_Motor(1, currentTheta2);
            Set_DM_Motor(0,  currentTheta3);
            HAL_Delay(10);
        }
    } else {
        // 坐标未变化：直接发目标角度（保持状态）
        Set_DM_Motor(1, targetTheta2);
        Set_DM_Motor(0, targetTheta3);
        // 关键：更新当前目标值，避免下次步进起点错误
        currentTheta2 = targetTheta2;
        currentTheta3 = targetTheta3;
    }

    // 最终修正（消除浮点累积）
    Set_DM_Motor(1, currentTheta2);
    Set_DM_Motor(0,  currentTheta3);

    // 5. 运动完成后，再更新 last 坐标为本次目标
    lastX = targetX;
    lastY = targetY;
    lastZ = targetZ;

		// 更新全局坐标记录（可选）
    currentX = targetX;
		currentY = targetY;
		currentZ = targetZ;

}



/**
 * @brief  控制底座旋转到目标 XY 对应的角度
 * @param  targetX, targetY: 末端目标水平坐标（单位：cm）
 * @param  steps: 步进插值的步数（如果 XY 变化了）
 * @note   内部用静态变量记录上一次目标，只有当 XY 变化时才执行运动，
 *         否则立即返回（不重复发送指令，节省通信且避免抖动）。
 */
//void Arm_Base_Move(float targetX, float targetY, uint16_t steps)
//{
//    static float last_base_X = 0.0f;
//    static float last_base_Y = 0.0f;
//    const float eps = 0.01f;

//    // 1. 如果 XY 没有明显变化，不做任何事，直接返回
//    if (fabsf(targetX - last_base_X) < eps && fabsf(targetY - last_base_Y) < eps) {
//        return;
//    }

//    // 2. 记录本次目标，供下次比较
//    last_base_X = targetX;
//    last_base_Y = targetY;

//    // 3. 计算目标底座角度（修正：用当前 targetX, targetY）
//    float targetTheta1 = atan2f(targetY, targetX) * 180.0f / PI;

//    // 4. 底座角度限幅（-90°~90°）
//    if (targetTheta1 < -90.0f || targetTheta1 > 90.0f) {
//        // 可以根据需要钳位或直接返回，这里选择钳位
//        targetTheta1 = (targetTheta1 < -90.0f) ? -90.0f : 90.0f;
//    }

//    // 5. 读取当前底座实际角度（用于步进起点）
//    Motor_Feedback_Process();
//    Motor_Feedback_TimeoutTask();
//    float currentTheta1 = motor_fb[6].theta / 39.7524f * 360.0f + offset1;

//    // 6. 步进模式：插值发送底座角度（与非步进统一处理，简单起见直接用步进）
//    float delta = (targetTheta1 - currentTheta1) / steps;
//    float ctrl = currentTheta1;
//    for (uint16_t i = 0; i < steps; i++) {
//        ctrl += delta;
//        float gimbal_deg = ctrl + offset1;          // 使用 offset1 统一偏移
//        hmotor3.Kp = 0.2f;
//        hmotor3.Kw = 0.01f;
//        gimbal_send_unitree(gimbal_deg);
//        HAL_Delay(10);
//    }
//    // 最终精准到位
//    gimbal_send_unitree(targetTheta1 + offset1);
//}
//void Arm_Base_Move(float targetX, float targetY, uint16_t steps)//闭环的
//{
//    static float last_target_X = 0.0f;
//    static float last_target_Y = 0.0f;
//    static float current_theta = 0.0f;          // 当前实际角度（不含offset1，单位：度）
//    static int first_call = 1;
//    const float eps = 0.01f;

//    // 1. 目标无变化则返回
//    if (fabsf(targetX - last_target_X) < eps && fabsf(targetY - last_target_Y) < eps) {
//        return;
//    }
//    last_target_X = targetX;
//    last_target_Y = targetY;

//    // 2. 计算目标底座角度（相对于基座坐标系）
//    float targetTheta1 = atan2f(targetY, targetX) * 180.0f / PI;
//    // 限幅
//    if (targetTheta1 <= -90.0f) targetTheta1 = -90.0f;
//    if (targetTheta1 >=  90.0f) targetTheta1 =  90.0f;

//    // 3. 首次调用时，从实际反馈初始化 current_theta（注意去掉 offset1）
//    if (first_call) {
//        Motor_Feedback_Process();
//        Motor_Feedback_TimeoutTask();
//        // 假设 motor_fb[6].theta 是原始编码器值，转换为度后加上 offset1 为实际角度
//        // 但我们要存储不带 offset1 的机械角度，因为后续发送会统一加 offset1
//        // 若 offset1 是机械零位偏移，则实际角度 = raw * scale + offset1，
//        // 但发送函数要求的是 raw * scale 还是 raw * scale + offset1 需明确。
//        // 这里推荐统一：current_theta 存储“发送指令值”（即电机期望接收的值），
//        // 而 targetTheta1 也转换成相同量纲。为了减少混淆，统一用法：
//        // 令 offset1 仅为反馈计算用，发送时不加 offset1，直接发送 targetTheta1。
//        // 但原代码发送加了 offset1，所以我们这里保持兼容：
//        // current_theta = motor_fb[6].theta / 39.7524f * 360.0f + offset1; // 这是原反馈值
//        // 但由于发送时也加 offset1，所以实际起点应减去 offset1 以匹配目标不带 offset1 的计算。
//        // 最简单：让 current_theta 存储不带 offset1 的值，同时修改发送，去掉多余偏移。
//        // 我建议修改发送方式：发送 (ctrl) 即可，不再加 offset1。
//        // 但为了最小改动，你需确认 offset1 用途。以下为推荐修改（发送不加 offset1）：
//        current_theta = motor_fb[10].theta / 39.7524f * 360.0f; // 原始编码器角度
//        first_call = 0;
//    }

//    // 4. 从 current_theta 插值到 targetTheta1（两者都不加 offset1）
//    float delta = (targetTheta1 - current_theta) / steps;
//    float ctrl = current_theta;
//    for (uint16_t i = 0; i < steps; i++) {
//        ctrl += delta;
//        // 发送指令时不再额外加 offset1（如果发送函数期望原始编码器角度）
//        // 若你确认发送函数需要 offset1，请保留，但务必与 current_theta 的量纲一致。
//        gimbal_send_unitree(ctrl);   // 注意：这里不再加 offset1
//        HAL_Delay(10);
//    }
//    // 最终精准到位
//    gimbal_send_unitree(targetTheta1);

//    // 5. 更新当前位置为本次目标，供下次作为起点
//    current_theta = targetTheta1;
//}





/**
 * @brief 底座开环步进到目标XY对应的角度（不依赖反馈）
 * @param targetX  目标 X 坐标（任意单位）
 * @param targetY  目标 Y 坐标（任意单位）
 * @param steps    插值步数（必须 > 0）
 */
void Arm_Base_Move(float targetX, float targetY, uint16_t steps)//开环的，读不到反馈值时用
{
    // 静态变量记录当前实际角度（物理角度，单位度），初始为0（机械零点）
    static float current_angle = 0.0f;
    const float eps = 0.01f;

    // 1. 计算目标角度（物理角度，单位度）
    float target_angle = atan2f(targetY, targetX) * 180.0f / PI;

    // 2. 角度限幅（-90° ~ 90°）
    if (target_angle < -90.0f) target_angle = -90.0f;
    if (target_angle >  90.0f) target_angle =  90.0f;

    // 3. 如果目标与当前角度几乎相同，直接返回（避免无效步进）
    if (fabsf(target_angle - current_angle) < eps) {
        return;
    }

    // 4. 步进参数保护（防止除零）
    if (steps == 0) steps = 1;

    // 5. 开环插值：从 current_angle 逐步移动到 target_angle
    float delta = (target_angle - current_angle) / steps;
    float cmd = current_angle;
    for (uint16_t i = 0; i < steps; i++) {
        cmd += delta;
        gimbal_send_unitree(cmd);   // 直接发送实际角度（不加偏移）
        HAL_Delay(10);              // 可根据需要调整步进间隔
    }

    // 6. 最终精准到位
    gimbal_send_unitree(target_angle);

    // 7. 更新记录为本次终点（供下次调用作为起点）
    current_angle = target_angle;
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

/**
 * 正运动学函数
 * -----------------------------------------------
 * 连杆定义（与逆解算一致）：
 *   L2 = 19.8cm = 第一连杆（肩→肘）
 *   L3 = 42.1cm = 第二连杆（肘→腕）
 *
 * 几何推导：
 *   第一连杆从肩出发，方向 θ₁（与垂直方向夹角）
 *   第二连杆从肘出发，方向 θ₁ + (π - θ₂)
 *     θ₂=π → 两连杆同向（直立）
 *     θ₂=π/2 → 第二连杆垂直于第一连杆
 *     θ₂=0 → 两连杆反向（折叠）
 *
 *   R = L2·sin(θ₁) + L3·sin(θ₁ + π - θ₂)
 *     = L2·sin(θ₁) - L3·sin(θ₁ - θ₂)
 *   δz = L2·cos(θ₁) + L3·cos(θ₁ + π - θ₂)
 *      = L2·cos(θ₁) - L3·cos(θ₁ - θ₂)
 *   Z = L1 + 12.8 + δz
 *
 * 验证（IK 输入 R=30, Z=50 → θ₁=-0.437, θ₂=1.554）：
 *   FK: R = 19.8×sin(-0.437) - 42.1×sin(-0.437-1.554)
 *     = -8.37 - 42.1×(-0.911) = -8.37 + 38.36 = 29.99 ≈ 30 ✅
 *   FK: δz = 19.8×cos(-0.437) - 42.1×cos(-1.991)
 *      = 17.94 - 42.1×(-0.412) = 17.94 + 17.35 = 35.29 ≈ 35.17 ✅
 *
 * 验证（直立 θ₁=0, θ₂=π）：
 *   R = 0 - 42.1×sin(-π) = 0 ✅
 *   δz = 19.8 - 42.1×cos(-π) = 19.8 + 42.1 = 61.9 ✅
 *   Z = 61.9 + 27.63 = 89.53 ✅
 */
void Arm_Forward_Kinematics_New(float big_arm_angle_deg, float small_arm_angle_deg,
                                 float *R_out, float *z_out)
{
    // 1. 角度转弧度
    float theta1 = big_arm_angle_deg * PI / 180.0f;

    // 2. 从小臂发送角度反推肘角 theta2
    // 逆解算：targetTheta3 = -(PI-theta2)*2/PI*180
    // 反推：theta2 = PI + small_arm_deg * PI / 360
    float theta2 = PI + (small_arm_angle_deg * PI / 360.0f);

    // 3. 正运动学公式
    // R = L2·sin(θ₁) - L3·sin(θ₁ - θ₂)
    // δz = L2·cos(θ₁) - L3·cos(θ₁ - θ₂)
    float R = L2*sinf(theta1) - L3*sinf(theta1 - theta2);
    float delta_z = L2*cosf(theta1) - L3*cosf(theta1 - theta2);
    float end_z = delta_z + L1 + 12.8f;

    // 输出结果（cm）
    *R_out = R;
    *z_out = end_z;
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM10)
//	{
//		if(currentTheta1)
//	}
//}
