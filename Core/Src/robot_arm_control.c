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

    // 计算关节角 theta1（肘部向上姿态）
    double alpha = atan2(R, delta_z);
    double cos_gamma = (L2*L2 + D*D - L3*L3) / (2 * L2 * D);
    if (cos_gamma >  1.0) cos_gamma =  1.0;
    if (cos_gamma < -1.0) cos_gamma = -1.0;
    double gamma = acos(cos_gamma);

    theta1 = alpha - gamma;
//}
//if(has_solution ==1){
double targetTheta2 = theta1/PI*180.0f;
double targetTheta3 = -(PI-theta2)*2.0f/PI*180.0f;


// has_solution = 1 时，theta1、theta2 即为有效结果（单位：弧度）

//    // ===== 几何逆解（仅大臂、小臂） =====
//    float z_new = targetZ - 12.8f;
//    float s = sqrtf(targetX * targetX + targetY * targetY);
//    float h = z_new - L1;
//    float dist = sqrtf(s * s + h * h);

//    // 工作空间钳位
//    if (dist > L2 + L3 - 0.01f) dist = L2 + L3 - 0.01f;
//    else if (dist < fabsf(L2 - L3) + 0.01f) dist = fabsf(L2 - L3) + 0.01f;

//    float cos_alpha = CLAMP_COS((L2*L2 + L3*L3 - dist*dist) / (2*L2*L3));
//    float alpha = acosf(cos_alpha) * 180.0f / PI;
//    float phi = atan2f(h, s) * 180.0f / PI;
//    float cos_beta = CLAMP_COS((L3*L3 + dist*dist - L2*L2) / (2*L3*dist));
//    float beta = acosf(cos_beta) * 180.0f / PI;

//    float targetTheta2 = 90.0f - (phi + beta) + offset2;
//    float targetTheta3 = -(180.0f - alpha) + offset3;

//    // 关节限幅
//    if (targetTheta2 > 60.5f) targetTheta2 = 60.0f;
//    else if (targetTheta2 < -73.5f) targetTheta2 = -73.5f;
//    if (targetTheta3 > 131.5f) targetTheta3 = 131.5f;
//    else if (targetTheta3 < -143.0f) targetTheta3 = -143.0f;

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
void Arm_Base_Move(float targetX, float targetY, uint16_t steps)
{
    static float last_base_X = 0.0f;
    static float last_base_Y = 0.0f;
    const float eps = 0.01f;

    // 1. 如果 XY 没有明显变化，不做任何事，直接返回
    if (fabsf(targetX - last_base_X) < eps && fabsf(targetY - last_base_Y) < eps) {
        return;
    }

    // 2. 记录本次目标，供下次比较
    last_base_X = targetX;
    last_base_Y = targetY;

    // 3. 计算目标底座角度（修正：用当前 targetX, targetY）
    float targetTheta1 = atan2f(targetY, targetX) * 180.0f / PI;

    // 4. 底座角度限幅（-90°~90°）
    if (targetTheta1 < -90.0f || targetTheta1 > 90.0f) {
        // 可以根据需要钳位或直接返回，这里选择钳位
        targetTheta1 = (targetTheta1 < -90.0f) ? -90.0f : 90.0f;
    }

    // 5. 读取当前底座实际角度（用于步进起点）
    Motor_Feedback_Process();
    Motor_Feedback_TimeoutTask();
    float currentTheta1 = motor_fb[6].theta / 39.7524f * 360.0f + offset1;

    // 6. 步进模式：插值发送底座角度（与非步进统一处理，简单起见直接用步进）
    float delta = (targetTheta1 - currentTheta1) / steps;
    float ctrl = currentTheta1;
    for (uint16_t i = 0; i < steps; i++) {
        ctrl += delta;
        float gimbal_deg = ctrl + offset1;          // 使用 offset1 统一偏移
        hmotor3.Kp = 0.2f;
        hmotor3.Kw = 0.01f;
        gimbal_send_unitree(gimbal_deg);
        HAL_Delay(10);
    }
    // 最终精准到位
    gimbal_send_unitree(targetTheta1 + offset1);
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
