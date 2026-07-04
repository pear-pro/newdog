/*gait.c
 * 
 * 说明: 1.注意区分状态机函数和阻塞式函数
 * 
 */
#include "gait.h"
#include "IMU.h"
#include "math.h"
#include "global_var.h"
#include "kinematic.h"
#include "imu.h"
#include "tim.h"
#include "motor.h"
#include "stm32f427xx.h"
#include "stm32f4xx_hal_rcc.h"
#include <stdint.h>

#define pi 3.141592f

// imu闭环用到的变量
float turn_omega_des =0.0f;// 目标旋转角度
float kp_turn_omega = 1.0f;
float turn_omega_corr = 0.0f; // 旋转纠正值
float deta_angle=0.0f;

float Forward_freq = 0.01f; // 0.004 
#define up_down_freq 0.004f
#define crawl_freq 0.005f

float support_Kp = 0.5f; // 0.6/0.7,0.25/0.3
float swing_Kp = 0.3f;
float Expect_kw = 0.01f;  // 直接用来初始化
float support_tau_ff = 0.00f; // 0.10
float swing_tau_ff = 0.0f;

float Frontflip_freq1 = 0.15f; // 
float Frontflip_freq2 = 0.004f; // 
float Frontflip_freq3 = 0.004f; // 

#define jump_freq0 0.08f   // 0.01
#define jump_freq1 0.01f   // 0.01
#define jump_freq2 0.999f   // 0.40 跳 //0.4999
#define jump_freq3 0.039f   // 0.07  0.0372
#define jump_freq4 0.005f   // 0.01  0.005
#define jump_freq5 0.003f   // 0.01

float walk_height = 22.0f; //32.0   // 步高，直接影响通过性，过大可能过不了，过小可能容易绊倒
float max_stride = 13.0f;// 13.0f  // 步幅，直接影响速度，过大可能打滑，过小可能慢

float tau = 0.0f;
float t = 0.0f;    

float kp_GyroZ = 0.08f;  // 40~50左右输入，20左右输出
float kp_VeloY = 10.0f; // 2.0输入，20.0左右输出
float GyroZ_max = 130.0f; // 转动角速度指令上限
float stride_max = 20.0f;


// 每个电机的发力表现不同
// float motor1_kp_offset = 0.35f;
// float motor2_kp_offset = 0.15f;
// float motor3_kp_offset = 0.3f;
// float motor4_kp_offset = 0.2f;
// float motor5_kp_offset = 0.37f;
// float motor6_kp_offset = 0.32f;
// float motor7_kp_offset = 0.37f;
// float motor8_kp_offset = 0.50f;

float motor1_kp_offset = 0.15f;
float motor2_kp_offset = 0.0f;
float motor3_kp_offset = 0.1f;
float motor4_kp_offset = 0.1f;
float motor5_kp_offset = 0.15f;
float motor6_kp_offset = 0.0f;
float motor7_kp_offset = 0.0f;
float motor8_kp_offset = 0.0f;

void Init_turn_omega_des(){
	turn_omega_des = body_yaw;
}

// state == 1 支撑相 ; state == 0 摆动相
void set_Motor_Kp(int hposition1_state, int hposition2_state, int hposition3_state, int hposition4_state)
{
    static int last_state = -1;

    int current_state = (hposition1_state << 0) | 
                        (hposition2_state << 1) | 
                        (hposition3_state << 2) | 
                        (hposition4_state << 3);
    
    // 检查参数组合是否与上一次相同
    if (current_state == last_state) {
        return;  // 所有参数都相同，直接返回
    }
    
    // 更新静态变量
    last_state = current_state;
    
    if (hposition1_state == 1) // 支撑相
        {
            hmotor1.Kp = support_Kp* (1.0f + motor1_kp_offset);
            hmotor2.Kp = support_Kp * (1.0f + motor2_kp_offset);
            hmotor1.Tau_ff = support_tau_ff;
            hmotor2.Tau_ff = support_tau_ff;

        }
    else                      // 摆动相
        {
            hmotor1.Kp = swing_Kp* (1.0f + motor1_kp_offset);
            hmotor2.Kp = swing_Kp * (1.0f + motor2_kp_offset);
            hmotor1.Tau_ff = swing_tau_ff;
            hmotor2.Tau_ff = swing_tau_ff;
        }

    if (hposition2_state == 1) // 支撑相
        {
            hmotor3.Kp = support_Kp* (1.0f + motor3_kp_offset);
            hmotor4.Kp = support_Kp * (1.0f + motor4_kp_offset);
            hmotor3.Tau_ff = support_tau_ff;
            hmotor4.Tau_ff = support_tau_ff;
        }
    else                      // 摆动相
        {
            hmotor3.Kp = swing_Kp* (1.0f + motor3_kp_offset);
            hmotor4.Kp = swing_Kp * (1.0f + motor4_kp_offset);
            hmotor3.Tau_ff = swing_tau_ff;
            hmotor4.Tau_ff = swing_tau_ff;
        }

    if (hposition3_state == 1) // 支撑相
        {
            hmotor5.Kp = support_Kp* (1.0f + motor5_kp_offset);
            hmotor6.Kp = support_Kp * (1.0f + motor6_kp_offset);
            hmotor5.Tau_ff = support_tau_ff;
            hmotor6.Tau_ff = support_tau_ff;
        }
    else                      // 摆动相
        {
            hmotor5.Kp = swing_Kp* (1.0f + motor5_kp_offset);
            hmotor6.Kp = swing_Kp * (1.0f + motor6_kp_offset);
            hmotor5.Tau_ff = swing_tau_ff;
            hmotor6.Tau_ff = swing_tau_ff;
        }

    if (hposition4_state == 1) // 支撑相
        {
            hmotor7.Kp = support_Kp* (1.0f + motor7_kp_offset);
            hmotor8.Kp = support_Kp * (1.0f + motor8_kp_offset);
            hmotor7.Tau_ff = support_tau_ff;
            hmotor8.Tau_ff = support_tau_ff;
        }
    else                      // 摆动相
        {
            hmotor7.Kp = swing_Kp* (1.0f + motor7_kp_offset);
            hmotor8.Kp = swing_Kp * (1.0f + motor8_kp_offset);
            hmotor7.Tau_ff = swing_tau_ff;
            hmotor8.Tau_ff = swing_tau_ff;
        }

}

void quick_set_kp(float kp_value,float kw_value)
{
    hmotor1.Kp = kp_value* (1.0f + motor1_kp_offset);
    hmotor2.Kp = kp_value * (1.0f + motor2_kp_offset);
    hmotor3.Kp = kp_value* (1.0f + motor3_kp_offset);
    hmotor4.Kp = kp_value * (1.0f + motor4_kp_offset);
    hmotor5.Kp = kp_value* (1.0f + motor5_kp_offset);
    hmotor6.Kp = kp_value * (1.0f + motor6_kp_offset);
    hmotor7.Kp = kp_value* (1.0f + motor7_kp_offset);
    hmotor8.Kp = kp_value * (1.0f + motor8_kp_offset);

    hmotor1.Kw = kw_value;
    hmotor2.Kw = kw_value;
    hmotor3.Kw = kw_value;
    hmotor4.Kw = kw_value;
    hmotor5.Kw = kw_value;
    hmotor6.Kw = kw_value;
    hmotor7.Kw = kw_value;
    hmotor8.Kw = kw_value;
}

void  Body_Roll_Stabilizer(){
	stab_roll += kp_roll*(0 - body_roll) - kd_roll * (body_roll - prev_body_roll);
	
	if (body_roll<-45.0f||body_roll>45.0f){
		emergency_stop=1;
		stab_roll = 0;
		return;
	}else{
		emergency_stop=0;	
	}
	
	if (stab_roll > 20.0f) { stab_roll = 20.0f;}
	if (stab_roll < -20.0f) { stab_roll = -20.0f;}
	
	prev_body_roll = body_roll;
}

uint8_t imu_emergency_stop(){
	if (body_roll<-37.0f||body_roll>37.0f){
		//emergency_stop=1;
		stab_roll = 0;
		return 1;
	}else{
		//emergency_stop=0;	
		return 0;
	}
}
 

// 单足摆线轨迹生成
GaitPhasesPoints  gaitGenerator(int state, float height, float step_height, float stride)
{
  GaitPhasesPoints phaseState;
  
  if(state == 0)         phaseState.sigma = 2 * pi * tau / (0.5f * Ts);               // 前半周期轨迹生成三角函数中的相位
  else if (state == 1)   phaseState.sigma = 2 * pi * (tau - 0.5f* Ts)/ (0.5f * Ts);   // 后半周期轨迹生成三角函数中的相位

  //摆动xy
  phaseState.xSwing = stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi)) - stride / 2;
  phaseState.ySwing = height - step_height * (1 - cos(phaseState.sigma)) / 2;
  //支撑x
  phaseState.xSupport = stride / 2  - stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi));
  phaseState.ySupport = height;
  return phaseState;
}

GaitPhasesPoints  crawl_gaitGenerator(int state, float height, float step_height, float stride, float x_pos)
{
  GaitPhasesPoints phaseState; 
  


  if(state == 0)         phaseState.sigma = 2 * pi * tau / (0.5f * Ts);               // 前半周期轨迹生成三角函数中的相位
  else if (state == 1)   phaseState.sigma = 2 * pi * (tau - 0.5f* Ts)/ (0.5f * Ts);   // 后半周期轨迹生成三角函数中的相位

  //摆动xy
  phaseState.xSwing = stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi)) - stride / 2 + x_pos;
  phaseState.ySwing = height - step_height * (1 - cos(phaseState.sigma)) / 2;
  //支撑x
  phaseState.xSupport = stride / 2  - stride * ((phaseState.sigma - sin(phaseState.sigma)) / (2 * pi)) + x_pos;
  phaseState.ySupport = height;
  return phaseState;
}

/* 新版匍匐步态生成器：单足摆动(25%·Ts) + 三足支撑(75%·Ts)
 * t:         当前腿的时间偏移量（秒），范围 [0, Ts)
 *   t ∈ [0,       0.25*Ts): 摆动相 → 摆线轨迹完成抬腿前移
 *   t ∈ [0.25*Ts, Ts     ): 支撑相 → 足端匀速后移推身体前进
 * height:    支撑时足端离电机轴心的垂直距离
 * step_height: 摆动最高点高度
 * stride:    步幅（单周期内足端前后移动总量）
 * x_pos:     该腿电机轴心的x坐标（前腿为正，后腿为负）
 *
 * Ts (gait.h) 为匍匐步态的完整控制周期（秒），tau 从 0 累加至 Ts 为一个完整步态。
 * 修改 Ts 即可等比例调节爬行速度：Ts 越大周期越长、爬行越慢。
 */
GaitPhasesPoints crawl_gaitGenerator_new(float t, float height, float step_height, float stride, float x_pos)
{
	GaitPhasesPoints phaseState;

	// 确保 t 在 [0, Ts) 区间内（容错，正常由调用方保证）
	if (t >= Ts) t -= Ts;
	if (t < 0.0f) t += Ts;

	float swing_end = 0.25f * Ts;   // 摆动相结束时刻 = 25% 周期处

	if (t < swing_end)  // ======== 摆动相 (25%·Ts) ========
	{
		// sigma 从 0 → 2π，完成一次完整的摆线运动
		phaseState.sigma = 2.0f * pi * t / swing_end;

		// 摆动水平：从后向前跨一步（modified sine 轨迹）
		phaseState.xSwing = stride * ((phaseState.sigma - sinf(phaseState.sigma)) / (2.0f * pi))
		                    - stride / 2.0f + x_pos;
		// 摆动垂直：抬腿→放下（cos 曲线）
		phaseState.ySwing = height - step_height * (1.0f - cosf(phaseState.sigma)) / 2.0f;

		// 支撑相字段置零（该腿在摆动，不使用支撑值）
		phaseState.xSupport = 0.0f;
		phaseState.ySupport = 0.0f;
	}
	else  // ======== 支撑相 (75%·Ts) ========
	{
		// support_t 从 0 → 1，表示支撑进度
		float support_duration = Ts - swing_end;   // 0.75f * Ts
		float support_t = (t - swing_end) / support_duration;

		// 支撑水平：摆线轨迹，起点终点速度为零，与摆动相平滑衔接
		float support_sigma = support_t * 2.0f * pi;
		phaseState.sigma = support_sigma;
		phaseState.xSupport = stride / 2.0f - stride * ((support_sigma - sinf(support_sigma)) / (2.0f * pi)) + x_pos;
		// 支撑垂直：保持触地高度
		phaseState.ySupport = height;

		// 摆动相字段置零
		phaseState.xSwing = 0.0f;
		phaseState.ySwing = 0.0f;
	}

	return phaseState;
}

// 把运动曲线赋值给每个足   height:支撑脚离电机轴心高度  step_height:摆动高度  stride:步幅
void motion_Forward(float height, float step_height, float stride)
{
    float freq = Forward_freq; // 步频
    tau = tau + freq; // 前进
    if(tau >= 1.0f){tau = 0.0f;}

    if (tau <= 0.5f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(0, height, step_height, stride);

        hposition1.B_y  = phaseState.ySwing;
        hposition1.B_x  =  phaseState.xSwing; 
        hposition2.B_y  = phaseState.ySupport;
        hposition2.B_x  =  phaseState.xSupport; 
        hposition3.B_y  = phaseState.ySwing;
        hposition3.B_x  =  phaseState.xSwing; 
        hposition4.B_y  = phaseState.ySupport;
        hposition4.B_x  =  phaseState.xSupport; 
        set_Motor_Kp(0,1,0,1);
    }
    else if (tau > 0.5f && tau <= 1.0f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(1, height, step_height, stride);

        hposition1.B_y  = phaseState.ySupport;
        hposition1.B_x  =  phaseState.xSupport; 
        hposition2.B_y  = phaseState.ySwing;
        hposition2.B_x  =  phaseState.xSwing;
        hposition3.B_y  = phaseState.ySupport;
        hposition3.B_x  =  phaseState.xSupport;
        hposition4.B_y  = phaseState.ySwing;
        hposition4.B_x  =  phaseState.xSwing;
        set_Motor_Kp(1,0,1,0);
    }

    inverseKinematic_All();
    Motor_SendCmd_AllAngle();  
}

/* ── 匍匐状态机：先静止下蹲，再启动步态 ── */
typedef enum {
	CRAWL_DESCEND = 0,  /* 阶段1：四腿不动，缓慢下蹲 */
	CRAWL_ACTIVE  = 1   /* 阶段2：正常匍匐步态     */
} CrawlState_t;

static CrawlState_t crawl_state = CRAWL_DESCEND;
static float crawl_descend_t = 0.0f;        /* 下降进度 [0, 1] */
static float crawl_descend_start_h = 22.0f; /* 下降起始高度      */

#define CRAWL_TARGET_HEIGHT 13.5f   /* 匍匐目标高度 */
#define CRAWL_DESCEND_STEP  0.015f  /* 每帧下降步长，0.015≈0.33s */

/* 进入匍匐时由 main.c 调用，重置下降状态 */
void motion_Crawl_Reset(void)
{
	crawl_descend_start_h = walk_height;  /* 捕获当前站立高度  */
	crawl_state = CRAWL_DESCEND;
	crawl_descend_t = 0.0f;
}

void motion_Crawl(float step_height, float stride)
{

	/* ====== 阶段1：静止下蹲，四腿不动 ====== */
	if (crawl_state == CRAWL_DESCEND) {
		float r = fabsf(CRAWL_TARGET_HEIGHT - crawl_descend_start_h) * 0.5f;
		float center_y = (CRAWL_TARGET_HEIGHT + crawl_descend_start_h) / 2.0f;
		float theta = 3.141592f * crawl_descend_t;
		float current_h = r * cosf(theta) + center_y;  /* t=0→站高, t=1→匍匐高 */

		hposition1.B_x = 0.0f; hposition1.B_y = current_h;
		hposition2.B_x = 0.0f; hposition2.B_y = current_h;
		hposition3.B_x = 0.0f; hposition3.B_y = current_h;
		hposition4.B_x = 0.0f; hposition4.B_y = current_h;

		motor1_kp_offset = 0.35f; motor2_kp_offset = 0.45f;
		motor3_kp_offset = 0.2f;  motor4_kp_offset = 0.15f;
		motor5_kp_offset = 0.17f; motor6_kp_offset = 0.32f;
		motor7_kp_offset = 0.47f; motor8_kp_offset = 0.60f;
		set_Motor_Kp(1,1,1,1);
		crawl_inverseKinematic_All();
		Motor_SendCmd_AllAngle();

		motor1_kp_offset = 0.25f; motor2_kp_offset = 0.15f;
		motor3_kp_offset = 0.3f;  motor4_kp_offset = 0.2f;
		motor5_kp_offset = 0.17f; motor6_kp_offset = 0.32f;
		motor7_kp_offset = 0.37f; motor8_kp_offset = 0.50f;

		crawl_descend_t += CRAWL_DESCEND_STEP;
		if (crawl_descend_t >= 1.0f) {
			crawl_descend_t = 0.0f;
			crawl_state = CRAWL_ACTIVE;
			tau = 0.0f;  /* 重置步态相位 */
		}
		return;  /* 下降期间不执行步态 */
	}
	/* ====== 阶段2：正常匍匐步态 ====== */
	tau += Forward_freq;
	if(tau >= Ts_crawl) { tau -= Ts_crawl; }

	// 匍匐步态 Kp 偏移
	motor1_kp_offset = 0.35f;
	motor2_kp_offset = 0.45f;
	motor3_kp_offset = 0.2f;
	motor4_kp_offset = 0.15f;
	motor5_kp_offset = 0.27f;
	motor6_kp_offset = 0.32f;
	motor7_kp_offset = 0.47f;
	motor8_kp_offset = 0.99f;
	
	support_tau_ff = 0.12f;

		float crawl_height = CRAWL_TARGET_HEIGHT;
	float front_pivot_x = crawl_height + 0.5f * stride;
	float back_x_pos    = -front_pivot_x;
	float back_stride   = stride + rcData.R_y * 3.0f;
	float swing_end = 0.25f * Ts;   // 摆动相结束时间点

	// 四条腿依次偏移 0.25*Ts（秒），保证任意时刻仅一腿摆动
	float t_FR = tau;
	float t_BR = tau - 0.25f * Ts;  if(t_BR < 0.0f) t_BR += Ts_crawl;
	float t_BL = tau - 0.50f * Ts;  if(t_BL < 0.0f) t_BL += Ts_crawl;
	float t_FL = tau - 0.75f * Ts;  if(t_FL < 0.0f) t_FL += Ts_crawl;

	// 生成每条腿的轨迹
	GaitPhasesPoints FR = crawl_gaitGenerator_new(t_FR, crawl_height, step_height, stride,       front_pivot_x);
	GaitPhasesPoints BR = crawl_gaitGenerator_new(t_BR, crawl_height, step_height, back_stride,  back_x_pos);
	GaitPhasesPoints BL = crawl_gaitGenerator_new(t_BL, crawl_height, step_height, back_stride,  back_x_pos);
	GaitPhasesPoints FL = crawl_gaitGenerator_new(t_FL, crawl_height, step_height, stride,       front_pivot_x);

	// 根据各腿时间决定取摆动值还是支撑值
	hposition1.B_y = (t_FR < swing_end) ? FR.ySwing : FR.ySupport;
	hposition1.B_x = (t_FR < swing_end) ? FR.xSwing : FR.xSupport;
	hposition2.B_y = (t_BR < swing_end) ? BR.ySwing : BR.ySupport;
	hposition2.B_x = (t_BR < swing_end) ? BR.xSwing : BR.xSupport;
	hposition3.B_y = (t_BL < swing_end) ? BL.ySwing : BL.ySupport;
	hposition3.B_x = (t_BL < swing_end) ? BL.xSwing : BL.xSupport;
	hposition4.B_y = (t_FL < swing_end) ? FL.ySwing : FL.ySupport;
	hposition4.B_x = (t_FL < swing_end) ? FL.xSwing : FL.xSupport;

	// Kp: 摆动腿=0(swing Kp)，支撑腿=1(support Kp)
	set_Motor_Kp(
		(t_FR < swing_end) ? 0 : 1,
		(t_BR < swing_end) ? 0 : 1,
		(t_BL < swing_end) ? 0 : 1,
		(t_FL < swing_end) ? 0 : 1
	);

	crawl_inverseKinematic_All();
	Motor_SendCmd_AllAngle();

	// 恢复默认 Kp 偏移
	motor1_kp_offset = 0.25f;
	motor2_kp_offset = 0.15f;
	motor3_kp_offset = 0.3f;
	motor4_kp_offset = 0.2f;
	motor5_kp_offset = 0.17f;
	motor6_kp_offset = 0.32f;
	motor7_kp_offset = 0.37f;
	motor8_kp_offset = 0.50f;
	
	support_tau_ff = 0.00f;
}

/* 参数说明：
* stride：步幅，参数范围（-max_stride 到 max_stride）
* turn_stride：转弯角速度，参数范围（-1.0f 到 1.0f）
*/

void motion_Mix(float height, float step_height, float stride, float turn_stride)
{    
//	static float _t = 0.0f;
//	_t += Forward_freq;  
//	if(_t >= 1.0f) { _t -= 1.0f; }

//    if (rcData.sw8 == 0x0320){
//        if (_t <= 0.5f) {
//            tau = (cosf(pi * _t * 2.0f - pi) + 1.0f) / 4.0f;        // 0 → 0.5
//        } else {
//            tau = (cosf(pi * (_t - 0.5f) * 2.0f - pi) + 1.0f) / 4.0f + 0.5f;  // 0.5 → 1.0
//        }		
//	}else{
        tau += Forward_freq;  
        if(tau >= 1.0f) { tau -= 1.0f; }
//    }

//   // 带陀螺仪的前进闭环控制
//    float R_stride = stride;
//    float L_stride = stride;  
//    float Exp_GZ = GyroZ_max*turn_omega;  
//    float yaw_correction = kp_GyroZ * (Exp_GZ - GyroZ);
//    float shift_suppression = kp_VeloY*(0-VeloY);
//    if (yaw_correction> stride_max) {yaw_correction = stride_max;}

//   if (rcData.R_y == 0){
//       if (turn_omega == 0){ VeloY = 0.0f;} // 清除零漂
//       if (turn_omega <-0.05f){ // 左转
//           R_stride = 0 + yaw_correction + shift_suppression;
//           L_stride = 0 - yaw_correction - shift_suppression;
//       }
//       if (turn_omega > 0.05f){ // 右转
//           R_stride = 0 + yaw_correction - shift_suppression;
//           L_stride = 0 - yaw_correction + shift_suppression;
//       }    
//       	if (R_stride> stride_max) {R_stride= stride_max; }
// 		if (R_stride<-stride_max) {R_stride=-stride_max; }
//       	if (L_stride> stride_max) {L_stride= stride_max; }
// 		if (L_stride<-stride_max) {L_stride=-stride_max; }
//   }else{
//       if (turn_omega < 0){ // 左转
//           R_stride = stride;
//           L_stride = stride - 2.0f*yaw_correction;
//       }else{ // 右转
//           R_stride = stride + 2.0f*yaw_correction;
//           L_stride = stride;
//       }
//   }

// 开环左右转控制
    float R_stride = stride;
    float L_stride = stride;
	
	float turn_curr_max = 30.0f;//需要实际测量
	
	 deta_angle=turn_omega_des-body_yaw;
	if(deta_angle>180.0f) deta_angle-=360.0f;
	else if(deta_angle<-180.0f) deta_angle+=360.0f;
	
	// 从turn_omega_des 映射到 turn_omega_corr
	turn_omega_corr = kp_turn_omega*(deta_angle);
                   if (turn_omega_corr>turn_curr_max){turn_omega_corr=turn_curr_max;}
                   if (turn_omega_corr<-turn_curr_max){turn_omega_corr=-turn_curr_max;}
	turn_omega_corr = turn_omega_corr/turn_curr_max;

		
	// turn_omega_corr 映射到 turn_stride
	turn_stride = turn_omega_corr;


	// turn_stride从-1到1，控制转动的差速步长
	if (turn_stride > 0.005f){
		R_stride = stride * (1.0f - 2.0f * turn_stride);
		L_stride = stride;
	}else if (turn_stride < -0.005f){
		R_stride = stride;
		L_stride = stride * (1.0f + 2.0f * turn_stride);
	}
    // 限制步幅在合理范围内
    if (tau <= 0.5f)  
    {
        GaitPhasesPoints RightState = gaitGenerator(0, height, step_height, R_stride);
        GaitPhasesPoints LeftState = gaitGenerator(0, height, step_height, L_stride);

        hposition1.B_y  = RightState.ySwing * (1.0f+tanf(pi*stab_roll/180.0f)) ;
        hposition1.B_x  =  RightState.xSwing; 
        hposition2.B_y  = RightState.ySupport * (1.0f+tanf(pi*stab_roll/180.0f));
        hposition2.B_x  =  RightState.xSupport; 
        hposition3.B_y  = LeftState.ySwing * (1.0f-tanf(pi*stab_roll/180.0f));
        hposition3.B_x  =  LeftState.xSwing; 
        hposition4.B_y  = LeftState.ySupport * (1.0f-tanf(pi*stab_roll/180.0f));
        hposition4.B_x  =  LeftState.xSupport; 
        set_Motor_Kp(0,1,0,1);
    }
    else if (tau > 0.5f && tau <= 1.0f)  //
    {
        GaitPhasesPoints RightState = gaitGenerator(1, height, step_height, R_stride);
        GaitPhasesPoints LeftState = gaitGenerator(1, height, step_height, L_stride);

        hposition1.B_y  = RightState.ySupport * (1.0f+tanf(pi*stab_roll/180.0f));
        hposition1.B_x  =  RightState.xSupport; 
        hposition2.B_y  = RightState.ySwing * (1.0f+tanf(pi*stab_roll/180.0f));
        hposition2.B_x  =  RightState.xSwing;
        hposition3.B_y  = LeftState.ySupport * (1.0f-tanf(pi*stab_roll/180.0f));
        hposition3.B_x  =  LeftState.xSupport;
        hposition4.B_y  = LeftState.ySwing * (1.0f-tanf(pi*stab_roll/180.0f));
        hposition4.B_x  =  LeftState.xSwing;
        set_Motor_Kp(1,0,1,0);  // 设置电机Kp参数
    }

    inverseKinematic_All(); // 计算每个电机的目标角度
    Motor_SendCmd_AllAngle();   // 发送命令给电机
}

void motion_StandBy(float height)
{
    hposition1.B_y = height * (1.0f+tanf(pi*stab_roll/180.0f));
    hposition2.B_y = height * (1.0f+tanf(pi*stab_roll/180.0f));
    hposition3.B_y = height * (1.0f-tanf(pi*stab_roll/180.0f));
    hposition4.B_y = height * (1.0f-tanf(pi*stab_roll/180.0f));
    hposition1.B_x = 0.0f;
    hposition2.B_x = 0.0f;
    hposition3.B_x = 0.0f;
    hposition4.B_x = 0.0f;
    set_Motor_Kp(1,1,1,1);
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
}

void StepInPlace(float height, float step_height)
{   
    float r = step_height / 2.0f;
    float center_y = height - r;

    if(tau >= 1.0f){tau = 0.0f;}
    tau += 0.03f;
    float theta = 2.0f * pi * tau/0.5f;
    if (tau <= 0.5f)
    {
        hposition1.B_x = 0.0f;
        hposition1.B_y = r + center_y;
        hposition2.B_x = 0.0f;
        hposition2.B_y = r * cosf(theta) + center_y;    
        hposition3.B_x = 0.0f;
        hposition3.B_y = r + center_y;
        hposition4.B_x = 0.0f;
        hposition4.B_y = r * cosf(theta) + center_y;
        set_Motor_Kp(1,0,1,0);
    }
    else
    {
        hposition1.B_x = 0.0f;
        hposition1.B_y = r * cosf(theta) + center_y;
        hposition2.B_x = 0.0f;
        hposition2.B_y = r + center_y;    
        hposition3.B_x = 0.0f;
        hposition3.B_y = r * cosf(theta) + center_y;
        hposition4.B_x = 0.0f;
        hposition4.B_y = r + center_y;
        set_Motor_Kp(0,1,0,1);
    }
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();    
}

// flip_state 1是正面，0是反面
void flip_body()
{
    static int flip_state = 1;

    hposition1.B_x = 0.0f;
    hposition1.B_y = 20.0f;
    hposition2.B_x = 0.0f;
    hposition2.B_y = 20.0f;    
    hposition3.B_x = 0.0f;
    hposition3.B_y = 20.0f;
    hposition4.B_x = 0.0f;
    hposition4.B_y = 20.0f;

    if (flip_state == 1){
        for (flip_offset = 0.0f; flip_offset <= 3.165f; flip_offset+=0.05f){
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
            HAL_Delay(3);
        }
        flip_state = 0;
    }else{
        for (flip_offset = 3.165f; flip_offset >= 0.0f; flip_offset-=0.05f){
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();    
            HAL_Delay(3);
        }
        flip_state = 1;
    }
}

void motion_Up(float start,float des)
{
	set_Motor_Kp(1,1,1,1);
    float r = fabsf(des - start) * 0.5f;
    float center_y = (des + start) / 2.0f;
    float theta = 0.0f;

    for (float i = 0.0f; i <= 1.0f; i += up_down_freq){
        theta = pi * i;
        hposition1.B_x = 0.0f;
        hposition1.B_y = -r * cosf(theta) + center_y;
        hposition2.B_x = 0.0f;
        hposition2.B_y = -r * cosf(theta) + center_y;    
        hposition3.B_x = 0.0f;
        hposition3.B_y = -r * cosf(theta) + center_y;
        hposition4.B_x = 0.0f;
        hposition4.B_y = -r * cosf(theta) + center_y;

        inverseKinematic_All();
        Motor_SendCmd_AllAngle(); 
    }
}

void motion_Down(float start, float des)
{
	set_Motor_Kp(1,1,1,1);
    float r = fabsf(des - start) * 0.5f;
    float center_y = (des + start) / 2.0f;
    float theta = 0.0f;

    for (float i = 0.0f; i <= 1.0f; i += up_down_freq){
        theta = pi * i;
        hposition1.B_x = 0.0f;
        hposition1.B_y = r * cosf(theta) + center_y;
        hposition2.B_x = 0.0f;
        hposition2.B_y = r * cosf(theta) + center_y;    
        hposition3.B_x = 0.0f;
        hposition3.B_y = r * cosf(theta) + center_y;
        hposition4.B_x = 0.0f;
        hposition4.B_y = r * cosf(theta) + center_y;

        inverseKinematic_All();
        Motor_SendCmd_AllAngle(); 
    }
}


// 往前跳, 15.5f~37.0f 
void motion_Jump(float stride)
{
    float height_des = 38.5f; // 蹬地腿长
    float jump_step_height = height_des - 13.0f; // 抬腿高度,参考点为轴心

    float forward_prep_step = 8.0f; // 起跳前水平准备距离
    float k0 = (15.5f - walk_height) / (forward_prep_step); // 斜率
    float x_start0 = 0.0f;
    float x_stop0 = forward_prep_step;
	
	// state0蹲下  (0,walk_height) → (forward_prep_step,15.5)
    for (float x = x_start0; x <= x_stop0; x += jump_freq0){
		if (emergency_stop==1){return;}
        float y = k0 * x + walk_height;
        hposition1.B_y = y;
        hposition1.B_x = x; 
        hposition2.B_y = y;
        hposition2.B_x = x; 
        hposition3.B_y = y;
        hposition3.B_x = x; 
        hposition4.B_y = y;
        hposition4.B_x = x;
        inverseKinematic_All();
        Motor_SendCmd_AllAngle(); 
    }
	
	// state1水平运腿 (9.0,15.5) → (9.0,-stride / 2.0f)
	float k2 = 2*height_des/stride; // 斜率
	float x_start2 = sqrt(15.5f*15.5f/(1+k2*k2)); // state2终点提前

    for (float i = 0.0f; i <=  1.0f; i += jump_freq1){
		if (emergency_stop==1){return;}
		
        float x = forward_prep_step + (-x_start2 - forward_prep_step) * i*i;
        float y = 15.5f;
        hposition1.B_y = y;
        hposition1.B_x = x; 
        hposition2.B_y = y;
        hposition2.B_x = x; 
        hposition3.B_y = y;
        hposition3.B_x = x; 
        hposition4.B_y = y;
        hposition4.B_x = x;
        inverseKinematic_All();
        Motor_SendCmd_AllAngle(); 
    }
        
		
	// state 2 蹬腿
    // ---------MIT版本蹬腿--------
	float y_start2 = k2 * x_start2;
	float x_stop2 = sqrt(height_des*height_des/(1+k2*k2));
	// float y_des = k2 * x_stop2;

	motor1_kp_offset *= 1.45f;
	motor2_kp_offset *= 1.35f;
	motor7_kp_offset *= 1.9f;
	motor8_kp_offset *= 1.9f;


    quick_set_kp(9.0f,0.00f); // 跳跃时增大Kp，提升响应速度
	for (float x = x_start2;x < x_stop2; x += jump_freq2*(fabsf(x_start2 - x_stop2))){
	if (emergency_stop==1){return;}
	
	 float y = k2 * x;
	 hposition1.B_y = y;
	 hposition1.B_x = -x; 
	 hposition2.B_y = y;
	 hposition2.B_x = -x; 
	 hposition3.B_y = y;
	 hposition3.B_x = -x; 
	 hposition4.B_y = y;
	 hposition4.B_x = -x; 

	 inverseKinematic_All();
	 Motor_SendCmd_AllAngle(); 
	 //HAL_Delay(3);
	}
	HAL_Delay(130); // 等完全蹬直
	
	motor1_kp_offset /= 1.3f;
	motor2_kp_offset /= 1.2f;
	motor7_kp_offset /= 1.3f;
	motor8_kp_offset /= 1.3f;
	
// ---------异步力矩版蹬腿----------

	
	
	// state 3收腿前送
	quick_set_kp(1.0f,Expect_kw); 
	 for (float angle = 0.0f;angle <=pi; angle += jump_freq3 * pi){
		if (emergency_stop==1){return;}

		float x = stride * ((angle - sinf(angle)) / (2 * pi)) - stride / 2;
		float y = height_des - jump_step_height * (1 - cos(angle)) / 2;
		y *= 0.9f;

		hposition1.B_y = y;
		hposition1.B_x = x; 
		hposition2.B_y = y;
		hposition2.B_x = x; 
		hposition3.B_y = y;
		hposition3.B_x = x; 
		hposition4.B_y = y;
		hposition4.B_x = x;

		inverseKinematic_All();
		Motor_SendCmd_AllAngle(); 
	 }
	

	 for (float angle = pi;angle <=2*pi; angle += jump_freq4 * pi){
		if (emergency_stop==1){return;}

		 float x = stride * ((angle - sinf(angle)) / (2 * pi)) - stride / 2;
		 float y = walk_height - (walk_height - 15.5f) * (1 - cosf(angle)) / 2;

		 hposition1.B_y = y;
		 hposition1.B_x = x; 
		 hposition2.B_y = y;
		 hposition2.B_x = x; 
		 hposition3.B_y = y;
		 hposition3.B_x = x; 
		 hposition4.B_y = y;
		 hposition4.B_x = x;

		 inverseKinematic_All();
		 Motor_SendCmd_AllAngle(); 
	 }
	 
	 quick_set_kp(0.4f,Expect_kw); // 落地时减小Kp，增加缓冲，防止过度震荡    

	// state4 水平回收
	 for (float x = stride / 2.0f; x > 0.0f; x -= jump_freq5 * stride){
			if (emergency_stop==1){return;}

		 float y = walk_height;

		 hposition1.B_y = y;
		 hposition1.B_x = x; 
		 hposition2.B_y = y;
		 hposition2.B_x = x; 
		 hposition3.B_y = y;
		 hposition3.B_x = x; 
		 hposition4.B_y = y;
		 hposition4.B_x = x;

		 inverseKinematic_All();
		 Motor_SendCmd_AllAngle(); 
	 }
	 inverseKinematic_All();
	 Motor_SendCmd_AllAngle(); 

}

/*
 * motion_SmallJump - 小跳上台阶，一跳一级
 * 设计目标：垂直方向为主，稳定跃上 10cm 高台阶，落于 30cm 深踏面
 * 遥控触发，按一次跳一级
 *
 * 调参入口（全部集中于此）：
 *   STEP_HEIGHT     台阶实际高度
 *   SAFETY_MARGIN   跳高安全余量（越大越稳，但落地冲击也大）
 *   CROUCH_H        下蹲深度（越小储能越多，但别太小导致运动学无解）
 *   PUSH_H          蹬直腿长（不要到机械极限 38.5）
 *   SMALL_STRIDE    水平跨度（越小越垂直，越大越往前窜）
 *   KP_BOOST        蹬腿时 Kp 基础值（越大越猛，过大会震荡/过流）
 */
void motion_SmallJump(void)
{
    /* ======== 可调参数 ======== */
    const float STEP_HEIGHT   = 10.0f;   // 台阶高差 (cm)
    const float SAFETY_MARGIN = 5.0f;    // 跳高余量 → 目标抬升 ≈15cm
    const float CROUCH_H      = 18.0f;   // 下蹲腿长 (< walk_height=22)
    const float PUSH_H        = 30.0f;   // 蹬直腿长 (< 机械极限 38.5)
    const float SMALL_STRIDE  = 12.0f;   // 水平"虚拟步幅"，越小=越垂直
    const float KP_BOOST      = 5.0f;    // 蹬腿时基础 Kp（越大越猛）

    float forward_prep = 4.0f;           // 下蹲时脚前移量

    /* 预计算蹬地方向（共用，避免多处重复算） */
    float k2       = 2.0f * PUSH_H / SMALL_STRIDE;
    float k2_sq_p1 = 1.0f + k2 * k2;    // 1 + k2²，复用

    /* 蹬地直线的起点/终点（腿长 = 到髋关节的欧式距离） */
    float x_start  = sqrtf(CROUCH_H * CROUCH_H / k2_sq_p1); // 在 CROUCH_H 腿长时
    float x_stop   = sqrtf(PUSH_H   * PUSH_H   / k2_sq_p1); // 在 PUSH_H   腿长时

    /* 空中摆线峰值：两段摆线在 θ=π 处的衔接高度
     * STEP_HEIGHT + SAFETY_MARGIN ≈ 15cm 是目标净抬升量，由
     * "蹬腿产生的身体上升 + 腿伸展量" 共同提供。
     * 这里取 PUSH_H 作为峰值，保证腿部有足够伸展余量。 */
    float peak_y = PUSH_H;

    /* ======== State 0: 下蹲蓄力 ======== */
    /* (0, walk_height)  →  (forward_prep, CROUCH_H)                     */
    {
        float k0 = (CROUCH_H - walk_height) / forward_prep;
        for (float x = 0.0f; x <= forward_prep; x += 0.06f) {
            if (emergency_stop == 1) return;
            float y = k0 * x + walk_height;
            hposition1.B_y = y;  hposition1.B_x = x;
            hposition2.B_y = y;  hposition2.B_x = x;
            hposition3.B_y = y;  hposition3.B_x = x;
            hposition4.B_y = y;  hposition4.B_x = x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
    }

    /* ======== State 1: 水平收腿到起跳位 ======== */
    /* 保持 CROUCH_H，二次插值平滑收腿至蹬地起点 (x = -x_start)          */
    {
        for (float i = 0.0f; i <= 1.0f; i += 0.015f) {
            if (emergency_stop == 1) return;
            float x = forward_prep + (-x_start - forward_prep) * i * i;
            float y = CROUCH_H;
            hposition1.B_y = y;  hposition1.B_x = x;
            hposition2.B_y = y;  hposition2.B_x = x;
            hposition3.B_y = y;  hposition3.B_x = x;
            hposition4.B_y = y;  hposition4.B_x = x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
    }

    /* ======== State 2: 蹬腿起跳 ======== */
    /* 沿 y = k2*x 直线蹬直，大幅提高 Kp 以产生爆发力                    */
    {
        /* 保存原始 KP */
        float saved[4] = {motor1_kp_offset, motor2_kp_offset,
                          motor7_kp_offset, motor8_kp_offset};

        /* 提升 KP */
        motor1_kp_offset *= 1.3f;
        motor2_kp_offset *= 1.2f;
        motor7_kp_offset *= 1.5f;
        motor8_kp_offset *= 1.5f;
        quick_set_kp(KP_BOOST, 0.0f);

        float range = fabsf(x_start - x_stop);
        for (float x = x_start; x < x_stop; x += 0.35f * range) {
            if (emergency_stop == 1) goto restore_kp;
            float y = k2 * x;
            hposition1.B_y = y;  hposition1.B_x = -x;
            hposition2.B_y = y;  hposition2.B_x = -x;
            hposition3.B_y = y;  hposition3.B_x = -x;
            hposition4.B_y = y;  hposition4.B_x = -x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
        HAL_Delay(60);   // 等待蹬直

    restore_kp:
        motor1_kp_offset = saved[0];
        motor2_kp_offset = saved[1];
        motor7_kp_offset = saved[2];
        motor8_kp_offset = saved[3];
        quick_set_kp(1.0f, Expect_kw);
    }
    /* 急停检查：goto 过来的话 KP 已恢复，直接退出 */
    if (emergency_stop == 1) return;

    /* ======== State 3: 空中收腿前摆（前半段摆线，0→π） ======== */
    /* y: CROUCH_H  →  peak_y(PUSH_H)，足端抬起清过台阶边缘              */
    {
        for (float angle = 0.0f; angle <= pi; angle += 0.035f * pi) {
            if (emergency_stop == 1) return;
            float x = SMALL_STRIDE * ((angle - sinf(angle)) / (2.0f * pi))
                      - SMALL_STRIDE / 2.0f;
            float y = CROUCH_H + (peak_y - CROUCH_H) * (1.0f - cosf(angle)) / 2.0f;

            hposition1.B_y = y;  hposition1.B_x = x;
            hposition2.B_y = y;  hposition2.B_x = x;
            hposition3.B_y = y;  hposition3.B_x = x;
            hposition4.B_y = y;  hposition4.B_x = x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
    }

    /* ======== State 4: 落腿着地（后半段摆线，π→2π） ======== */
    /* y: peak_y(PUSH_H)  →  walk_height，两段在 θ=π 处连续               */
    /* 降低 Kp 以缓冲落地冲击                                            */
    {
        quick_set_kp(0.3f, Expect_kw);   // 落地时低 Kp，软着陆
        for (float angle = pi; angle <= 2.0f * pi; angle += 0.006f * pi) {
            if (emergency_stop == 1) return;
            float x = SMALL_STRIDE * ((angle - sinf(angle)) / (2.0f * pi))
                      - SMALL_STRIDE / 2.0f;
            float y = walk_height + (peak_y - walk_height) * (1.0f - cosf(angle)) / 2.0f;

            hposition1.B_y = y;  hposition1.B_x = x;
            hposition2.B_y = y;  hposition2.B_x = x;
            hposition3.B_y = y;  hposition3.B_x = x;
            hposition4.B_y = y;  hposition4.B_x = x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
    }

    /* ======== State 5: 水平回收，站稳 ======== */
    {
        for (float x = SMALL_STRIDE / 2.0f; x > 0.0f; x -= 0.005f * SMALL_STRIDE) {
            if (emergency_stop == 1) return;
            float y = walk_height;
            hposition1.B_y = y;  hposition1.B_x = x;
            hposition2.B_y = y;  hposition2.B_x = x;
            hposition3.B_y = y;  hposition3.B_x = x;
            hposition4.B_y = y;  hposition4.B_x = x;
            inverseKinematic_All();
            Motor_SendCmd_AllAngle();
        }
        inverseKinematic_All();
        Motor_SendCmd_AllAngle();
    }
}

void motion_Frontflip(void){
    /*
    1.后脚蹬，给个初始翻转速度
    2.前后脚都收到90度
    3.身体自然往前倒10度左右
    4.前后一起蹬
    5.前后脚都收到翻转后的坐标系的90度，保持蹬地状态直到落地
    6.翻转坐标系

    存储参考角：1.起跳前的站立  2.后推蹬地的坐标（3，17）对应的角度  3.原地腿伸直的角度

    注意：全程慎用inverseKinematic，完全使用角度控制。完成动作后记得翻转坐标系
    */
    float front_retract_angle = 90.0f; // 前腿收腿角度
    float back_retract_angle = 90.0f; // 后腿收腿角度
    float front_skip_Pitch = 100.0f; // 前空翻时身体前倾的角度
    float back_skip_Pitch = -50.0f; // 后腿蹬地直到身体前倾到back_skip_Pitch
    float skip_Height = 35.0f; // 前空翻时蹬地的高度

    float back_support_angle_max = 130.0f; // 后脚蹬地的最大角度
	float front_support_angle_max = 60.0f; // 前脚蹬地的最大角度

    // step0:准备起跳，存储初始角度
    // 存储参考角：1.起跳前的站立
	motion_StandBy(17.0f); // 准备起跳高度
	HAL_Delay(3000);
    float start_h1_alpha1 = hposition1.alpha;
    float start_h1_beta1 = hposition1.beta;
    float start_h2_alpha1 = hposition2.alpha;
    float start_h2_beta1 = hposition2.beta;
    float start_h3_alpha1 = hposition3.alpha;
    float start_h3_beta1 = hposition3.beta;
    float start_h4_alpha1 = hposition4.alpha;
    float start_h4_beta1 = hposition4.beta;

    // 存储参考角：2.后推蹬地的坐标（3，17）对应的角度
    float x2 = 3.0f; 
    float y2 = 17.0f; 
    hposition2.B_y = y2;
    hposition2.B_x = -x2; 
    hposition3.B_y = y2;
    hposition3.B_x = -x2; 
	crawl_inverseKinematic(&hposition2,BACK);
	crawl_inverseKinematic(&hposition3,BACK);
    float start_h2_alpha2 = hposition2.alpha; // 后推蹬地的坐标（3，17）对应的角度  
    float start_h2_beta2 = hposition2.beta;
    float start_h3_alpha2 = hposition3.alpha;
    float start_h3_beta2 = hposition3.beta;

    //  存储参考角：3.原地腿伸直的角度
    hposition1.B_y = 33.0f;
    hposition2.B_y = 33.0f;
    hposition3.B_y = 33.0f;
    hposition4.B_y = 33.0f;
    hposition1.B_x = 0.0f;
    hposition2.B_x = 0.0f;
    hposition3.B_x = 0.0f;
    hposition4.B_x = 0.0f;
    inverseKinematic_All();
    float start_h1_alpha3 = hposition1.alpha;
    float start_h1_beta3 = hposition1.beta;
    float start_h2_alpha3 = hposition2.alpha;
    float start_h2_beta3 = hposition2.beta;
    float start_h3_alpha3 = hposition3.alpha;
    float start_h3_beta3 = hposition3.beta;
    float start_h4_alpha3 = hposition4.alpha;
    float start_h4_beta3 = hposition4.beta;

    // step1.后脚蹬，给个初始翻转速度
    // step2：前后脚都收到90度
    while (body_pitch < front_retract_angle){
        if (emergency_stop==1){return;}

        hposition1.alpha = start_h1_alpha1 + body_pitch;  // motor1
        hposition1.beta =  start_h1_beta1  - body_pitch;    // motor2
        hposition4.alpha = start_h4_alpha1 + body_pitch;   // motor7
        hposition4.beta =  start_h4_beta1  - body_pitch;   // motor8
//        hposition2.alpha = start_h2_alpha1 + body_pitch;   // motor3
//        hposition2.beta =  start_h2_beta1  - body_pitch;   // motor4
//        hposition3.alpha = start_h3_alpha1 + body_pitch;  // motor5
//        hposition3.beta =  start_h3_beta1  - body_pitch;    // motor6


    uint8_t temp = 0;
		if (body_pitch > back_skip_Pitch){ // 后脚蹬
			static float flip_tau1 = 0.0f;
			flip_tau1 += Frontflip_freq1;
			if (flip_tau1 >= 1.0f){ flip_tau1 = 1.0f;}

			 float back_support_angle = flip_tau1*back_support_angle_max; // 后脚蹬地的角度，逐渐增加到最大值
			 hposition2.alpha = start_h2_alpha1 - back_support_angle;   // motor3
			 hposition2.beta =  start_h2_beta1  - back_support_angle;   // motor4
			 hposition3.alpha = start_h3_alpha1 - back_support_angle;  // motor5
			 hposition3.beta =  start_h3_beta1  - back_support_angle;    // motor6
			//HAL_Delay(90);
		temp=1;
		}else{
			hposition2.alpha = start_h2_alpha1 - body_pitch*back_retract_angle/front_retract_angle;   // motor3
			hposition2.beta =  start_h2_beta1  + body_pitch*back_retract_angle/front_retract_angle;   // motor4
			hposition3.alpha = start_h3_alpha1 - body_pitch*back_retract_angle/front_retract_angle;  // motor5
			hposition3.beta =  start_h3_beta1  + body_pitch*back_retract_angle/front_retract_angle;    // motor6
		temp=0;

		}

		 Motor_SendCmd_AllAngle(); 
	 }

}
	
 // --------------------------by LiShuai ,BEGIN
// 
//    for (float x = stride / 2.0f; x > 0.0f; x -= jump_freq4 * stride){
//        float y = walk_height;
 // --------------------------by LiShuai ,BEGIN
// 
//    for (float x = stride / 2.0f; x > 0.0f; x -= jump_freq4 * stride){
//        float y = walk_height;

//        hposition1.B_y = y;
//        hposition1.B_x = x; 
//        hposition2.B_y = y;
//        hposition2.B_x = x; 
//        hposition3.B_y = y;
//        hposition3.B_x = x; 
//        hposition4.B_y = y;
//        hposition4.B_x = x;
//        hposition1.B_y = y;
//        hposition1.B_x = x; 
//        hposition2.B_y = y;
//        hposition2.B_x = x; 
//        hposition3.B_y = y;
//        hposition3.B_x = x; 
//        hposition4.B_y = y;
//        hposition4.B_x = x;

//        inverseKinematic_All();
//        Motor_SendCmd_AllAngle(); 
//    }
//	quick_set_kp(support_Kp); // 落地时减小Kp，增加缓冲，防止过度震荡    
//        inverseKinematic_All();
//        Motor_SendCmd_AllAngle(); 
//    }
//	quick_set_kp(support_Kp); // 落地时减小Kp，增加缓冲，防止过度震荡    


//}
//}



//void testCircle()
//{   
//    if(tau >= 1.0f){tau = 0.0f;}
//void testCircle()
//{   
//    if(tau >= 1.0f){tau = 0.0f;}

//    float r = 8.0f;
//    tau += 0.01f;
//    float theta = 2.0f * pi * tau/1.0f;
//    hposition1.B_x = r * cosf(theta);
//    hposition1.B_y = r * sinf(theta) + 27.0f;
//    hposition2.B_x = -r * cosf(theta);
//    hposition2.B_y = -r * sinf(theta) + 27.0f;
//    hposition3.B_x = r * cosf(theta);
//    hposition3.B_y = r * sinf(theta) + 27.0f;
//    hposition4.B_x = -r * cosf(theta);
//    hposition4.B_y = -r * sinf(theta) + 27.0f;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//}
//    float r = 8.0f;
//    tau += 0.01f;
//    float theta = 2.0f * pi * tau/1.0f;
//    hposition1.B_x = r * cosf(theta);
//    hposition1.B_y = r * sinf(theta) + 27.0f;
//    hposition2.B_x = -r * cosf(theta);
//    hposition2.B_y = -r * sinf(theta) + 27.0f;
//    hposition3.B_x = r * cosf(theta);
//    hposition3.B_y = r * sinf(theta) + 27.0f;
//    hposition4.B_x = -r * cosf(theta);
//    hposition4.B_y = -r * sinf(theta) + 27.0f;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//}

///*
//前后两腿依次按照椭圆轨迹向前移动stride
//前后腿向后移动stride，身体向前送
//后腿依次按照椭圆轨迹向前移动stride
//*/
//void test_low_walk(float stride,float b) //b是椭圆的参数
//{
//    float walk_H=15.5f;
//    float low_H=5.5f;
//    float theta=0.0f;
//    //蹲下去
//  for(theat=0.0f;theat<pi;theat+=up_down_freq*pi)
//  {
//    float r=(walk_H-low_H)/2;
//    float center_y=(walk_H+low_H)/2;
//    hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
//    hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
//    hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
//    hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  float s=stride;
//  float r=stride/2.0f;
//  float p=r*b/sqrt(b*b*(cosf(theat)*cosf(theat))+r*r*(sinf(theta)*sinf(theta))); //椭圆的极径,中心在(r,0)处 r=a
//  //右前腿按照椭圆轨迹移动至s
//  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//   // float x=r*(1+cosf(theat));
//   // float y=r*sinf(theat);
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=x; hposition1.B_y=low_H-y;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=0.0f; hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //左前腿按照椭圆轨迹移动至s
//  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=s; hposition1.B_y=low_H;
//    hposition4.B_x=x; hposition4.B_y=low_H-y;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=0.0f; hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //1,4回收 2,3后送
//  for(theat=0.0f;theat<pi;theta+=up_down_freq*pi)
//  {
//    float x=s*(cosf(1+theat)/2.0f);
///*
//前后两腿依次按照椭圆轨迹向前移动stride
//前后腿向后移动stride，身体向前送
//后腿依次按照椭圆轨迹向前移动stride
//*/
//void test_low_walk(float stride,float b) //b是椭圆的参数
//{
//    float walk_H=15.5f;
//    float low_H=5.5f;
//    float theta=0.0f;
//    //蹲下去
//  for(theat=0.0f;theat<pi;theat+=up_down_freq*pi)
//  {
//    float r=(walk_H-low_H)/2;
//    float center_y=(walk_H+low_H)/2;
//    hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
//    hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
//    hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
//    hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  float s=stride;
//  float r=stride/2.0f;
//  float p=r*b/sqrt(b*b*(cosf(theat)*cosf(theat))+r*r*(sinf(theta)*sinf(theta))); //椭圆的极径,中心在(r,0)处 r=a
//  //右前腿按照椭圆轨迹移动至s
//  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//   // float x=r*(1+cosf(theat));
//   // float y=r*sinf(theat);
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=x; hposition1.B_y=low_H-y;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=0.0f; hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //左前腿按照椭圆轨迹移动至s
//  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=s; hposition1.B_y=low_H;
//    hposition4.B_x=x; hposition4.B_y=low_H-y;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=0.0f; hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //1,4回收 2,3后送
//  for(theat=0.0f;theat<pi;theta+=up_down_freq*pi)
//  {
//    float x=s*(cosf(1+theat)/2.0f);

//    hposition1.B_x=x; hposition1.B_y=low_H;
//    hposition4.B_x=x; hposition4.B_y=low_H;
//    hposition2.B_x=-s*(cosf(1-theat)/2.0f); hposition2.B_y=low_H;
//    hposition3.B_x=-s*(cosf(1-theat)/2.0f); hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //右后腿按照椭圆轨迹移动至0.0f
//   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=0.0f; hposition1.B_y=low_H;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=-s+x; hposition2.B_y=low_H-y;
//    hposition3.B_x=-s; hposition3.B_y=low_H;
//     inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //右后腿按照椭圆轨迹移动至0.0f || 最终回到状态1
//   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=0.0f; hposition1.B_y=low_H;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=-s+x; hposition3.B_y=low_H-y;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//}
//    hposition1.B_x=x; hposition1.B_y=low_H;
//    hposition4.B_x=x; hposition4.B_y=low_H;
//    hposition2.B_x=-s*(cosf(1-theat)/2.0f); hposition2.B_y=low_H;
//    hposition3.B_x=-s*(cosf(1-theat)/2.0f); hposition3.B_y=low_H;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //右后腿按照椭圆轨迹移动至0.0f
//   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=0.0f; hposition1.B_y=low_H;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=-s+x; hposition2.B_y=low_H-y;
//    hposition3.B_x=-s; hposition3.B_y=low_H;
//     inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//  //右后腿按照椭圆轨迹移动至0.0f || 最终回到状态1
//   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
//  {
//    float x=r+p*cosf(theta);
//    float y=p*sinf(theat);
//    hposition1.B_x=0.0f; hposition1.B_y=low_H;
//    hposition4.B_x=0.0f; hposition4.B_y=low_H;
//    hposition2.B_x=0.0f; hposition2.B_y=low_H;
//    hposition3.B_x=-s+x; hposition3.B_y=low_H-y;
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//  }
//}

//// void test_jump_filp(float stride)
//// {
////   float walk_hight=0.0f;
////   float height_des=1.0f;
////   float long_hight=20.0f;
////   float angle=0.0f;
////   uint8_t pitch_triggered=0;
////   //ready
////   for(float theta=0.0f;theta<pi;theta+=0.05f)
////   {
////     float r=(long_hight-walk_hight)/2;
////     float center_y=(long_hight+walk_hight)/2;
////     hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
////     hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
////     hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
////     hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
////     inverseKinematic_All();
////     Motor_SendCmd_AllAngle();   
////   }
////   //前脚起跳
////     float k=2*long_hight/stride;
////     float x_start = sqrt(walk_hight*walk_hight/(1+k*k));
//// 	float y_start = k * x_start;
////     float x_des = sqrt(height_des*height_des/(1+k*k));
////     float y_des = k * x_des;
////     quick_set_kp(1.5f); // 跳跃时增大Kp，提升响应速度
////   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
////   {
////     float y=k*x;
////     hposition1.B_x=x; hposition1.B_y=y;
////     hposition4.B_x=x; hposition4.B_y=y;
////     hposition2.B_x=0.0f; hposition2.B_y=walk_hight;
////     hposition3.B_x=0.0f; hposition3.B_y=walk_hight;
////     inverseKinematic_All();
////     Motor_SendCmd_AllAngle();   
////      if(body_pitch<SET_BODY_ANGLE){
////         pitch_triggered=1;
////         break;
////      }
////   }
////     //后脚起跳
////   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
////     {
////         float y=k*x;
////         if(pitch_triggered){quick_set_kp(2.0f);}
////         else{quick_set_kp(1.0f);}
////         hposition1.B_x=x_des; hposition1.B_y=k*x_des;
////         hposition4.B_x=x_des; hposition4.B_y=k*x_des;
////         hposition2.B_x=x; hposition2.B_y=y;
////         hposition3.B_x=x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();   
////     }
////     //空中前半段，收腿
////    for(angle=0;angle<pi;angle+=jump_freq3*pi)
////    {
////       float x=x_des*(1.0f+cos(angle))/2.0f;
////       float y = k*x_des+(FILP_BACK_Y-k*x_des)*(1.0f-cosf(angle))/2.0f;
////       hposition1.B_x=x; hposition1.B_y=y;
////       hposition4.B_x=x; hposition4.B_y=y;
////       hposition2.B_x=x; hposition2.B_y=y;
////       hposition3.B_x=x; hposition3.B_y=y;
////       inverseKinematic_All();
////       Motor_SendCmd_AllAngle();
////    }
////    //空中后半段，展腿，准备落地支撑
////    quick_set_kp(1.3f);
////    for(angle=pi;angle<2*pi;angle+=jump_freq3*pi)
////    {
////      float x=stride/4.0f*(1.0f+cos(angle))/2.0f;
////      float y=LANDING_Y+(FILP_BACK_Y-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
////      hposition1.B_x=x; hposition1.B_y=y;
////      hposition4.B_x=x; hposition4.B_y=y;
////      hposition2.B_x=-x; hposition2.B_y=y;
////      hposition3.B_x=-x; hposition3.B_y=y;
////      inverseKinematic_All();
////      Motor_SendCmd_AllAngle();
////    }
////    //落地缓冲
////     quick_set_kp(0.35f);
////     for(angle=2*pi;angle<3*pi;angle+=jump_freq3*pi)
////     {
////         float x=stride/4.0f;
////         float y=LANDING_Y+(walk_hight-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
////         hposition1.B_x=x; hposition1.B_y=y;
////         hposition4.B_x=x; hposition4.B_y=y;
////         hposition2.B_x=-x; hposition2.B_y=y;
////         hposition3.B_x=-x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();
//// void test_jump_filp(float stride)
//// {
////   float walk_hight=0.0f;
////   float height_des=1.0f;
////   float long_hight=20.0f;
////   float angle=0.0f;
////   uint8_t pitch_triggered=0;
////   //ready
////   for(float theta=0.0f;theta<pi;theta+=0.05f)
////   {
////     float r=(long_hight-walk_hight)/2;
////     float center_y=(long_hight+walk_hight)/2;
////     hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
////     hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
////     hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
////     hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
////     inverseKinematic_All();
////     Motor_SendCmd_AllAngle();   
////   }
////   //前脚起跳
////     float k=2*long_hight/stride;
////     float x_start = sqrt(walk_hight*walk_hight/(1+k*k));
//// 	float y_start = k * x_start;
////     float x_des = sqrt(height_des*height_des/(1+k*k));
////     float y_des = k * x_des;
////     quick_set_kp(1.5f); // 跳跃时增大Kp，提升响应速度
////   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
////   {
////     float y=k*x;
////     hposition1.B_x=x; hposition1.B_y=y;
////     hposition4.B_x=x; hposition4.B_y=y;
////     hposition2.B_x=0.0f; hposition2.B_y=walk_hight;
////     hposition3.B_x=0.0f; hposition3.B_y=walk_hight;
////     inverseKinematic_All();
////     Motor_SendCmd_AllAngle();   
////      if(body_pitch<SET_BODY_ANGLE){
////         pitch_triggered=1;
////         break;
////      }
////   }
////     //后脚起跳
////   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
////     {
////         float y=k*x;
////         if(pitch_triggered){quick_set_kp(2.0f);}
////         else{quick_set_kp(1.0f);}
////         hposition1.B_x=x_des; hposition1.B_y=k*x_des;
////         hposition4.B_x=x_des; hposition4.B_y=k*x_des;
////         hposition2.B_x=x; hposition2.B_y=y;
////         hposition3.B_x=x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();   
////     }
////     //空中前半段，收腿
////    for(angle=0;angle<pi;angle+=jump_freq3*pi)
////    {
////       float x=x_des*(1.0f+cos(angle))/2.0f;
////       float y = k*x_des+(FILP_BACK_Y-k*x_des)*(1.0f-cosf(angle))/2.0f;
////       hposition1.B_x=x; hposition1.B_y=y;
////       hposition4.B_x=x; hposition4.B_y=y;
////       hposition2.B_x=x; hposition2.B_y=y;
////       hposition3.B_x=x; hposition3.B_y=y;
////       inverseKinematic_All();
////       Motor_SendCmd_AllAngle();
////    }
////    //空中后半段，展腿，准备落地支撑
////    quick_set_kp(1.3f);
////    for(angle=pi;angle<2*pi;angle+=jump_freq3*pi)
////    {
////      float x=stride/4.0f*(1.0f+cos(angle))/2.0f;
////      float y=LANDING_Y+(FILP_BACK_Y-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
////      hposition1.B_x=x; hposition1.B_y=y;
////      hposition4.B_x=x; hposition4.B_y=y;
////      hposition2.B_x=-x; hposition2.B_y=y;
////      hposition3.B_x=-x; hposition3.B_y=y;
////      inverseKinematic_All();
////      Motor_SendCmd_AllAngle();
////    }
////    //落地缓冲
////     quick_set_kp(0.35f);
////     for(angle=2*pi;angle<3*pi;angle+=jump_freq3*pi)
////     {
////         float x=stride/4.0f;
////         float y=LANDING_Y+(walk_hight-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
////         hposition1.B_x=x; hposition1.B_y=y;
////         hposition4.B_x=x; hposition4.B_y=y;
////         hposition2.B_x=-x; hposition2.B_y=y;
////         hposition3.B_x=-x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();

////     }
////     //落地后站立
////     quick_set_kp(support_Kp);
////     for(angle=3*pi;angle<4*pi;angle+=jump_freq3*pi)
////     {
////         float x=stride/4.0f+(-stride/4.0f)*(1.0f+cos(angle))/2.0f;
////         float y=walk_hight;
////         hposition1.B_x=x; hposition1.B_y=y;
////         hposition4.B_x=x; hposition4.B_y=y;
////         hposition2.B_x=-x; hposition2.B_y=y;
////         hposition3.B_x=-x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();
////     }
//// }
//	
//void state_filp_jump(float stride)
//{
//   static filp_jump_state_t j_state=FILP_IDLE;
//   static float ready_height=12.8f;
//   static float walk_height=20.0f;
//   static float height_des=33.0f;
//   static float back_x=0.0f; 
//   static float back_y=0.0f;
//   float fqu=0.02f;
//   switch(j_state)
//   {
//    case FILP_IDLE:
//    {
//        quick_set_kp(support_Kp);
//        hposition1.B_x = 0.0f; hposition1.B_y = walk_height;
//        hposition4.B_x = 0.0f; hposition4.B_y = walk_height;
//        hposition2.B_x = 0.0f; hposition2.B_y = walk_height;
//        hposition3.B_x = 0.0f; hposition3.B_y = walk_height;
//        j_state=FILP_READY;
//        break;
//    }
//    case FILP_READY:
//    {
//        //前后腿高度到ready_height,后退往前送stride/2
//        float r=(walk_height-ready_height)/2.0f;
//        float center_y=(ready_height+walk_height)/2.0f;
//        static float theta=0.0f;
//        hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
//        hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
//        hposition2.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition2.B_y=center_y+r*cos(theta);
//        hposition3.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition3.B_y=center_y+r*cos(theta);
//        theta+=fqu*pi;
//        if(theta>=pi){theta=0.0f; j_state=FILP_BACK_JUMP;}
//        break;
//    }
//    case FILP_BACK_JUMP:
//    {
//         //后脚起跳
//        float s=stride/2.0f; 
//        float back_ready_long=sqrtf(s*s+ready_height*ready_height);
//        float k=2*height_des/s;
//        float x_start = sqrt(back_ready_long*back_ready_long/(1+k*k)); //sqrt(ready_height*ready_height/(1+k*k));
//        float y_start = k * x_start;
//        float x_des = sqrt(height_des*height_des/(1+k*k));
//        float y_des = k * x_des-;
//        static float x=0.0f;
//        static uint8_t inited = 0;
//        if(!inited){x=x_start; inited=1; quick_set_kp(1.5f);}
//        float y=k*x;
//        hposition2.B_x=-x; hposition2.B_y=y;
//        hposition3.B_x=-x; hposition3.B_y=y;
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height;    
//        if(body_pitch<=SET_BODY_PITCH) //腿可能还没到位置，但重心已经能满足翻滚条件了
//        {
//            inited=0;
//            back_x=x; back_y=y;  //记录此时真实的坐标，用于后续计算,取正值方便算
//            j_state=FILP_READY_FONT_JUMP;
//        }
//        else if(x<=x_des)
//        {
//            inited=0;
//            x=x_start;
//            j_state=FILP_JUMP_RECOVER;
//        }else
//        {
//            x+=jump_freq4*fabsf(x_start-x_des);
//        }
//        break;
//    }
//    case FILP_JUMP_RECOVER:
//    {
//       static float s=0.0f;
//       float y=back_y+(ready_height-back_y)*(1-cosf(s))/2.0f;
//       float x=back_x*(1+cosf(s))/2.0f;
//        hposition2.B_x=-x; hposition2.B_y=y;
//        hposition3.B_x=-x; hposition3.B_y=y;
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height; 
//        s+=fqu;
//        if(s>=pi)
//        {
//            s=0.0f;
//            if(body_pitch<=LIMIT_PITCH){j_state=FILP_WALK_STATE;}
//        }
//        break;
//    }
//    case FILP_READY_FONT_JUMP:
//    {
//        //后退转至相对身体水平位置，前腿先直接向前移(jump=0,只做翻转 || jump=1,跳加翻转)
//       float inside_r=ready_height;
//       float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
//       float theta_begin=atan2f(ready_height,fabsf(back_x));
//       float theta_end=0.0f;
//       static float theta=0.0f;
//       static uint8_t inited=0;
//       if(!inited){ inited=1; theta=theta_begin;} //只再第一次进入时赋值
//       float x=outside_r*cosf(theta);
//       float y=outside_r*sinf(theta); //相当于back_y的值了
//       if(jump==0)
//       {
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height;
//        hposition2.B_x=-x;   hposition2.B_y=y;
//        hposition3.B_x=-x;   hposition3.B_y=y;
//        theta-=fqu*pi;
//        if(theta<=theta_end){theta=theta_begin; inited=0; j_state=FILP_ONLY;}  
//       }
//       break;
//    }
//    case FILP_ONLY:
//    {
//        float x_begin=0.0f;
//        float x_end=stride/2.0f;
//        float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
//        float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
//        static float theta=0.0f;
//        hposition1.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
//        hposition1.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
//        hposition4.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
//        hposition4.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
//        hposition2.B_x=-outside_r;   hposition2.B_y=0.0f;
//        hposition3.B_x=-outside_r;   hposition3.B_y=0.0f;
//        theta+=fqu*pi;
//        if(theta>=pi){theta=0.0f; j_state=FILP_LANDING;}
//        break;
//    }
//    case FILP_HALFS_SKY:
//    {
////     }
////     //落地后站立
////     quick_set_kp(support_Kp);
////     for(angle=3*pi;angle<4*pi;angle+=jump_freq3*pi)
////     {
////         float x=stride/4.0f+(-stride/4.0f)*(1.0f+cos(angle))/2.0f;
////         float y=walk_hight;
////         hposition1.B_x=x; hposition1.B_y=y;
////         hposition4.B_x=x; hposition4.B_y=y;
////         hposition2.B_x=-x; hposition2.B_y=y;
////         hposition3.B_x=-x; hposition3.B_y=y;
////         inverseKinematic_All();
////         Motor_SendCmd_AllAngle();
////     }
//// }
//	
//void state_filp_jump(float stride)
//{
//   static filp_jump_state_t j_state=FILP_IDLE;
//   static float ready_height=12.8f;
//   static float walk_height=20.0f;
//   static float height_des=33.0f;
//   static float back_x=0.0f; 
//   static float back_y=0.0f;
//   float fqu=0.02f;
//   switch(j_state)
//   {
//    case FILP_IDLE:
//    {
//        quick_set_kp(support_Kp);
//        hposition1.B_x = 0.0f; hposition1.B_y = walk_height;
//        hposition4.B_x = 0.0f; hposition4.B_y = walk_height;
//        hposition2.B_x = 0.0f; hposition2.B_y = walk_height;
//        hposition3.B_x = 0.0f; hposition3.B_y = walk_height;
//        j_state=FILP_READY;
//        break;
//    }
//    case FILP_READY:
//    {
//        //前后腿高度到ready_height,后退往前送stride/2
//        float r=(walk_height-ready_height)/2.0f;
//        float center_y=(ready_height+walk_height)/2.0f;
//        static float theta=0.0f;
//        hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
//        hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
//        hposition2.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition2.B_y=center_y+r*cos(theta);
//        hposition3.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition3.B_y=center_y+r*cos(theta);
//        theta+=fqu*pi;
//        if(theta>=pi){theta=0.0f; j_state=FILP_BACK_JUMP;}
//        break;
//    }
//    case FILP_BACK_JUMP:
//    {
//         //后脚起跳
//        float s=stride/2.0f; 
//        float back_ready_long=sqrtf(s*s+ready_height*ready_height);
//        float k=2*height_des/s;
//        float x_start = sqrt(back_ready_long*back_ready_long/(1+k*k)); //sqrt(ready_height*ready_height/(1+k*k));
//        float y_start = k * x_start;
//        float x_des = sqrt(height_des*height_des/(1+k*k));
//        float y_des = k * x_des-;
//        static float x=0.0f;
//        static uint8_t inited = 0;
//        if(!inited){x=x_start; inited=1; quick_set_kp(1.5f);}
//        float y=k*x;
//        hposition2.B_x=-x; hposition2.B_y=y;
//        hposition3.B_x=-x; hposition3.B_y=y;
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height;    
//        if(body_pitch<=SET_BODY_PITCH) //腿可能还没到位置，但重心已经能满足翻滚条件了
//        {
//            inited=0;
//            back_x=x; back_y=y;  //记录此时真实的坐标，用于后续计算,取正值方便算
//            j_state=FILP_READY_FONT_JUMP;
//        }
//        else if(x<=x_des)
//        {
//            inited=0;
//            x=x_start;
//            j_state=FILP_JUMP_RECOVER;
//        }else
//        {
//            x+=jump_freq4*fabsf(x_start-x_des);
//        }
//        break;
//    }
//    case FILP_JUMP_RECOVER:
//    {
//       static float s=0.0f;
//       float y=back_y+(ready_height-back_y)*(1-cosf(s))/2.0f;
//       float x=back_x*(1+cosf(s))/2.0f;
//        hposition2.B_x=-x; hposition2.B_y=y;
//        hposition3.B_x=-x; hposition3.B_y=y;
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height; 
//        s+=fqu;
//        if(s>=pi)
//        {
//            s=0.0f;
//            if(body_pitch<=LIMIT_PITCH){j_state=FILP_WALK_STATE;}
//        }
//        break;
//    }
//    case FILP_READY_FONT_JUMP:
//    {
//        //后退转至相对身体水平位置，前腿先直接向前移(jump=0,只做翻转 || jump=1,跳加翻转)
//       float inside_r=ready_height;
//       float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
//       float theta_begin=atan2f(ready_height,fabsf(back_x));
//       float theta_end=0.0f;
//       static float theta=0.0f;
//       static uint8_t inited=0;
//       if(!inited){ inited=1; theta=theta_begin;} //只再第一次进入时赋值
//       float x=outside_r*cosf(theta);
//       float y=outside_r*sinf(theta); //相当于back_y的值了
//       if(jump==0)
//       {
//        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
//        hposition4.B_x=0.0f; hposition4.B_y=ready_height;
//        hposition2.B_x=-x;   hposition2.B_y=y;
//        hposition3.B_x=-x;   hposition3.B_y=y;
//        theta-=fqu*pi;
//        if(theta<=theta_end){theta=theta_begin; inited=0; j_state=FILP_ONLY;}  
//       }
//       break;
//    }
//    case FILP_ONLY:
//    {
//        float x_begin=0.0f;
//        float x_end=stride/2.0f;
//        float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
//        float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
//        static float theta=0.0f;
//        hposition1.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
//        hposition1.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
//        hposition4.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
//        hposition4.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
//        hposition2.B_x=-outside_r;   hposition2.B_y=0.0f;
//        hposition3.B_x=-outside_r;   hposition3.B_y=0.0f;
//        theta+=fqu*pi;
//        if(theta>=pi){theta=0.0f; j_state=FILP_LANDING;}
//        break;
//    }
//    case FILP_HALFS_SKY:
//    {

//    }
//    case FILP_HALFE_SKY:
//    {
//     
//    }
//    case FILP_LANDING:
//    {
//       float inside_r=ready_height;
//       float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
//       float x_end=stride/2.0f;
//       float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
//       static float theta=0.0f;
//       hposition2.B_x=outside_r+(x_end-outside_r)*(1-cosf(theta))/2.0f;
//       hposition2.B_y=y_end*cosf(theta);
//       hposition2.B_x=outside_r+(x_end-outside_r)*(1-cosf(theta))/2.0f;
//       hposition2.B_y=y_end*cosf(theta);
//       hposition1.B_x=x_end; hposition1.B_y=y_end;
//       hposition4.B_x=x_end; hposition4.B_y=y_end;
//       theta+=fqu*pi;
//       if(theta>=pi){theta=0.0f; j_state=FILP_WALK_STATE;}
//       break;
//    }
//    case FILP_WALK_STATE:
//    {
//        //第一次翻转后，狗的方向变了，要注意
//        float x_end=stride/2.0f;
//        float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
//        static float theta=0.0f;
//        hposition1.B_x=x_end*(1+cosf(theta))/2.0f;
//        hposition1.B_y=y_end+(walk_height-y_end)*(1-cosf(theta))/2.0f;
//        hposition4.B_x=x_end*(1+cosf(theta))/2.0f;
//        hposition4.B_y=y_end+(walk_height-y_end)*(1-cosf(theta))/2.0f;
//        theta+=fqu*pi;
//        if(theta>=pi){theta=0.0f; j_state=FILP_IDLE;}
//        break;
//    }
//   }
//    inverseKinematic_All();
//    Motor_SendCmd_AllAngle();   
//}

//void inverse_x(void)
//{
//    hposition1.B_x = -hposition1.B_x;
//    hposition2.B_x = -hposition2.B_x;
//    hposition3.B_x = -hposition3.B_x;
//    hposition4.B_x = -hposition4.B_x;
//}

 // ----------------------by LiShuai ,END