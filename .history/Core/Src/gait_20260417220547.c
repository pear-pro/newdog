/*gait.c
 * 
 * 说明: 1.注意区分状态机函数和阻塞式函数
 * 
 */
#include "gait.h"
#include "math.h"
#include "global_var.h"
#include "kinematic.h"
#include "imu.h"
#include "tim.h"
#include "motor.h"

#define pi 3.141592f

float Forward_freq = 0.0001f; // 0.004 
#define up_down_freq 0.004f

#define jump_freq0 0.08f   // 0.01
#define jump_freq1 0.01f   // 0.01
#define jump_freq2 0.333f   // 0.40 跳 //0.4999
#define jump_freq3 0.039f   // 0.07  0.037
#define jump_freq4 0.005f   // 0.01  0.005
#define jump_freq5 0.003f   // 0.01

float walk_height = 22.0f; 
float max_stride = 20.0f;

float tau = 0.0f;
float t = 0.0f;    

float support_Kp = 0.70f; // 0.6/0.7,0.25/0.3
float swing_Kp = 0.3f;
float Expect_kw = 0.01f;  // 直接用来初始化
float support_tau_ff = 0.00f; // 0.10
float swing_tau_ff = 0.0f;

float kp_GyroZ = 0.08f;  // 40~50左右输入，20左右输出
float kp_VeloY = 10.0f; // 2.0输入，20.0左右输出
float GyroZ_max = 130.0f; // 转动角速度指令上限
float stride_max = 20.0f;

// 每个电机的发力表现不同
 float motor1_kp_offset = 0.25f;
 float motor2_kp_offset = 0.15f;
 float motor3_kp_offset = 0.3f;
 float motor4_kp_offset = 0.2f;
 float motor5_kp_offset = 0.17f;
 float motor6_kp_offset = 0.32f;
 float motor7_kp_offset = 0.37f;
 float motor8_kp_offset = 0.50f;

//float motor1_kp_offset = 0.15f;
//float motor2_kp_offset = 0.0f;
//float motor3_kp_offset = 0.1f;
//float motor4_kp_offset = 0.1f;
//float motor5_kp_offset = 0.15f;
//float motor6_kp_offset = 0.0f;
//float motor7_kp_offset = 0.0f;
//float motor8_kp_offset = 0.0f;

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
	
	if (stab_roll > 20.0f) { stab_roll = 20.0f;}
	if (stab_roll < -20.0f) { stab_roll = -20.0f;}
	
	prev_body_roll = body_roll;
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


void motion_Mix(float height, float step_height, float stride)
{
	static float _t = 0.0f;
	_t += Forward_freq;  
	if(_t >= 1.0f) { _t -= 1.0f; }

    if (rcData.sw8 == 0x0320){
        if (_t <= 0.5f) {
            tau = (cosf(pi * _t * 2.0f - pi) + 1.0f) / 4.0f;        // 0 → 0.5
        } else {
            tau = (cosf(pi * (_t - 0.5f) * 2.0f - pi) + 1.0f) / 4.0f + 0.5f;  // 0.5 → 1.0
        }		
	}else{
        tau += Forward_freq;  
        if(tau >= 1.0f) { tau -= 1.0f; }
    }

//   // 带陀螺仪的前进闭环控制
//    float R_stride = stride;
//    float L_stride = stride;  
//    float Exp_GZ = GyroZ_max*rcData.R_x;  
//    float yaw_correction = kp_GyroZ * (Exp_GZ - GyroZ);
//    float shift_suppression = kp_VeloY*(0-VeloY);
//    if (yaw_correction> stride_max) {yaw_correction = stride_max;}

//   if (rcData.R_y == 0){
//       if (rcData.R_x == 0){ VeloY = 0.0f;} // 清除零漂
//       if (rcData.R_x <-0.05f){ // 左转
//           R_stride = 0 + yaw_correction + shift_suppression;
//           L_stride = 0 - yaw_correction - shift_suppression;
//       }
//       if (rcData.R_x > 0.05f){ // 右转
//           R_stride = 0 + yaw_correction - shift_suppression;
//           L_stride = 0 - yaw_correction + shift_suppression;
//       }    
//       	if (R_stride> stride_max) {R_stride= stride_max; }
// 		if (R_stride<-stride_max) {R_stride=-stride_max; }
//       	if (L_stride> stride_max) {L_stride= stride_max; }
// 		if (L_stride<-stride_max) {L_stride=-stride_max; }
//   }else{
//       if (rcData.R_x < 0){ // 左转
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
	
	if (rcData.sw5==0x0320&&rcData.sw7==0xFCE0){
		float yaw_correction = kp_GyroZ * (0 - GyroZ);
		if (yaw_correction> stride_max) {yaw_correction = stride_max;}
		
		if (rcData.R_x > 0.005f){
			R_stride = stride * (1.0f - 2.0f * rcData.R_x);
			L_stride = stride;
		}else if (rcData.R_x < -0.005f){
			R_stride = stride;
			L_stride = stride * (1.0f + 2.0f * rcData.R_x);
		}else{ // GyroZ逆时针为正
			R_stride = stride + yaw_correction;
			L_stride = stride - yaw_correction;
		}	
	}else {
		if (rcData.R_x > 0.005f){
			R_stride = stride * (1.0f - 2.0f * rcData.R_x);
			L_stride = stride;
		}else if (rcData.R_x < -0.005f){
			R_stride = stride;
			L_stride = stride * (1.0f + 2.0f * rcData.R_x);
		}
	}

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
    else if (tau > 0.5f && tau <= 1.0f)
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
        set_Motor_Kp(1,0,1,0);
    }

    inverseKinematic_All();
    Motor_SendCmd_AllAngle();  
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


// 往前跳, 15.5f~37.0f ？
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
	float y_start2 = k2 * x_start2;
	float x_stop2 = sqrt(height_des*height_des/(1+k2*k2));
	// float y_des = k2 * x_stop2;

	quick_set_kp(14.4f,0.0f); // 跳跃时增大Kp，提升响应速度
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
	}
	HAL_Delay(90); // 等完全蹬直
	
	// state 3收腿前送
	quick_set_kp(2.0f,Expect_kw); 
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



	
