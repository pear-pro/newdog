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

float turn_omega_corr = 0.0f; // 旋转纠正值
float deta_angle=0.0f;
float turn_stride=0.0f;

float Forward_freq = 0.0075f; // 0.004 
#define up_down_freq 0.004f
#define crawl_freq 0.002f

float support_Kp = 0.7f; // 0.6/0.7,0.25/0.3
float swing_Kp = 0.3f;
float Expect_kw = 0.01f;  // 直接用来初始化
float support_tau_ff = 0.00f; // 0.10
float swing_tau_ff = 0.0f;

float Frontflip_freq1 = 0.15f; // 
float Frontflip_freq2 = 0.004f; // 
float Frontflip_freq3 = 0.004f; // 

#define jump_freq0 0.08f   // 0.01
#define jump_freq1 0.01f   // 0.01
#define jump_freq2 0.999f   // 0.40 跳 //0.4999 // 0.2499
#define jump_freq3 0.039f   // 0.07  0.0372
#define jump_freq4 0.005f   // 0.01  0.005
#define jump_freq5 0.003f   // 0.01

float walk_height = 22.0f; //32.0
float max_stride = 11.0f;
float max_stride2 = 9.0f;
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

float motor1_kp_offset = 0.5f;
float motor2_kp_offset = 0.0f;
float motor3_kp_offset = 0.1f;
float motor4_kp_offset = 0.1f;
float motor5_kp_offset = 0.15f;
float motor6_kp_offset = 0.0f;
float motor7_kp_offset = 0.25f;
float motor8_kp_offset = 0.4f;


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

void motion_Crawl(float step_height, float stride){
    tau += crawl_freq; // 步频
    if(tau >= 1.0f) { tau = 0.0f; }

  motor1_kp_offset = 0.35f;
  motor2_kp_offset = 0.45f;
  motor3_kp_offset = 0.2f;
  motor4_kp_offset = 0.15f;
  motor5_kp_offset = 0.17f;
  motor6_kp_offset = 0.32f;
  motor7_kp_offset = 0.47f;
  motor8_kp_offset = 0.60f;

    float Crawl_height = 13.5f; // y轴方向上电机轴心到地面的距离14.5 - 足底高度4.0
    float front_pivot_x = 13.5f + 0.5f*stride; // 前摆线方程原点=x方向上离电机轴心最近的距离+步幅的一半
	//                    12.5
    if (tau <= 0.5f)
    {
        GaitPhasesPoints FrontRightState = crawl_gaitGenerator(0, Crawl_height, step_height, stride, front_pivot_x);
        GaitPhasesPoints FrontLeftState = crawl_gaitGenerator(0, Crawl_height, step_height, stride, front_pivot_x);
        GaitPhasesPoints BackRightState = crawl_gaitGenerator(0, Crawl_height, step_height, stride+rcData.R_y*3.0f, -front_pivot_x);
        GaitPhasesPoints BackLeftState = crawl_gaitGenerator(0, Crawl_height, step_height, stride+rcData.R_y*3.0f, -front_pivot_x);

        hposition1.B_y  = FrontRightState.ySwing;
        hposition1.B_x  =  FrontRightState.xSwing; 
        hposition2.B_y  = BackRightState.ySupport;
        hposition2.B_x  =  BackRightState.xSupport; 
        hposition3.B_y  = BackLeftState.ySwing ;
        hposition3.B_x  =  BackLeftState.xSwing; 
        hposition4.B_y  = FrontLeftState.ySupport;
        hposition4.B_x  =  FrontLeftState.xSupport; 
        set_Motor_Kp(1,1,1,1);
    }
    else if (tau > 0.5f && tau <= 1.0f)
    {
        GaitPhasesPoints FrontRightState = crawl_gaitGenerator(1, Crawl_height, step_height, stride, front_pivot_x);
        GaitPhasesPoints FrontLeftState = crawl_gaitGenerator(1, Crawl_height, step_height, stride, front_pivot_x);
        GaitPhasesPoints BackRightState = crawl_gaitGenerator(1, Crawl_height, step_height, stride+rcData.R_y*3.0f, -front_pivot_x);
        GaitPhasesPoints BackLeftState = crawl_gaitGenerator(1, Crawl_height, step_height, stride+rcData.R_y*3.0f, -front_pivot_x);

        hposition1.B_y  = FrontRightState.ySupport;
        hposition1.B_x  =  FrontRightState.xSupport; 
        hposition2.B_y  = BackRightState.ySwing;
        hposition2.B_x  =  BackRightState.xSwing;
        hposition3.B_y  = BackLeftState.ySupport;
        hposition3.B_x  =  BackLeftState.xSupport;
        hposition4.B_y  = FrontLeftState.ySwing;
        hposition4.B_x  =  FrontLeftState.xSwing;
        set_Motor_Kp(1,1,1,1);
    }
	
	crawl_inverseKinematic_All();
    Motor_SendCmd_AllAngle();  
	
  motor1_kp_offset = 0.25f;
  motor2_kp_offset = 0.15f;
  motor3_kp_offset = 0.3f;
  motor4_kp_offset = 0.2f;
  motor5_kp_offset = 0.17f;
  motor6_kp_offset = 0.32f;
  motor7_kp_offset = 0.37f;
  motor8_kp_offset = 0.50f;
}

/* 参数说明：
* stride：步幅，参数范围（-max_stride 到 max_stride）
* turn_omega：转弯角速度，参数范围（-1.0f 到 1.0f）
*/

uint8_t Flag=1;
void motion_Mix(float height, float step_height, float stride)
{    
    tau += Forward_freq;  
    if(tau >= 1.0f) { tau -= 1.0f; }
			
	// 开环左右转控制
	float kp_turn_omega = 1.60f;
//	float turn_omega_integral = 0.0f;
//	const float ki_turn = 0.0170f; // 积分系数
//	const float integral_max =6.0f; // 积分限幅防饱和
//	float last_deta_angle = 0.0f; // 上一次角度误差
//	const float kd_turn = 0.35f;  // 微分系数
	
    float R_stride = stride;
    float L_stride = stride;
	
	float turn_curr_max = 30.0f;//需要实际测量
	
	if((fabs(rcData.R_y)<0.35f)&&(fabs(rcData.R_x)<0.35f)&&Flag==0)
	{
		Init_turn_omega_des();
		Flag=1;
//		turn_omega_integral=0;
	}
	
	//解决陀螺仪转过180变号
	deta_angle=turn_omega_des-body_yaw;
	if(deta_angle>180.0f) deta_angle-=360.0f;
	else if(deta_angle<-180.0f) deta_angle+=360.0f;
	
//	turn_omega_integral += ki_turn *deta_angle;
//	// 积分限幅
//	if(turn_omega_integral > integral_max) turn_omega_integral = integral_max;
//	if(turn_omega_integral < -integral_max) turn_omega_integral = -integral_max;

//	// PID输出
//	float d_err = deta_angle - last_deta_angle;
//	turn_omega_corr = kp_turn_omega * deta_angle + turn_omega_integral + kd_turn * d_err;
//	last_deta_angle = deta_angle;
	turn_omega_corr = kp_turn_omega * deta_angle;
	// 从turn_omega_des 映射到 turn_omega_corr
                   if (turn_omega_corr>turn_curr_max){turn_omega_corr=turn_curr_max;}
                   if (turn_omega_corr<-turn_curr_max){turn_omega_corr=-turn_curr_max;}
	turn_omega_corr = turn_omega_corr/turn_curr_max;
				   
	// turn_omega_corr 映射到 turn_stride
	turn_stride = turn_omega_corr;
	if((fabs(rcData.R_y)<0.1f)&&(fabs(rcData.R_x)<0.1f))
	{
		
	if(fabs(deta_angle)>5.0f)
	{
       if (turn_stride >0.01f){ 
           R_stride =  - turn_stride*max_stride2;
           L_stride =  + turn_stride*max_stride2;
       }
	   if (turn_stride <-0.01f){
		
           R_stride =  - turn_stride*max_stride2;
           L_stride =  + turn_stride*max_stride2;
       }
		
     }
	}
	else if ((fabs(rcData.R_y)<0.35f)&&(rcData.R_x>0.35f||rcData.R_x<-0.35f)){
		Flag=0;
       if (turn_stride >0.01f){ 
           R_stride =  - turn_stride*max_stride2;
           L_stride =  + turn_stride*max_stride2;
       }
	   if (turn_stride <-0.01f){
           R_stride =  - turn_stride*max_stride2;
           L_stride =  + turn_stride*max_stride2;
       }
   }
	else if((fabs(rcData.R_y)>0.35f)&&(fabs(rcData.R_x)<0.35f))
	{
		Flag=0;
		if (turn_stride > 0.1f){
			if(rcData.R_y>0.35){
				R_stride = stride;
				L_stride = stride*(1.0f - 1.5f * turn_stride);
			}
			else if(rcData.R_y<-0.35)  
				 
			{
				R_stride = stride*(1.0f - 1.5f * turn_stride);
				L_stride = stride;
			}
		}			
		if (turn_stride < -0.1f){
			if(rcData.R_y>0.35)
			{
				R_stride = stride* (1.0f +1.5f * turn_stride);
				L_stride = stride;
			}
			else if(rcData.R_y<-0.35)
			{
				R_stride = stride;
				L_stride = stride* (1.0f +1.5f * turn_stride);
			}
		}
	}
	else if((fabs(rcData.R_y)>0.35f)&&(fabs(rcData.R_x)>0.18f))
	{
		Flag=0;
		if (turn_stride > 0.01f){
			if(rcData.R_y>0.35)
			{
				R_stride = stride *(1.0f + 1.1f * turn_stride);
				L_stride = stride;
			}
			else if(rcData.R_y<-0.35)
			{
				R_stride = stride;
				L_stride = stride *(1.0f + 1.1f * turn_stride);
			}
		}
		if (turn_stride < -0.01f){
			if(rcData.R_y>0.35)
			{
				R_stride = stride ;
				L_stride = stride * (1.0f - 1.1f * turn_stride);
			}
			else if(rcData.R_y<-0.35)
			{
				R_stride = stride*(1.0f - 1.1f * turn_stride);
				L_stride = stride;
			}
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