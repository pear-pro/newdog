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
#include "motor.h"
#include "stm32f427xx.h"
#include "stm32f4xx_hal_rcc.h"
#include <stdint.h>

#define pi 3.141592f

#define Forward_freq 0.04f 
#define up_down_freq 0.04f
#define jump_freq1 0.01f
#define jump_freq2 0.4f
#define jump_freq3 0.07f
#define jump_freq4 0.01f

#define SET_BODY_ROLL 0.0f //后续调整
#define SET_BODY_PITCH 10.0f //狗直立时的pitch
#define FILP_BACK_Y 10.0f //后续调整
#define LANDING_Y 12.0f //后续调整
#define LIMIT_PITCH 0.2f
#define TRACK 30.0f //左右轴距，手测，有误差
#define WHEELBASE 50.0f //前后轴距
volatile static uint8_t jump=0.0f;

float walk_height = 20.0f; 

float tau = 0.0f;
float t = 0;    
float support_Kp = 0.4f; // 0.6,0.25
float swing_Kp = 0.4f;
float support_tau_ff = 0.0f;
float swing_tau_ff = 0.0f;

// 每个电机的发力表现不同
float motor1_kp_offset = 0.2f;
float motor2_kp_offset = 0.0f;
float motor3_kp_offset = 0.0f;
float motor4_kp_offset = 0.2f;
float motor5_kp_offset = 0.1f;
float motor6_kp_offset = 0.25f;
float motor7_kp_offset = 0.1f;
float motor8_kp_offset = 0.35f;

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

void quick_set_kp(float kp_value)
{
    hmotor1.Kp = kp_value* (1.0f + motor1_kp_offset);
    hmotor2.Kp = kp_value * (1.0f + motor2_kp_offset);
    hmotor3.Kp = kp_value* (1.0f + motor3_kp_offset);
    hmotor4.Kp = kp_value * (1.0f + motor4_kp_offset);
    hmotor5.Kp = kp_value* (1.0f + motor5_kp_offset);
    hmotor6.Kp = kp_value * (1.0f + motor6_kp_offset);
    hmotor7.Kp = kp_value* (1.0f + motor7_kp_offset);
    hmotor8.Kp = kp_value * (1.0f + motor8_kp_offset);
}

void  Body_Roll_Stabilizer(){
	stab_roll += kp_roll*(0 - body_roll) - kd_roll * (body_roll - prev_body_roll);
	
	if (stab_roll > 40.0f) { stab_roll = 40.0f;}
	if (stab_roll < -40.0f) { stab_roll = -40.0f;}
	
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
    tau = tau + Forward_freq; // 前进
    if(tau >= 1.0f){tau = 0.0f;}

    // 带陀螺仪的前进闭环控制
    //     if (rc_x > 0){
    //     R_stride = stride + kp_GyroZ * (GyroZ_max*rc_x/rc_x_max - GyroZ);
    // }else{
    //     L_stride = stride - kp_GyroZ * (GyroZ_max*rc_x/rc_x_max - GyroZ);
    // }

    


	// 左右转控制
    float R_stride = stride; // 右脚步幅
    float L_stride = stride; // 左脚步幅
    if (rc_x > 0){
        R_stride = stride * 20.0f*(rc_x/rc_x_max);
    }else{
        L_stride = stride * 20.0f*(rc_x/rc_x_max);
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
    float height_des = 33.0f; // 蹬地腿长
    float step_height = height_des - 13.0f; // 抬腿高度

    float r = fabsf(15.5f - walk_height) * 0.5f;
    float center_y = (15.5f + walk_height) / 2.0f;
    float theta = 0.0f;

    for (float i = 0.0f; i <= 1.0f; i += jump_freq1){
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
	
    float k = 2*height_des/stride; // 斜率
    float x_start = sqrt(15.5f*15.5f/(1+k*k));
	float y_start = k * x_start;
    float x_des = sqrt(height_des*height_des/(1+k*k));
    float y_des = k * x_des;

    quick_set_kp(1.3f); // 跳跃时增大Kp，提升响应速度
    for (float x = x_start;x < x_des; x += jump_freq2*(fabsf(x_start - x_des))){
        float y = k * x;
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
	
    for (float angle = 0.0f;angle <=pi; angle += jump_freq3 * pi){

        float x = stride * ((angle - sin(angle)) / (2 * pi)) - stride / 2;
        float y = height_des - step_height * (1 - cos(angle)) / 2;
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
	

    for (float angle = pi;angle <=2*pi; angle += jump_freq2 * pi){

        float x = stride * ((angle - sin(angle)) / (2 * pi)) - stride / 2;
        float y = walk_height - (walk_height - 15.5f) * (1 - cos(angle)) / 2;

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
	
    for (float x = stride / 2.0f; x > 0.0f; x -= jump_freq4 * stride){
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
	quick_set_kp(support_Kp); // 落地时减小Kp，增加缓冲，防止过度震荡    


}



void testCircle()
{   
    if(tau >= 1.0f){tau = 0.0f;}

    float r = 8.0f;
    tau += 0.01f;
    float theta = 2.0f * pi * tau/1.0f;
    hposition1.B_x = r * cosf(theta);
    hposition1.B_y = r * sinf(theta) + 27.0f;
    hposition2.B_x = -r * cosf(theta);
    hposition2.B_y = -r * sinf(theta) + 27.0f;
    hposition3.B_x = r * cosf(theta);
    hposition3.B_y = r * sinf(theta) + 27.0f;
    hposition4.B_x = -r * cosf(theta);
    hposition4.B_y = -r * sinf(theta) + 27.0f;
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
}

/*
前后两腿依次按照椭圆轨迹向前移动stride
前后腿向后移动stride，身体向前送
后腿依次按照椭圆轨迹向前移动stride
*/
void test_low_walk(float stride,float b) //b是椭圆的参数
{
    float walk_H=15.5f;
    float low_H=5.5f;
    float theta=0.0f;
    //蹲下去
  for(theat=0.0f;theat<pi;theat+=up_down_freq*pi)
  {
    float r=(walk_H-low_H)/2;
    float center_y=(walk_H+low_H)/2;
    hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
    hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
    hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
    hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
  float s=stride;
  float r=stride/2.0f;
  float p=r*b/sqrt(b*b*(cosf(theat)*cosf(theat))+r*r*(sinf(theta)*sinf(theta))); //椭圆的极径,中心在(r,0)处 r=a
  //右前腿按照椭圆轨迹移动至s
  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
  {
   // float x=r*(1+cosf(theat));
   // float y=r*sinf(theat);
    float x=r+p*cosf(theta);
    float y=p*sinf(theat);
    hposition1.B_x=x; hposition1.B_y=low_H-y;
    hposition4.B_x=0.0f; hposition4.B_y=low_H;
    hposition2.B_x=0.0f; hposition2.B_y=low_H;
    hposition3.B_x=0.0f; hposition3.B_y=low_H;
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
  //左前腿按照椭圆轨迹移动至s
  for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
  {
    float x=r+p*cosf(theta);
    float y=p*sinf(theat);
    hposition1.B_x=s; hposition1.B_y=low_H;
    hposition4.B_x=x; hposition4.B_y=low_H-y;
    hposition2.B_x=0.0f; hposition2.B_y=low_H;
    hposition3.B_x=0.0f; hposition3.B_y=low_H;
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
  //1,4回收 2,3后送
  for(theat=0.0f;theat<pi;theta+=up_down_freq*pi)
  {
    float x=s*(cosf(1+theat)/2.0f);

    hposition1.B_x=x; hposition1.B_y=low_H;
    hposition4.B_x=x; hposition4.B_y=low_H;
    hposition2.B_x=-s*(cosf(1-theat)/2.0f); hposition2.B_y=low_H;
    hposition3.B_x=-s*(cosf(1-theat)/2.0f); hposition3.B_y=low_H;
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
  //右后腿按照椭圆轨迹移动至0.0f
   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
  {
    float x=r+p*cosf(theta);
    hposition1.B_x=0.0f; hposition1.B_y=low_H;
    hposition4.B_x=0.0f; hposition4.B_y=low_H;
    hposition2.B_x=-s+x; hposition2.B_y=low_H;
    hposition3.B_x=-s; hposition3.B_y=low_H;
     inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
  //右后腿按照椭圆轨迹移动至0.0f || 最终回到状态1
   for(theat=pi;theat>0.0f;theta-=up_down_freq*pi)
  {
    float x=r+p*cosf(theta);
    hposition1.B_x=0.0f; hposition1.B_y=low_H;
    hposition4.B_x=0.0f; hposition4.B_y=low_H;
    hposition2.B_x=0.0f; hposition2.B_y=low_H;
    hposition3.B_x=-s+x; hposition3.B_y=low_H;
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
  }
}

// void test_jump_filp(float stride)
// {
//   float walk_hight=0.0f;
//   float height_des=1.0f;
//   float long_hight=20.0f;
//   float angle=0.0f;
//   uint8_t pitch_triggered=0;
//   //ready
//   for(float theta=0.0f;theta<pi;theta+=0.05f)
//   {
//     float r=(long_hight-walk_hight)/2;
//     float center_y=(long_hight+walk_hight)/2;
//     hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
//     hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
//     hposition2.B_x=0.0f; hposition2.B_y=center_y+r*cos(theta);
//     hposition3.B_x=0.0f; hposition3.B_y=center_y+r*cos(theta);
//     inverseKinematic_All();
//     Motor_SendCmd_AllAngle();   
//   }
//   //前脚起跳
//     float k=2*long_hight/stride;
//     float x_start = sqrt(walk_hight*walk_hight/(1+k*k));
// 	float y_start = k * x_start;
//     float x_des = sqrt(height_des*height_des/(1+k*k));
//     float y_des = k * x_des;
//     quick_set_kp(1.5f); // 跳跃时增大Kp，提升响应速度
//   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
//   {
//     float y=k*x;
//     hposition1.B_x=x; hposition1.B_y=y;
//     hposition4.B_x=x; hposition4.B_y=y;
//     hposition2.B_x=0.0f; hposition2.B_y=walk_hight;
//     hposition3.B_x=0.0f; hposition3.B_y=walk_hight;
//     inverseKinematic_All();
//     Motor_SendCmd_AllAngle();   
//      if(body_pitch<SET_BODY_ANGLE){
//         pitch_triggered=1;
//         break;
//      }
//   }
//     //后脚起跳
//   for(float x=x_start;x<x_des;x+=jump_freq4*fabsf(x_start-x_des))
//     {
//         float y=k*x;
//         if(pitch_triggered){quick_set_kp(2.0f);}
//         else{quick_set_kp(1.0f);}
//         hposition1.B_x=x_des; hposition1.B_y=k*x_des;
//         hposition4.B_x=x_des; hposition4.B_y=k*x_des;
//         hposition2.B_x=x; hposition2.B_y=y;
//         hposition3.B_x=x; hposition3.B_y=y;
//         inverseKinematic_All();
//         Motor_SendCmd_AllAngle();   
//     }
//     //空中前半段，收腿
//    for(angle=0;angle<pi;angle+=jump_freq3*pi)
//    {
//       float x=x_des*(1.0f+cos(angle))/2.0f;
//       float y = k*x_des+(FILP_BACK_Y-k*x_des)*(1.0f-cosf(angle))/2.0f;
//       hposition1.B_x=x; hposition1.B_y=y;
//       hposition4.B_x=x; hposition4.B_y=y;
//       hposition2.B_x=x; hposition2.B_y=y;
//       hposition3.B_x=x; hposition3.B_y=y;
//       inverseKinematic_All();
//       Motor_SendCmd_AllAngle();
//    }
//    //空中后半段，展腿，准备落地支撑
//    quick_set_kp(1.3f);
//    for(angle=pi;angle<2*pi;angle+=jump_freq3*pi)
//    {
//      float x=stride/4.0f*(1.0f+cos(angle))/2.0f;
//      float y=LANDING_Y+(FILP_BACK_Y-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
//      hposition1.B_x=x; hposition1.B_y=y;
//      hposition4.B_x=x; hposition4.B_y=y;
//      hposition2.B_x=-x; hposition2.B_y=y;
//      hposition3.B_x=-x; hposition3.B_y=y;
//      inverseKinematic_All();
//      Motor_SendCmd_AllAngle();
//    }
//    //落地缓冲
//     quick_set_kp(0.35f);
//     for(angle=2*pi;angle<3*pi;angle+=jump_freq3*pi)
//     {
//         float x=stride/4.0f;
//         float y=LANDING_Y+(walk_hight-LANDING_Y)*(1.0f-cosf(angle))/2.0f;
//         hposition1.B_x=x; hposition1.B_y=y;
//         hposition4.B_x=x; hposition4.B_y=y;
//         hposition2.B_x=-x; hposition2.B_y=y;
//         hposition3.B_x=-x; hposition3.B_y=y;
//         inverseKinematic_All();
//         Motor_SendCmd_AllAngle();

//     }
//     //落地后站立
//     quick_set_kp(support_Kp);
//     for(angle=3*pi;angle<4*pi;angle+=jump_freq3*pi)
//     {
//         float x=stride/4.0f+(-stride/4.0f)*(1.0f+cos(angle))/2.0f;
//         float y=walk_hight;
//         hposition1.B_x=x; hposition1.B_y=y;
//         hposition4.B_x=x; hposition4.B_y=y;
//         hposition2.B_x=-x; hposition2.B_y=y;
//         hposition3.B_x=-x; hposition3.B_y=y;
//         inverseKinematic_All();
//         Motor_SendCmd_AllAngle();
//     }
// }
	
void state_filp_jump(float stride)
{
   static filp_jump_state_t j_state=FILP_IDLE;
   static float ready_height=12.8f;
   static float walk_height=20.0f;
   static float height_des=33.0f;
   static float back_x=0.0f; 
   static float back_y=0.0f;
   float fqu=0.02f;
   switch(j_state)
   {
    case FILP_IDLE:
    {
        quick_set_kp(support_Kp);
        hposition1.B_x = 0.0f; hposition1.B_y = walk_height;
        hposition4.B_x = 0.0f; hposition4.B_y = walk_height;
        hposition2.B_x = 0.0f; hposition2.B_y = walk_height;
        hposition3.B_x = 0.0f; hposition3.B_y = walk_height;
        j_state=FILP_READY;
        break;
    }
    case FILP_READY:
    {
        //前后腿高度到ready_height,后退往前送stride/2
        float r=(walk_height-ready_height)/2.0f;
        float center_y=(ready_height+walk_height)/2.0f;
        static float theta=0.0f;
        hposition1.B_x=0.0f; hposition1.B_y=center_y+r*cos(theta);
        hposition4.B_x=0.0f; hposition4.B_y=center_y+r*cos(theta);
        hposition2.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition2.B_y=center_y+r*cos(theta);
        hposition3.B_x=stride/2.0f*(cosf(1-theta)/2.0f); hposition3.B_y=center_y+r*cos(theta);
        theta+=fqu*pi;
        if(theta>=pi){theta=0.0f; j_state=FILP_BACK_JUMP;}
        break;
    }
    case FILP_BACK_JUMP:
    {
         //后脚起跳
        float s=stride/2.0f; 
        float back_ready_long=sqrtf(s*s+ready_height*ready_height);
        float k=2*height_des/s;
        float x_start = sqrt(back_ready_long*back_ready_long/(1+k*k)); //sqrt(ready_height*ready_height/(1+k*k));
        float y_start = k * x_start;
        float x_des = sqrt(height_des*height_des/(1+k*k));
        float y_des = k * x_des-;
        static float x=0.0f;
        static uint8_t inited = 0;
        if(!inited){x=x_start; inited=1; quick_set_kp(1.5f);}
        float y=k*x;
        hposition2.B_x=-x; hposition2.B_y=y;
        hposition3.B_x=-x; hposition3.B_y=y;
        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
        hposition4.B_x=0.0f; hposition4.B_y=ready_height;    
        if(body_pitch<=SET_BODY_PITCH) //腿可能还没到位置，但重心已经能满足翻滚条件了
        {
            inited=0;
            back_x=x; back_y=y;  //记录此时真实的坐标，用于后续计算,取正值方便算
            j_state=FILP_READY_FONT_JUMP;
        }
        else if(x<=x_des)
        {
            inited=0;
            x=x_start;
            j_state=FILP_JUMP_RECOVER;
        }else
        {
            x+=jump_freq4*fabsf(x_start-x_des);
        }
        break;
    }
    case FILP_JUMP_RECOVER:
    {
       static float s=0.0f;
       float y=back_y+(ready_height-back_y)*(1-cosf(s))/2.0f;
       float x=back_x*(1+cosf(s))/2.0f;
        hposition2.B_x=-x; hposition2.B_y=y;
        hposition3.B_x=-x; hposition3.B_y=y;
        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
        hposition4.B_x=0.0f; hposition4.B_y=ready_height; 
        s+=fqu;
        if(s>=pi)
        {
            s=0.0f;
            if(body_pitch<=LIMIT_PITCH){j_state=FILP_WALK_STATE;}
        }
        break;
    }
    case FILP_READY_FONT_JUMP:
    {
        //后退转至相对身体水平位置，前腿先直接向前移(jump=0,只做翻转 || jump=1,跳加翻转)
       float inside_r=ready_height;
       float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
       float theta_begin=atan2f(ready_height,fabsf(back_x));
       float theta_end=0.0f;
       static float theta=0.0f;
       static uint8_t inited=0;
       if(!inited){ inited=1; theta=theta_begin;} //只再第一次进入时赋值
       float x=outside_r*cosf(theta);
       float y=outside_r*sinf(theta); //相当于back_y的值了
       if(jump==0)
       {
        hposition1.B_x=0.0f; hposition1.B_y=ready_height;
        hposition4.B_x=0.0f; hposition4.B_y=ready_height;
        hposition2.B_x=-x;   hposition2.B_y=y;
        hposition3.B_x=-x;   hposition3.B_y=y;
        theta-=fqu*pi;
        if(theta<=theta_end){theta=theta_begin; inited=0; j_state=FILP_ONLY;}  
       }
       break;
    }
    case FILP_ONLY:
    {
        float x_begin=0.0f;
        float x_end=stride/2.0f;
        float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
        float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
        static float theta=0.0f;
        hposition1.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
        hposition1.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
        hposition4.B_x=x_begin+(x_end-x_begin)*(1-cosf(theta))/2.0f; 
        hposition4.B_y=ready_height+(y_end-ready_height)*(1-cosf(theta))/2.0f;
        hposition2.B_x=-outside_r;   hposition2.B_y=0.0f;
        hposition3.B_x=-outside_r;   hposition3.B_y=0.0f;
        theta+=fqu*pi;
        if(theta>=pi){theta=0.0f; j_state=FILP_LANDING;}
        break;
    }
    case FILP_HALFS_SKY:
    {

    }
    case FILP_HALFE_SKY:
    {
     
    }
    case FILP_LANDING:
    {
       float inside_r=ready_height;
       float outside_r=sqrt(ready_height*ready_height+back_x*back_x);
       float x_end=stride/2.0f;
       float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
       static float theta=0.0f;
       hposition2.B_x=outside_r+(x_end-outside_r)*(1-cosf(theta))/2.0f;
       hposition2.B_y=y_end*cosf(theta);
       hposition2.B_x=outside_r+(x_end-outside_r)*(1-cosf(theta))/2.0f;
       hposition2.B_y=y_end*cosf(theta);
       hposition1.B_x=x_end; hposition1.B_y=y_end;
       hposition4.B_x=x_end; hposition4.B_y=y_end;
       theta+=fqu*pi;
       if(theta>=pi){theta=0.0f; j_state=FILP_WALK_STATE;}
       break;
    }
    case FILP_WALK_STATE:
    {
        //第一次翻转后，狗的方向变了，要注意
        float x_end=stride/2.0f;
        float y_end=sqrtf(x_end*x_end+ready_height*ready_height)-ready_height;
        static float theta=0.0f;
        hposition1.B_x=x_end*(1+cosf(theta))/2.0f;
        hposition1.B_y=y_end+(walk_height-y_end)*(1-cosf(theta))/2.0f;
        hposition4.B_x=x_end*(1+cosf(theta))/2.0f;
        hposition4.B_y=y_end+(walk_height-y_end)*(1-cosf(theta))/2.0f;
        theta+=fqu*pi;
        if(theta>=pi){theta=0.0f; j_state=FILP_IDLE;}
        break;
    }
   }
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
}

void inverse_x(void)
{
    hposition1.B_x = -hposition1.B_x;
    hposition2.B_x = -hposition2.B_x;
    hposition3.B_x = -hposition3.B_x;
    hposition4.B_x = -hposition4.B_x;
}