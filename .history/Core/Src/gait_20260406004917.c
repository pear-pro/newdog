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
#include "motor.h"

#define pi 3.141592f

#define Forward_freq 0.005f 
#define up_down_freq 0.04f
#define jump_freq1 0.01f
#define jump_freq2 0.01f


float walk_height = 20.0f; 

float tau = 0.0f;
float t = 0;    
float support_Kp = 0.2f; // 0.6,0.25
float swing_Kp = 0.2f;
float support_tau_ff = 0.2f;
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

	// 左右转控制
    float R_stride = stride; // 右脚步幅
    float L_stride = stride; // 左脚步幅
    if (rc_x > 0){
        R_stride = stride * (1.0f - 2.0f * rc_x/rc_x_max);
    }else{
        L_stride = stride * (1.0f + 2.0f * rc_x/rc_x_max);
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
    float height_des = 37.0f; // 蹬地腿长
    float step_height = 20.0f; // 抬腿高度

    // motion_Down(walk_height,15.5f);

    float k = 2*height_des/stride; // 跳跃高度调节系数，越大跳得越高
    float x_start = sqrt(15.5f*15.5f/(1+k*k));
	float y_start = k * x_start;
    float x_des = sqrt(height_des*height_des/(1+k*k));
    float y_des = k * x_des;

    // for (float x = x_start;x < x_des; x += jump_freq1){
    //     float y = k * x;
    //     hposition1.B_y = y;
    //     hposition1.B_x = x; 
    //     hposition2.B_y = y;
    //     hposition2.B_x = x; 
    //     hposition3.B_y = y;
    //     hposition3.B_x = x; 
    //     hposition4.B_y = y;
    //     hposition4.B_x = x; 

    //     inverseKinematic_All();
    //     Motor_SendCmd_AllAngle(); 
    // }
    // for (float x = x_des;x > x_start; x -= jump_freq1){
    //     float y = k * x;
    //     hposition1.B_y = y;
    //     hposition1.B_x = x; 
    //     hposition2.B_y = y;
    //     hposition2.B_x = x; 
    //     hposition3.B_y = y;
    //     hposition3.B_x = x; 
    //     hposition4.B_y = y;
    //     hposition4.B_x = x; 

    //     inverseKinematic_All();
    //     Motor_SendCmd_AllAngle(); 
    // }
	
    for (float angle = 0.0f;angle <=2*pi; angle += jump_freq2){

        float x = stride * ((angle - sin(angle)) / (2 * pi)) - stride / 2;
        float y = height - step_height * (1 - cos(angle)) / 2;

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
    
    for (float angle = 2*pi;angle >=0.0f; angle -= jump_freq2){
        float x = stride * ((angle - sin(angle)) / (2 * pi)) - stride / 2;
        float y = height - step_height * (1 - cos(angle)) / 2;

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
