#include "gait.h"
#include "math.h"
#include "global_var.h"
#include "kinematic.h"

#define pi 3.141592f

float tau = 0.0f;
float t = 0;    
float support_Kp = 0.9f;
float swing_Kp = 0.2f;

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
            hmotor1.Kp = support_Kp;
            hmotor2.Kp = support_Kp;
        }
    else                      // 摆动相
        {
            hmotor1.Kp = swing_Kp;
            hmotor2.Kp = swing_Kp;
        }

    if (hposition2_state == 1) // 支撑相
        {
            hmotor3.Kp = support_Kp;
            hmotor4.Kp = support_Kp;
        }
    else                      // 摆动相
        {
            hmotor3.Kp = swing_Kp;
            hmotor4.Kp = swing_Kp;
        }

    if (hposition3_state == 1) // 支撑相
        {
            hmotor5.Kp = support_Kp;
            hmotor6.Kp = support_Kp;
        }
    else                      // 摆动相
        {
            hmotor5.Kp = swing_Kp;
            hmotor6.Kp = swing_Kp;
        }

    if (hposition4_state == 1) // 支撑相
        {
            hmotor7.Kp = support_Kp;
            hmotor8.Kp = support_Kp;
        }
    else                      // 摆动相
        {
            hmotor7.Kp = swing_Kp;
            hmotor8.Kp = swing_Kp;
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
    float freq = 0.1f; // 步频
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

void motion_Backward(float height, float step_height, float stride)
{
    float freq = 0.1f; // 步频
    tau = tau - freq; // 后退
    if(tau < 0.0f){tau = 1.0f;}

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
void motion_TurnRight(float height, float step_height, float stride)
{
    float freq = 0.02f; // 步频
    tau = tau + freq; // 前进
    if(tau >= 1.0f){tau = 0.0f;}

    if (tau <= 0.5f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(0, height, step_height, stride);

        hposition1.B_y  = phaseState.ySwing;
        hposition1.B_x  = phaseState.xSwing; 
        hposition2.B_y  = phaseState.ySupport;
        hposition2.B_x  = phaseState.xSupport; 
        hposition3.B_y  = phaseState.ySwing;
        hposition3.B_x  = -phaseState.xSwing; 
        hposition4.B_y  = phaseState.ySupport;
        hposition4.B_x  = -phaseState.xSupport; 
        set_Motor_Kp(0,1,0,1);
    }
    else if (tau > 0.5f && tau <= 1.0f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(1, height, step_height, stride);

        hposition1.B_y  = phaseState.ySupport;
        hposition1.B_x  = phaseState.xSupport; 
        hposition2.B_y  = phaseState.ySwing;
        hposition2.B_x  = phaseState.xSwing;
        hposition3.B_y  = phaseState.ySupport;
        hposition3.B_x  = -phaseState.xSupport;
        hposition4.B_y  = phaseState.ySwing;
        hposition4.B_x  = -phaseState.xSwing;
        set_Motor_Kp(1,0,1,0);
    }
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();   
}

void motion_TurnLeft(float height, float step_height, float stride)
{
float freq = 0.02f; // 步频
    tau = tau + freq; // 前进
    if(tau >= 1.0f){tau = 0.0f;}

    if (tau <= 0.5f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(0, height, step_height, stride);

        hposition1.B_y  = phaseState.ySwing;
        hposition1.B_x  = -phaseState.xSwing; 
        hposition2.B_y  = phaseState.ySupport;
        hposition2.B_x  = -phaseState.xSupport; 
        hposition3.B_y  = phaseState.ySwing;
        hposition3.B_x  = phaseState.xSwing; 
        hposition4.B_y  = phaseState.ySupport;
        hposition4.B_x  = phaseState.xSupport; 
        set_Motor_Kp(0,1,0,1);
    }
    else if (tau > 0.5f && tau <= 1.0f)
    {
        GaitPhasesPoints phaseState = gaitGenerator(1, height, step_height, stride);

        hposition1.B_y  = phaseState.ySupport;
        hposition1.B_x  = -phaseState.xSupport; 
        hposition2.B_y  = phaseState.ySwing;
        hposition2.B_x  = -phaseState.xSwing;
        hposition3.B_y  = phaseState.ySupport;
        hposition3.B_x  = phaseState.xSupport;
        hposition4.B_y  = phaseState.ySwing;
        hposition4.B_x  = phaseState.xSwing;
        set_Motor_Kp(1,0,1,0);
    }
    inverseKinematic_All();
    Motor_SendCmd_AllAngle();    
}

void motion_StandBy(float height)
{
    hposition1.B_y = height;
    hposition2.B_y = height;
    hposition3.B_y = height;
    hposition4.B_y = height;
    hposition1.B_x = 0.0f;
    hposition2.B_x = 0.0f;
    hposition3.B_x = 0.0f;
    hposition4.B_x = 0.0f;
    set_Motor_Kp(0,0,0,0);
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

//往前跳
void motion_Jump()
{


}


// 往上跳，实际上要写成往前跳的动作
//void testJump() // 调用一次跳一次 
//{
//    float faai = 0.9f; // 前90%的时间收脚，后10%时间跳跃
//    float r = 9.0f;
//    float theta = 0.0f;
//    float center_y = 27.0f;
//	tau = 0.0f;

//    while(tau <= 1.0f)
//    {
//        tau += 0.2f;
//        if (tau <= faai){
//            theta =  pi * tau/(faai);
//            hposition1.B_x = 0.0f;
//            hposition1.B_y = r * cosf(theta) + center_y;  // (27-r,27+r)
//            hposition2.B_x = 0.0f;
//            hposition2.B_y = r * cosf(theta) + center_y;
//            hposition3.B_x = 0.0f;
//            hposition3.B_y = r * cosf(theta) + center_y;
//            hposition4.B_x = 0.0f;
//            hposition4.B_y = r * cosf(theta) + center_y; 
//        }else{
//            hposition1.B_x = 0.0f;
//            hposition1.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;     
//            hposition2.B_x = 0.0f;      
//            hposition2.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
//            hposition3.B_x = 0.0f;
//            hposition3.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
//            hposition4.B_x = 0.0f;
//            hposition4.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
//        }
//        inverseKinematic_All();
//        Motor_SendCmd_AllAngle();   
//    }
//}

void testJump(float stride)// 调用一次跳一次 
{
    float faai = 0.9f; // 前90%的时间收脚，后10%时间跳跃
    float r = 9.0f;
    float theta = 0.0f;
    float center_y = 27.0f;
	
	
	tau = 0.0f;

//    while(tau <= 1.0f)
//    {
//        tau += 0.05f;
//        if (tau <= faai){
//            theta =  pi * tau/(faai);
//            hposition1.B_x = 0.0f;
//            hposition1.B_y = r * cosf(theta) + center_y;  // (27-r,27+r)
//        }else{
//           hposition1.B_x = 0.0f;
//           hposition1.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;           
//        }
	while(tau <= 2.0f)
    {
        tau += 0.05f;
        if (tau <= faai){
            theta =  pi * tau/(faai);
            hposition1.B_x = 0.0f;
            hposition1.B_y = r * cosf(theta) + center_y;  // (27-r,27+r)
            hposition2.B_x = 0.0f;
            hposition2.B_y = r * cosf(theta) + center_y;
            hposition3.B_x = 0.0f;
            hposition3.B_y = r * cosf(theta) + center_y;
            hposition4.B_x = 0.0f;
            hposition4.B_y = r * cosf(theta) + center_y; 
        }else{
            hposition1.B_x = 0.0f;
            hposition1.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;     
            hposition2.B_x = 0.0f;      
            hposition2.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
            hposition3.B_x = 0.0f;
            hposition3.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
            hposition4.B_x = 0.0f;
            hposition4.B_y = 2.0f*r * (tau - faai)/(1.0f - faai) + center_y - r;
        }
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
