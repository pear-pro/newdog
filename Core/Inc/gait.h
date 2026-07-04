#ifndef __GAIT__
#define __GAIT__

#define Ts 1.00f    // 控制周期
#define Ts_crawl 1.00f   
#include "stm32f4xx.h"

/*某只脚的两个电机驱动的角度，alpha和beta*/
typedef struct
{
	float alpha;
	float beta;
	float B_x;  // B点的x坐标
	float B_y;  // B点的y坐标

}Position_HandleTypeDef;

// 全局变量
extern float walk_height;
extern float max_stride;
extern float Forward_freq;

extern float support_Kp; 
extern float swing_Kp;
extern float Expect_kw; 
extern float support_tau_ff; 
extern float swing_tau_ff;
extern float turn_omega_des;
extern float deta_angle;

typedef struct 
{
  float xSwing;    // 前半周期生成的摆动相的x坐标
  float ySwing;    // 前半周期生成的摆动相的y坐标
  float xSupport;  // 前半周期生成的支撑相的x坐标
  float ySupport;  // 前半周期生成的支撑相的y坐标
  float sigma;     // 轨迹生成三角函数中的相位
}GaitPhasesPoints;

typedef enum
{
  FILP_IDLE=0,
  FILP_READY,
  FILP_BACK_JUMP,
  FILP_READY_JUMP,
  FILP_FRONT_JUMP,
  FILP_JUMP_RECOVER,
  FILP_ONLY,
  FILP_HALFS_SKY,
  FILP_HALFE_SKY,
  FILP_LANDING,
  FILP_WALK_STATE
}filp_jump_state_t;

void Init_turn_omega_des(void);
void Body_Roll_Stabilizer(void);
void motion_Forward(float height, float step_height, float stride);
void motion_StandBy(float height);
void StepInPlace(float height, float step_height);
void motion_Jump(float stride);
void motion_SmallJump(void);
void motion_Mix(float height, float step_height, float stride);
void flip_body(void);
void motion_Down(float start,float des);
void motion_Up(float start,float des);
void motion_Crawl(float step_height, float stride);
void motion_Crawl_Reset(void);   // 进入匍匐时调用，触发缓慢下蹲过渡
uint8_t imu_emergency_stop(void);
void motion_Frontflip(void);
void test_circle(void);
 void motion_Crawl1(float step_height, float stride);
void Init_turn_omega_des();


#endif
