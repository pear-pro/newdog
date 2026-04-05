#ifndef __GAIT__
#define __GAIT__

#define Ts 1.00f          // 控制周期

/*某只脚的两个电机转动的角度，alpha和beta*/
typedef struct
{
	float alpha;
	float beta;
	float B_x;  // B点的x坐标
	float B_y;  // B点的y坐标

}Position_HandleTypeDef;

typedef struct 
{
  float xSwing;    // 前半周期生成的摆动相的x坐标
  float ySwing;    // 前半周期生成的摆动相的y坐标
  float xSupport;  // 前半周期生成的支撑相的x坐标
  float ySupport;  // 前半周期生成的支撑相的y坐标
  float sigma;     // 轨迹生成三角函数中的相位
}GaitPhasesPoints;

void motion_Forward(float height, float step_height, float stride);
void motion_StandBy(float height);
void StepInPlace(float height, float step_height);
void motion_Jump();
void motion_Mix(float height, float step_height, float stride);
void flip_body();


void testCircle();
void testJump(float stride);

#endif
