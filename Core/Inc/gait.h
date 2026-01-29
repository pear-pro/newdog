#ifndef __GAIT__
#define __GAIT__
#include "stm32f4xx_hal.h"

/*某只脚的两个电机转动的角度，alpha和beta*/
typedef struct
{
	float alpha;
	float beta;
	float B_x;  // B点的x坐标
	float B_y;  // B点的y坐标

}Position_HandleTypeDef;
typedef struct {
    float freq;    // 时间步长
    float height;  // 步高（跳跃高度）
    float stride;  // 步长（前进距离）
    float T;       // 步态周期
    float maxHeight;    // 最大高度
} Gait_Config;


extern Gait_Config g_gait_config;


void Jump(Position_HandleTypeDef *hposition);




#endif




