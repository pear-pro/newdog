#include "imu.h"

/*
使用到三轴参数的地方：
1.main.c里面的翻身flip_body()函数的条件
2.gait.c的行走代码需要用到body_roll来保持身体左右平衡
*/

// 三轴角度，单位 度°
float body_roll = 0.0f; // 横滚角，X轴，身体向左右倾斜
float body_pitch = 0.0f; // 俯仰角，Y轴，身体向前点头或向后仰头
float body_yaw = 0.0f; // 偏航角，Z轴，身体在水平面内旋转

float stab_roll = 0.0f; // 对 body_roll 积分得到地面倾角过程写在主循环里
float kp_roll = 0.01f; 

