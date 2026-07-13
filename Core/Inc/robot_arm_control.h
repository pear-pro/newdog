#ifndef ROBOT_ARM_CONTROL_H
 #define ROBOT_ARM_CONTROL_H
 
 #include "main.h"
 #include "motor_4310.h"
 #include "tim.h"
 #include <math.h>
 
 #define PI 3.1415926535f
 
 #define L1 14.83f//??
 #define L2 19.799f//大臂（短的）
 #define L3 42.1f//小臂（长的）
// #define offset2 -20.0f//?????
// #define offset3 (-90.0f / 2)//?????
 #define offset2 0.0f
 #define offset3 0.0f
 #define offset1  0.0f

 /**
  * 达妙4310电机反馈角度读取宏（Watch窗口使用）
  * ------------------------------------------------
  * damiao[0] = 小臂电机（有2:1减速比）
  * damiao[1] = 大臂电机
  *
  * Rxmsg.Angle 单位：弧度（电机轴角度）
  * 转换为关节角度（度）：
  *   小臂 = Rxmsg.Angle * 57.2958f / 2.0f  （减速比2:1，需除以2）
  *   大臂 = Rxmsg.Angle * 57.2958f
  *
  * 示例（Watch窗口）：
  *   damiao[0].Rxmsg.Angle * 57.2958f / 2.0f   → 小臂关节角度（度）
  *   damiao[1].Rxmsg.Angle * 57.2958f           → 大臂关节角度（度）
  *   damiao[0].Rxmsg.Speed                      → 小臂速度（rad/s）
  *   damiao[1].Rxmsg.Torque                     → 大臂扭矩（Nm）
  */
 
 extern motor_info_t damiao[4];
 extern TIM_HandleTypeDef htim8;
 extern float currentTheta1;

void Set_DM_Motor(uint8_t id, float angle_deg);
//uint8_t Arm_Move_To_Start(float x, float y, float z);
//uint8_t Arm_Move_To(float x, float y, float z,float *theta1, float *theta2,  float *theta3, float *servo_angle);
void  Arm_Move_Smooth(float targetX, float targetY, float targetZ, uint16_t steps);
void Set_Camera_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Set_Sucker_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Arm_Base_Move(float targetX, float targetY, uint16_t steps);
void Set_Servo_Angle_TIM8(uint32_t Channel, float angle);
void Arm_Forward_Kinematics(float *x, float *y, float *z);

/**
 * 正运动学函数
 * -----------------------------------------------
 * 连杆：L2=19.8cm 第一连杆（肩→肘），L3=42.1cm 第二连杆（肘→腕）
 *
 * 输入：
 *   big_arm_angle_deg   - 大臂关节角度（度），damiao[1].Rxmsg.Angle * 57.2958f
 *   small_arm_angle_deg - 小臂关节角度（度），damiao[0].Rxmsg.Angle * 57.2958f / 2.0f
 *
 * 输出：
 *   *R_out - 水平距离（cm），即 sqrt(x²+y²)
 *   *z_out - 高度（cm）
 *
 * 公式：
 *   R = L2·sin(θ₁) - L3·sin(θ₁ - θ₂)
 *   Z = L1 + 12.8 + L2·cos(θ₁) - L3·cos(θ₁ - θ₂)
 */
void Arm_Forward_Kinematics_New(float big_arm_angle_deg, float small_arm_angle_deg,
                                 float *R_out, float *z_out);

 #endif
 