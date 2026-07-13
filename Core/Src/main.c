/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <math.h>
#include "motor.h"
#include "gait.h"
#include "crc_ccitt.h"
#include "kinematic.h"
#include "global_var.h"
#include "remote_control.h"
#include "key.h"
#include "pg_led.h"
#include "sucker.h"
#include "imu.h"
#include "pwm_app.h"
#include "ht_10a_remote_control.h"
#include "robot_arm_control.h"
#include "motor_feedback.h"
#include "debug_uart.h"
#include "IMU.h"
#include "filter.h"

#include "usart_demo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t alive_tick = 0; 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 9个电机的结构体
Motor_HandleTypeDef hmotor1;
Motor_HandleTypeDef hmotor2;
Motor_HandleTypeDef hmotor3;
Motor_HandleTypeDef hmotor4;
Motor_HandleTypeDef hmotor5;
Motor_HandleTypeDef hmotor6;
Motor_HandleTypeDef hmotor7;
Motor_HandleTypeDef hmotor8;  // 前8个都是腿部电机
Motor_HandleTypeDef hmotor10; // 云台电机

// 四只脚的位置相关结构体
Position_HandleTypeDef hposition1;
Position_HandleTypeDef hposition2;
Position_HandleTypeDef hposition3;
Position_HandleTypeDef hposition4;

/* hposition与电机对应关系
hposition1 : hmotor1(α) , hmotor2(β)
hposition2 : hmotor3(α) , hmotor4(β)
hposition3 : hmotor5(α) , hmotor6(β)
hposition4 : hmotor7(α) , hmotor8(β)
α送奇数号电机，顺时针角度为正的定义为奇数号电机
                        (前)
          hposition4            hposition1

            α      β             β       α
          hmotor7 hmotor8      hmotor2 hmotor1
            \    /                \    /
             \  /                  \  /
              \/                    \/
              /\                    /\
             /  \                  /  \
            /    \                /    \
          hmotor5 hmotor6      hmotor4 hmotor3
          α        β             β        α
          hposition3            hposition2

                        (后)

*/


/* 引入 usart.c 的全局变量 */
//extern volatile uint8_t RS485_RxBuf[16];
//extern volatile uint8_t Receive_OK;

int temp_state = 0;
//static int up_trigger_count = 0;
//static int down_trigger_count = 0;

/* 达妙电机反馈角度（方便 debug 查看） */
float dm_small_arm_angle_deg = 0.0f;  // 小臂关节角度（度）
float dm_big_arm_angle_deg = 0.0f;    // 大臂关节角度（度）
float dm_small_arm_speed = 0.0f;      // 小臂速度（rad/s）
float dm_big_arm_torque = 0.0f;       // 大臂扭矩（Nm）

/* 正运动学输出（末端坐标，单位 cm） */
float fk_R = 0.0f;  // 水平距离 sqrt(x²+y²)
float fk_z = 0.0f;  // 高度
// 注意：X 和 Y 需要底座旋转角（宇树电机），暂未计算


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

   
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_UART8_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start(&htim9);
	PWM_Init(); 
	

	//remote_control_init(); // 初始化遥控器
	sbus_remote_control_init(); // 初始化遥控器hot rc

	init_motor_parameters();// 初始化电机参数
	remap_motor_ids(); // 重映射id
 Motor_Feedback_Init();
  UART8_Demo_Init(); // 初始化 UART8 的 DMA 接收和中断
	
	
//	HAL_Delay(100);
	Init_turn_omega_des();
	
// motor_release();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
float GyroZ_filtered = 0.0f;
// 滤波系数，0~1，越小越平滑，也越滞后
float alpha = 0.008f;
Arm_Base_Move( 11, 0, 200); //先发底盘为零度，防止第一次调用函数是跳变



Set_dm_enable(&hcan1,0);
Set_dm_enable(&hcan1,1);


Set_dm_zeropoint(&hcan1,0);
Set_dm_zeropoint(&hcan1,1);
	damiao[0].KP=2.0f;   // KP 小值：软保持，手动可推动
	damiao[0].KD = 0.9f;
	damiao[0].tor = 0.0f;
	damiao[1].KP=2.0f;   // KP 小值：软保持，手动可推动
	damiao[1].KD = 0.9f;
	damiao[1].tor = 0.0f;
//	
//	PWM_Set(PWM_OUT);//吸
//Arm_Move_Smooth(10, 10 , 40, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(20, 23 , 40, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(20, 23 , 12, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(20, 23 , 50, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(10, 10 , 50, 400);
//HAL_Delay(5000);
//	float kp_value = 0.1f;
//	float kw_value = 0.01f;
//	
//    hmotor1.Kp = kp_value* (1.0f );
//    hmotor2.Kp = kp_value * (1.0f );
//    hmotor3.Kp = kp_value* (1.0f );
//    hmotor4.Kp = kp_value * (1.0f );
//    hmotor5.Kp = kp_value* (1.0f );
//    hmotor6.Kp = kp_value * (1.0f );
//    hmotor7.Kp = kp_value* (1.0f );
//    hmotor8.Kp = kp_value * (1.0f );

//    hmotor1.Kw = kw_value;
//    hmotor2.Kw = kw_value;
//    hmotor3.Kw = kw_value;
//    hmotor4.Kw = kw_value;
//    hmotor5.Kw = kw_value;
//    hmotor6.Kw = kw_value;
//    hmotor7.Kw = kw_value;
//    hmotor8.Kw = kw_value;
		
		while (1)
  {  
Arm_Base_Move( 11, 0, 200); 
		while(1){
//		//电机接收
//	  Motor_Feedback_Process();    
//    Motor_Feedback_TimeoutTask();
//	hmotor1.Theta_des = 0.0f;
//	hmotor2.Theta_des = 0.0f;
//	hmotor3.Theta_des = 0.0f;
//	hmotor4.Theta_des = 0.0f;
//	hmotor5.Theta_des = 0.0f;
//	hmotor6.Theta_des = 0.0f;
//	hmotor7.Theta_des = 0.0f;
//	hmotor8.Theta_des = 0.0f;		  
//	Motor_SendCmd_AllAngle(); 
//  
//		  
  }
//		
//		while(1){
//		
//		hmotor10.Tau_ff = 0.0f;
//    hmotor10.Kp = 0.0f;
//    hmotor10.Kw = 0.0f;
//			Motor_SendCmd(&hmotor10);
//			HAL_Delay(1);
//		Motor_Feedback_Process();    
//    Motor_Feedback_TimeoutTask();
//		}
//while(1){

//float x = 6.0f;
//float y = 37.0f;
//	    hposition1.B_y = y;
//        hposition1.B_x = x; 
//        hposition2.B_y = y;
//        hposition2.B_x = x; 
//        hposition3.B_y = y;
//        hposition3.B_x = x; 
//        hposition4.B_y = y;
//        hposition4.B_x = x;
//        inverseKinematic_All();
//        Motor_SendCmd_AllAngle(); 

//		  Change_Angle(&hmotor3,0.2f,0.0f);
//HAL_Delay(2000);
//test_jump(7.0f);
//}
//		while(1){
//		  Change_Angle(&hmotor3,0.2f,0.0f);
//			HAL_Delay(2000);
//			Change_Angle(&hmotor3,0.2f,20.0f);
//		  HAL_Delay(2000);
//		

//		}
		
		
//	  Motor_Feedback_Process();    
//    Motor_Feedback_TimeoutTask();
// 		
//		for(int i=1;i<9;i++){
//			uint8_t err  = motor_fb[i].error;
//			if(err!=0){
//			motor_release();
//				while(1){}
//			}
//		}
//		
//		
//	  //      滤波测试
////	  while(1)
////	  {
////	  GyroZ_filtered = alpha * GyroZ + (1 - alpha) * GyroZ_filtered;
//////	  float yaw_Kalman;
//////	  yaw_Kalman = Kalman_Filter(&KF_Yaw, body_yaw,GyroZ );
//////		float f[3]={yaw_Kalman,body_yaw,GyroZ};
//////		  Vofa_JustFloat(f,3);
//////     Kalman(&GyroZ_Kalman,body_yaw);
//////		body_yaw=GyroZ_Kalman.Out;
//////		       Kalman(&GyroZ_Kalman,GyroZ_filtered);
//////		GyroZ_filtered=GyroZ_Kalman.Out;

////	  float f[3]={GyroZ,GyroZ_filtered,body_yaw};
////		  Vofa_JustFloat(f,3);
////	  HAL_Delay(1);
////  }
//  // --------------循环配置----------------
//    UART8_Demo_Process(); // 处理 UART8 接收的树莓派数据，更新 rcData 结构体

//	  stab_roll = 0.0f;// 平衡角度归零
//  
//    // -------------急停模式调试------------------
////  if (motor_release_flag == 1){
////	motor_release_flag=0;
////	init_motor_parameters();
////  }

//    // -------------机械臂调试------------------
	while(1){
	/**
	 * 达妙4310电机反馈接收测试代码
	 * ----------------------------
	 * 原理：达妙电机 MIT 模式是"命令-响应"机制
	 *       只有收到控制命令后，电机才会发送反馈数据
	 *
	 * 当前配置：KP=0（不跟踪位置），KD=0.9（阻尼），电机不会移动
	 *           持续发送命令（10ms间隔），电机才会持续响应反馈
	 *
	 * 反馈读取（Watch窗口）：
	 *   小臂关节角度(度): damiao[0].Rxmsg.Angle * 57.2958f / 2.0f
	 *   大臂关节角度(度): damiao[1].Rxmsg.Angle * 57.2958f
	 *   小臂速度(rad/s): damiao[0].Rxmsg.Speed
	 *   大臂扭矩(Nm): damiao[1].Rxmsg.Torque
	 *
	 * 调试变量：
	 *   can1_rx_count: 收到的CAN1消息总数
	 *   can1_rx_id: 最后收到的电机ID（0=小臂，1=大臂）
	 *   can1_rx_data[8]: 原始响应数据
	 */
//	damiao[0].KP = 0.0f;
//	damiao[0].KD = 0.9f;
//	damiao[0].angle = 0.0f;
//	damiao[0].speed = 0.0f;
//	damiao[0].tor = 0.0f;
//	damiao[1].KP = 0.0f;
//	damiao[1].KD = 0.9f;
//	damiao[1].angle = 0.0f;
//	damiao[1].speed = 0.0f;
//	damiao[1].tor = 0.0f;
//	Set_dm_mit(&hcan1, 0);  // 发送小臂控制命令
//	Set_dm_mit(&hcan1, 1);  // 发送大臂控制命令
//	HAL_Delay(10);  // 10ms 间隔，100Hz

//	// 更新全局变量（debug 中可直接查看）
//	dm_small_arm_angle_deg = (damiao[0].Rxmsg.Angle / 2.0f) * 57.2958f;  // 小臂关节角度（度）
//	dm_big_arm_angle_deg = damiao[1].Rxmsg.Angle * 57.2958f;            // 大臂关节角度（度）
//	dm_small_arm_speed = damiao[0].Rxmsg.Speed;                          // 小臂速度（rad/s）
//	dm_big_arm_torque = damiao[1].Rxmsg.Torque;                          // 大臂扭矩（Nm）

//	// 正运动学：计算 R 和 Z（X、Y 需要底座角度，暂未实现）
//	Arm_Forward_Kinematics_New(dm_big_arm_angle_deg, dm_small_arm_angle_deg,
//	                           &fk_R, &fk_z);

//用于看反馈角度定零点
//while(1){
////  hmotor10.Kp=0.0f;
////  hmotor10.Kw=0.0f;
////  hmotor10.Theta_des=0.0f;
////	hmotor10.Omega_des=0.0f;
////	hmotor10.Tau_ff=0.0f;
////	Motor_SendCmd(&hmotor10);
////	HAL_Delay(1);
////	Motor_Feedback_Process();    
////  Motor_Feedback_TimeoutTask();
//	
////gimbal_send_unitree(0.0f);
//}

//Arm_Move_Smooth(30, 30 , 50, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(30, 30 , 40, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(30, 30 , 35, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(30, 30 , 30, 400);
//HAL_Delay(10000);


//Arm_Move_Smooth(30, 30 , 20, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(30, 25 , 20, 400);
//HAL_Delay(5000);
Arm_Move_Smooth(15, 15 , 40, 400);
HAL_Delay(5000);
Arm_Move_Smooth(20, 30 , 40, 400);
HAL_Delay(5000);
Arm_Move_Smooth(20, 30 , 20, 400);
HAL_Delay(5000);
Arm_Move_Smooth(20, 30 , 40, 400);
HAL_Delay(5000);
Arm_Move_Smooth(15, 15 , 40, 400);
HAL_Delay(5000);
//Arm_Move_Smooth(25, 20 , 20, 400);
//HAL_Delay(5000);
//Arm_Move_Smooth(20, 20 , 20, 400);
//HAL_Delay(10000);


//Arm_Base_Move( 49.29, -29.58, 200); 
//Arm_Move_Smooth(49.29, -29.58 , 40, 500);
////HAL_Delay(5000);
//Arm_Move_Smooth(20 , 15 , 25, 500);
//HAL_Delay(40000);
//Arm_Base_Move( 30, 15, 200); 
//Arm_Move_Smooth(30 , 15 , 25, 500);
//HAL_Delay(40000);
//Arm_Base_Move( 20, 25, 200); 
//Arm_Move_Smooth(20 , 25 , 25, 500);
//HAL_Delay(40000);
//Arm_Base_Move( -9, 0, 200); 
//Arm_Move_Smooth(10 , 15 , 40, 500);
//HAL_Delay(30000); 
//Arm_Base_Move( 11, 20, 200); 
//Arm_Move_Smooth(10 , 15 , 40, 500);
//HAL_Delay(30000); 




////Arm_Move_Smooth(10 , 10 , 40, 100);
////HAL_Delay(2000);  
////Arm_Base_Move( 0, 1, 200);
////HAL_Delay(2000);
////while(1){
//////float x=0.0f;

//////float y=0.0f;
//////float z=0.0f;

//////Arm_Move_Smooth(x , y , z, 100);  
//////Arm_Base_Move( x, y, 200);
////	
//Arm_Move_Smooth(20 , 15 , 40, 500);
//HAL_Delay(2000);  
//Arm_Base_Move( 0, 1, 200);
//HAL_Delay(2000);
//Arm_Base_Move( 1, 0, 200);
//HAL_Delay(2000);	
//Arm_Base_Move( 0, -1, 200);
//HAL_Delay(2000);
//}


//Arm_Move_Smooth(0 , 25 , 50.0f, 100);
//HAL_Delay(2000);
//Arm_Move_Smooth(25 , 25 , 50.0f, 100);
//HAL_Delay(2000);
//Arm_Move_Smooth(20 , 0 , 50.0f, 100);  
//HAL_Delay(2000);

//Arm_Base_Move( 0, 10, 200);  
//HAL_Delay(3000);  
//Arm_Base_Move( 10,10, 200);  
//HAL_Delay(3000);  
//Arm_Base_Move( 10,0, 200);  
//HAL_Delay(3000);


//		gimbal_send_unitree(80.0f);
//	//	gimbal_send_unitree(50.0f);// motor_release();
//while(1){
//	  Set_DM_Motor(1, 0);//大臂调节零点
//	  Set_DM_Motor(0,0);//小臂调节零点
//}
//		Set_dm_mit(&hcan1,0);
//		Set_dm_mit(&hcan1,1);
	// Arm_Move_Smooth(10.0,-20,60,100);
	// HAL_Delay(2000);
		
	//	Arm_Move_Smooth(0, 0, 75.629,50);	
	//	Arm_Move_Smooth(41,0-10,34.629+12.8-2.8,10);
		//	Arm_Move_Smooth(41,0-20,34.629+12.8-2.8,10);
 //依次跑这 5 个点，观察底座是否转到对应方向

//Arm_Base_Move( 0,25 , 200);  HAL_Delay(1000);  // θ1=0°    正右
//Arm_Base_Move( 32,  32, 200);  HAL_Delay(1000);  // θ1=45°   右前
//Arm_Base_Move( 0,  25, 200);  HAL_Delay(1000);  // θ1=90°   正前
//Arm_Base_Move( -32, 32, 200);  HAL_Delay(1000);  // θ1=-45°  右后

//Arm_Base_Move(  0, -45, 200);  HAL_Delay(40000);  // θ1=-90°  正后
		
//		HAL_Delay(4000);
// Arm_Base_Move( 39, -21, 200);  HAL_Delay(400);  // θ1≈-28°  右后
// Arm_Base_Move( 15.03, 93.74, 200);  HAL_Delay(40000);  // θ1≈-17°  右后
//Arm_Base_Move( 45,  -4, 200);  HAL_Delay(40000);  // θ1≈ -5°  近正右
//Arm_Base_Move( 45,   5, 200);  HAL_Delay(40000);  // θ1≈  6°  近正右
//Arm_Base_Move( 43,  14, 200);  HAL_Delay(40000);  // θ1≈ 18°  右前
//Arm_Base_Move( 35.9,  -27.4, 200);  HAL_Delay(10000000);  // θ1≈ 29°  右前

//		Arm_Base_Move(  0, 6.4, 100);  HAL_Delay(2000);  // θ1=90°   正前

// 假设画一个在高为30.0f的矩形（xoy面上）

//float Z = 30.0f;
//uint16_t steps = 150;

// // 角1: 右下 (45, -10)
// Arm_Base_Move( 45, -10, steps);
// Arm_Move_Smooth(45, -10, Z, steps);
// HAL_Delay(1000);

// // 角2: 右上 (45, 10)
// Arm_Base_Move( 45,  10, steps);
// Arm_Move_Smooth(45,  10, Z, steps);
// HAL_Delay(1000);

// // 角3: 左上 (25, 10)
// Arm_Base_Move( 25,  10, steps);
// Arm_Move_Smooth(25,  10, Z, steps);
// HAL_Delay(1000);

// // 角4: 左下 (25, -10)
// Arm_Base_Move( 25, -10, steps);
// Arm_Move_Smooth(25, -10, Z, steps);
// HAL_Delay(1000);

// // 回到角1 闭合矩形
// Arm_Base_Move( 45, -10, steps);
// Arm_Move_Smooth(45, -10, Z, steps);


//  Arm_Base_Move(10.0,-20,100);
//  HAL_Delay(2000);
  
//	Arm_Move_Smooth(5.0,-20,60,100);
//	HAL_Delay(2000);
//  Arm_Move_Smooth(10.0,-20,35,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,35,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,20,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,35,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(10.0,-20,35,100);+
//	HAL_Delay(2000);
//	
	
//  Arm_Move_Smooth(10.0,-20,35,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,35,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,20,100);
//	HAL_Delay(2000);
//	Arm_Move_Smooth(30.0,-20,35,100);
//	HAL_Delay(2000);
//Arm_Move_Smooth( 32, -32, 34.629, 100);  // θ1=-45° 右后方
//		HAL_Delay(3000);
//Arm_Move_Smooth(  0, -45, 34.629, 100);  // θ1=-90° 正后方
//    HAL_Delay(3000);

		
	//		Arm_Move_Smooth(41,0,34.629+12.8-2.8,10);
		
	//		Arm_Move_Smooth(0,-41,34.629+12.8-2.8,50);

//    初始状态
//    Arm_Move_To_Start(0, 0, 60.629);
//    Arm_Move_To(x,y,z);//到达位置
//    Arm_Move_To();//平衡位置
//    Arm_Move_To(x,y,z);//到达位置
//    Arm_Move_To();//平衡位置
//			Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 10.0f);//直 y
//
//		HAL_Delay(3000);
//		Arm_Move_Smooth(-35.0,0,34.629+12.8,5);
//		HAL_Delay(3000);
//		Arm_Move_Smooth(-44.0+7.5,6.0f,34.629+12.8-2.8,50);
//		Arm_Move_Smooth(-9.629,0,50.0+12.8,5);//平衡位置
//		HAL_Delay(5000);
	//	Arm_Move_Smooth(-35,0,34.629+10+12.8,5);//吸取状态
//		HAL_Delay(5000);
//		Arm_Move_To(40,0,0);//
//		Arm_Move_To(0,0,69.629);//完全竖直	
//		Arm_Move_To(-35,0,34.629+12.8);
//		PWM_Set(PWM_IN);//吸
//		HAL_Delay(1000);
//		Arm_Move_To(-9.629,0,50.0+12.8);//平衡位置
	//	HAL_Delay(10000);
//Arm_Move_To(-35,0,40.629+10+12.8);
//HAL_Delay(10000);
//		HAL_Delay(1000);
//  	HAL_Delay(10);
		
		//摄像机舵机给值
		Set_Camera_Servo_Angle_TIM8(TIM_CHANNEL_1,270.0f);//w下面
    Set_Camera_Servo_Angle_TIM8(TIM_CHANNEL_2, 270.0f);//x上面
		
		//吸盘舵机
		Set_Sucker_Servo_Angle_TIM8(TIM_CHANNEL_3, 10.0f);//垂直 y
//		Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 170.0f);//直线
//				Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 45.0f);//向上
//		Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 90.0f);
		
//		//吸盘
//		PWM_Set(PWM_IN);//吸
//		HAL_Delay(2000);
//		
//		PWM_Set(PWM_OUT);//放
//		HAL_Delay(2000);
//		
//		//PWM_Set(PWM_IDLE);//错误，都不进行

//		HAL_Delay(1000);
  }
//    // -------------机械臂调试结束------------------
////    // -------------位置控制测试-----------------

////// while(1){
//////	float x = 0.0f;
//////	float y =25.0f;
//////	hposition1.B_y = y;
//////	hposition1.B_x = x; 
//////	hposition2.B_y = y;
//////	hposition2.B_x = x; 
//////	hposition3.B_y = y;
//////	hposition3.B_x = x; 
//////	hposition4.B_y = y;
//////	hposition4.B_x = x;
//////	crawl_inverseKinematic_All();
//////	Motor_SendCmd_AllAngle(); 
//////	HAL_Delay(10);
////// }


////	//----------4/4单电机通信调试----------
////接收

//

  // -------------遥控控制部分------------------
	// 说明：此处主要控制非状态机函数
 

  // // 树莓派请求行走，没用到，看后续怎么进入行走状态
  // if (uart8_walk_request) {
  //     rcData.sw5 = 0x0320;
  //     rcData.sw7 = 0x0320;
  //     uart8_walk_request = 0;
  // }

  // 遥控取值：上   0xFCE0
  //                0x0320
  //           下   0x0000
  /* 功能说明
  *    sw5    sw6    sw7    sw8    代码位置    功能              state
  *   0xFCE0   -      -      -       tim10     急停               -(不在switch中)
  *   0x0000   -    0xFCE0   -       mian      跳跃               3
  *   0x0000   -    0x0320   -       mian      站立               4
  *   0x0320   -      -      0xFCE0  mian      遥控控制行走        1
  *   0x0320   -      -      0x0000  mian      树莓派控制行走      2
  *     -    0xFCE0   -      -       tim10     调腿高
  *     -    0x0320   -      -       tim10     调步频
  * 
  */

	//if (rcData.sw5 == 0xFCE0||imu_emergency_stop()){emergency_stop = 1;} // 侧翻急停开启版
	if (rcData.sw5 == 0xFCE0){ emergency_stop = 1;} // 侧翻急停关闭版
  else{ emergency_stop = 0;}

  if (rcData.sw5 == 0x0000 && rcData.sw7 == 0xFCE0){ temp_state=3;} // 跳跃，注意是非状态机函数
	if (rcData.sw5 == 0x0000 && rcData.sw7 == 0x0320){ temp_state=4;}	// 站立
  if (rcData.sw5 == 0x0320 && rcData.sw8 == 0xFCE0){ temp_state=1;} // 遥控控制
  if (rcData.sw5 == 0x0320 && rcData.sw8 == 0x0000){                  // 树莓派控制
      temp_state = uart8_walk_request ? 2 : 12;                     // 有数据→走(case2)，超时→站(case12)
  }


  // -------------一些未用上的功能------------------
// 捡箱子功能
//		static int16_t sucker_state_count = 0;
//		sucker_state_count++;
//		motion_Down(15.0f, walk_height);
//		PWM_Set((sucker_state_count % 2 == 1) ? PWM_OUT : PWM_IN);
//		HAL_Delay(2000);
//		motion_Up(walk_height, 15.0f);

// 过限高杆
//		temp_state=6; 
	
// 坐标系翻转
// flip_body();
// HAL_Delay(1000);

// 		Body_Roll_Stabilizer();// 体滚转稳定


	// -----------状态机函数----------------
	//temp_state=1;
	
if (emergency_stop==1){
	motor_release();
}else{
    switch (temp_state)
    {
        case 1:
        // 遥控控制行走
		    motion_Mix(walk_height, 8.0f, max_stride*rcData.R_y,rcData.R_x);
        break;
                
        case 2:
        // 树莓派控制行走
		    motion_Mix(walk_height, 8.0f, max_stride*front_speed, turn_omega);
        break;

        case 3: // 跳跃
       //树莓派测试的时候不用，安全起见 motion_Jump(28.0f);
        break;
                
        case 4: // 站立
		    motion_Mix(walk_height, 0.0000001f, 0.0f, 0.0f);
        break;   
          
        case 5:

        break;
                   
        case 6: // 匍匐，过限高杆
			motion_Crawl(5.0f,6.0f);
        break;     

        case 7:// 前空翻
			motion_Frontflip();

        break; 

        case 8:
			//test_circle();
        break; 

	    	case 12: // RPi 超时自动站立
            motion_Mix(walk_height, 0.0000001f, 0.0f, 0.0f);
        break;

        default:
			motion_StandBy(walk_height) ;
            break;
    }
}

	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// --------private functions---------

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
