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
#include "motor_4310.h"
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
  MX_CAN2_Init();
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
	
	HAL_Delay(100);
	Init_turn_omega_des();
	
// motor_release();

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    Set_dm_enable(&hcan1, 0);
	  Set_dm_enable(&hcan1, 1);
float GyroZ_filtered = 0.0f;
// 滤波系数，0~1，越小越平滑，也越滞后
float alpha = 0.008f;
//		volatile  float accum_x = 0.0f;
//		volatile  float accum_y = 0.0f;



//	hmotor1.Theta_des = 0.0f;
//	hmotor2.Theta_des = 0.0f;
//	hmotor3.Theta_des = 0.0f;
//	hmotor4.Theta_des = 0.0f;
//	hmotor5.Theta_des = 0.0f;
//	hmotor6.Theta_des = 0.0f;
//	hmotor7.Theta_des = 0.0f;
//	hmotor8.Theta_des = 0.0f;		  
//	Motor_SendCmd_AllAngle(); 

while (1)
  {  
//	  while(1){
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
//	  }
		
//		float f[4]={AccY,AccX,accum_y,accum_x};
//		Vofa_JustFloat(f,4);
	  
// 		
//		for(int i=2;i<10;i++){
//			uint8_t err  = motor_fb[i].error;
//			if(err!=0){
//			motor_release();
//				while(1){}
//			}
//		}
		
		
	  //      滤波测试
//	  while(1)
//	  {
//	  GyroZ_filtered = alpha * GyroZ + (1 - alpha) * GyroZ_filtered;
////	  float yaw_Kalman;
////	  yaw_Kalman = Kalman_Filter(&KF_Yaw, body_yaw,GyroZ );
////		float f[3]={yaw_Kalman,body_yaw,GyroZ};
////		  Vofa_JustFloat(f,3);
////     Kalman(&GyroZ_Kalman,body_yaw);
////		body_yaw=GyroZ_Kalman.Out;
////		       Kalman(&GyroZ_Kalman,GyroZ_filtered);
////		GyroZ_filtered=GyroZ_Kalman.Out;

//	  float f[3]={GyroZ,GyroZ_filtered,body_yaw};
//		  Vofa_JustFloat(f,3);
//	  HAL_Delay(1);
//  }
  // --------------循环配置----------------
    UART8_Demo_Process(); // 处理 UART8 接收的树莓派数据，更新 rcData 结构体

	  stab_roll = 0.0f;// 平衡角度归零
  
    // -------------急停模式调试------------------
//  if (motor_release_flag == 1){
//	motor_release_flag=0;
//	init_motor_parameters();
//  }

    // -------------机械臂调试（注释保留）------------------
//	hmotor10.Kp =0.8f;
//	gimbal_send_unitree(10.0f);
//	Set_DM_Motor(1, -50);//大臂调节
//	Set_DM_Motor(0, -20);//小臂调节
//	Arm_Move_To(40,0,50);
//	Set_Servo_Angle_TIM8(TIM_CHANNEL_1,30.0f);//摄像机w
//  Set_Servo_Angle_TIM8(TIM_CHANNEL_2, 135.0f);//摄像机x
//	Set_Servo_Angle_TIM8(TIM_CHANNEL_3, 265.0f);//吸盘垂直
	


    // -------------位置控制测试-----------------

// while(1){
//	float x = 0.0f;
//	float y =25.0f;
//	hposition1.B_y = y;
//	hposition1.B_x = x; 
//	hposition2.B_y = y;
//	hposition2.B_x = x; 
//	hposition3.B_y = y;
//	hposition3.B_x = x; 
//	hposition4.B_y = y;
//	hposition4.B_x = x;
//	crawl_inverseKinematic_All();
//	Motor_SendCmd_AllAngle(); 
//	HAL_Delay(10);
// }


//	//----------4/4单电机通信调试----------
//接收

//while(1){
////	MotorTest_Sweep(1, 0.4f);
////	MotorTest_Sweep(2, 0.4f);
//	MotorTest_Sweep(3, 0.4f);
//	MotorTest_Sweep(4, 0.4f);
////	MotorTest_Sweep(5, 0.4f);  
////	MotorTest_Sweep(6, 0.4f);
////	MotorTest_Sweep(7, 0.4f);
////	MotorTest_Sweep(8, 0.4f);
//// 	MotorTest_Sweep(9, 0.4f); 
////	MotorTest_Sweep(10, 0.4f);
////	MotorTest_Sweep(11, 0.4f);
//// 	MotorTest_Sweep(12, 0.4f);
//// 	MotorTest_Sweep(13, 0.4f); 
////	MotorTest_Sweep(14, 0.4f);
////	MotorTest_Sweep(15, 0.4f);
//}


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

//	//if (rcData.sw5 == 0xFCE0||imu_emergency_stop()){emergency_stop = 1;} // 侧翻急停开启版
//	if (rcData.sw5 == 0xFCE0){ emergency_stop = 1;} // 侧翻急停关闭版
//  else{ emergency_stop = 0;}

//  if (rcData.sw5 == 0x0000 && rcData.sw7 == 0xFCE0){ temp_state=6;} // 跳跃，注意是非状态机函数
//	if (rcData.sw5 == 0x0000 && rcData.sw7 == 0x0320){ temp_state=4;}	// 站立
//  if (rcData.sw5 == 0x0320 && rcData.sw8 == 0xFCE0){ temp_state=1;} // 遥控控制
//  if (rcData.sw5 == 0x0320 && rcData.sw8 == 0x0000){                  // 树莓派控制
//      temp_state = uart8_walk_request ? 2 : 12;                     // 有数据→走(case2)，超时→站(case12)
//  }

if (rcData.sw8 == 0xFCE0){ emergency_stop = 1;} // 侧翻急停关闭版
  else{ emergency_stop = 0;}

  if (rcData.sw8 == 0x0000 && rcData.sw7 == 0xFCE0){ temp_state=6;} // 跳跃，注意是非状态机函数
	if (rcData.sw8 == 0x0000&& rcData.sw7 == 0x0320){ temp_state=4;}	// 站立

  if (rcData.sw8 == 0x0320&& rcData.sw7 == 0x0320 ){ temp_state=1;} // 遥控控制
   if (rcData.sw8 == 0x0320&& rcData.sw7 == 0xFCE0 ){ temp_state=2;} // 树莓派控制
   
 			if((fabsf(rcData.R_x)<0.05f)&&(fabsf(rcData.R_y)<0.05f)) {
//				state_cnt++;
//				if(state_cnt>=300){
				stay_state=1;
				Init_turn_omega_des();
//					state_cnt=0;
	//			}
				
//				grap_cnt++;
//				if(grap_cnt==300){
//					grap_cnt =0;
//				}
							
			}
			else{
				stay_state=0;
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
//	temp_state=1;
	
	float control_omega = 0.0f;

	
	flip_offset = 0.0f;
if (emergency_stop==1){
	motor_release();
}else{
	    static uint8_t prev_temp_state = 0;  /* 上一帧的状态，用于检测切换 */
    switch (temp_state)
    {
        case 1:
        // 遥控控制行走
//		if(fabs(rcData.R_y)<0.35) rcData.R_y=0.0f;
		if(stay_state==0){
			if(fabsf(rcData.R_x)<0.05f){
		    motion_mix7(walk_height, 7.0f, -(rcData.R_y+0.1582f));
			}
			else{
		    motion_mix5(walk_height, 7.0f, -rcData.R_y,-rcData.R_x);
			}
		}
		else if(stay_state==1){
			motion_StandBy(walk_height);
		}
//		
//		    motion_mix7(walk_height, 7.5f, -(rcData.R_y+0.1582f));
		
        break;
                
        case 2:
        // 树莓派控制行走
//			control_omega = powf(fabs(turn_omega),0.2);
//			control_omega *= turn_omega/fabs(turn_omega);
//		    motion_mix5(walk_height, 7.0f, -front_speed,control_omega);
			if(stay_state==0){
		    motion_mix6(walk_height, 7.5f, -front_speed);
			}
			else if(stay_state==1){
				motion_StandBy(walk_height);
			}
//           motion_mix6(walk_height, 7.5f, -(front_speed+0.1f));

        break;

        case 3: // 跳跃
       //树莓派测试的时候不用，安全起见
//				motion_Jump(15.0f);
        break;
                
        case 4: // 站立
			
		motion_StandBy(walk_height);
		   // motion_Mix(walk_height, 0.0000001f, 0.0f);
		//Init_turn_omega_des();
			
        break;   
          
        case 5:

        break;
                   
        case 6: // 匍匐，过限高杆
//			    if (prev_temp_state != 6) {
//			        motion_Crawl_Reset();  /* 刚进入匍匐，触发缓慢下蹲 */
//			    }
//			motion_Crawl(5.0f,12.0f);
//				motion_Jump(15.0f);
        break;     

        case 7:// 前空翻
			motion_Frontflip();

        break; 

        case 8:
			//test_circle();
        break; 

	    	case 12: // RPi 超时自动站立
            motion_Mix(walk_height, 0.0000001f, 0.0f);
        break;

        default:
			motion_StandBy(walk_height) ;
            break;
    }
	    prev_temp_state = temp_state;
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
