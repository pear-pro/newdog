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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 统一设置电机控制参数
float Expect_Kp = 0.4;
float EXpect_kw = 0.01;  // 0.01
float Expect_Tau_ff = 0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t alive_tick = 0; 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 8个电机的结构体
Motor_HandleTypeDef hmotor1;
Motor_HandleTypeDef hmotor2;
Motor_HandleTypeDef hmotor3;
Motor_HandleTypeDef hmotor4;
Motor_HandleTypeDef hmotor5;
Motor_HandleTypeDef hmotor6;
Motor_HandleTypeDef hmotor7;
Motor_HandleTypeDef hmotor8;
Motor_HandleTypeDef hmotor9;

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
extern volatile uint8_t RS485_RxBuf[16];
extern volatile uint8_t Receive_OK;

int temp_state = 0;
//static int up_trigger_count = 0;
//static int down_trigger_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start(&htim9);
	PWM_Init(); 
	//remote_control_init(); // 初始化遥控器
	sbus_remote_control_init();

	init_motor_parameters();// 设置角度模式参数
	remap_motor_ids(); // 重映射id
	
	motor_release();
	
	motion_Jump(6.0f);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
  
    // pd控制横向平衡
	//Body_Roll_Stabilizer();

// ----------遥控传参说明----------

    /*
    遥控传参：运动状态 move_state
             支撑高度 height
             抬腿高度 step_height
             步幅 stride
    动作说明：1.前进，后退，左转，右转
                - 单足轨迹都是摆线方程
                - 传入height决定支撑高度，stride决定步幅
             2.motion_StandBy是原地站立
                - 传入height决定站立的高度
             3.StepInPlace是原地踏步
                - 传入height决定支撑腿高度，step_height决定抬起高度
    */
    

	
//	//----------4/4单电机通信测试----------

//	MotorTest_Sweep(1, 0.4f);
//	MotorTest_Sweep(2, 0.4f);
//	MotorTest_Sweep(3, 0.4f);
//	MotorTest_Sweep(4, 0.4f);
//	MotorTest_Sweep(5, 0.4f);  
//	MotorTest_Sweep(6, 0.4f);
//	MotorTest_Sweep(7, 0.4f);
//	MotorTest_Sweep(8, 0.4f);

    // --------------- 电机偏置角调整-------
//while(1){
//  Motor_SendCmd_AllAngle();
//}	

	// ------------imu控制翻身的条件写这---------
//	if (height > 42.0f){
//		flip_body();
//	}


// --------- 左轮遥感控制 ----------
	// 捡起箱子的代码

//	if (height > 42.0f) {
//		up_trigger_count++;
//		motion_Down(15.0f, 20.0f);
//		PWM_Set((up_trigger_count % 2 == 1) ? PWM_OUT : PWM_IN);
//		HAL_Delay(1000);
//		motion_Up(20.0f, 15.0f);
//	}
//	
//	if (height < 16.0f) {
//		down_trigger_count++;
//		temp_state = (down_trigger_count % 2 == 1) ? 1 : 0;
//	}

	// -----------非状态机函数----------------
  //0x0320,0x0000,0xFCE0
    if (rcData.sw7 == 0xFCE0){
      motion_Jump(6.0f);
    }


	// -----------状态机----------------
	// 原本：move_state

//	static uint32_t start_time = 0;
//	static uint8_t state_active = 1;  // 1表示正在执行状态机，0表示已超时

//	// 首次进入时记录开始时间
//	if (start_time == 0)
//	{
//		start_time = HAL_GetTick();
//	}

//	// 检查14秒超时
//	if (HAL_GetTick() - start_time >= 8000)
//	{
//		temp_state = 0;      // 状态设为0
//		state_active = 0;    // 标记已超时
//	}
	float temp_rc_y = 0.0f;
    walk_height = 20.0f;
	temp_state = 2;
    switch (temp_state)
    {
        case 1:

        motion_Forward(walk_height, 0.001f, temp_rc_y);
		//motion_Forward(26.0f, 13.0f, 10);
        break;
                
        case 2:
			  motion_Mix(walk_height, 0.000001f, temp_rc_y);
        break;

        case 3:
		    motion_Forward(walk_height, 0.0f, 10.0f);
        break;
                
        case 4:
			flip_body();
			HAL_Delay(1000);
        break;   
          
        case 5:
			motion_Jump(6.0f);
            //StepInPlace(20.0f, step_height);
        break;
                
        case 6:

        break;     

        case 7:
            testCircle(); // 测试画圆用
        break; 

        case 8:
        break; 
		
        default:
			motion_StandBy(walk_height) ;
            break;
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
