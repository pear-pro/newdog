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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// 电机1~8初始偏置�?
#define motor1_bias 0.0f
#define motor2_bias 0.0f
#define motor3_bias 0.0f // 目前阶段还没用到后面的电�?
#define motor4_bias 0.13f
#define motor5_bias 4.88f
#define motor6_bias 0.0f
#define motor7_bias 0.0f
#define motor8_bias 0.0f

// 统一设置电机控制参数
#define Expect_Kp 0.3
#define EXpect_kw 0.01 
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 8个电机的结构�?
//Motor_HandleTypeDef hmotor1;
//Motor_HandleTypeDef hmotor2; // 目前阶段还没用到后面的电�?
//Motor_HandleTypeDef hmotor3;
Motor_HandleTypeDef hmotor4;
//Motor_HandleTypeDef hmotor5;
//Motor_HandleTypeDef hmotor6;
//Motor_HandleTypeDef hmotor7;
//Motor_HandleTypeDef hmotor8;

// 四只脚的位置相关结构�?
Position_HandleTypeDef hposition1;
//Position_HandleTypeDef hposition2; // 目前阶段还没用到后面的脚
//Position_HandleTypeDef hposition3;
//Position_HandleTypeDef hposition4;
Gait_Config g_gait_config = {
    .freq = 0.03f,   // 时间步长
	  .T = 1.0f,       // 步�?�周�?
    .height = 5.0f, // 步高（跳跃高度）
    .stride = 5.0f, // 步长（前进距离）
    .maxHeight = 5.0f    // �?大高�?
};




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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	Motor_Feedback_Init();
  
  Motors_Init(&huart6);
//	HAL_UART_Receive_IT(&huart6, &usart6_rx_byte, 1);
//	if (HAL_UART_Receive_IT(&huart6, &usart6_rx_byte, 1) != HAL_OK)
//  {
//    Error_Handler(); 
//  }
//  初始化电机结构体,角度环控制的话只�?要初始化kp和kw
//  Motor_Init(&hmotor1, &huart6, 1);
//	hmotor1.Kp = Expect_Kp;
//	hmotor1.Kw = EXpect_kw;	
//    Motor_Init(&hmotor2, &huart6, 1);
//	hmotor2.Kp = Expect_Kp;
//	hmotor2.Kw = EXpect_kw;
//    Motor_Init(&hmotor3, &huart6, 3);
//	hmotor3.Kp = Expect_Kp;
//	hmotor3.Kw = EXpect_kw;	
//    Motor_Init(&hmotor4, &huart6, 4);
//    hmotor4.Kp = Expect_Kp;
//    hmotor4.Kw = EXpect_kw;    
//    Motor_Init(&hmotor5, &huart6, 5);
//    hmotor5.Kp = Expect_Kp;
//    hmotor5.Kw = EXpect_kw;	
//    Motor_Init(&hmotor6, &huart6, 4);
//	hmotor6.Kp = Expect_Kp;
//	hmotor6.Kw = EXpect_kw;    
//	Motor_Init(&hmotor7, &huart6, 5);
//	hmotor7.Kp = Expect_Kp;
//	hmotor7.Kw = EXpect_kw;	
//    Motor_Init(&hmotor8, &huart6, 4);
//	hmotor8.Kp = Expect_Kp;
//	hmotor8.Kw = EXpect_kw;    
    motors[0].Kp = Expect_Kp;
    motors[0].Kw = EXpect_kw; 
		motors[1].Kp = Expect_Kp;
    motors[1].Kw = EXpect_kw; 
	  motors[2].Kp = Expect_Kp;
    motors[2].Kw = EXpect_kw; 
		motors[3].Kp = Expect_Kp;
    motors[3].Kw = EXpect_kw; 
		motors[4].Kp = Expect_Kp;
    motors[4].Kw = EXpect_kw; 
		motors[5].Kp = Expect_Kp;
    motors[5].Kw = EXpect_kw; 
		motors[6].Kp = Expect_Kp;
    motors[6].Kw = EXpect_kw; 
		motors[7].Kp = Expect_Kp;
    motors[7].Kw = EXpect_kw; 
		


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
				//ID
//		motors[0].Tau_ff = 0.0f;
//		motors[0].Theta_des =0.0f;
//		motors[0].Omega_des =0.0f;
//		
//		motors[1].Tau_ff = 0.0f;
//		motors[1].Theta_des =0.0f;
//		motors[1].Omega_des =0.0f;
//				
//		motors[2].Tau_ff = 0.0f;
//		motors[2].Theta_des =0.0f;
//		motors[2].Omega_des =0.0f;
//		
//		motors[3].Tau_ff = 0.0f;
//		motors[3].Theta_des =3.0f;
//		motors[3].Omega_des =0.0f;
//		
		motors[4].Tau_ff = 0.0f;
		motors[4].Theta_des = 2.0f;
		motors[4].Omega_des =0.0f;
//		
		motors[5].Tau_ff = 0.0f;
		motors[5].Theta_des =2.0f;
		motors[5].Omega_des =0.0f;
		
//		motors[6].Tau_ff = 0.0f;
//		motors[6].Theta_des =0.0f;
//		motors[6].Omega_des =0.0f;
//		
//		motors[7].Tau_ff = 0.0f;
//		motors[7].Theta_des =0.0f;
//		motors[7].Omega_des =0.0f;
//		
		Motors_SendAll();
		
	
    Motor_Feedback_Process();
	    if (motor_timeout_flag)//掉线检测
        {
            motor_timeout_flag = 0;
            Motor_Feedback_TimeoutTask();
        }

      
//        if (motor_fb[0].online)//读取0号电机
//        {
//            float pos = motor_fb[0].theta;
//            float vel = motor_fb[0].omega;
//            float tau = motor_fb[0].tau;
//            (void)pos;
//            (void)vel;
//            (void)tau;
//        }   


       
		
		
    // 输入B点坐�?
//	hposition1.B_x = 0.0f;
//	hposition1.B_y = 20.0f;
//		   Jump(& hposition1 );
		//输入B点坐�?
//		hposition1.B_x = 25.0f;
//		hposition1.B_y = 20.0f;
//    //根据B点坐标，逆解算出alpha和beta
//		inverseKinematic(&hposition1);
    //发�?�角度数�?
//     hmotor5.Theta_des = hposition1.alpha;
//		 Motor_SendCmd(&hmotor5);
//	   HAL_Delay(1);
//		
//		 hmotor4.Theta_des = hposition1.beta;
//		 Motor_SendCmd(&hmotor4);
//	   HAL_Delay(1);
//    // 根据B点坐标，逆解算出alpha和beta

//	 inverseKinematic(&hposition1);
//	// 发�?�角度数�?
//	 hmotor4.Theta_des = hposition1.alpha;
//	 Motor_SendCmd(&hmotor4);
//	 HAL_Delay(1);
//	 hmotor5.Theta_des = hposition1.beta;
//	 Motor_SendCmd(&hmotor5);
	
	// 单次置数
//	hposition1.alpha = 20.0f;
//	hposition1.beta = 20.0f;
//	
//	//motor1 id = 5,bias = 3.33
//	hmotor1.Theta_des = 3.64f / 6.33f + 6.28 * hposition1.beta / 360.0f;
//	hmotor1.Kp = 0.3;
//	hmotor1.Kw = 0.05;
//	Motor_SendCmd(&hmotor1);
//	
//	HAL_Delay(1);
//	
//	//motor1 id = 4,bias = 4.58
//	hmotor2.Theta_des = 4.75f / 6.33f + 6.28 * hposition1.alpha / 360.0f;
//	hmotor2.Kp = 0.3;
//	hmotor2.Kw = 0.05;
//	Motor_SendCmd(&hmotor2);

//	HAL_Delay(1000);
//	
	
	// 循环置数
	
//	hposition1.alpha = 10.0f;
//	hposition1.beta = 10.0f;
//		
//	for (int j =0;j<100;j++)
//	{
//		hmotor4.Theta_des = motor4_bias / 6.33f + 6.28 * hposition1.beta / 360.0f;
//		Motor_SendCmd(&hmotor4);
//		HAL_Delay(1);
//		hmotor5.Theta_des = motor5_bias / 6.33f + 6.28 * hposition1.alpha / 360.0f;
//		Motor_SendCmd(&hmotor5);

//		HAL_Delay(1);
//		
//		hposition1.alpha += 1.5f;
//		hposition1.beta += 1.5f;
//	}
//	
//	for (int j =0;j<100;j++)
//	{
//		hmotor4.Theta_des = motor4_bias / 6.33f + 6.28 * hposition1.beta / 360.0f;
//		Motor_SendCmd(&hmotor4);
//		HAL_Delay(1);
//		hmotor5.Theta_des = motor5_bias / 6.33f + 6.28 * hposition1.alpha / 360.0f;
//		Motor_SendCmd(&hmotor5);
//		HAL_Delay(1);
//		
//		hposition1.alpha -= 1.5f;
//		hposition1.beta -= 1.5f;
//	}

	
	



	
	  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
