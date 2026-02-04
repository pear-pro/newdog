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
#include "dma.h"
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



// 统一设置电机控制参数
#define Expect_Kp 0.5
#define EXpect_kw 0.01 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
  /* USER CODE BEGIN 2 */

  remote_control_init(); // 初始化遥控器

//  初始化电机结构体,角度环控制只需要初始化kp和kw
    Motor_Init(&hmotor1, &huart6, 1);
    hmotor1.Kp = Expect_Kp;
    hmotor1.Kw = EXpect_kw;	
    Motor_Init(&hmotor2, &huart6, 2);
    hmotor2.Kp = Expect_Kp;
    hmotor2.Kw = EXpect_kw;
    Motor_Init(&hmotor3, &huart6, 3);
    hmotor3.Kp = Expect_Kp;
    hmotor3.Kw = EXpect_kw;	
    Motor_Init(&hmotor4, &huart6, 4);
    hmotor4.Kp = Expect_Kp;
    hmotor4.Kw = EXpect_kw;    
    Motor_Init(&hmotor5, &huart6, 5);
    hmotor5.Kp = Expect_Kp;
    hmotor5.Kw = EXpect_kw;	
    Motor_Init(&hmotor6, &huart6, 6);
    hmotor6.Kp = Expect_Kp;
    hmotor6.Kw = EXpect_kw;    
    Motor_Init(&hmotor7, &huart6, 7);
    hmotor7.Kp = Expect_Kp;
    hmotor7.Kw = EXpect_kw;	
    Motor_Init(&hmotor8, &huart6, 8);
    hmotor8.Kp = Expect_Kp;
    hmotor8.Kw = EXpect_kw;  


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 

    // if (t >= Ts){t = 0;}
    // 根据t给B点坐标赋值
    // 注意下面的trot和StepInPlace一次只能取消注释其中一个，StepInPlace是原地踏步
    // trot(1,30,15,40); // 前进
    //StepInPlace(0.01,34,5); // 原地踏步
    // testCircle();
	// testStep();
	// HAL_Delay(6);

  //   // 根据B点坐标，逆解算出alpha和beta
  //   inverseKinematic(&hposition1);

  //   // 发送角度数据
  //   hmotor5.Theta_des = motor5_bias / 6.33f + 6.28 * hposition1.alpha / 360.0f;;
  //   Motor_SendCmd(&hmotor5);
  //   hmotor4.Theta_des = motor4_bias / 6.33f + 6.28 * hposition1.beta / 360.0f;
  //   Motor_SendCmd(&hmotor4);


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
    


    switch (move_state)
    {
        case 1:

            motion_Forward(height, 13.0f, stride);
        break;
                
        case 2:

            motion_Backward(height,  13.0f, stride);
        break;

        case 3:
      
            motion_TurnRight(height,  13.0f, stride);
        break;
                
        case 4:

            motion_TurnLeft(height,  13.0f, stride);
        break;   
          
        case 5:

            StepInPlace(height, step_height);
        break;
                
        case 6:
            motion_Jump();
			move_state = 0;
        break;     

        case 7:
            testCircle(); // 测试画圆用，实际上没有这个动作
        break; 

        case 8:
            testJump(stride); // 测试跳跃用，实际上没有这个动作
        break; 

        default:
            motion_StandBy(height) ;
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
