/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Left;
uint16_t Right;
uint16_t Middle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}


/**
  * 函数功能: 左转
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：htim3控制右侧电机
  */
void TurnLeft(void)
{
	HAL_GPIO_TogglePin(Right_Motors_0_GPIO_Port,GPIO_PIN_7);
	HAL_GPIO_TogglePin(Right_Motors_1_GPIO_Port,GPIO_PIN_9);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,150);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,40);
	
}


/**
  * 函数功能: 右转
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：htim3控制右侧电机
  */
void TurnRight(void)
{	HAL_GPIO_TogglePin(Right_Motors_0_GPIO_Port,GPIO_PIN_6);
	HAL_GPIO_TogglePin(Right_Motors_1_GPIO_Port,GPIO_PIN_8);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,40);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,150);
	
}

/**
  * 函数功能: 掉头
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：Delay 用来确定掉头正确进行
  */
void TurnBack(void)
{
	HAL_GPIO_TogglePin(Left_Motors_0_GPIO_Port,GPIO_PIN_6);
	HAL_GPIO_TogglePin(Left_Motors_1_GPIO_Port,GPIO_PIN_8);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,80);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,80);
	HAL_Delay(1000);
}

/**
  * 函数功能: 直行
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Gohead(void)
{
	HAL_GPIO_WritePin(Left_Motors_0_GPIO_Port,GPIO_PIN_6,0);
	HAL_GPIO_WritePin(Left_Motors_1_GPIO_Port,GPIO_PIN_8,1);
	
	HAL_GPIO_WritePin(Right_Motors_0_GPIO_Port,GPIO_PIN_7,0);
	HAL_GPIO_WritePin(Right_Motors_1_GPIO_Port,GPIO_PIN_9,1);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,80);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,80);
}
void Car_Init(void)
{
	HAL_GPIO_WritePin(Left_Motors_0_GPIO_Port,GPIO_PIN_6,0);
	HAL_GPIO_WritePin(Left_Motors_1_GPIO_Port,GPIO_PIN_8,1);
	
	HAL_GPIO_WritePin(Right_Motors_0_GPIO_Port,GPIO_PIN_7,0);
	HAL_GPIO_WritePin(Right_Motors_1_GPIO_Port,GPIO_PIN_9,1);
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	Car_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Left=1,Middle=1,Right=1;
  while (1)
  {
	Left=HAL_GPIO_ReadPin(Left_GPIO_Port,Left_Pin);  
	Middle=HAL_GPIO_ReadPin(Middle_GPIO_Port,Middle_Pin);
	Right=HAL_GPIO_ReadPin(Right_GPIO_Port,Right_Pin);
    
	Gohead();
	printf("直行");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	if(Left==1&&Middle==1&&Right==0)
	{
		TurnLeft();
		HAL_Delay(500);
		printf("左转");
	}
	else if(Left==0&&Middle==1&&Right==1)
	{
		TurnRight();
		HAL_Delay(500);
		printf("右转");
	}
	else if(Middle==0)
	{
		TurnBack();
		printf("掉头");
	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
