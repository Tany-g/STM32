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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern UART_HandleTypeDef huart1;   //声明串口


/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
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
    * 函数功能: 控制小车左侧两电机
    * 输入参数: Direction 输入1————前进  输入0————后退 
    * 返 回 值: 无
    * 说    明：无
    */
void LeftControl(int Direction)
{
	if(Direction==0)  
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
	}
	else			
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	}
	  	
}
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
	uint16_t Case=1;
	uint16_t angle=1;//角度控制
	uint16_t p3=0;
	uint16_t p2=0;
	uint16_t b3=0;
	uint16_t b4=0;
	uint16_t b5=0;
	uint16_t b6=0;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Case++;
	  switch (Case)
      {
      	case 1://0
			angle = 5;
      		break;
      	case 2://45
			angle = 10;
      		break;
		case 3://90
			angle = 15;
      		break;
		case 4://135
			angle = 20;
      		break;
		case 5://180
			angle = 25;
			break;
      	default:
			
			printf("    角度配置失败");
      		break;
      }
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,angle);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,200-angle);
	  
	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_3);
	  if(Case==5)
	  {
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,0);
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
	  }
		  
	  p3=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	  p2=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
	  if(p2==1)
		  printf("|| 电机正转 p2=%d,p3=%d ||",p2,p3);
	  else if(p2==p3)
		  printf("|| 电机制动 p2=%d,p3=%d ||",p2,p3);
	  else
		  printf("|| 电机反转 p2=%d,p3=%d ||",p2,p3);

	  HAL_Delay(5000);
	  if(Case==5)
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,1);
	  printf("______舵机角度为：%d  \n",((angle/5)-1)*45);
	  
	  if (Case==5)
		  Case=0;
		  
		  
	b3=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
	b4=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
	b5=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
	b6=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
	
	
	printf("  b3=%d,b4=%d,b5=%d,b6=%d",b3,b4,b5,b6);// 0 代表接受到红外线 即前方有障碍物
    HAL_Delay(1000);

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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
    {
    	case GPIO_PIN_0:
			
		
    		break;
    	case GPIO_PIN_1:
			
		
    		break;
    	case GPIO_PIN_2:
			
		
    		break;
    	case GPIO_PIN_3:
			
		
    		break;
    	case GPIO_PIN_4:
			
		
    		break;
    	default:
    		break;
    }
	
	
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
