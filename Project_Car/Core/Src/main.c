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
typedef struct Coordinate 
{
	uint16_t X;
	uint16_t Y;
}Coordinate;

typedef struct MovingStateOfCar//С���˶������־λ
{
	uint16_t L;
	uint16_t R;
	uint16_t B;
	uint16_t F;
}CarState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define test
#define COUNT 8
#define SPEED 60
#define FBcorrectTime 20
#define LRcorrectTime 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 
enum Turn
{
	Turn_left,
	Turn_right,
	Turn_front,
	Turn_back,
	Turn_leftFront,
	Turn_rightFront,
	Turn_leftBack,
	Turn_rightBack
};




//����
uint16_t BLast=0;
uint16_t BThis=0;

uint16_t FLast=0;
uint16_t FThis=0;

uint16_t LLast=0;
uint16_t LThis=0;

uint16_t RLast=0;
uint16_t RThis=0;
//����
uint16_t isBackLine=0;
uint16_t isFrontLine=0;
uint16_t isLeftLine=0;
uint16_t isRightLine=0;

// �ĸ�����ĺ���
uint16_t LeftLt;
uint16_t RightLt;
uint16_t FrontLt;
uint16_t BackLt;

//С������
Coordinate CarPoint={0,0};
uint16_t Car_X;
uint16_t Car_Y;


uint16_t To=0;
//�����߱�־λ
uint16_t FEnter=0;
uint16_t FExit=0;

uint16_t BEnter=0;
uint16_t BExit=0;

uint16_t LEnter=0;
uint16_t LExit=0;

uint16_t REnter=0;
uint16_t RExit=0;
//С��״̬

CarState  Carsta;
CarState* pCarsta=&Carsta;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef test
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

#endif
/**
  * ��������: ����A����ת������ ���ٶ�
  * �������: Direction:�����������ת  Speed��0-100ֵԽ���ٶȿ�
  * �� �� ֵ: ��
  * ˵    ������
  */

void A_control(uint16_t Direction,uint16_t Speed)
{
	if(Direction)
	{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Speed);
	}
	else
	{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,Speed);	
	}
}

/**
  * ��������: ����B����ת������ ���ٶ�
  * �������: Direction:�����������ת  Speed��0-100ֵԽ���ٶȿ�
  * �� �� ֵ: ��
  * ˵    ������
  */
void B_control(uint16_t Direction,uint16_t Speed)
{
	if(Direction)
	{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Speed);
	}
	else
	{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Speed);	
	}	
}

/**
  * ��������: ����С���˶�����
  * �������: enum ���͵İ˸�����
  * �� �� ֵ: ��
  * ˵    ������
  */
void Car_control(int Turn,CarState* CarSta)
{
	switch (Turn)
    {
    	case Turn_front://front
			#ifdef test
			printf("______ǰ��   F=%d" ,CarSta->F);
			#endif
			CarSta->L=0;
			CarSta->F=1;
			CarSta->B=0;
			CarSta->R=0;
			A_control(1,SPEED);
			B_control(1,SPEED);
    		break;
		
    	case  Turn_back://left
			#ifdef test

			printf("����   B=%d" ,CarSta->B);
			#endif
			CarSta->L=0;
			CarSta->F=0;
			CarSta->B=1;
			CarSta->R=0;
			A_control(0,SPEED);
			B_control(0,SPEED);
    		break;
		
		case  Turn_left:
			#ifdef test

			printf("����  L=%d" ,CarSta->L);
			#endif
			CarSta->L= 1;
			CarSta->F= 0;
			CarSta->B= 0;
			CarSta->R= 0;
			A_control(0,SPEED);
			B_control(1,SPEED);
    		break;
		
		case  Turn_right:
			#ifdef test

			printf("����  R=%d" ,CarSta->R);
			#endif
			CarSta->L= 0;
			CarSta->F= 0;
			CarSta->B= 0;
			CarSta->R= 1;
			A_control(1,SPEED);
			B_control(0,SPEED);
    		break;
		
		case  Turn_rightBack:
			#ifdef test

			printf("�Һ�   R=%d B=%d " ,CarSta->R,CarSta->B);
			#endif
			CarSta->L= 0;
			CarSta->F= 0;
			CarSta->B= 1;
			CarSta->R= 1;
			A_control(0,SPEED);
			B_control(0,0);
    		break;
		
		case  Turn_rightFront:
			#ifdef test

			printf("�Һ�   R=%d F=%d " ,CarSta->R,CarSta->F);
			#endif
			CarSta->L= 0;
			CarSta->F= 1;
			CarSta->B= 0;
			CarSta->R= 1;			
			A_control(1,SPEED);
			B_control(0,0);
    		break;
		case  Turn_leftBack:
			#ifdef test

			printf("�Һ�   L=%d B=%d " ,CarSta->L,CarSta->B);
			#endif
			CarSta->L= 1;
			CarSta->F= 0;
			CarSta->B= 1;
			CarSta->R= 0;	
			A_control(1,0);
			B_control(0,SPEED);
    		break;
		case  Turn_leftFront:
			#ifdef test

			printf("�Һ�   L=%d F=%d " ,CarSta->L,CarSta->F);
			#endif
			CarSta->L= 1;
			CarSta->F= 1;
			CarSta->B= 0;
			CarSta->R= 0;	
			A_control(1,0);
			B_control(1,SPEED);
    		break;
    	default:
    		break;
    }
}


/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void Car_To(int Turn)
{
	switch (Turn)
    {
    	case Turn_front://front
			#ifdef test
			printf("______ǰ��   F=%d" ,Carsta.F);
			#endif
			A_control(1,SPEED);
			B_control(1,SPEED);
    		break;
		
    	case  Turn_back://left
			#ifdef test
			printf("����   B=%d" ,Carsta.B);
			#endif
			A_control(0,SPEED);
			B_control(0,SPEED);
    		break;
		
		case  Turn_left:
			#ifdef test
			printf("����  L=%d" ,Carsta.L);
			#endif
			A_control(0,SPEED);
			B_control(1,SPEED);
    		break;
		
		case  Turn_right:
			#ifdef test
			printf("����  R=%d" ,Carsta.R);
			#endif
			A_control(1,SPEED);
			B_control(0,SPEED);
    		break;
    	default:
    		break;
    }
}


/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
//void isLine(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
//{
//	Last=0;
//	This=0;
//	This=HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
//	if ((Last!=This)&&This==1)
//	{
//		printf("����");
//	}
//	if ((Last!=This)&&This==0)
//	{
//		printf("����");
//	}
//	Last=This;
//}


/**
  * ��������: ʹС���ƶ����̶�����
  * �������: Ŀ�������
  * �� �� ֵ: ״ֵ̬
  * ˵    ������
  */
//uint16_t ToPoint(Coordinate *Pxy)
//{
//	int Move_x,Move_y;
//	Move_x=Pxy->X-CarPoint.X;
//	Move_y=Pxy->Y-CarPoint.Y;
//	
//	if(Move_x>0)
//	{
//		if(Move_y>0)
//		{
//			Car_control(Turn_rightFront,pCarsta);
//		}
//		
//		else if(Move_y==0)
//		{
//			Car_control(Turn_right,pCarsta);
//		}
//		else
//		{
//			Car_control(Turn_rightBack,pCarsta);
//		}
//	}
//	
//	else if(Move_x==0)
//	{
//		if(Move_y>0)
//		{
//			Car_control(Turn_front,pCarsta);
//		}
//		
//		else if(Move_y==0)
//		{
//			return 1;
//		}
//		
//		else
//		{
//			Car_control(Turn_back,pCarsta);
//		}
//	}
//	
//	else
//	{
//		if(Move_y>0)
//		{
//			Car_control(Turn_leftFront,pCarsta);
//		}
//		
//		else if(Move_y==0)
//		{
//			Car_control(Turn_left,pCarsta);
//		}

//		else
//		{
//			Car_control(Turn_leftBack,pCarsta);

//		}
//	}
//}

/**
  * ��������: ֹͣ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */

void Stop(CarState* Carsta)
{
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	Carsta->B=0;
	Carsta->L=0;
	Carsta->F=0;
	Carsta->R=0;
	#ifdef test 
	printf("�յ�");
	#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t Flag=1;
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_Delay(1000);
HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
HAL_Delay(1000);
HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
HAL_Delay(1000);
Car_control(Turn_front,pCarsta);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  
//	  HAL_Delay(1500);
//	 Car_To(Turn_right);
//	  
//	  HAL_Delay(3000);
//	  Car_To((Turn_back));
//	  
//	  HAL_Delay(500);
//	  Car_To((Turn_right));
//	  
//	  HAL_Delay(3000);
//	  Car_To((Turn_back));
//	  
//	  HAL_Delay(500);
//	  Car_To((Turn_right));
//	 HAL_Delay(3000);
//	  Car_To(Turn_front);
//	  
	  
	  
	  
	  
//	This=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
//	if ((Last!=This)&&This==1)
//	{
//		Enter=1;
//	}
//	else if ((Last!=This)&&This==0)
//	{
//		Exit=1;
//	}
//	Last=This;
	  //�����߼�����
	  #ifdef test
	  printf("start test \n");
	  #endif
//	  Car_control(Turn_back,pCarsta);
//	  Car_control(Turn_left,pCarsta);
	  //��ȡ��·����״̬ �ж���������Ӧ���Ƿ��ǵ��ϵ���
	  
//A��������
////	  BackLt=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
//	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==1)
//	  {
//		  isBackLine++;
//	  }
////	  LeftLt=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
//	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==1)
//	  {
//		  isLeftLine++;
//	  }
////	  RightLt=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
//	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==1)
//	  {
//		  isRightLine++;
//	  }	  
////	  FrontLt=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
//	  if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==1)
//	  {
//		  isFrontLine++;
//	  }
//A��������
	  
////�����Լ�������  A����
//	  if(isRightLine>COUNT)
//	  {
//		  isRightLine=0;
//		  CarPoint.X+=Carsta.R;
//		  CarPoint.X-=Carsta.L;
//	  }
//	  
//	  if(isLeftLine>COUNT)
//	  {
//		  isLeftLine=0;
//		  CarPoint.X-=Carsta.L;
//		  CarPoint.X+=Carsta.R;
//	  }
//	  
//	  if(isFrontLine>COUNT)
//	  {
//		  isFrontLine=0;
//		  CarPoint.Y+=Carsta.F;
//		  CarPoint.Y-=Carsta.B;
//	  }
//	  
//	  if(isBackLine>COUNT)
//	  {
//		  isBackLine=0;
//		  CarPoint.Y-=Carsta.B;
//		  CarPoint.Y+=Carsta.F;
//	  }
////A����

//B��������
//��
	BThis=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
	if ((BLast!=BThis)&&BThis==1)
	{
		BEnter=1;
	}
	else if ((BLast!=BThis)&&BThis==0)
	{
		BExit=1;
	}
	BLast=BThis;
//ǰ
	FThis=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
	if ((FLast!=FThis)&&FThis==1)
	{
		FEnter=1;
	}
	else if ((FLast!=FThis)&&FThis==0)
	{
		FExit=1;
	}
	FLast=FThis;
//��
	LThis=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4);
	if ((LLast!=LThis)&&LThis==1)
	{
		LEnter=1;
	}
	else if ((LLast!=LThis)&&LThis==0)
	{
		LExit=1;
	}
	LLast=LThis;
//��
	RThis=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
	if ((RLast!=RThis)&&RThis==1)
	{
		FEnter=1;
	}
	else if ((RLast!=RThis)&&RThis==0)
	{
		FExit=1;
	}
	RLast=RThis;
//B ���������
	
	
//�����Լ�������  B����
	  if(REnter&&RExit)
	  {
		  REnter=0;
		  RExit=0;
		  CarPoint.X+=Carsta.R;
		  CarPoint.X-=Carsta.L;
	  }
	  
	  if(LEnter&&LExit)
	  {
		  LEnter=0;
		  LExit=0;
		  
		  CarPoint.X-=Carsta.L;
		  CarPoint.X+=Carsta.R;
	  }
	  
	  if(FEnter&&FExit)
	  {
		  FEnter=0;
		  FExit=0;
		  
		  CarPoint.Y+=Carsta.F;
		  CarPoint.Y-=Carsta.B;
	  }
	  
	  if(BEnter&&BExit)
	  {
		  BEnter=0;
		  BExit=0;
		  
		  CarPoint.Y-=Carsta.B;
		  CarPoint.Y+=Carsta.F;
	  }
//B����
	  
	  
//ָʾ�ƿ���
	  if ((CarPoint.Y==8&&CarPoint.X==0)||(CarPoint.Y==12&&CarPoint.X==12)||(CarPoint.Y==24&&CarPoint.X==6)||(CarPoint.Y==28&&CarPoint.X==14))
	  {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
	  }
	  
//����ִ��
	  if (CarPoint.X==0&&CarPoint.Y==12)
		Car_control(Turn_right,pCarsta);
	  if ((CarPoint.X==11||CarPoint.X==12)&&(CarPoint.Y==11||CarPoint.Y==10))
		Car_control(Turn_front,pCarsta);
	  if(((CarPoint.X==10||CarPoint.X==11||CarPoint.X==12)&&CarPoint.Y==24)&&Flag)
		Car_control(Turn_left,pCarsta);
	  if(CarPoint.X==6&&CarPoint.Y==24)
		{
		  Flag=0;
		  Car_control(Turn_right,pCarsta);
		}
	  if(CarPoint.X==14&&CarPoint.Y==24)
		  Car_control(Turn_front,pCarsta);
	  if(CarPoint.X==14&&CarPoint.Y==28)
		  Stop(pCarsta);
	  HAL_Delay(10);
	  
	  
//С������
	  //ǰ���� ����
	  if(!(CarPoint.Y%4))
	  {
		if ((Carsta.F||Carsta.B)&&(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)||HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)))
		{
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))
		{
			Car_To(Turn_left);
		}
		else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))
		{
			Car_To(Turn_right);			  
		}
		HAL_Delay(FBcorrectTime);
		//����ʱ��
		if(Carsta.B==1)
		{
			Car_To(Turn_back);
		}
		else if(Carsta.F==1)
		{
			Car_To(Turn_front);
		}
	  }
	  }
	  
	  //������ ���� 
	  if(!(CarPoint.X%2))
	  {
		if ((Carsta.L||Carsta.R)&&(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)||HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)))
	  {
		  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6))
		  {
			  Car_To(Turn_back);
		  }
		  else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3))
		  {
			  Car_To(Turn_front);			  
		  }
		  HAL_Delay(LRcorrectTime);
		  //����ʱ��
		  if(Carsta.L==1)
		  {
			  Car_To(Turn_left);
		  }
		  else if(Carsta.R==1)
		  {
			  Car_To(Turn_right);
		  }
	  }
  
	  }
	  
//	  if ((CarPoint.X==0&&CarPoint.Y==2)||(CarPoint.X==6&&CarPoint.Y==2)||(CarPoint.X==6&&CarPoint.Y==6)
//		||(CarPoint.X==4&&CarPoint.Y==6)||(CarPoint.X==7&&CarPoint.Y==6)
//	  )
//	  {
//		  To++;
//	  }
//	  switch (To)
//      {
//      	case 1:
//			Car_control(Turn_right,pCarsta);
//      		break;
//      	case 2:
//			Car_control(Turn_front,pCarsta);
//      		break;
//      	case 3:
//			Car_control(Turn_left,pCarsta);
//      		break;
//		case 4:
//			Car_control(Turn_right,pCarsta);
//			break;
//		case 5:
//			Car_control(Turn_front,pCarsta);
//      		break;
//		case 6:
//			if (CarPoint.X==7&&CarPoint.Y==7)
//			{
//				Stop();
//			}
//			break;
//      }
		  
	  
	  
	  
	  
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  #ifdef test
	  printf("FrontLt %d |",FrontLt);
	  printf("RightLt %d |",RightLt);
	  printf("LeftLt  %d |",LeftLt);
	  printf("BackLt  %d |",BackLt);
//	  Car_control(Turn_back,pCarsta);
//	  Car_control(Turn_left,pCarsta);
	  printf("CarPoint.X %d  CarPoint.y %d\n",CarPoint.X,CarPoint.Y);
	  printf("State L---%d F---%d B---%d R---%d ",Carsta.L,Carsta.F,Carsta.B,Carsta.R);
//	  Car_control(Turn_front,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_back,pCarsta);//left
//	  HAL_Delay(1000);
//	  Car_control(Turn_left,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_right,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_rightBack,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_leftBack,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_rightFront,pCarsta);
//	  HAL_Delay(1000);
//	  Car_control(Turn_leftFront,pCarsta);


	  #endif
	  
//	  HAL_Delay(100);
	  
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
