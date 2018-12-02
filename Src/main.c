/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "FindRoad.h"
#include "decodemessage.h"
#include "utils.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN PV */
//read wifi
uint8_t receive[1];
uint8_t temptext[150];
volatile uint8_t rawtext[70];
int current=0;
volatile uint8_t refreshed=0;
//read BT
uint8_t btreceive[1];
volatile uint8_t bluetooth=0;
//timer for 6050
volatile uint8_t TimeUp=0;
//findroad result
int routestep;
int currentstep;
int destx,desty;
//current situation
double current_angle=180;
uint8_t isA;
uint8_t fre=10;  
volatile int leftspeed=0,rightspeed=0;
int idlespeed=0;
float lefterror[3]={0,0,0},righterror[3]={0,0,0};
int leftpwm=0,rightpwm=0;
float lastdis,thisdis=1000;
int reachsituation=0;
int lastx=0,lasty=0;
int thisx=0,thisy=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN PFP */

double turntoangle(int thisx,int thisy,int lastx,int lasty){
	if(thisx == lastx){
		if(thisy>lasty) return 90;
		else return 270;
	}
	else{
		double temptan=1.0*(thisy-lasty)/(thisx-lastx);
		double tempangle=atan(temptan)*180/3.1416;
		if(thisx>lastx){
			if(tempangle>0) return tempangle;
			else return tempangle+360;
		}
		else{
			return tempangle+180;
		}
	}
}

void go(int l,int r){
	if(r>0){
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	}
	else if(r<0){
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,-r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	else if(r==0){
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,-r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-r);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	if(l>0){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
	}
	else if(l<0){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,-l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	else if(l==0){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,-l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,-l);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	}
}

/*-----MPU Part Start-----*/
#define	SMPLRT_DIV		0x19	
#define	CONFIG			0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B  
#define	WHO_AM_I		0x75   
#define SlaveAddress 0xD0 

#define FS_SEL_2000 0x18
#define FS_SEL_1000 0x10
#define FS_SEL_500 0x8
#define FS_SEL_250 0x0
#define FS_SEL FS_SEL_2000
#define GYRO_SCALE_RANGE 2000

int16_t data; 
float angle_speed; 
float angle=0.0; 
float gyro_z_offset=0.0; 

void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_Data)
{
	HAL_I2C_Mem_Write(&hi2c1, SlaveAddress, REG_Address ,1 ,&REG_Data, 1, 1000);
}

uint8_t Single_ReadI2C(uint8_t REG_Address) 
{
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, SlaveAddress, REG_Address ,1 ,&data, 1, 1000);
	return data;
}

uint16_t Get_MPU_Data(uint8_t REG_Address) 
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return ((uint16_t)H)*256+L;
}

void InitMPU6050()
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, FS_SEL);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);
	
	int i;
	int16_t _data;
	HAL_Delay(1000); 
	for(i=1;i<=10;i++)
	{
		_data=Get_MPU_Data(GYRO_ZOUT_H);
		gyro_z_offset+=_data;
		HAL_Delay(100); 
	}
	gyro_z_offset/=10;
	printf("offset:%f",gyro_z_offset);
}
/*-----MPU Part end-----*/

double __angle;
//turning left
void turnleft(double destangle){
	destangle-=1.5;
	//-----code changed-----
	//printf("turning left %f\n",destangle);
	go(-80,80);
	double tempangle=0;
	while(tempangle<=0.5*destangle && tempangle>=-0.5*destangle){
		if (TimeUp==1){
			data=Get_MPU_Data(GYRO_ZOUT_H);
			angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
			tempangle+=angle_speed*0.01;
			__angle+=angle_speed*0.01;
			TimeUp=0;
		}
	}
	go(-75,75);
	while(tempangle<=destangle && tempangle>=-1*destangle){
		if (TimeUp==1){
			data=Get_MPU_Data(GYRO_ZOUT_H);
			angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
			tempangle+=angle_speed*0.01;
			__angle+=angle_speed*0.01;
			//printf("%lf;%f\n",tempangle,angle_speed);
			TimeUp=0;
		}
	}
	go(80,-80);
	HAL_Delay(10);
	go(0,0);
}

void turnright(double destangle){
	destangle-=1.5;
	//-----code changed-----
	//printf("turning right %f\n",destangle);
	go(80,-80);
	double tempangle=0;
	while(tempangle<=0.5*destangle && tempangle>=-0.5*destangle){
		if (TimeUp==1){
			data=Get_MPU_Data(GYRO_ZOUT_H);
			angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
			tempangle+=angle_speed*0.01;
			
			TimeUp=0;
			//printf("angle=%lf\n",tempangle);
		}
	}
	go(75,-75);
	while(tempangle<=destangle && tempangle>=-1*destangle){
		if (TimeUp==1){
			data=Get_MPU_Data(GYRO_ZOUT_H);
			angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
			tempangle+=angle_speed*0.01;
			//printf("%lf;%f\n",tempangle,angle_speed);
			TimeUp=0;
		}
	}
	go(-80,80);
	HAL_Delay(10);
	go(0,0);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN 2 */
	setInitHandle(&huart3);
	uint8_t a[]="--Starting Initializing--\r\n";
	printf("%s",a); 
	InitMPU6050();
	printf("mpu6050 initializd");
	uint8_t isA=(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)?0:1);
	isA=1;
	current_angle=270;
	
	
	printf("AT\r\n");
	HAL_UART_Transmit(&huart2,"AT\r\n",4,100);
	HAL_Delay(3000);
	
	printf("AT+CWMODE=3\r\n");
	HAL_UART_Transmit(&huart2,"AT+CWMODE=3\r\n",13,100);
	HAL_Delay(3000);
  
	printf("AT+RST\r\n");
	HAL_UART_Transmit(&huart2,"AT+RST\r\n",8,100);
	HAL_Delay(3000);
	
	printf("AT+CWJAP=\"EDC20\",\"12345678\"\r\n");
	HAL_UART_Transmit(&huart2,"AT+CWJAP=\"EDC20\",\"12345678\"\r\n",30,100);
	HAL_Delay(5000);

	printf("AT+CIPSTART=\"TCP\",\"192.168.1.100\",20000\r\n");
	HAL_UART_Transmit(&huart2,"AT+CIPSTART=\"TCP\",\"192.168.1.100\",20000\r\n",41,100);
	HAL_Delay(2000);
	
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_UART_Receive_IT(&huart2,receive,1);
	HAL_UART_Receive_IT(&huart3,btreceive,1);
	MessageInfo* message=(MessageInfo*)malloc(sizeof(MessageInfo));
	//initialize position
	PosList route;
	while(1){
		if (refreshed==1){			
			msgrefresh((char*)rawtext,message,isA);
			printf("isA=%d",isA);
			printf("\nmy_x=%d my_y=%d passengerNum=%d\n",
			message->my_x,message->my_y,message->passengerNum);
			thisx=message->my_x;
			thisy=message->my_y;
			thisdis=sqrt((message->my_x-destx)*(message->my_x-destx)*1.0+(message->my_y-desty)*(message->my_y-desty)*1.0);
			refreshed=0;
			route=GetPosListWithAngle(*message, 0);
			//
			print_pos_list(route);
			break;
		}
	}
	
	printf("initialization finished\n");
	int reachsituation=0;
	/*
	int testroute[36]={68,185,30,185,35,235,69,240,105,205,141,169,169,141,240,69,240,40,230,30,185,30,125,30,90,50,84,71,71,84,50,90,30,25,30,185};
	routestep=18;
	currentstep=0;
	currentstep++;
	destx=testroute[currentstep*2];
	desty=testroute[currentstep*2+1];
	printf("destx=%d,desty=%d\n",destx,desty);
	*/
	currentstep=0;
	destx=route.data[currentstep].x;
	desty=route.data[currentstep].y;
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/*
	__angle=0.0;
		while(1)
	{
		turnleft(88.5);
		int i=0;
		while(1)
		{
			if (TimeUp==1){
			data=Get_MPU_Data(GYRO_ZOUT_H);
			angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
			__angle+=angle_speed*0.01;
			TimeUp=0;
			i++;
			}
			if(i%50==0) printf("%lf;%f\n",__angle,angle_speed);
			if(200==i)break;
		}
	}
	*/
		
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN 3 */
		if (refreshed==1){			
			msgrefresh((char*)rawtext,message,isA);
			//printf("isA=%d",isA);
			//printf("\nmy_x=%d my_y=%d passengerNum=%d\n",
			//message->my_x,message->my_y,message->passengerNum);
			/*
			for (int i=0;i<message->passengerNum;i++){
				printf("pass_status=%hhd xs_pos=%d ys_pos=%d xe_pos=%d ye_pos=%d\n",
				message->pass_status[i],message->xs_pos[i],message->ys_pos[i],message->xe_pos[i],message->ye_pos[i]);
			}
			*/
			refreshed=0;
			//go straight
			if(reachsituation==0){
				go(50,50);
				lastdis=thisdis;
				thisdis=sqrt((message->my_x-destx)*(message->my_x-destx)*1.0+(message->my_y-desty)*(message->my_y-desty)*1.0);
				//printf("dis=%f\n",thisdis);
				if(lastdis<thisdis && thisdis<10){
					go(0,0);
					//printf("reach dest:%d,%d\n",destx,desty);
					reachsituation=1;
					currentstep++;
					if(currentstep<route.num){			
						destx=route.data[currentstep].x;
						desty=route.data[currentstep].y;
					}
					else{
						deletePosList(&route);
						route = GetPosListWithAngle(*message, 0);
						print_pos_list(route);
						currentstep=0;
					}
					//printf("next dest:%d,%d\n",destx,desty);
					HAL_Delay(500);
				}
				else{
					//printf("going\n");
					go(50,50); //go straight ***pid***
				}
			}
			//turning
			else if(reachsituation==1){
				reachsituation=0;
				lastx=thisx;
				lasty=thisy;
				thisx=message->my_x;
				thisy=message->my_y;
				current_angle=turntoangle(thisx,thisy,lastx,lasty);
				double destangle=turntoangle(destx,desty,thisx,thisy);
				destangle=destangle-current_angle;
				if(destangle>180) destangle-=360;
				if(destangle<-180) destangle+=360;
				//printf("turn left:%f\n",destangle);
				if(destangle>0){
					//if(destangle<60)
					turnleft(destangle);
				}
				else{
					//if(destangle>-60)
					turnright(-1*destangle);
				}
				go(0,0);
				//printf("ready to go straight\n");
				HAL_Delay(500);
				//
				go(50,50);
				HAL_Delay(100);
				//
			}
		}//end if situation =1/0
  }//end main while
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 3;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//////////////////////////////////////////////////////////////////////////////////* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if(huart->Instance==USART3){
		refreshed=1;
		bluetooth=btreceive[0]-48;
		printf("%d",bluetooth);
		HAL_UART_Receive_IT(&huart3,btreceive,1);
	}
	if (huart->Instance==USART2){
		temptext[current]=receive[0];
		current++;
		if (temptext[current-1]==0x0A && temptext[current-2]==0x0D && current>=64){
			for(int i=0;i<64;i++){
				rawtext[i]=temptext[current-64+i];
			}
		  current=0;
			refreshed=1;
		}
		HAL_UART_Receive_IT(&huart2,receive,1);
	}
	if (huart->Instance==USART1){
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM1){
		rightspeed = (int)(__HAL_TIM_GET_COUNTER(&htim4))*fre/8;
		leftspeed = (int)(__HAL_TIM_GET_COUNTER(&htim5))*fre/8;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		__HAL_TIM_SET_COUNTER(&htim5, 0);	
	}
	if (htim->Instance==TIM8){
		TimeUp=1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
