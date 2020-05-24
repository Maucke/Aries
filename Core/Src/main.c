/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
#include "math.h"
#include "config.h"
#include "oled.h"
#include "ds3231.h"
//#include "mpu6050.h"
#include "flash.h"
#include "stdio.h"
#include "vfd.h"	 
//#include "apds9960.h"
#include "sk6812.h"	 
#include "remote.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLEEPTIME    20*60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t Flag_Refrash = False;
uint8_t Flag_Blink = False;
uint8_t Flag_Sleep = False;
uint8_t Flag_Continue = False;
uint8_t Flag_Reception = True;
uint8_t OfflineCount = 10;
uint8_t SystemActive = False;

char Display_Time[6]="--:--";
char Display_State[5] = "----";
char Display_Notification[6]="V1.00";

uint8_t Remote_Value;
uint8_t RandomP = 6;
uint8_t RandomPosX = 30,RandomPosY = 1;
u8 BTVPostionX = 1;//124
u8 BTVPostionY = 5;//30

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#define TrumHeight   64
#define TrumWidth    1
#define TrumInterval 1
#define TrumNum	   	 128

uint8_t fall_pot[250]={0};	//记录下落点的坐标
uint8_t flow_pot[250]={0};

u16 Display_Mode = MODE_OFFLINE;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void OLED_Boot(void);
void TIME_Mode(void);

u8 BootGRAM[304*48];

int cont_str(char *s)
{
	int i = 0;      
	while ( s[++i] != '\0')   ;
	return i-1;
}
float temp;
void temp_data(void)
{
	float Rt=0;
	float Rp=10000;
	float T2=273.15+25;
	float Bx=3435;
	float Ka=273.15;
	float vol=0;
	//   vol=(float)((Get_Adc_Average(ADC_Channel_5,10))*(3.3/4096));
	//   Rt=(3.3-vol)*10000/vol;
	Rt = 22050;
	temp=1/(1/T2+log(Rt/Rp)/Bx)-Ka+0.5;
	printf("Temp is %f",temp);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1,Uart_Recv1_Buf,Uart_Max_Length);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim8);
	
	OLED_AllMotion_Init();
	InitData();
	sprintf(Device_Str.vcputemp,"---#");	
	sprintf(Device_Str.vgputemp,"---#");	
	printf("Flash ID is 0x%X",SPI_Flash_ReadID());
	HAL_UART_Receive_IT(&huart3,&Uart_Recv3_Data,1);
	Update_Pos();

	SPI_Flash_Init();
//	Recvcmd();
	printf("Init OK\r\n");
	if(Device_Cmd.commandset&0x2)
	{
		OLED_Boot();
	}
	while(1){ OLED_Boot();temp_data();}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
	{
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch(Display_Mode)
		{
//#if GEMINI == 1
			case MODE_DEFALUT:DATE_Mode();break;
			case MODE_DATE:DATE_Mode();break;
//#else
//			case MODE_DEFALUT:TIME_Mode();break;
//			case MODE_DATE:TIME_Mode();break;
//#endif
			case MODE_NORMAL:NORMAL_Mode();break;
			case MODE_GAME:GAME_Mode();break;
			case MODE_OFFLINE:DATE_Mode();break;
			case MODE_SLEEP:SLEEP_Mode();break;
			case MODE_MUSIC:MUSIC_Mode();break;
			default:Display_Mode = MODE_DEFALUT;break;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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

void OLED_Boot(void)
{
	int i;
	for(i=179;i>0;i--)
	{
		SystemActive = False;
		SPI_Flash_Read(BootGRAM,i*256*32,256*32);   
		OLED_Clear();
		OLED_Full(BootGRAM);
		OLED_Refresh_Gram();
		SystemActive = True;
	}
#if GEMINI == 1
	
	int i;
	for(i=0;i<56;i++)
	{
		SystemActive = False;
		SPI_Flash_Read(BootGRAM,i*304*48,304*48);   
		OLED_Clear();
		OLED_Full(BootGRAM);
		OLED_Refresh_Gram();
		SystemActive = True;
	}
	HAL_Delay(100);
	for(i=0;i<96;i+=8)
	{
		OLED_Clear();
		OLED_Full(BootGRAM);
		OLED_Refresh_Gram();
	}
#elif MARKC == 1
	int i;
	for(i=0;i<90;i++)
	{
		SPI_Flash_Read(BootGRAM,i*304*12,304*12);   
		OLED_Clear();
		OLED_Convert(0,4,152,BootGRAM);
		OLED_Refresh_Gram();
	}
	HAL_Delay(100);
	for(i=4;i<96;i+=8)
	{
		OLED_Clear();
		OLED_Convert(0,i,152,BootGRAM);
		OLED_Refresh_Gram();
	}
#endif
}




void Display_Style3(void)
{
	uint16_t i = 0;
	uint16_t j = 0;
	int Temp = 0;
	uint16_t Index;
	static float BackFlag[100]={0};
	/*******************显示*******************/
	for(i = 0; i < 76; i++)	
	{
		Index = (float)i*180/TrumNum;
		Temp = TrumHeight - (u16)(Device_Msg.fft[Index]/2.5f);
		if(Temp < 4)
			Temp = 4;
		if(flow_pot[i]+6 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i]+2 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i] < Temp)
			flow_pot[i] += 1;
		
		if(flow_pot[i] > (Temp+6))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > (Temp+2))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp)
			flow_pot[i] -= 1;
		
		if(fall_pot[i]+3 > flow_pot[i]) 
		{
			fall_pot[i] = flow_pot[i]-3;
			BackFlag[i] = 0;
		}
		else if(fall_pot[i]+3 < flow_pot[i]) 
		{
			if(!BackFlag[i])
			{
				if(fall_pot[i]>=4)
					fall_pot[i] -= 1.5f;
				else if(fall_pot[i])
					BackFlag[i] = 1;
			}
			else
				fall_pot[i] += 2;
		}
		
		GUI_RLine(4*i+0,fall_pot[i],fall_pot[i]+1,4);
		GUI_RLine(4*i+1,fall_pot[i],fall_pot[i]+1,8);
		GUI_RLine(4*i+2,fall_pot[i],fall_pot[i]+1,4);
		
		GUI_RLine(4*i+0,flow_pot[i],TrumHeight-1,10);
		GUI_RLine(4*i+1,flow_pot[i],TrumHeight-1,15);
		GUI_RLine(4*i+2,flow_pot[i],TrumHeight-1,10);	
		for(j=TrumHeight-1;j>=flow_pot[i];j-=2)
			GUI_HLine(4*i+0,j,4*i+2,0);
	}
}


void Display_Style2(void)
{
	uint16_t i = 0;
	int Temp = 0;
	uint16_t Index;
	/*******************显示*******************/
	for(i = 0; i < 102; i++)	
	{
		Index = (float)i*180/100;
		Temp = 48 - (u16)(Device_Msg.fft[Index]/4);
		if(Temp < 2)
			Temp = 2;
//		else if(Temp<4) Temp = 4;
		
		if(flow_pot[i]+6 < Temp)
			flow_pot[i] += 3;
		else if(flow_pot[i]+2 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i]+1 < Temp)
			flow_pot[i] += 1;
		
		if(flow_pot[i] > (Temp+6))
			flow_pot[i] -= 3;
		else if(flow_pot[i] > (Temp+2))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp+1)
			flow_pot[i] -= 1;
		
		if(fall_pot[i]+3 > flow_pot[i]) 
		{
			fall_pot[i] = flow_pot[i]-3;
		}
		else if(fall_pot[i]+3 < flow_pot[i]) 
		{
			fall_pot[i] ++;
		}
		else
			fall_pot[i] = flow_pot[i]-3;
		GUI_RLine(3*i,flow_pot[i],47,15);
		GUI_RLine(3*i+1,flow_pot[i],47,15);
		GUI_RLine(3*i,(47-flow_pot[i])/3+47,46,5);
		GUI_RLine(3*i+1,(47-flow_pot[i])/3+47,46,5);
		
		GUI_RLine(3*i,fall_pot[i]+1,fall_pot[i]+1,6);
		GUI_RLine(3*i+1,fall_pot[i]+1,fall_pot[i]+1,6);
	}
}

void Display_Style1(void)
{
	uint16_t i = 0;
	int Temp = 0;
	uint16_t Index;
	/*******************显示*******************/
	for(i = 0; i < TrumNum; i++)	
	{
		Index = (float)i*180/TrumNum;
		Temp = TrumHeight - (u16)(Device_Msg.fft[Index]/2.5f);
		if(Temp < 2)
			Temp = 2;
//		else if(Temp<4) Temp = 4;
		
		if(flow_pot[i]+6 < Temp)
			flow_pot[i] += 4;
		else if(flow_pot[i]+2 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i]+1 < Temp)
			flow_pot[i] += 1;
		
		if(flow_pot[i] > (Temp+6))
			flow_pot[i] -= 4;
		else if(flow_pot[i] > (Temp+2))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp+1)
			flow_pot[i] -= 1;
		
		if(fall_pot[i]+3 > flow_pot[i]) 
		{
			fall_pot[i] = flow_pot[i]-3;
		}
		else if(fall_pot[i]+3 < flow_pot[i]) 
		{
			fall_pot[i] ++;
		}
		GUI_RLine((TrumWidth+TrumInterval)*i,flow_pot[i],TrumHeight-1,15);
		GUI_RLine((TrumWidth+TrumInterval)*i,fall_pot[i]+1,fall_pot[i]+1,6);
	}
}

u16 SampPoint[4][192*2];

void Display_Style4()
{

	uint16_t i = 0;
	int Temp = 0;
	uint16_t Index;
	/*******************显示*******************/
	for(i = 0; i < 102; i++)	
	{
		Index = (float)i*180/100;
		Temp = TrumHeight - (u16)(Device_Msg.fft[Index]/2.5f);
		if(Temp < 2)
			Temp = 2;
		if(flow_pot[i]+10 < Temp)
			flow_pot[i] += 4;
		else if(flow_pot[i]+3 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i] < Temp)
			flow_pot[i] += 1;
		if(flow_pot[i] > (Temp+10))
			flow_pot[i] -= 4;
		else if(flow_pot[i] > (Temp+3))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp)
			flow_pot[i] -= 1;
		
//		GUI_Line(3*i,flow_pot[i],3*i+3,flow_pot[i+1],13);
		SampPoint[0][i*2] = 3*i;
		SampPoint[0][i*2+1] = flow_pot[i];
		GUI_RLine(3*i,flow_pot[i],TrumHeight-1,3);
		SampPoint[1][i*2] = 3*i;
		SampPoint[1][i*2+1] = flow_pot[i]-1;
	}
	GUI_LineS(SampPoint[0],102,13);
	GUI_LineS(SampPoint[1],102,9);
}

void Display_Style6()
{
	uint16_t i = 0;
	int Temp = 0;
	uint16_t Index;
	/*******************显示*******************/
	for(i = 0; i < 152; i++)	
	{
		Index = (float)i*180/152;
		Temp = 32 - (u16)(Device_Msg.fft[Index]/5);
		if(Temp < 2)
			Temp = 2;
		if(flow_pot[i]+10 < Temp)
			flow_pot[i] += 4;
		else if(flow_pot[i]+3 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i] < Temp)
			flow_pot[i] += 1;
		if(flow_pot[i] > (Temp+10))
			flow_pot[i] -= 4;
		else if(flow_pot[i] > (Temp+3))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp)
			flow_pot[i] -= 1;
		
//		GUI_Line(3*i,flow_pot[i],3*i+3,flow_pot[i+1],13);
		if(i%2)
		{
			SampPoint[0][i*2] = 2*i;
			SampPoint[0][i*2+1] = flow_pot[i];
			SampPoint[1][i*2] = 2*i;
			SampPoint[1][i*2+1] = flow_pot[i]-1;
			
			SampPoint[2][i/2*2] = 2*i;
			SampPoint[2][i/2*2+1] = flow_pot[i];
			
		}
		else
		{
			SampPoint[0][i*2] = 2*i;
			SampPoint[0][i*2+1] = - flow_pot[i] + 64;
			SampPoint[1][i*2] = 2*i;
			SampPoint[1][i*2+1] = - flow_pot[i] + 64-1;
			
			SampPoint[3][i/2*2] = 2*i;
			SampPoint[3][i/2*2+1] = - flow_pot[i] + 64;
		}
			
		
//		GUI_RLine(3*i,SampPoint[0][i*2+1],SampPoint[1][i*2+1],5);
		
	}
	GUI_LineS(SampPoint[2],76,5);
	GUI_LineS(SampPoint[3],76,5);
	
	GUI_LineS(SampPoint[0],152,13);
	GUI_LineS(SampPoint[1],152,13);
}

void Display_Style5()
{

	uint16_t i = 0;
	int Temp = 0;
	uint16_t Index;
	/*******************显示*******************/
	for(i = 0; i < 102; i++)	
	{
		Index = (float)i*180/100;
		Temp = 32 - (u16)(Device_Msg.fft[Index]/5);
		if(Temp < 2)
			Temp = 2;
		if(flow_pot[i]+10 < Temp)
			flow_pot[i] += 3;
		else if(flow_pot[i]+3 < Temp)
			flow_pot[i] += 2;
		else if(flow_pot[i] < Temp)
			flow_pot[i] += 1;
		if(flow_pot[i] > (Temp+10))
			flow_pot[i] -= 3;
		else if(flow_pot[i] > (Temp+3))
			flow_pot[i] -= 2;
		else if(flow_pot[i] > Temp)
			flow_pot[i] -= 1;
		
//		GUI_Line(3*i,flow_pot[i],3*i+3,flow_pot[i+1],13);
		SampPoint[0][i*2] = 3*i;
		SampPoint[0][i*2+1] = flow_pot[i];
		SampPoint[1][i*2] = 3*i;
		SampPoint[1][i*2+1] = - flow_pot[i] + 64;
		
		GUI_RLine(3*i,SampPoint[0][i*2+1],SampPoint[1][i*2+1],5);
		
	}
	GUI_LineS(SampPoint[0],102,13);
	GUI_LineS(SampPoint[1],102,13);
}

void MUSICUI_Init()
{
	u8 i;
	for(i=0;i<TrumNum;i++)
	{
		fall_pot[i] = 63;
		flow_pot[i] = 63;
	}
	
	Flag_Reception = False;
	ClearFFT();
	for(i=0;i<15;)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			i++;
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			switch(Device_Cmd.commandstyle)
			{
				case 1:Display_Style1();break;
				case 2:Display_Style2();break;
				case 3:Display_Style3();break;
				case 4:Display_Style4();break;
				case 5:Display_Style5();break;
				case 6:Display_Style6();break;
				default:Display_Style1();break;
			}
			
			OLED_Refresh_Gram();
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	Flag_Reception = True;
	OLED_Clear();
}

void MUSICUI_Out()
{
	u8 i;
	
	Flag_Reception = False;
	ClearFFT();
	for(i=0;i<30;)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			i++;
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			switch(Device_Cmd.commandstyle)
			{
				case 1:Display_Style1();break;
				case 2:Display_Style2();break;
				case 3:Display_Style3();break;
				case 4:Display_Style4();break;
				case 5:Display_Style5();break;
				case 6:Display_Style6();break;
				default:Display_Style1();break;
			}
			
			OLED_Refresh_Gram();
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	Flag_Reception = True;
	OLED_Clear();
}



void MUSIC_Mode(void)
{
	MUSICUI_Init();
	while(Display_Mode == MODE_MUSIC)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			if(Flag_Sleep)
			{
				Flag_Sleep = False;
				Display_Mode = MODE_SLEEP;
				break;
			}
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			switch(Device_Cmd.commandstyle)
			{
				case 1:Display_Style1();break;
				case 2:Display_Style2();break;
				case 3:Display_Style3();break;
				case 4:Display_Style4();break;
				case 5:Display_Style5();break;
				case 6:Display_Style6();break;
				default:Display_Style1();break;
			}
			
			OLED_Refresh_Gram();
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	MUSICUI_Out();
}

void Judge_Notification(void)
{
//	if(!Clock_Msg.aiwork)
//	{
		sprintf(Display_Notification,"%5s",ds3231.Time);
		if(Flag_Blink)
			Display_Notification[2] = ' '; 
//	}
//	else if(Clock_Msg.aiwork)
//		sprintf(Display_Notification,"XIOAI");
}



typedef struct
{
	u16 MonLen ;
	u16 WeekLen ;
	u16 LineLen;
	u8 MonBtWeek;
}TIMED;

TIMED timed;

void CalcTIMED(char *Mon,char *Week)
{
	timed.MonLen = cont_str(Mon)*5;
	timed.WeekLen = cont_str(Week)*6;
	if(timed.WeekLen>timed.MonLen)
	{
		timed.MonLen = timed.WeekLen;
		timed.MonBtWeek = True;
	}
	else
		timed.MonBtWeek = False;
	timed.LineLen = (timed.MonLen+38)*2;
}

u8 VoiceFlag = False;
void DATE_MainDisplay()
{
	OLED_SF22x40(RandomPosX,RandomPosY,ds3231.Day);
	if(timed.MonBtWeek)
	{
		OLED_SF12x24(RandomPosX+33,RandomPosY+15,ds3231.Week);
	}
	else
		OLED_SF12x24(RandomPosX+33+timed.MonLen - timed.WeekLen,RandomPosY+15,ds3231.Week); 
	
	OLED_SF10x16(RandomPosX,RandomPosY+47,ds3231.Mon);
	OLED_SF10x16(RandomPosX+timed.MonLen+18,RandomPosY+47,ds3231.Year);
	GUI_HLine(RandomPosX*2,RandomPosY+44,RandomPosX*2+timed.LineLen,0xF);
	
	if(VoiceFlag)
	{
		SHOW_CORN.BTV = False;
		if(Flow_BTV() == OLED_IDLE)
		{
			if(BTVPostionX>50)
				Motion_VOICE(BTVPostionX-6,15);
			else
				Motion_VOICE(BTVPostionX,15);
		}
	}
	else
		Flow_BTV();
}

void DATEUI_Init()
{
	int i;
	SHOW_CORN.BTV = True;
	CalcTIMED(ds3231.Mon,ds3231.Week);
	for(i = 0;i<20;i++)
	{
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			
			Flow_BTV();
			
			OLED_SF22x40(RandomPosX ,RandomPosY-40+i*2,ds3231.Day);
			if(timed.MonBtWeek)
				OLED_SF12x24(RandomPosX+33,RandomPosY+15-40+i*2,ds3231.Week);
			else
				OLED_SF12x24(RandomPosX+33+timed.MonLen - timed.WeekLen,RandomPosY+15-40+i*2,ds3231.Week); 
			OLED_SF10x16(RandomPosX,RandomPosY+47+40-i*2,ds3231.Mon);
			OLED_SF10x16(RandomPosX+timed.MonLen+18,RandomPosY+47+40-i*2,ds3231.Year);
			
			GUI_HLine(RandomPosX*2,RandomPosY+44,RandomPosX*2+(float)timed.LineLen/(20-i),0xF); //注意
		
		OLED_Refresh_Gram();
	}
}

void DATEUI_Out()
{
	int i;
	float line;
	SHOW_CORN.BTV = False;
	Motion_MovmeteorInit();
	CalcTIMED(ds3231.Mon,ds3231.Week);
	line = timed.LineLen;
	for(i = 19;i>=0;i--)
	{
		OLED_Clear();
		
		OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
		
			Flow_BTV();
		
		OLED_SF22x40(RandomPosX ,RandomPosY-40+i*2,ds3231.Day);
		if(timed.MonBtWeek)
			OLED_SF12x24(RandomPosX+33,RandomPosY+15-40+i*2,ds3231.Week);
		else
			OLED_SF12x24(RandomPosX+33+timed.MonLen - timed.WeekLen,RandomPosY+15-40+i*2,ds3231.Week); 
		OLED_SF10x16(RandomPosX,RandomPosY+47+40-i*2,ds3231.Mon);
		OLED_SF10x16(RandomPosX+timed.MonLen+18,RandomPosY+47+40-i*2,ds3231.Year);
		
		GUI_HLine(RandomPosX*2,RandomPosY+44,RandomPosX*2+line/(20-i),0xF); //注意
		
		OLED_Refresh_Gram();
	}
}


void Update_Pos(void)
{
	if(BTVPostionX > 100)
	{
		BTVPostionX = rand()%4+1;
//		BTVPostionY = rand()%10+20;
		RandomPosX = rand()%4 + 34;
//		RandomPosY = rand()%22 + 10;
		RandomP = rand()%13;
	}
	else
	{
		BTVPostionX = rand()%4+100;
//		BTVPostionY = rand()%10+20;
		RandomPosX = rand()%4 + 10;
//		RandomPosY = rand()%22 + 10;
		RandomP = rand()%13;
	}
}


void DATE_Mode(void)
{
	Update_Pos();
	DATEUI_Init();
	while(Display_Mode == MODE_DATE||Display_Mode == MODE_DEFALUT||Display_Mode == MODE_OFFLINE)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			if(Flag_Sleep)
			{
				Flag_Sleep = False;
				Display_Mode = MODE_SLEEP;
				break;
			}
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			DATE_MainDisplay();
			OLED_Refresh_Gram();


			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	DATEUI_Out();
}

void TIMEUI_Init()
{
	static char str[20];
	int i;sprintf(str,"%s.%4s.%s",ds3231.Year,ds3231.Mon,ds3231.Day);
	for(i = 0;i<20;i++)
	{
		OLED_Clear();
		
		OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
		
		OLED_SF32x46(RandomP+4-10+i/2,31,ds3231.Hour,'B');
		OLED_SF32x46(RandomP+48,32-40+i*2,ds3231.Min,'T');
		GUI_CircleFill(RandomP*2+84,43-40+i*2,2,15);
		GUI_CircleFill(RandomP*2+84,68+20-i,2,15);
		OLED_SF10x16(RandomP+90,39-40+i*2,ds3231.Week);
		
		OLED_SF8x16(RandomP+90+60-i*3,56,str);
		
		GUI_HLine(RandomP*2+180,56,RandomP*2+180+i*5,15);
		
		OLED_Refresh_Gram();
	}
	
}

void TIMEUI_Out()
{
	static char str[20];
	int i;sprintf(str,"%s.%4s.%s",ds3231.Year,ds3231.Mon,ds3231.Day);
	for(i = 20;i>=0;i--)
	{
		OLED_Clear();
		
		OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
		
		OLED_SF32x46(RandomP+4-10+i/2,31,ds3231.Hour,'B');
		OLED_SF32x46(RandomP+48,32-40+i*2,ds3231.Min,'T');
		GUI_CircleFill(RandomP*2+84,43-40+i*2,2,15);
		GUI_CircleFill(RandomP*2+84,68+20-i,2,15);
		OLED_SF10x16(RandomP+90,39-40+i*2,ds3231.Week);
		
		OLED_SF8x16(RandomP+90+60-i*3,56,str);
		GUI_HLine(RandomP*2+180,56,RandomP*2+180+i*5,15);
		
		OLED_Refresh_Gram();
	}
	
}
void TIME_MainDisplay(void)
{
	static char str[20];
	OLED_SF32x46(RandomP+4,31,ds3231.Hour,'B');
	OLED_SF32x46(RandomP+48,32,ds3231.Min,'T');
	GUI_CircleFill(RandomP*2+84,43,2,15);
	GUI_CircleFill(RandomP*2+84,68,2,15);
	OLED_SF10x16(RandomP+90,39,ds3231.Week);
	sprintf(str,"%s.%4s.%s",ds3231.Year,ds3231.Mon,ds3231.Day);
	OLED_SF8x16(RandomP+90,56,str);
	GUI_HLine(RandomP*2+180,56,RandomP*2+278,15);
}

void TIME_Mode(void)
{
	TIMEUI_Init();
	while(Display_Mode == MODE_DATE)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			if(Flag_Sleep)
			{
				Flag_Sleep = False;
				Display_Mode = MODE_SLEEP;
				break;
			}
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			TIME_MainDisplay();
			OLED_Refresh_Gram();

			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
//	TIMEUI_Out();
}

void SLEEP_Mode(void)
{
	u16 i;
	for(i=0;i<1000;)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;i++;
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			OLED_OlRefresh_Gram();
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	OLED_Clear();
	CalcTIMED(ds3231.Mon,ds3231.Week);
}

void OFFLINE_Mode(void)
{
	while(Display_Mode == MODE_OFFLINE)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			OLED_Clear();
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			OLED_Refresh_Gram();
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		}
	}
	OLED_Clear();
}

void NORMAL_MainDisplay(void)
{
	OLED_SF8x16(30,0,Device_Str.cputemp);
	OLED_SF8x16(30,16,Device_Str.cpuclock);
	OLED_BAR64x10(60+2,36,Device_Msg.cpuload/10,0);
	OLED_SF8x16(30,50+2,Device_Str.gputemp);
	OLED_SF8x16(30,66+2,Device_Str.gpuclock);
	OLED_BAR64x10(60+2,86+2,Device_Msg.gpuload/10,1);
	
	Flow_Coordinate(2);
	Showbar(Display_Notification);
	
	Flow_Corn();
	Flow_Line();
	Flow_USB();
	
	OLED_Part(30,33,32,10,Show_Load);
	OLED_Part(30,83+2,32,10,Show_Load);
	OLED_Part(24,0,5,16,Corn_Temp);
	OLED_Part(24,16,5,16,Corn_Freq);
	OLED_Part(24,32,5,10,Corn_Load);
	OLED_Part(24,50+2,5,16,Corn_Temp);
	OLED_Part(24,66+2,5,16,Corn_Freq);
	OLED_Part(24,82+2,5,10,Corn_Load);
	if(VoiceFlag)
	{
		SHOW_CORN.ASUS = False;
		if(Flow_ASUS() == OLED_IDLE)
		{
			Motion_VOICE(115,28);
		}
	}
	else
		Flow_ASUS();
}

void NORMALUI_Init(void)
{
	Notbarautoflag = True;
	Notbarrefsflag = True;
	OLED_Clear();
	SHOW_CORN.ASUS = True;
	SHOW_CORN.CPUGPU = True;
	SHOW_CORN.LINE = True;
	SHOW_CORN.USBD = True;
	Clear_FRAM();
	OLED_RefrashCoordinate(Device_Msg.cpuload/10);
	OLED_Part(30,33,32,10,Show_Load);
	OLED_Part(30,83+2,32,10,Show_Load);
	OLED_Part(24,0,5,16,Corn_Temp);
	OLED_Part(24,16,5,16,Corn_Freq);
	OLED_Part(24,32,5,10,Corn_Load);
	OLED_Part(24,50+2,5,16,Corn_Temp);
	OLED_Part(24,66+2,5,16,Corn_Freq);
	OLED_Part(24,82+2,5,10,Corn_Load);
	HAL_TIM_Base_Start_IT(&htim6);
	Flag_Reception = False;
	InitData();
	Device_Msg.cpuload = 1000;
	Device_Msg.gpuload = 1000;
	while(!Flag_Continue)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;Judge_Notification();
			OLED_Fill(0);
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			NORMAL_MainDisplay();
			OLED_Refresh_Gram();
		}
	}
	Flag_Continue = False;
	HAL_TIM_Base_Start_IT(&htim6);
	Flag_Reception = True;
	while(!Flag_Continue)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;Judge_Notification();
			OLED_Fill(0);
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			Device_Msg.cpuload = 0;
			Device_Msg.gpuload = 0;
			NORMAL_MainDisplay();
			OLED_Refresh_Gram();
		}
	}
	Flag_Continue = False;
}

void NORMALUI_Out(void)
{
	u8 i;
	SHOW_CORN.ASUS = False;
	SHOW_CORN.CPUGPU = False;
	SHOW_CORN.LINE = False;
	SHOW_CORN.USBD = False;
	sprintf(Display_Notification,"-----");
	Notbarautoflag = False;
	for(i=0;i<36;i++)
	{
//		while(!Flag_Refrash) __ASM("NOP");
		Flag_Refrash = False;
		Device_Msg.cpuload=0;
		Device_Msg.cpuload=0;
		OLED_Fill(0);
		NORMAL_MainDisplay();if(i>2)
		GUI_RectangleFill(44,0,128,95,0);
		OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
		Flow_Coordinate(0);
		OLED_Refresh_Gram();
	}
	OLED_Fill(0);
	DampValueClear();
}

void NORMAL_Mode(void)
{
	static u16 SleepTypo = 0;
	static u8 FlashFlag = 0;
	NORMALUI_Init();
	while(Display_Mode == MODE_NORMAL)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			if(!FlashFlag++%64)
				OLED_RefrashCoordinate(Device_Msg.cpuload/10);
			if(Flag_Sleep)
			{
				sprintf(Display_Notification,"Sleep");
				SleepTypo++;
				if(SleepTypo >= 250)
				{
					SleepTypo = 0;
					Flag_Sleep = False;
					Display_Mode = MODE_SLEEP;
				}
			}
			else
			{
				Judge_Notification();
			}
			OLED_Fill(0);
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			NORMAL_MainDisplay();
			OLED_Refresh_Gram();
		}
	}
	NORMALUI_Out();
}

void GAME_MainDisplay(void)
{
	SHOW_CORN.GAMEBK=True;
	OLED_SF8x16(10,40,Device_Str.ncpufan);
	OLED_SF8x16(10+76,40,Device_Str.ngpufan);
	OLED_SF10x16(2,72,Device_Str.ncpuclock);
	OLED_SF10x16(27,72,Device_Str.ngputemp);
	OLED_SF10x16(52,72,Display_State);

	OLED_BAR(74,46+2,75,8,Device_Msg.cpufan/30,1,2);
	OLED_BAR(74+152,46+2,75,8,Device_Msg.gpufan/30,1,3);

	OLED_BAR(152+2,73,146,21,Device_Msg.ramload/10,2,4);

	OLED_BAR(0+2,12,146,15,Device_Msg.cpuload/10,2,5);
	OLED_BAR(152+2,12,146,15,Device_Msg.gpuload/10,2,6);
}

void GAME_Out(void)
{
	int i;
	Flag_Reception = False;
	InitData();
	sprintf(Display_State,"%4s",Device_Str.ramusrdata);
	Device_Msg.cpuload = 0;
	Device_Msg.gpuload = 0;
	Device_Msg.cpufan = 0;
	Device_Msg.gpufan = 0;
	Device_Msg.ramload = 0;
	for(i = 300 ;i>=0;i-=6)
	{
		OLED_Fill(0);
		SportUIMt(i);
		
		if(i>100)
		{
			GAME_MainDisplay();
		}
		
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			SportUI2Mt(i);
			SportUIMt(i);
			if(i>100)
			{
				GAME_MainDisplay();
			}

			OLED_Refresh_Gram();
		}
		if(i>420&&!DampValueBusy())
			break;
	}
	DampValueClear();
	Flag_Reception = True;
}	

void GAME_Init(void)
{
	int i;
	Flag_Reception = False;
	InitData();
	sprintf(Display_State,"%4s",Device_Str.ramusrdata);
	Device_Msg.cpuload = 1000;
	Device_Msg.gpuload = 1000;
	Device_Msg.cpufan = 3000;
	Device_Msg.gpufan = 3000;
	Device_Msg.ramload = 1000;
	for(i= 0 ;i<300;i+=6)
	{
		OLED_Fill(0);
		SportUIMt(i);
		
		if(i>100)
		{
			GAME_MainDisplay();
		}
		
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			SportUIMt(i);
			SportUI2Mt(i);
			if(i>100)
			{
				GAME_MainDisplay();
			}

			OLED_Refresh_Gram();
		}
	}
	Flag_Reception = True;
}	

void GAME_Mode(void)
{
	static u16 SleepTypo = 0;
	GAME_Init();
	while(Display_Mode == MODE_GAME)
	{
		if(Flag_Refrash)
		{
			Flag_Refrash = False;
			if(Flag_Sleep)
			{
				sprintf(Display_State,"REST");
				SleepTypo++;
				if(SleepTypo >= 250)
				{
					SleepTypo = 0;
					Flag_Sleep = False;
					Display_Mode = MODE_SLEEP;
				}
			}
			else
				sprintf(Display_State,"%4s",Device_Str.ramusrdata);
			OLED_Fill(0);
			OLED_AllMotion(Device_Cmd.commandmotion,Device_Cmd.commandspeed);
			OLED_Full(UI_Sport);
			GAME_MainDisplay();
			OLED_Refresh_Gram();
		}
	}
	GAME_Out();
}

u8 TimeSystem_Convert(u8 Hour,u8 Sys)
{
	if(!Sys)
	{
		if(Hour == 0x12)
			return 0;
		else
			return Hour;
	}
	else
	{
		if(Hour == 0x12)
			return 0x12;
		else
			return HEX2BCD(BCD2HEX(Hour) + 12);
	}	
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static u16 SleepCount = 0;
//	static u8 InitFlag = True;
	static u8 U3WatchDogCount = 0;
	static u16 ContinueCount = 0;
	if (htim->Instance == htim4.Instance)
	{
		Flag_Refrash = True;
		
	}
	if(htim->Instance==TIM5)
	{
	}
	if (htim->Instance == htim6.Instance)
	{
		if(ContinueCount++>20)
		{
			Flag_Continue = True;
			ContinueCount = 0;
			HAL_TIM_Base_Stop_IT(&htim6);
		}
	}
	if (htim->Instance == htim8.Instance)
	{
	}
	if (htim->Instance == htim7.Instance)
	{
		if(HAL_GPIO_ReadPin(WIFI_CH1_GPIO_Port,WIFI_CH1_Pin))
			VoiceFlag = True;
		U3WatchDogCount++;
		if(OfflineCount<1)
		{
			OfflineCount++;
		}
		else if(OfflineCount<2)
		{
			ClearFFT();
			OfflineCount++;
		}
		else if(OfflineCount <10)
		{
			InitData();
			OfflineCount++;
		}
		else if(OfflineCount == 10)
		{
			Display_Mode = MODE_OFFLINE;
		}
		Flag_Blink = (~Flag_Blink)&1;
		
		
			
//		
//		printf("R:%d G:%d B:%d\r\n",RGBRAM[1][0],RGBRAM[1][1],RGBRAM[1][2]);
//		if(HAL_GPIO_ReadPin(HUMAN_GPIO_Port,HUMAN_Pin) == 0)
//				printf("没人\r\n");
//			else
//				printf("有人\r\n");
		Time_Handle();
		if(Uart_Overflow1_Flag)
		{
			DS3231_SetUart();
			OfflineCount = 0;
//			VFD_Bright = 0xff;
			
			Uart_Overflow1_Flag = False;
		}
		if(Device_Cmd.commandset&0x4)
		{
			if(SleepCount++>SLEEPTIME*2)
			{
				Flag_Sleep = True;
				SleepCount = 0;
			}
		}
		else
			SleepCount = 0;
//		Update_Pos();
		
		if(SaveFlag && SystemActive)
		{
			SaveFlag = False;
			Tranfcmd();
		}
	}
}


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
