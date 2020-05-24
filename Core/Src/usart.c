/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include "sys.h"
#include "main.h"
#include "flash.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
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
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
	return 1;
}


uint8_t Nix2Num(uint8_t Input)
{
	switch(Input)
	{
		case 0x3f:return 0;
		case 0x06:return 1;
		case 0x5b:return 2;
		case 0x4f:return 3;
		case 0x66:return 4;
		case 0x6d:return 5;
		case 0x7d:return 6;
		case 0x07:return 7;
		case 0x7f:return 8;
		case 0x6f:return 9;
		default:return 0xff;
	}
}

CLOCK_MSG Clock_Msg;
DEVICE_CMD Device_Cmd;
DEVICE_MSG Device_Msg;
DEVICE_STR Device_Str;

void ConvertData(void)
{
	if(Device_Msg.cputemp)
	sprintf(Device_Str.cputemp,"%02d.%1d$ ",Device_Msg.cputemp/10,Device_Msg.cputemp%10);
	if(Device_Msg.cpuclock)
	sprintf(Device_Str.cpuclock,"%dMhz ",Device_Msg.cpuclock);
	sprintf(Device_Str.cpuload,"%02.1f%%  ",(double)Device_Msg.cpuload/10);
	if(Device_Msg.cpufan)
	sprintf(Device_Str.cpufan,"%dRpm ",Device_Msg.cpufan);
	
	if(Device_Msg.gputemp)
	sprintf(Device_Str.gputemp,"%02d.%1d$ ",Device_Msg.gputemp/10,Device_Msg.gputemp%10);
	if(Device_Msg.gpuclock)
	sprintf(Device_Str.gpuclock,"%dMhz ",Device_Msg.gpuclock);
	sprintf(Device_Str.gpuload,"%02.1f%%  ",(double)Device_Msg.gpuload/10);
	if(Device_Msg.gpufan)
	sprintf(Device_Str.gpufan,"%dRpm ",Device_Msg.gpufan);
	
	if(Device_Msg.maintemp)
	sprintf(Device_Str.maintemp,"%03.1f$ ",(double)Device_Msg.maintemp/10);
	if(Device_Msg.mainfan)
	sprintf(Device_Str.mainfan,"%dRpm ",Device_Msg.mainfan);
	
	sprintf(Device_Str.ramload,"%02.1f%% ",(double)Device_Msg.ramload/10);
	
	if(Device_Msg.ramusrdata)
	sprintf(Device_Str.ramusrdata,"%d",Device_Msg.ramusrdata);
	
	if(Device_Msg.cpufan)	
	sprintf(Device_Str.ncpufan,"%04d",Device_Msg.cpufan);
	if(Device_Msg.cpuclock)
	sprintf(Device_Str.ncpuclock,"%04d",Device_Msg.cpuclock);
	if(Device_Msg.gpufan)
	sprintf(Device_Str.ngpufan,"%04d",Device_Msg.gpufan);
	if(Device_Msg.gputemp)
	sprintf(Device_Str.ngputemp,"%02d.%1d",Device_Msg.gputemp/10,Device_Msg.gputemp%10);
	
	
	if(Device_Msg.cputemp)
		sprintf(Device_Str.vcputemp,"%03d#",Device_Msg.cputemp);	
	if(Device_Msg.gputemp)
		sprintf(Device_Str.vgputemp,"%03d#",Device_Msg.gputemp);	
}

void InitData(void)
{
	sprintf(Device_Str.cputemp,"--.-$ ");
	sprintf(Device_Str.cpuclock,"----Mhz ");
	sprintf(Device_Str.cpuload,"--.-%%  ");
	sprintf(Device_Str.cpufan,"----Rpm ");
	
	sprintf(Device_Str.gputemp,"--.-$ ");
	sprintf(Device_Str.gpuclock,"----Mhz ");
	sprintf(Device_Str.gpuload,"--.-%%  ");
	sprintf(Device_Str.gpufan,"----Rpm ");
	
	sprintf(Device_Str.maintemp,"--.-$ ");
	sprintf(Device_Str.mainfan,"----Rpm ");
	
	sprintf(Device_Str.ramload,"--.-%% ");
	sprintf(Device_Str.ramusrdata,"----");
	
	sprintf(Device_Str.ngputemp,"--.-");	
	sprintf(Device_Str.ncpuclock,"----");	
	sprintf(Device_Str.ngpufan,"----");	
	sprintf(Device_Str.ncpufan,"----");	
	
	memset(&Device_Msg,0,sizeof(Device_Msg));
}


void Judge_Mode()
{
	static u8 normalcount=6,gamecount=6;
	if(Device_Msg.gpuload>800)
	{
		gamecount ++;
		if(gamecount>=10)
		{
			gamecount = 10;Display_Mode = MODE_GAME;
			normalcount = 0;
		}
	}
	else if(Device_Msg.gpuload<200)
	{
		normalcount ++;
		if(normalcount>=10)
		{
			normalcount = 10;Display_Mode = MODE_NORMAL;
			gamecount = 0;
		}
	}
}
uint8_t Uart_Recv1_Buf[Uart_Max_Length] = {0};
uint16_t Uart_Recv1_Length = 0;
uint8_t Uart_Overflow1_Flag = False;

#if GEMINI == 1
u8 ReponseID[]={0xFF,0x55,'O','K',0x06,'G','e','m','i','n','i'};
#else
u8 ReponseID[]={0xFF,0x55,'O','K',0x05,'A','r','i','e','s'};
#endif

void AnalysisComputermsg(uint8_t *Buf)
{
	if(Buf[0] == 0xFF &&Buf[1] == 0x55)
	{
		if(Buf[4] == 0x2)
		{
			switch (MAKEWORD(Buf[3], Buf[2]))
			{
			case CPU_Temp:
				Device_Msg.cputemp = MAKEWORD(Buf[6],Buf[5]);
				break;
			case CPU_Clock:
				Device_Msg.cpuclock = MAKEWORD(Buf[6],Buf[5]);
				break;
			case CPU_Load:
				Device_Msg.cpuload = MAKEWORD(Buf[6],Buf[5]);
				break;
			case CPU_Fan:
				Device_Msg.cpufan = MAKEWORD(Buf[6],Buf[5]);
				break;
			case CPU_POWER:
				Device_Msg.cpupower = MAKEWORD(Buf[6],Buf[5]);
				break;

			case GPU_Temp:
				Device_Msg.gputemp = MAKEWORD(Buf[6],Buf[5]);
				break;
			case GPU_Clock:
				Device_Msg.gpuclock = MAKEWORD(Buf[6],Buf[5]);
				break;
			case GPU_Load:
				Device_Msg.gpuload = MAKEWORD(Buf[6],Buf[5]);
				break;
			case GPU_Fan:
				Device_Msg.gpufan = MAKEWORD(Buf[6],Buf[5]);
				break;


			case Main_Temp:
				Device_Msg.maintemp = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Main_Fan:
				Device_Msg.mainfan = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Main_Vol:
				Device_Msg.mainvoltage = MAKEWORD(Buf[6],Buf[5]);
				break;

			case RAM_Load:
				Device_Msg.ramload = MAKEWORD(Buf[6],Buf[5]);
				break;
			case RAM_Data:
				Device_Msg.ramusrdata = MAKEWORD(Buf[6],Buf[5]);
				break;

			case HDD_Load:
				Device_Msg.hddload = MAKEWORD(Buf[6],Buf[5]);
				break;
			case HDD_Temp:
				Device_Msg.hddtemp = MAKEWORD(Buf[6],Buf[5]);
				break;

			case Uart_Year:
				Device_Msg.uartyear = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Month:
				Device_Msg.uartmonth = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Day:
				Device_Msg.uartday = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Week:
				Device_Msg.uartweek = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Hour:
				Device_Msg.uarthour = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Minute:
				Device_Msg.uartminute = MAKEWORD(Buf[6],Buf[5]);
				break;
			case Uart_Second:
				Device_Msg.uartsecond = MAKEWORD(Buf[6],Buf[5]);
				break;
			case End_Frame_ADDR:
				if(MAKEWORD(Buf[6],Buf[5]) == 0x5A5A)
				{
					ConvertData();		
					Uart_Overflow1_Flag = True;
					if(Device_Cmd.commandmode == 1) 
					{
						Judge_Mode();
					}
				}break;
			case 0x3F3F:
				ReponseID[2] = (Buf[5] & 0x5A) | (Buf[6] & 0xA5);
				ReponseID[3] = (Buf[5] ^ 0x57) | (Buf[6] & 0x57);
				HAL_UART_Transmit_IT(&huart1,ReponseID,10);break;
			}
		}
	}
}

u8 SaveFlag = False;

void AnalysisCommand(uint8_t *Buf)
{
	if(Buf[0] == 0xFF &&Buf[1] == 0x55)
	{
		if(Buf[4] == 0x2)
		{
			switch (MAKEWORD(Buf[3], Buf[2]))
			{
				case Command_Mode:
//					if(Device_Cmd.commandmode != MAKEWORD(Buf[6],Buf[5]))
//					{
						Device_Cmd.commandmode = MAKEWORD(Buf[6],Buf[5]); 
						if(Device_Cmd.commandmode>1) Display_Mode = Device_Cmd.commandmode;
//					}
				break;
				case Command_Motion:
					if(Device_Cmd.commandmotion != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandmotion = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_Style:
					if(Device_Cmd.commandstyle != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandstyle = MAKEWORD(Buf[6],Buf[5]);
					}break;
				case Command_Speed:
					if(Device_Cmd.commandspeed != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandspeed = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_Brightness:
					if(Device_Cmd.commandbrightness != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandbrightness = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_Set:
					if(Device_Cmd.commandset != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandset = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_RGBMode:
					if(Device_Cmd.commandrgbmode != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandrgbmode = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_RGBBrightness:
					if(Device_Cmd.commandrgbbrightness != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandrgbbrightness = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
				case Command_RGBColor:
					if(Device_Cmd.commandrgbcolor != MAKEWORD(Buf[6],Buf[5]))
					{
						Device_Cmd.commandrgbcolor = MAKEWORD(Buf[6],Buf[5]);
						SaveFlag = True;
					}break;
			}
		}
	}
}

void Tranfcmd(void)
{
	static u8 CMD[32];
	CMD[0] = Device_Cmd.commandmode>>8;
	CMD[1] = Device_Cmd.commandmode&0xff;
	
	CMD[2] = Device_Cmd.commandmotion>>8;
	CMD[3] = Device_Cmd.commandmotion&0xff;
	
	CMD[4] = Device_Cmd.commandstyle>>8;
	CMD[5] = Device_Cmd.commandstyle&0xff;
	
	CMD[6] = Device_Cmd.commandspeed>>8; 
	CMD[7] = Device_Cmd.commandspeed&0xff;
	
	CMD[8] = Device_Cmd.commandbrightness>>8;
	CMD[9] = Device_Cmd.commandbrightness&0xff;
	
	CMD[10] = Device_Cmd.commandset>>8;
	CMD[11] = Device_Cmd.commandset&0xff;
	
	CMD[12] = Device_Cmd.commandrgbmode>>8; 
	CMD[13] = Device_Cmd.commandrgbmode&0xff;
	
	CMD[14] = Device_Cmd.commandrgbcolor>>8;
	CMD[15] = Device_Cmd.commandrgbcolor&0xff;
	
	CMD[16] = Device_Cmd.commandrgbbrightness>>8;
	CMD[17] = Device_Cmd.commandrgbbrightness&0xff;
	SPI_Flash_Write(CMD,1024*1024,32);
}

void Recvcmd(void)
{
	static u8 CMD[32];
	SPI_Flash_Read(CMD,1024*1024,32);
//	Device_Cmd.commandmode = MAKEWORD(CMD[1],CMD[0]);
//	if(Device_Cmd.commandmode>1) Display_Mode = Device_Cmd.commandmode;
	Device_Cmd.commandmotion = MAKEWORD(CMD[3],CMD[2]);
	Device_Cmd.commandstyle = MAKEWORD(CMD[5],CMD[4]);
	Device_Cmd.commandspeed = MAKEWORD(CMD[7],CMD[6]);
	Device_Cmd.commandbrightness = MAKEWORD(CMD[9],CMD[8]);
	Device_Cmd.commandset = MAKEWORD(CMD[11],CMD[10]);

	Device_Cmd.commandmode = MAKEWORD(CMD[13],CMD[12]);
	Device_Cmd.commandrgbcolor = MAKEWORD(CMD[15],CMD[14]);
	Device_Cmd.commandrgbbrightness = MAKEWORD(CMD[17],CMD[16]);
}

void FindBrightness(void)
{
	static u8 CMD[32];
	SPI_Flash_Read(CMD,1024*1024,32);
//	Device_Cmd.commandmode = MAKEWORD(CMD[1],CMD[0]);
//	if(Device_Cmd.commandmode>1) Display_Mode = Device_Cmd.commandmode;
	Device_Cmd.commandbrightness = MAKEWORD(CMD[9],CMD[8]);
	Device_Cmd.commandrgbbrightness = MAKEWORD(CMD[17],CMD[16]);
	
}

void AnalysisComputername(uint8_t *Buf)
{
	u8 i;
	if(Buf[0] == 0xFF &&Buf[1] == 0x55)
	{
		if(Buf[4] > 0x2)
		{
			switch (MAKEWORD(Buf[3], Buf[2]))
			{
			case 0x101:for(i=0;i<Buf[4];i++) Device_Msg.cpuname[i] = Buf[i + 5];break;
			case 0x102:for(i=0;i<Buf[4];i++) Device_Msg.gpuname[i] = Buf[i + 5];break;
			case 0x103:for(i=0;i<Buf[4];i++) Device_Msg.mainname[i] = Buf[i + 5];break;
			}
		}
	}
}


void ClearFFT(void)
{
	int i;
	for(i=0;i<Uart_FFT_Length;i++) 
		Device_Msg.fft[i] = 0;
}


void AnalysisFFT(uint8_t *Buf)
{
	u16 i;
	if(Buf[0] == 0xFF &&Buf[1] == 0x55)
	{
		if(Buf[4] >= 0x2)
		{
			switch (MAKEWORD(Buf[3], Buf[2]))
			{
				case FFT_Data:for(i=0;i<Buf[4];i++) Device_Msg.fft[i] = Buf[i + 5];
				if(Device_Cmd.commandmode == 1) 
					Display_Mode = MODE_MUSIC;
					break;
				case Left_Vol:
					Device_Msg.leftvol = MAKEWORD(Buf[6],Buf[5]);break;
				case Right_Vol:
					Device_Msg.rightvol = MAKEWORD(Buf[6],Buf[5]);break;
			}
			Uart_Overflow1_Flag = True;
		}
	}
}


void AnalysisMiStaus(uint8_t Data)
{
	printf("%x\r\n",Data);
	if(Data&8)
		Clock_Msg.tmsys = 0;
	else if(Data&0x20)
		Clock_Msg.tmsys = 1;
	if(Data&1)
		Clock_Msg.cation = True;
	else if((Data&1)==0)
		Clock_Msg.cation = False;
	if(Data&2)
		Clock_Msg.mute = True;
	else if((Data&2)==0)
		Clock_Msg.mute = False;
	if(Data&4)
		Clock_Msg.alarm = True;
	else if((Data&4)==0)
		Clock_Msg.alarm = False;
	if(Data&0x10)
		Clock_Msg.wifi = True;
	else if((Data&0x10)==0)
		Clock_Msg.wifi = False;
}
u8 Uart_Recv3_Data;
u8 Uart_Overflow3_Flag = False;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t Command,Data;
	static uint8_t Uart3_Busy = False;
	if(huart->Instance == USART3)   // 判断是由哪个串口触发的中断
	{
		if(!Uart3_Busy)
		{
			if((Uart_Recv3_Data&0x1F) == 0x8)
			{
				Command = Uart_Recv3_Data;
				Uart3_Busy = True;
			}
		}
		else
		{
			if(Uart_Recv3_Data == 0x28)
			{
				Uart3_Busy = False;
				Command = 0;
				HAL_UART_Receive_IT(&huart3,&Uart_Recv3_Data,1);
				return;
			}
			if(Command == 0x28)
			{
				if(Uart_Recv3_Data==0)
					Clock_Msg.aiwork = True;
				else
					AnalysisMiStaus(Uart_Recv3_Data);
			}
			else
			{
				if((Uart_Recv3_Data&0x7f)==0)
				{
					if(Command==0xc8||Command==0x8)
					Clock_Msg.aiwork = True;
				}
				else if(Command==0xc8)
					Clock_Msg.aiwork = False;
				Data = Nix2Num(Uart_Recv3_Data&0x7F);
				if(Data!=0x7F)
				{
					switch(Command)
					{
						case 0x08:if(Data<2) {Clock_Msg.hour&=0xF;	Clock_Msg.hour|= Data<<4;}break;
						case 0x88:Clock_Msg.hour&=0xF0; Clock_Msg.hour|= Data;break;
						case 0x48:if(Data<6) {Clock_Msg.minute&=0xF;  Clock_Msg.minute |= Data<<4;}break;
						case 0xC8:Clock_Msg.minute&=0xF0; Clock_Msg.minute |= Data;break;
						
					}
				}
			}
			Uart_Overflow3_Flag = True;
			Uart3_Busy = False;
			Command = 0;
		}
		HAL_UART_Receive_IT(&huart3,&Uart_Recv3_Data,1);      // 重新使能串口1接收中断
	}
}

u8 U3errorFlag = False;
/* 中断错误处理函数，在此处理overrun错误 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)   // 判断是由哪个串口触发的中断
	{
    if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_ORE) != RESET) 
    {
//			U3errorFlag = True;
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        HAL_UART_Receive_IT(&huart3,&Uart_Recv3_Data,1);
    }
	}
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
