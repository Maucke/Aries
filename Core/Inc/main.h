/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define MODE_DEFALUT      	0x00
#define MODE_OFFLINE    	  0x01
#define MODE_DATE    	  		0x02
#define MODE_MUSIC     			0x03
#define MODE_NORMAL   		  0x04
#define MODE_GAME  					0x05
#define MODE_SLEEP		  		0x06
#define MODE_AIWORK		  		0x07


#define GEMINI False
#define MARKC False
#define TEACH False
#define ARIES True

extern u8 BTVPostionX ;//124
extern u8 BTVPostionY ;//30
extern u8 BootGRAM[];
extern u8 VoiceFlag;
extern uint8_t SystemActive;
extern uint8_t Flag_Reception;
extern u16 Display_Mode;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
void Judge_Notification(void);

void SLEEP_Mode(void);
void OFFLINE_Mode(void);
void NORMAL_MainDisplay(void);
void NORMALUI_Init(void);
void NORMALUI_Out(void);
void NORMAL_Mode(void);
void GAME_MainDisplay(void);
void GAME_Out(void);
void GAME_Init(void);
void GAME_Mode(void);
void TIME_VFDDisplay(void);
void Display_Beat(void);
void DATE_Mode(void);
void Update_Pos(void);
void MUSIC_Mode(void);
void AIWORK_Mode(void);
u8 TimeSystem_Convert(u8 Hour,u8 Sys);
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_D0_Pin GPIO_PIN_0
#define OLED_D0_GPIO_Port GPIOC
#define OLED_D1_Pin GPIO_PIN_1
#define OLED_D1_GPIO_Port GPIOC
#define OLED_D2_Pin GPIO_PIN_2
#define OLED_D2_GPIO_Port GPIOC
#define OLED_D3_Pin GPIO_PIN_3
#define OLED_D3_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define OLED_D4_Pin GPIO_PIN_4
#define OLED_D4_GPIO_Port GPIOC
#define OLED_D5_Pin GPIO_PIN_5
#define OLED_D5_GPIO_Port GPIOC
#define DS3231_SCL_Pin GPIO_PIN_0
#define DS3231_SCL_GPIO_Port GPIOB
#define DS3231_SDA_Pin GPIO_PIN_1
#define DS3231_SDA_GPIO_Port GPIOB
#define DS3231_INT_Pin GPIO_PIN_2
#define DS3231_INT_GPIO_Port GPIOB
#define WIFI_CH4_Pin GPIO_PIN_12
#define WIFI_CH4_GPIO_Port GPIOB
#define WIFI_CH3_Pin GPIO_PIN_13
#define WIFI_CH3_GPIO_Port GPIOB
#define WIFI_CH2_Pin GPIO_PIN_14
#define WIFI_CH2_GPIO_Port GPIOB
#define WIFI_CH1_Pin GPIO_PIN_15
#define WIFI_CH1_GPIO_Port GPIOB
#define OLED_D6_Pin GPIO_PIN_6
#define OLED_D6_GPIO_Port GPIOC
#define OLED_D7_Pin GPIO_PIN_7
#define OLED_D7_GPIO_Port GPIOC
#define OLED_CS_Pin GPIO_PIN_3
#define OLED_CS_GPIO_Port GPIOB
#define OLED_RES_Pin GPIO_PIN_4
#define OLED_RES_GPIO_Port GPIOB
#define OLED_A0_Pin GPIO_PIN_5
#define OLED_A0_GPIO_Port GPIOB
#define OLED_WR_Pin GPIO_PIN_6
#define OLED_WR_GPIO_Port GPIOB
#define OLED_RD_Pin GPIO_PIN_7
#define OLED_RD_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
