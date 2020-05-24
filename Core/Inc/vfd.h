#ifndef __PT6302_H
#define __PT6302_H	

#include "stm32f1xx_hal.h"
#include "sys.h"
#include "spi.h"

typedef enum
{
  VFD_OK       = 0x00U,
  VFD_ERROR    = 0x01U,
  VFD_BUSY     = 0x02U,
  VFD_IDLE     = 0x03U,
} VFD_STATUS;

typedef enum
{
  DOLBY    		 = 0x00U,
  TIME    		 = 0x01U,
  CIRCLE     	 = 0x02U,
  MESSG    		 = 0x03U,
  HDCORN   		 = 0x04U,
} VFDCORN;

#define VFD_DATA_Clr() PBout(15)=0//DATA
#define VFD_DATA_Set() PBout(15)=1

#define VFD_CLK_Clr() PBout(13)=0//CLK
#define VFD_CLK_Set() PBout(13)=1
 		     
#define VFD_STB_Clr()  PBout(12)=0//STB
#define VFD_STB_Set()  PBout(12)=1

#define Delt 6
extern u8 VRAM[24*7][2];
extern u8 VBAK[24*7][2];

void VFD_Init(void);
void VFD_Refresh_Vram(void);
void VFD_F5x7(int x,u8 y,u8 Num,u8 Mtflag);
void VFD_SF5x7(int x,u8 y,char *ch,u8 Mtflag);
void VFD_SFs5x7(int x,u8 y,char *ch,u8 Font);
void VFD_Corn(VFDCORN corn, u8 State);
void VFD_Clear(void);
void VFD_Fill(u8 Color);
void VFD_Point(int x,uint8_t y,uint8_t t);
u8 VFD_ReadPoint(int x,uint8_t y);
void VFD_Part(int x,u8 y,u8 Wid,u8 He,u8 *ch);
void VFD_ClearPart(int x,u8 y,u8 Wid,u8 He);
void VFD_Brightness(u8 Bright);

VFD_STATUS VFD_MovVfdflake(u8 Index);
void Motion_VfdflakeInit(void);
void Motion_Vfdflake(void);

VFD_STATUS VFD_FucBlin(u8 Index);
void Motion_BlinInit(void);
void Motion_Blin(void);
void VFD_PNTTIME(void);
void VFD_PNTMSG(void);
void VFD_Bow(u8 X,u8 H,u8 Y,u8 PixNum,u8 Ch);
void VFD_PNTCls(void);

#endif  
	 
	 



