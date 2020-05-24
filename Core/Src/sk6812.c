#include "sk6812.h"
#include "usart.h"

u8 RGBTAG[64][3]={0};
float RGBRAM[64][3]={0};

u16 RGBBright = 100;

void SK6812_send_L(void)
{
	WSDATA=1;
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 
	__ASM("NOP");
	WSDATA=0;
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
}

void SK6812_send_H(void)
{
	WSDATA=1;			 //发送“1”码
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	WSDATA=0;
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");		 //不可省略的nop(),延时指定时间作用，晶振频率33MHz
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
}


void SK6812_Write(u8 Color_Red,u8 Color_Green,u8 Color_Blue)
{  
	unsigned char i;
	for(i=0;i<8;i++) 
	{
		if(Color_Green&0x80)
			SK6812_send_H();
		else 
			SK6812_send_L();                  
		Color_Green<<=1;    
	} 

	for(i=0;i<8;i++) 
	{
		if(Color_Red&0x80)
			SK6812_send_H();
		else 
			SK6812_send_L();                  
		Color_Red<<=1;    
	}

	for(i=0;i<8;i++) 
	{
		if(Color_Blue&0x80)
			SK6812_send_H();
		else 
			SK6812_send_L();                
		Color_Blue<<=1;    
	}
}

float RunStep = 2.0;

void SK6812_Flow()
{
	u8 i,j;
	
	if(RGBBright>10)
		RunStep = RGBBright*2.0f/255;
	else
		RunStep = 1;
//	printf("RunStep:%f\r\n",RunStep);
	for(i=0;i<RGBIndex;i++)
	{
		for(j=0;j<3;j++)
		{	
			if(RGBRAM[i][j]>=RGBTAG[i][j]+RunStep)
				RGBRAM[i][j]-=RunStep;
			else if(RGBRAM[i][j]+RunStep<=RGBTAG[i][j])
				RGBRAM[i][j]+=RunStep;
		}
	}
}


void SK6812_Refrash(void)
{
	u8 i;
	for(i=0;i<RGBIndex;i++)
	{
		SK6812_Write(RGBRAM[i][0],RGBRAM[i][1],RGBRAM[i][2]);
	}
}

void SK6812_WheelAll(u16 WheelPos)
{
	u8 i;
	WheelPos = WheelPos * 3.0 * RGBBright/255;
	for(i=0;i<RGBIndex;i++)
	{
		if(WheelPos < RGBBright) 
		{
			RGBTAG[i][0] = 0;
			RGBTAG[i][1] = RGBBright - WheelPos;
			RGBTAG[i][2] = WheelPos;
		}
		else if(WheelPos < (RGBBright*2)) 
		{          
			RGBTAG[i][0] = WheelPos-RGBBright;
			RGBTAG[i][1] = 0;
			RGBTAG[i][2] = RGBBright *2 - WheelPos;
		}
		else
		{
			RGBTAG[i][0] = (RGBBright*3) - WheelPos;
			RGBTAG[i][1] = WheelPos-(RGBBright*2);
			RGBTAG[i][2] = 0;
		}
	}
}

void SK6812_Wheel(u8 Type,u16 WheelPos) 
{
	u8 i;
	for(i=0;i<RGBIndex;i++)
	{
		WheelPos = WheelPos%(RGBBright*3);
		if(WheelPos < RGBBright) 
		{
			RGBTAG[i][0] = Type*RGBBright/2;
			RGBTAG[i][1] = RGBBright - WheelPos;
			RGBTAG[i][2] = WheelPos;
		}
		else if(WheelPos < (RGBBright*2)) 
		{          
			RGBTAG[i][0] = WheelPos-RGBBright;
			RGBTAG[i][1] = Type*RGBBright/2;
			RGBTAG[i][2] = RGBBright *2 - WheelPos;
		}
		else
		{
			RGBTAG[i][0] = (RGBBright*3) - WheelPos;
			RGBTAG[i][1] = WheelPos-(RGBBright*2);
			RGBTAG[i][2] = Type*RGBBright/2;
		}
		WheelPos +=16;
	}
}

void SK6812_WheelS(u8 Type,u16 WheelPos) 
{
	u8 i;
	for(i=0;i<RGBIndex;i++)
	{
		WheelPos = WheelPos%(RGBBright*3);
		if(WheelPos < RGBBright) 
		{
			RGBTAG[i][0] = 0;
			RGBTAG[i][1] = RGBBright - WheelPos;
			RGBTAG[i][2] = WheelPos;
		}
		else if(WheelPos < (RGBBright*2)) 
		{          
			RGBTAG[i][0] = WheelPos-RGBBright;
			RGBTAG[i][1] = 0;
			RGBTAG[i][2] = RGBBright *2 - WheelPos;
		}
		else
		{
			RGBTAG[i][0] = (RGBBright*3) - WheelPos;
			RGBTAG[i][1] = WheelPos-(RGBBright*2);
			RGBTAG[i][2] = 0;
		}
		WheelPos +=24;
	}
}

void SK6812_Same(u8 Color_Red,u8 Color_Green,u8 Color_Blue)
{
	u8 i;
	for(i=0;i<RGBIndex;i++)
	{
		RGBTAG[i][0] = Color_Red;
		RGBTAG[i][1] = Color_Green;
		RGBTAG[i][2] = Color_Blue;
	}
}

void SK6812_Init(void)
{
	WSDATA=1;
	__ASM("NOP");
	__ASM("NOP");
	WSDATA=0;
	HAL_Delay(1);
	SK6812_Refrash();
}

