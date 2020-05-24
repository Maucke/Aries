#include "oled.h"
#include "stdlib.h"
#include "gpio.h"
#include "config.h"
#include "oledfont.h" 
#include "math.h"  	 
#include "main.h"  	
#include "usart.h"
#include "spi.h"
#include "flash.h"
#include "stdio.h"

uint8_t OLED_GRAM[152][96];	
DISPLAY_CORN SHOW_CORN;

void OLED_WR_CMD(u8 cmd)
{		
//	OLED_WR=LOW;
	OLED_A0=OLED_CMD;
	OLED_CS=LOW;
	DATAOUT(cmd);
	OLED_CS=HIGH;
	OLED_A0=OLED_DATA;
//	OLED_WR=HIGH;
} 

void OLED_WR_DATA(u8 dat)
{		
//	OLED_WR=LOW;
//	OLED_A0=OLED_DATA;
	OLED_CS=LOW;
	DATAOUT(dat);
	OLED_CS=HIGH;
//	OLED_A0=OLED_DATA;
//	OLED_WR=HIGH;
} 

void OLED_Refresh_AddBack(u8 Set)
{
#if GEMINI == 1
	static int YStep = 96;
#else
	static int YStep = 0;
#endif
	uint16_t i,n;  
	if(Set)
	{
		if(YStep < 96) {YStep+=2;}
	}
	else
		if(YStep > 0) {YStep-=2;}
	for(n=0;n<152;n++) for(i=0;i<YStep;i++) OLED_GRAM[n][i]|=0x11;  
}

u16 OLEDBright = 100;
u8 OLEDBk = True;
//更新显存到LCD		 
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		   
//	static u16 Brightness = 0;
//	static u8 Contrast = 5;
//	if(Brightness>OLEDBright+7)
//	{
//		Brightness -= 7;
//		if(Brightness<14)
//			Contrast = 5;
//		else
//			Contrast = 254;
//		OLED_WR_CMD(0xBE);     //Set VSEGM Level
//		OLED_WR_CMD(Brightness>>4);		//40
//		OLED_WR_CMD(0xC1);     //The Contrast Control Mode Set
//		OLED_WR_CMD(Contrast);		//40
//	}
//	else if(Brightness+7<OLEDBright)
//	{
//		Brightness += 7;
//		if(Brightness<14)
//			Contrast = 5;
//		else
//			Contrast = 254;
//		OLED_WR_CMD(0xBE);     //Set VSEGM Level
//		OLED_WR_CMD(Brightness>>4);		//40
//		OLED_WR_CMD(0xC1);     //The Contrast Control Mode Set
//		OLED_WR_CMD(Contrast);		//40
////		printf("Contrast:%X\r\n",Contrast);
//	}
//		Brightness = OLEDBright;
//	OLED_Refresh_AddBack(Device_Cmd.commandset&0x8);	
//    
    OLED_WR_CMD(0x15);
    OLED_WR_DATA(0x1c);
    OLED_WR_DATA(0x5b);
    OLED_WR_CMD(0x75);
    OLED_WR_DATA(0x00);
    OLED_WR_DATA(0x3f);
    OLED_WR_CMD(0x5C);
    
	for(i=0;i<64;i++)  
	{  
		for(n=0;n<128;n++)
			OLED_WR_DATA(OLED_GRAM[n][i]); 
	}   
}

void OLED_OlRefresh_Gram(void)
{
	uint8_t i,n;		   	
	OLED_Refresh_AddBack(0);	
	for(i=0;i<96;i++)  
	{  
		OLED_WR_CMD(START_PAGE);
		OLED_WR_CMD(i);     
		OLED_WR_CMD(START_HIGH_BIT+(START_LOW_BIT)/16);
		OLED_WR_CMD((START_LOW_BIT)%16);
		for(n=0;n<152;n++)
			OLED_WR_DATA(OLED_GRAM[n][i]); 
	}   
}

void OLED_Full(const u8 *ch)
{
	uint16_t i,n;  
	for(i=0;i<64;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=~ch[i*128+n];  
}

void OLED_Refresh_Gram_Part(u16 x,u16 y,u16 l,u16 s)
{
	uint8_t i,n;	
	if(x+l>304||y+s>96) return;//超出范围了.	    
	x>>=1;l>>=1;
	for(i=y;i<y+s;i++)  
	{  
		OLED_WR_CMD(START_PAGE);
		OLED_WR_CMD(i);     
		OLED_WR_CMD(START_HIGH_BIT+(START_LOW_BIT+x)/16);
		OLED_WR_CMD((START_LOW_BIT+x)%16);
		for(n=x;n<x+l;n++)
			OLED_WR_DATA(OLED_GRAM[n][i]); 
	}   
}

void OLED_Clear(void)  
{  
	uint16_t i,n;  
	for(i=0;i<96;i++)for(n=0;n<152;n++)OLED_GRAM[n][i]=0;  
}
#define OFFSET 512
#define REAL(x) (x - OFFSET)
#define FAKE(x) (x + OFFSET)

void GUI_Point(uint16_t x,uint8_t y,uint8_t dot)
{
	if(x>303||y>95) return;//超出范围了.
	
	if(dot)
	{
		OLED_GRAM[x/2][y] &= 0xF<<((x&1)<<2);
		OLED_GRAM[x/2][y] |= (dot<<4)>>((x&1)<<2);
	} 
	else 
		OLED_GRAM[x/2][y] &= 0xF<<((x&1)<<2);	    
}

void OLED_Fill(uint8_t dot) 
{  
	uint16_t i,n;  
	for(i=0;i<96;i++)for(n=0;n<152;n++)OLED_GRAM[n][i]=dot;  										    
}

void OLED_Init(void)
{ 	 				 	 					 
	OLED_RD = HIGH;  			 
	OLED_WR = LOW;     
  OLED_RST = LOW;  
	HAL_Delay(10);
  OLED_RST = HIGH; 
	OLED_WR_CMD(0xFD);    /*SET COMMAND LOCK*/
	OLED_WR_DATA(0x12);   /* UNLOCK */
		 
	OLED_WR_CMD(0xAE);    /*DISPLAY OFF*/

	OLED_WR_CMD(0xB3);    /*DISPLAY DIVIDE CLOCKRADIO/OSCILLATAR FREQUANCY*/
	OLED_WR_DATA(0x91); 

	OLED_WR_CMD(0xCA);    /*multiplex ratio*/
	OLED_WR_DATA(0x3F);    /*duty = 1/64*/

	OLED_WR_CMD(0xA2);    /*set offset*/
	OLED_WR_DATA(0x00);  

	OLED_WR_CMD(0xA1);    /*start line*/
	OLED_WR_DATA(0x00);

	OLED_WR_CMD(0xA0);    /*set remap*/
	OLED_WR_DATA(0x14);
	OLED_WR_DATA(0x11);  

	/*OLED_WR_CMD(0xB5);    GPIO
	OLED_WR_CMD(0x00);   */ 

	OLED_WR_CMD(0xAB);    /*funtion selection*/
	OLED_WR_DATA(0x01);    /* selection external vdd */   

	OLED_WR_CMD(0xB4);    /*  */
	OLED_WR_DATA(0xA0); 
	OLED_WR_DATA(0xfd);

	OLED_WR_CMD(0xC1);    /*set contrast current */
	OLED_WR_DATA(0x9f); 


	OLED_WR_CMD(0xC7);    /*master contrast current control*/
	OLED_WR_DATA(0x0f); 

	/* OLED_WR_CMD(0xB9);    GRAY TABLE*/

	OLED_WR_CMD(0xB1);    /*SET PHASE LENGTH*/
	OLED_WR_DATA(0xE2); 

	OLED_WR_CMD(0xD1);    /**/
	OLED_WR_DATA(0x82); 
	OLED_WR_DATA(0x20);           


	OLED_WR_CMD(0xBB);    /*SET PRE-CHANGE VOLTAGE*/
	OLED_WR_DATA(0x1F); 

	OLED_WR_CMD(0xB6);    /*SET SECOND PRE-CHARGE PERIOD*/
	OLED_WR_DATA(0x08); 


	OLED_WR_CMD(0xBE);    /* SET VCOMH */
	OLED_WR_DATA(0x07);

	OLED_WR_CMD(0xA6);    /*normal display*/     

	OLED_Clear();
	OLED_Refresh_Gram();
	OLED_WR_CMD(0xAF);     //Set Display On  
}  

void OLED_XF4x5(u8 x,u8 y,u8 Num)
{
	u8 i;
	for(i=0;i<5;i++)
	{
		OLED_GRAM[x][i+y] = Defu_XF4x5[i*2+Num*10];
		OLED_GRAM[x+1][i+y] = Defu_XF4x5[i*2+1+Num*10];
	}
}

void OLED_Convert(int x,int y,u8 w,u8 *ch)
{
	u8 i,j,k;
	for(k=0;k<12;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<w;j++)
			{
				if(7-i+8*k+y<96)
					OLED_GRAM[x+j][7-i+8*k+y] = CovSc[(((~ch[w*2*k+j*2]>>i)&1)<<1)|((~ch[2*w*k+j*2+1]>>i)&1)];
			}
		}
	}
}	
//default:8
void OLED_F8x16(int x,int y,u8 Num,u8 Offset)
{
	u8 i,j,k;
	u8 Temp;
	for(k=0;k<2;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<4;j++)
			{
				if(i+8*k+y>=0&&i+8*k+y<96)
				{
					Temp = CovSc[(((OCRB_F8x16[(Num+Offset)*16+8*k+j*2]>>i)&1)<<1)|((OCRB_F8x16[(Num+Offset)*16+8*k+j*2+1]>>i)&1)+j*3*4];
					if(Temp)
							OLED_GRAM[x+j][i+8*k+y] = Temp;
				}
			}
		}
	}
}	

void OLED_SF8x16(int x,int y,char *ch)
{
	u8 c=0,j=0;
	while(ch[j]!='\0')
	{
		c=ch[j]-' ';
		if(x<152)
		OLED_F8x16(x,y,c,0);
		x+=4;
		j++;
	}
}

//default:16  纵向渐淡：+(i+k*8)*4
void OLED_F10x16(int x,int y,u8 Num,u8 Offset)
{
	u8 i,j,k;
	u8 Temp;
	for(k=0;k<2;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<5;j++)
				if(i+8*k+y>=0&&i+8*k+y<96)
				{
					Temp = CovSc[(((OCR_F10x16[(Num+Offset)*20+10*k+j*2]>>i)&1)<<1)|((OCR_F10x16[(Num+Offset)*20+10*k+j*2+1]>>i)&1)+j*2*4];
					if(Temp)
						OLED_GRAM[x+j][i+8*k+y] = Temp;
				}
		}
	}
}	

void OLED_SF10x16(int x,int y,char *ch)
{
	u8 c=0,j=0;
	while(ch[j]!='\0')
	{
		c=ch[j]-' ';
		if(x<152)
		OLED_F10x16(x,y,c,0);
		x+=5;
		j++;
	}
}

void OLED_F12x24(int x,int y,u8 Num,u8 Offset)
{
	u8 i,j,k;
	u8 Temp;
	for(k=0;k<3;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<6;j++)
				if(i+8*k+y>=0&&i+8*k+y<96)
				{
					Temp = CovSc[(((Agency_F12x24[(Num+Offset)*36+12*k+j*2]>>i)&1)<<1)|((Agency_F12x24[(Num+Offset)*36+12*k+j*2+1]>>i)&1)+j*2*4];
					if(Temp)
						OLED_GRAM[x+j][i+8*k+y] = Temp;
				}
		}
	}
}	

void OLED_SF12x24(int x,int y,char *ch)
{
	u8 c=0,j=0;
	while(ch[j]!='\0')
	{
		c=ch[j]-' ';
		if(x<152)
		OLED_F12x24(x,y,c,0);
		x+=6;
		j++;
	}
}

void OLED_F22x40(int x,int y,u8 Num,u8 Offset)
{
	u8 i,j,k;
	u8 Temp;
	for(k=0;k<5;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<11;j++)
				if(i+8*k+y>=0&&i+8*k+y<96)
				{
					Temp = CovSc[(((Agency_F22x40[(Num+Offset)*110+22*k+j*2]>>i)&1)<<1)|((Agency_F22x40[(Num+Offset)*110+22*k+j*2+1]>>i)&1)+j*4];
					if(Temp)
						OLED_GRAM[x+j][i+8*k+y] = Temp;
				}
		}
	}
}	

void OLED_SF22x40(int x,int y,char *ch)
{
	u8 c=0,j=0;
	while(ch[j]!='\0')
	{
		c=ch[j]-'0';
		if(x<152)
		OLED_F22x40(x,y,c,0);
		x+=11;
		j++;
	}
}

void OLED_F32x46(int x,int y,u8 Num,char Type)
{
	u8 i,j,k;
	u8 Temp;
	for(k=0;k<6;k++)
	{
		for(i=0;i<8;i++)
		{
			for(j=0;j<16;j++)
			{
				if(i+8*k+y>=0&&i+8*k+y<96)
				{
					switch(Type)
					{
						case 'T':Temp = CovSc[(((RobotT_F32x46[Num*192+32*k+j*2]>>i)&1)<<1)|((RobotT_F32x46[Num*192+32*k+j*2+1]>>i)&1)+j/2*4];break;
						case 'B':Temp = CovSc[(((RobotB_F32x46[Num*192+32*k+j*2]>>i)&1)<<1)|((RobotB_F32x46[Num*192+32*k+j*2+1]>>i)&1)+j/2*4];break;
						default:Temp = CovSc[(((Robot_F32x46[Num*192+32*k+j*2]>>i)&1)<<1)|((Robot_F32x46[Num*192+32*k+j*2+1]>>i)&1)+j/2*4];break;
					}
					if(Temp)
						OLED_GRAM[x+j][i+8*k+y] = Temp;
				}
			}
		}
	}
}	

void OLED_SF32x46(int x,int y,char *ch,char Type)
{
	u8 c=0,j=0;
	while(ch[j]!='\0')
	{
		c=ch[j]-'0';
		if(x<152)
		OLED_F32x46(x,y,c,Type);
		x+=18;
		j++;
	}
}


void OLED_Clar(int x,int y,u16 l,u16 s,const u8 *ch)
{
	u8 i,j;
	if(x>=0&&y>=0)
	{
		if(s+y<96&&x+l<152)
			for(i=MIN;i<s;i++)
				for(j=MIN;j<l;j++)
					OLED_GRAM[x+j][i+y]=ch[i*l+j];
		else if(s+y>=96&&x+l<152)
			for(i=MIN;i+y<96;i++)
				for(j=MIN;j<l;j++)
					OLED_GRAM[x+j][i+y]=ch[i*l+j];
		else if(s+y<96&&x+l>=152)
			for(i=MIN;i<s;i++)
				for(j=MIN;x+j<152;j++)
					OLED_GRAM[x+j][i+y]=ch[i*l+j];
		else
			for(i=MIN;i+y<96;i++)
				for(j=MIN;x+j<152;j++)
					OLED_GRAM[x+j][i+y]=ch[i*l+j];
	}
	else if(x>=0&&y<0)
	{
		for(i=-y;i<s;i++)
			if(i+y<96)
				for(j=MIN;j<l;j++)
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
	}
	else if(x<0&&y>=0)
	{
		for(i=MIN;i<s;i++)
			if(i+y<96)
				for(j=-x;j<l;j++)
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
	}
	else			
		for(i=-y;i<s;i++)
			if(i+y<96)
				for(j=-x;j<l;j++)
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
}

void OLED_Dim(int x,int y,u16 l,u16 s,const u8 *ch)
{
	u8 i,j;
	for(i=MIN;i<s;i++)
		for(j=MIN;j<l;j++)
		{
			if((ch[i*l+j]&0xF0)>0x60)
				OLED_GRAM[x+j][i+y]|=(ch[i*l+j]&0xF0);
			if((ch[i*l+j]&0xF)>0x6)
				OLED_GRAM[x+j][i+y]|=(ch[i*l+j]&0xF);
		}
}
void OLED_Part(int x,int y,u16 l,u16 s,const u8 *ch)
{
	u8 i,j;
	if(x>=0&&y>=0)
	{
		if(s+y<96&&x+l<152)
		{
			for(i=MIN;i<s;i++)
				for(j=MIN;j<l;j++)
					if(ch[i*l+j])
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
		}
		else if(s+y>=96&&x+l<152)
		{
			for(i=MIN;i+y<96;i++)
				for(j=MIN;j<l;j++)
					if(ch[i*l+j])
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
		}
		else if(s+y<96&&x+l>=152)
		{
			for(i=MIN;i<s;i++)
				for(j=MIN;x+j<152;j++)
					if(ch[i*l+j])
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
				}
		else
		{
			for(i=MIN;i+y<96;i++)
				for(j=MIN;x+j<152;j++)
					if(ch[i*l+j])
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
		}
	}
	else if(x>=0&&y<0)
	{
		for(i=-y;i<s;i++)
			if(i+y<96)
				for(j=MIN;j<l;j++)
					if(ch[i*l+j])
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
	}
	else if(x<0&&y>=0)
	{
		for(i=MIN;i<s;i++)
			if(i+y<96)
				for(j=-x;j<l;j++)
					if(ch[i*l+j])
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
	}
	else			
		for(i=-y;i<s;i++)
			if(i+y<96)
				for(j=-x;j<l;j++)
					if(ch[i*l+j])
					if(x+j<152)
						OLED_GRAM[x+j][i+y]=ch[i*l+j];
}
//提示条Gemini\Gemini.axf: Error: L6218E: Undefined symbol NotificationCount (referred from main.o).
u8 GRAM_Data[28*16];
void OLED_Putchar2GRAM(char *ch)
{
	u8 i,j,k,l;
	for(k=0;k<2;k++)
		for(i=0;i<8;i++)
		{
			GRAM_Data[k*8*28+i*28]=0;
			GRAM_Data[k*8*28+i*28+26]=0;
			GRAM_Data[k*8*28+i*28+1]=CovSd[(i+k*8)*4];
			GRAM_Data[k*8*28+i*28+27]=CovSd[(i+k*8)*4];
			for(l=0;l<5;l++)
				for(j=0;j<5;j++)
					GRAM_Data[k*8*28+i*28+l*5+j+2]=CovSd[(i+k*8)*4+(((OCR_F10x16[(ch[l]-' ')*20+10*k+j*2]>>i)&1)<<1)|((OCR_F10x16[(ch[l]-' ')*20+10*k+j*2+1]>>i)&1)];
		}
	GRAM_Data[1]=CovSd[2];
	GRAM_Data[1*8*28+7*28+1]=CovSd[(7+1*8)*4+2];
}	

//u8 NotificationCount;
//OLED_STATUS GRAM_Notification(char *ch)
//{
//	u8 i;
//	static char FRAM_Data[5] ;
//	static char DRAM_Data[5] ;
//	static OLED_STATUS Flag_Busy = OLED_IDLE;
//	
//	if(Flag_Busy == OLED_BUSY)
//	{
//		if(NotificationCount==0)
//		{
//			OLED_Putchar2GRAM(FRAM_Data);
//			OLED_Part(124,0,28,16,GRAM_Data);
//		}
//		else if(NotificationCount<=28)
//			OLED_Part(124+NotificationCount,0,28,16,GRAM_Data);
//		else if(NotificationCount==29)
//		{		
//			OLED_Putchar2GRAM(DRAM_Data);
//		}
//		else if(NotificationCount<=56)
//			OLED_Part(180-NotificationCount,0,28,16,GRAM_Data);
//		else
//		{
//			for(i=0;i<5;i++)
//				FRAM_Data[i] = DRAM_Data[i];
//			Flag_Busy = OLED_IDLE;
//			NotificationCount = 0;
//			OLED_Part(124,0,28,16,GRAM_Data);
//			return Flag_Busy;
//		}
//		NotificationCount ++;
//	}
//	else
//	{
//		OLED_Putchar2GRAM(ch);
//		if(FRAM_Data[4] != ch[4])
//		{
//			Flag_Busy = OLED_BUSY;
//			for(i=0;i<5;i++)
//			{
//				DRAM_Data[i] = ch[i];
//			}
//		}
//		else
//			OLED_Part(124,0,28,16,GRAM_Data);
//	}
//	return Flag_Busy;
//}

u8 Notbarrefsflag = False;
u8 Notbarautoflag = True;

void Showbar(char *ch)
{
	static u8 Notbarshowflag = False;
	static u8 Notbarpossflag = False;
	static u8 RunCount=0;
	static char FRAM_Data[5] ;
	int i;
	if(!Notbarpossflag)
	{
		if(FRAM_Data[4] != ch[4])
		{
			Notbarrefsflag = True;
			for(i=0;i<5;i++)
				FRAM_Data[i] = ch[i];
		}
	}
		
	if(!Notbarrefsflag)
	{
		if(Notbarshowflag)
		{
			OLED_Putchar2GRAM(ch);
			OLED_Part(125,0,28,16,GRAM_Data);
		}
		return;
	}
	if(Notbarshowflag)
	{
		if(RunCount<26)
		{
			Notbarpossflag = True;
			OLED_Part(125+RunCount,0,28,16,GRAM_Data);RunCount++;
		}
		else
		{
			Notbarshowflag = False;
			RunCount= 0;
			if(!Notbarautoflag)
			{
				Notbarpossflag = False;
				Notbarrefsflag = False;
			}
		}
	}
	if(!Notbarshowflag)
	{
		if(RunCount<26)
		{
			OLED_Putchar2GRAM(ch);
			Notbarpossflag = True;
			OLED_Part(150-RunCount,0,28,16,GRAM_Data);RunCount++;
		}
		else
		{
			RunCount= 0;
			Notbarshowflag = True;
			Notbarrefsflag = False;
			Notbarpossflag = False;
			OLED_Part(125,0,28,16,GRAM_Data);
			for(i=0;i<5;i++)
				FRAM_Data[i] = ch[i];
		}	
	}
		
}

//void GRAM_Notification(char *ch)
//{
//	u8 i;
//	static char FRAM_Data[5];
//	if(FRAM_Data[4] != ch[4])
//	{
//		if(NotificationCount==0)
//		{
//			OLED_Putchar2GRAM(FRAM_Data);
//			OLED_Part(124,0,28,16,GRAM_Data);
//		}
//		else if(NotificationCount<=28)
//			OLED_Part(124+NotificationCount,0,28,16,GRAM_Data);
//		else if(NotificationCount==29)
//		{
//			OLED_Putchar2GRAM(ch);
//			
//		}
//		else if(NotificationCount<=56)
//			OLED_Part(180-NotificationCount,0,28,16,GRAM_Data);
//		else
//		{
//			for(i=0;i<5;i++)
//				FRAM_Data[i]=ch[i];
//			NotificationCount = 0;
////			OLED_Putchar2GRAM("     ");
//			OLED_Part(124,0,28,16,GRAM_Data);
//		}
//		NotificationCount++;
//	}
//	else
//	{
//		NotificationCount=0;
//		for(i=0;i<5;i++)
//			FRAM_Data[i]=ch[i];
//		OLED_Putchar2GRAM(FRAM_Data);
//		OLED_Part(124,0,28,16,GRAM_Data);
//	}
//}
//坐标轴
u8 GRAM_Flow[114*96/2];
u8 FRAM_Flow[100];
void Clear_FRAM(void)
{
	u16 i;
	for(i=0;i<100;i++)
		FRAM_Flow[i]=0;
}

OLED_STATUS Flow_Coordinate(u8 Func)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	static int MotionStep = -98;
	if(Func==1)
	{
		if(MotionStep>2)
		{
			MotionStep-=2;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
		OLED_Part(64,MotionStep,57,95,GRAM_Flow);
	}
	else if(Func==2)
	{
		if(MotionStep<0)
		{
			MotionStep+=2;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
		OLED_Part(64,MotionStep+1,57,95,GRAM_Flow);
	}
	else if(Func==0)
	{
		if(MotionStep<96&&MotionStep>=0)
		{
			MotionStep+=3;
			Flag_Busy = OLED_BUSY;
		}
		else
		{
			Flag_Busy = OLED_IDLE;
			MotionStep = -98;
		}
		OLED_Part(64,MotionStep+1,57,95,GRAM_Flow);
	}
	return Flag_Busy;
}

void OLED_RefrashCoordinate(u16 Value)
{
	u8 i,j;
	u16 k;
	for(i=0;i<99;i++)
		FRAM_Flow[i] = FRAM_Flow[i+1];
	if(Value>100)
		Value = 100;
	FRAM_Flow[99] = Value*0.8+1;
	for(k=0;k<114*92/2;k++)
		GRAM_Flow[k+114*4/2] = Corn_Coordinate[k];
	
	//将Flash中的数据传递给RAM
	for(i=0;i<50;i++)
	{
		for(j=0;j<FRAM_Flow[2*i];j++)
		{
//			GRAM_Flow[114*(86-j)/2 + 2 + i+114*4/2] &= 0xF0;
			GRAM_Flow[114*(86-j)/2 + 2 + i+114*4/2] |= 0x0A;
		}
		for(j=0;j<FRAM_Flow[2*i+1];j++)
		{
//			GRAM_Flow[114*(86-j)/2 + 2 + i + 1+114*4/2] &= 0x0F;
			GRAM_Flow[114*(86-j)/2 + 2 + i + 1+114*4/2] |= 0xA0;
		}
	}
}	

void OLED_DisplayCoordinate(void)
{
	OLED_Part(64,1,57,95,GRAM_Flow);
}

u8 ASUSMotionStep = 0;

OLED_STATUS Flow_ASUS(void)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	if(SHOW_CORN.ASUS)
	{
		if(ASUSMotionStep<30)
		{
			Flag_Busy = OLED_BUSY;
			ASUSMotionStep++;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	else 
	{
		if(ASUSMotionStep>0)
		{
			Flag_Busy = OLED_BUSY;
			ASUSMotionStep--;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	OLED_Part(152-ASUSMotionStep,51,30,46,Corn_ASUS); 
	return Flag_Busy;
}

void Out_ASUS(void)
{
	ASUSMotionStep = 0;
}


OLED_STATUS Flow_Corn(void)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	static u8 MotionStep = 0;
	if(SHOW_CORN.CPUGPU)
	{
		if(MotionStep<23)
		{
			MotionStep++;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	else
	{
		if(MotionStep>0)
		{
			MotionStep--;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	OLED_Part(MotionStep-23,0,23,38,Corn_CPU);
	OLED_Part(MotionStep-23,50+2,23,35,Corn_GPU);
	return Flag_Busy;
}
//152-64-28
OLED_STATUS Flow_Line(void)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	static u8 MotionStep = 0;
	if(SHOW_CORN.LINE)
	{
		if(MotionStep<130)
		{
			MotionStep+=2;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
		GUI_HLine(0, 47, MotionStep, 0x88);	
	}
	else
	{
		if(MotionStep>0)
		{
			GUI_HLine(0, 47, MotionStep, 0);	
			MotionStep-=2;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	return Flag_Busy;
}

//void Flow_Top(void)
//{
//	static int MotionStep = 0;
//	if(SHOW_CORN.GAMEBK)
//	{
//		if(MotionStep < 75)
//		{
//			GUI_HLine(0, 0, MotionStep*2, 0xF);	
//			GUI_HLine(0+152, 0, MotionStep*2+152, 0xF);	
//		}
//		else if(MotionStep < 75+31)
//		{
//			GUI_RLine(149, 0,MotionStep-75 ,0xF);
//			GUI_RLine(149+152, 0,MotionStep-75 ,0xF);
//		}
//		else if(MotionStep < 75+31+75)
//		{
//			GUI_HLine(148-(MotionStep-31-75)*2, 30, 148, 0xF);	
//			GUI_HLine(148-(MotionStep-31-75)*2+152, 30, 148+152, 0xF);	
//		}
//		else if(MotionStep < 75+31+75+30)
//		{
//			GUI_RLine(0, 29-(MotionStep-31-75-75),29 ,0xF);
//			GUI_RLine(0+152, 29-(MotionStep-31-75-75),29,0xF);
//		}
//		if(MotionStep < 150+62+150+58)
//			MotionStep+=2;     
//	}
//}


OLED_STATUS Flow_USB(void)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	static u8 MotionStep = 0;
	if(SHOW_CORN.USBD)
	{
		if(MotionStep<28)
		{
			MotionStep++;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
		OLED_Part(152-MotionStep,20,28,16,Corn_USB);
	}
	else
	{
		if(MotionStep>0)
		{
			MotionStep--;
			Flag_Busy = OLED_BUSY;
		}
		else
			Flag_Busy = OLED_IDLE;
	}
	OLED_Part(152-MotionStep,20,28,16,Corn_USB);
	return Flag_Busy;
}
float DampValue[8];
OLED_STATUS OLED_BAR64x10(u8 x,u8 y,u16 Load,u8 Channel)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	Load=(float)Load*60/100;
	Flag_Busy = OLED_BUSY;
	if(DampValue[Channel]>Load)
		DampValue[Channel]--;
	else if(DampValue[Channel]<Load)
		DampValue[Channel]++;
	else
		Flag_Busy = OLED_IDLE;

	GUI_RectangleFill(x,y,x+DampValue[Channel],y,8);
	GUI_RectangleFill(x,y+1,x+DampValue[Channel],y+1,13);
	GUI_RectangleFill(x,y+2,x+DampValue[Channel],y+2,15);
	GUI_RectangleFill(x,y+3,x+DampValue[Channel],y+3,13);
	GUI_RectangleFill(x,y+4,x+DampValue[Channel],y+4,8);
	
	return Flag_Busy;
}

OLED_STATUS OLED_BAR(u8 x,u8 y,u16 w,u16 h,u16 Load,float Step,u8 Channel)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	if(Load>100)
		Load=100;
	Load=(float)Load*w/100;
	Flag_Busy = OLED_BUSY;
	if(DampValue[Channel]>Load+Step)
		DampValue[Channel]-=Step;
	else if(DampValue[Channel]<Load)
		DampValue[Channel]+=Step;
	else
		Flag_Busy = OLED_IDLE;
	if(DampValue[Channel]>w)
		DampValue[Channel] = w;
	else if(DampValue[Channel]<0)
		DampValue[Channel] = 0;
		
	GUI_RectangleFill(x,y,x+DampValue[Channel],y+h-1,15);
	return Flag_Busy;
}

void DampValueClear(void)
{
	u8 i;
	for(i=0;i<8;i++)
		DampValue[i] = 0;
}

u8 DampValueBusy(void)
{
	u8 Out = False;
	u8 i;
	for(i=0;i<8;i++)
	{
		if(DampValue[i]!=0)
			Out = True;
	}
	return Out;
}

float RandMind[2][MINDMAX],MovMind[2][MINDMAX];
float MindStep_X[MINDMAX],MindStep_Y[MINDMAX];
u8 MindNum = 5;
u8 CircleNum = 1;
u8 SnowflakeNum = 1;
u8 MeteoNum = 1;

OLED_STATUS OLED_MovMind(u8 Index)
{
	if(MovMind[0][Index]<=3)
	{	
		MovMind[0][Index] = 4;
		return OLED_IDLE;
	}
	else if(MovMind[0][Index]>=GUI_LCM_XMAX-2)
	{
		MovMind[0][Index] = GUI_LCM_XMAX-3;
		return OLED_IDLE;
	}
	else if(MovMind[1][Index]<=0)
	{
		MovMind[1][Index] = 1;
		return OLED_IDLE;
	}
	else if(MovMind[1][Index]>=GUI_LCM_YMAX-2)
	{
		MovMind[1][Index] = GUI_LCM_YMAX-3;
		return OLED_IDLE;
	}
	else
	{
		MovMind[0][Index] += MindStep_X[Index];
		MovMind[1][Index] += MindStep_Y[Index];
	}
  return OLED_BUSY;
}

u16 GetMindDistanceSquare(u16 x0,u16 y0,u16 x1,u16 y1)
{
	return ((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
}

void Motion_MindInit(void)
{
	static float DistanceX,DistanceY;
	int i;
	for(i=0;i<MINDMAX;i++)
	{
		RandMind[0][i] = rand()%(GUI_LCM_XMAX-4)+4;
		__ASM("NOP");
		RandMind[1][i] = rand()%GUI_LCM_YMAX;
		__ASM("NOP");
		MovMind[0][i] = rand()%(GUI_LCM_XMAX-4)+4;
		__ASM("NOP");
		MovMind[1][i] = rand()%GUI_LCM_YMAX;
		DistanceX = RandMind[0][i] - MovMind[0][i];
		DistanceY = RandMind[1][i] - MovMind[1][i];
		if(DistanceX<0&&DistanceY>=0)
		{
			if(-DistanceX>DistanceY)
			{
				MindStep_X[i]=-1;
				MindStep_Y[i]=-DistanceY/DistanceX;
			}
			else
			{
				MindStep_X[i]=DistanceX/DistanceY;;
				MindStep_Y[i]=1;
			}
		}
		else if(DistanceX>=0&&DistanceY<0)
		{
			if(DistanceX>-DistanceY)
			{
				MindStep_X[i]=1;
				MindStep_Y[i]=DistanceY/DistanceX;
			}
			else
			{
				MindStep_X[i]=-DistanceX/DistanceY;;
				MindStep_Y[i]=-1;
			}
		}
		else if(DistanceX>=0&&DistanceY>=0)
		{
			if(DistanceX>DistanceY)
			{
				MindStep_X[i]=1;
				MindStep_Y[i]=DistanceY/DistanceX;
			}
			else
			{
				MindStep_X[i]=DistanceX/DistanceY;;
				MindStep_Y[i]=1;
			}
		}
		else if(DistanceX<0&&DistanceY<0)
		{
			if(-DistanceX>-DistanceY)
			{
				MindStep_X[i]=-1;
				MindStep_Y[i]=-DistanceY/DistanceX; 
			}
			else
			{
				MindStep_X[i]=-DistanceX/DistanceY;;
				MindStep_Y[i]=-1;
			}
		}
	}
	MindNum = 5;
}


void Motion_Mind(void)
{
	static float DistanceX,DistanceY;
	int i,j;
	for(i=0;i<MindNum;i++)
	{
		if(OLED_MovMind(i) == OLED_IDLE)
		{
			RandMind[0][i] = rand()%(GUI_LCM_XMAX-4)+4;
			__ASM("NOP");
			RandMind[1][i] = rand()%GUI_LCM_YMAX;
			DistanceX = RandMind[0][i] - MovMind[0][i];
			DistanceY = RandMind[1][i] - MovMind[1][i];
			if(DistanceX<0&&DistanceY>=0)
			{
				if(-DistanceX>DistanceY)
				{
					MindStep_X[i]=-1;
					MindStep_Y[i]=-DistanceY/DistanceX;
				}
				else
				{
					MindStep_X[i]=DistanceX/DistanceY;;
					MindStep_Y[i]=1;
				}
			}
			else if(DistanceX>=0&&DistanceY<0)
			{
				if(DistanceX>-DistanceY)
				{
					MindStep_X[i]=1;
					MindStep_Y[i]=DistanceY/DistanceX;
				}
				else
				{
					MindStep_X[i]=-DistanceX/DistanceY;;
					MindStep_Y[i]=-1;
				}
			}
			else if(DistanceX>=0&&DistanceY>=0)
			{
				if(DistanceX>DistanceY)
				{
					MindStep_X[i]=1;
					MindStep_Y[i]=DistanceY/DistanceX;
				}
				else
				{
					MindStep_X[i]=DistanceX/DistanceY;;
					MindStep_Y[i]=1;
				}
			}
			else if(DistanceX<0&&DistanceY<0)
			{
				if(-DistanceX>-DistanceY)
				{
					MindStep_X[i]=-1;
					MindStep_Y[i]=-DistanceY/DistanceX;
				}
				else
				{
					MindStep_X[i]=-DistanceX/DistanceY;;
					MindStep_Y[i]=-1;
				}
			}
			if(MindNum<MINDMAX )
			{
				MindNum++;
			}
		}
	}
	
	for(i=0;i<MindNum;i++)
	{
		for(j=0;j<MindNum;j++)
		{
//			Temp = GetMindDistanceSquare(MovMind[0][i],MovMind[1][i],MovMind[0][j],MovMind[1][j]);
			if((MovMind[0][i]-MovMind[0][j])*(MovMind[0][i]-MovMind[0][j])+(MovMind[1][i]-MovMind[1][j])*(MovMind[1][i]-MovMind[1][j])<1600)
			{
				GUI_Line(MovMind[0][j],MovMind[1][j],MovMind[0][i],MovMind[1][i],2);
			}
		}
	}
	for(i=0;i<MindNum;i++)
		GUI_CircleFill(MovMind[0][i],MovMind[1][i],2,9);
}

u16 FucCircle[3][CIRCLEMAX];

OLED_STATUS OLED_FucCircle(u8 Index,int Size)
{
	static float Motion[CIRCLEMAX]={0};
	if(Motion[Index]<FucCircle[0][Index]%6+16)
	{
		if(Size-Motion[Index]<0)
		{
			Motion[Index] = 0;
			return OLED_IDLE;
		}
		GUI_CircleFill(FucCircle[0][Index],FucCircle[1][Index],Size-Motion[Index],12-Motion[Index]);
		GUI_Circle(FucCircle[0][Index],FucCircle[1][Index],Size+Motion[Index],12-Motion[Index]);
		Motion[Index]+=0.4;
		return OLED_BUSY;
	}
	else
	{
		Motion[Index] = 0;
		return OLED_IDLE;
	}
}
void Motion_CircleInit(void)
{
	int i;
	for(i=0;i<CIRCLEMAX;i++)
	{
		FucCircle[0][i] = rand()%(GUI_LCM_XMAX-12)+6;
		__ASM("NOP");
		FucCircle[1][i] = rand()%GUI_LCM_YMAX;
		__ASM("NOP");
		FucCircle[2][i] = rand()%10+2;
	}
	CircleNum=1;
}

void Motion_Circle(void)
{
	int i;
	for(i=0;i<CircleNum;i++)
	{
		if(OLED_FucCircle(i,FucCircle[2][i]) == OLED_IDLE)
		{
			FucCircle[0][i] = rand()%(GUI_LCM_XMAX-12)+6;
			__ASM("NOP");
			FucCircle[1][i] = rand()%GUI_LCM_YMAX;
			__ASM("NOP");
			FucCircle[2][i] = rand()%10+2;

			if(CircleNum<CIRCLEMAX )
			{
				CircleNum++;
			}
		}
	}
}

u16 MovSnowflake[5][STARMAX];

OLED_STATUS OLED_MovSnowflake(u8 Index)
{
	 int32   dx;						// 直线x轴差值变量
    int32   dy;          			// 直线y轴差值变量
    int8    dx_sym;					// x轴增长方向，为-1时减值方向，为1时增值方向
    int8    dy_sym;					// y轴增长方向，为-1时减值方向，为1时增值方向
    int32   dx_x2;					// dx*2值变量，用于加快运算速度
    int32   dy_x2;					// dy*2值变量，用于加快运算速度
   static int32   di[STARMAX];						// 决策变量
//   if(Mo150vSnowflake[1][Index]==0)
	dx = MovSnowflake[3][Index] - MovSnowflake[0][Index] ;			// 求取两点之间的差值
	 
   dy = (GUI_LCM_YMAX-1) - MovSnowflake[1][Index];
	
		if(MovSnowflake[1][Index]>90)
			return OLED_IDLE;
	
   /* 判断增长方向，或是否为水平线、垂直线、点 */
   if(dx>0)							// 判断x轴方向
   {  dx_sym = 1;					// dx>0，设置dx_sym=1
   }
   else
   {  if(dx<0)
      {  dx_sym = -1;				// dx<0，设置dx_sym=-1
      }
      else
      {  // dx==0，画垂直线，或一点
				
					 MovSnowflake[1][Index]+=1;
      	 return OLED_BUSY;
      }
   }
   
   if(dy>0)							// 判断y轴方向
   {  dy_sym = 1;					// dy>0，设置dy_sym=1
   }
   else
   {  if(dy<0)
      {  dy_sym = -1;				// dy<0，设置dy_sym=-1
      }
      else
      {  // dy==0，画水平线，或一点
				
					 MovSnowflake[0][Index]+=1;
      	 return OLED_BUSY;
      }
   }
    
   /* 将dx、dy取绝对值 */
   dx = dx_sym * dx;
   dy = dy_sym * dy;
 
   /* 计算2倍的dx及dy值 */
   dx_x2 = dx*2;
   dy_x2 = dy*2;
   
   /* 使用Bresenham法进行画直线 */
   if(dx>=dy)						// 对于dx>=dy，则使用x轴为基准
   {
			di[Index] = dy_x2 - dx;

         MovSnowflake[0][Index] += dx_sym;
         if(di[Index]<0)
         {  di[Index] += dy_x2;			// 计算出下一步的决策值
         }
         else
         {  di[Index] += dy_x2 - dx_x2;
            MovSnowflake[1][Index] += dy_sym;
         }
   }
   else								// 对于dx<dy，则使用y轴为基准
   {  
		 di[Index] = dx_x2 - dy;

         MovSnowflake[1][Index] += dy_sym;
         if(di[Index]<0)
         {  di[Index] += dx_x2;
         }
         else
         {  di[Index] += dx_x2 - dy_x2;
            MovSnowflake[0][Index] += dx_sym;
         }
   } 
  return OLED_BUSY;
}

void Motion_SnowflakeInit(void)
{
	int i;
	for(i=0;i<STARMAX;i++)
	{
			MovSnowflake[0][i] = rand()%(GUI_LCM_XMAX-12)+6;
			MovSnowflake[1][i] = 1;
			MovSnowflake[3][i] = MovSnowflake[0][i]+32;
			__ASM("NOP");
			MovSnowflake[2][i] = rand()%3;
			MovSnowflake[4][i] = rand()%14+1;
	}
	SnowflakeNum = 1;
}

void Motion_Snowflake(void)
{
	int i,j;
//	GUI_Line(MovSnowflake[0][0],MovSnowflake[1][i],MovSnowflake[3][i],95,15);
	for(i=0;i<SnowflakeNum;i++)
	{
		for(j=0;j<MovSnowflake[4][i]%5+1;j++)
		{
			if(OLED_MovSnowflake(i) == OLED_IDLE)
			{
				MovSnowflake[0][i] = rand()%(GUI_LCM_XMAX-12)+6;
				MovSnowflake[1][i] = 1;
				MovSnowflake[3][i] = MovSnowflake[0][i]+32;
				__ASM("NOP");
				MovSnowflake[2][i] = rand()%3;
				MovSnowflake[4][i] = rand()%14+1;
				if(SnowflakeNum<STARMAX )
				{
					SnowflakeNum++;
				}
			}
		}
		switch(MovSnowflake[2][i])
		{
			case 0:GUI_CircleFill(MovSnowflake[0][i],MovSnowflake[1][i],1,MovSnowflake[4][i]);break;
			case 1:GUI_CircleFill(MovSnowflake[0][i],MovSnowflake[1][i],1,MovSnowflake[4][i]);GUI_Point(MovSnowflake[0][i],MovSnowflake[1][i],0);break;
			case 2:GUI_Point(MovSnowflake[0][i],MovSnowflake[1][i],MovSnowflake[4][i]);break;
		}
	}
}

float MovmeteorStep = 60;
u16 Movmeteor[6][METEORMAX];
float MovmeteorStep_X;
float Movmeteor_X[METEORMAX];

OLED_STATUS OLED_Movmeteor(u8 Index)
{
	Movmeteor[1][Index] ++;
	Movmeteor_X[Index] += MovmeteorStep_X;
	if(Movmeteor[1][Index]>GUI_LCM_YMAX)
		return OLED_IDLE;
	else
		return OLED_BUSY;
}

void Motion_MovmeteorInit(void)
{
	int i;
	for(i=0;i<METEORMAX;i++)
	{
			Movmeteor[0][i] = rand()%GUI_LCM_XMAX;
			Movmeteor_X[i] = Movmeteor[0][i];
			Movmeteor[1][i] = 0;
			Movmeteor[2][i] = rand()%5+1;//Speed
			Movmeteor[3][i] = rand()%5;//Type
			Movmeteor[4][i] = rand()%15;//LineColor
			Movmeteor[5][i] = Movmeteor[4][i] + 1;//SnowflakeColor
	}
	MeteoNum = 1;
	MovmeteorStep_X = MovmeteorStep / GUI_LCM_YMAX;
}

void Motion_Movmeteor(void)
{
	int i,j;
	for(i=0;i<MeteoNum;i++)
	{
		for(j=0;j<Movmeteor[2][i];j++)
		{
			if(OLED_Movmeteor(i) == OLED_IDLE)
			{
				Movmeteor[0][i] = rand()%GUI_LCM_XMAX;
				Movmeteor_X[i] = Movmeteor[0][i];
				Movmeteor[1][i] = 0;
				Movmeteor[2][i] = rand()%5+1;//Speed
				Movmeteor[3][i] = rand()%4;//Type
				Movmeteor[4][i] = rand()%15+1;//LineColor
				Movmeteor[5][i] = Movmeteor[4][i];//SnowflakeColor
				if(MeteoNum<METEORMAX )
				{
					MeteoNum++;
				}
			}
		}
		if(Movmeteor[4][i]>1)
			GUI_Line(Movmeteor[0][i],0,Movmeteor_X[i],Movmeteor[1][i],Movmeteor[4][i]--);
		else if((Movmeteor[1][i]-(Movmeteor[2][i]*3))>0)
			GUI_Line((Movmeteor[1][i]-(Movmeteor[2][i]*3))*MovmeteorStep_X+Movmeteor[0][i],Movmeteor[1][i]-(Movmeteor[2][i]*3),Movmeteor_X[i],Movmeteor[1][i],3);
		switch(Movmeteor[3][i])
		{
			case 0:GUI_Point(Movmeteor_X[i],Movmeteor[1][i],Movmeteor[5][i]);break;
			case 1:GUI_Point(Movmeteor_X[i]+1,Movmeteor[1][i],Movmeteor[5][i]);GUI_Point(Movmeteor_X[i],Movmeteor[1][i]+1,Movmeteor[5][i]);break;
			case 2:GUI_CircleFill(Movmeteor_X[i],Movmeteor[1][i],1,Movmeteor[5][i]);break;
			case 3:GUI_Circle(Movmeteor_X[i],Movmeteor[1][i],1,Movmeteor[5][i]);GUI_Point(Movmeteor_X[i],Movmeteor[1][i],0);break;
		}
	}
}

void OLED_AllMotion_Init(void)
{
	 Motion_MindInit();
	 Motion_CircleInit();
	 Motion_SnowflakeInit();
	 Motion_MovmeteorInit();
}

void OLED_CustormMotion(u8 Channel)
{
	switch(Channel)
	{
	case 0:Motion_Mind();break;
	case 1:Motion_Movmeteor();break;
	case 2:Motion_Snowflake();break;
	case 3:Motion_Circle();break;
	}
}

void OLED_AllMotion(u8 Type,u16 Speed)
{
//	static int SwitchLst = -1;
	static u32 Count = 0;
	static u8 NowMotion = 0;
	static u16 SpeedSave = 0;
	Speed++;
	if(SpeedSave!=Speed)
	{
		SpeedSave = Speed;Count = 0;
	}
	Count++;

//	if(SwitchFlag)
//	{
//		OLED_AllMotion_Init();
//		SwitchFlag = False;
//	}
	if((Type&0xf) == 0)
		return;
	
	switch(Count/(5000-Speed*40))
	{
		case 0:
		if(Type&8){
 CircleNum = 1;
 SnowflakeNum = 1;
 MeteoNum = 1;NowMotion=0; }else {Count=(5000-Speed*40);}   break;
		case 1:
		if(Type&4){ 
 MindNum = 5;
 CircleNum = 1;
 SnowflakeNum = 1;NowMotion=1;}else {Count=(5000-Speed*40)*2;}   break;
		case 2:
		if(Type&2){
 MindNum = 5;
 CircleNum = 1;
 MeteoNum = 1; NowMotion=2;}else {Count=(5000-Speed*40)*3;}   break;
		case 3:
		if(Type&1){
 MindNum = 5;
 MeteoNum = 1;
 SnowflakeNum = 1; NowMotion=3;}else {Count=0;}   break;
		default:Count=0;break;
	}OLED_CustormMotion(NowMotion);
}

float DefLineStep = 64;
u16 DefLine[5][DEFLINEMAX];
float DefLineStep_X;
float DefLine_X[DEFLINEMAX];

OLED_STATUS OLED_DefLine(u8 Index)
{
	DefLine[1][Index] ++;
	DefLine_X[Index] += DefLineStep_X;
	if(DefLine[1][Index]>GUI_LCM_YMAX)
		return OLED_IDLE;
	else
		return OLED_BUSY;
}


OLED_STATUS Flow_BTV(void)
{
	static OLED_STATUS Flag_Busy = OLED_IDLE;
	static u8 MotionStep = 0;
	static u8 delays = 0;
	if(SHOW_CORN.BTV)
	{
		if(delays>5)
		{
			if(MotionStep<10)
			{
				MotionStep++;
				Flag_Busy = OLED_BUSY;
				OLED_Part(BTVPostionX,BTVPostionY+60-MotionStep*6,28,63,MotionBiTV[1]);
			}
			else
			{
				Flag_Busy = OLED_IDLE;
				Motion_BTV(BTVPostionX,BTVPostionY);
			}
		}
		else
			delays++;
	}
	else
	{
		if(MotionStep>0)
		{
			MotionStep--;
			Flag_Busy = OLED_BUSY;
			OLED_Part(BTVPostionX,BTVPostionY+60-MotionStep*6,28,63,MotionBiTV[6]);
		}
		else
		{
			delays = 0;
			Flag_Busy = OLED_IDLE;
		}
	}
	return Flag_Busy;
}

void Motion_BTV(u8 x,u8 y)
{
	static u16 Step = 6;
	OLED_Part(x,y,28,63,MotionBiTV[Step>>1]);
	if(Step++>=13)
		Step = 0;
}

void Motion_VOICE(u8 x,u8 y)
{
	static u16 Step = 0,VoiceIndex;
	SystemActive = False;
	SPI_Flash_Read(BootGRAM,2048000+Step*80*40+VoiceIndex*547200,80*40);
	SystemActive = True;   
	OLED_Dim(x,y,40,80,BootGRAM);
	Step+=2;
	if(Step>170)
	{
		Step = 0;
		if(!Clock_Msg.aiwork)
		{
			VoiceFlag = False;
			SHOW_CORN.BTV  = True;
			SHOW_CORN.ASUS = True;
		}
		VoiceIndex++;
		if(VoiceIndex==12)
			VoiceIndex=0;
	}
}

void Motion_Shab(u8 x,u8 y)
{
	static u16 Step = 0;
	OLED_Part(x,y,28,56,MotionShab[Step>>1]);
	if(Step++>=52)
		Step = 0;
}

void SportUIMt(int Step)
{
	int i;
	if(Step<150)
	{
		GUI_HLine(152+0,60,152+Step,15);
	}
	else
	{
		GUI_HLine(152+0,60,152+150,15);
		if(Step<185)
		{
			GUI_RLine(152+150,60,60+Step-150,15);
		}
		else
		{
			GUI_RLine(152+150,60,95,15);
			if(Step<335)
			{
				GUI_HLine(152+150,95,152+150-(Step-185),15);
			}
			else
			{
				GUI_HLine(152+150,95,152,15);
				if(Step<370)
				{
					GUI_RLine(152+0,95,95-(Step-335),15);
				}
				else
				{
					GUI_RLine(152+0,95,60,15);
				}
			}
		}
	}
	
	
	if(Step<150)
	{
		GUI_HLine(0,0,Step,15);
		GUI_HLine(152+0,0,152+Step,15);
	}
	else
	{
		GUI_HLine(0,0,150,15);
		GUI_HLine(152+0,0,152+150,15);
		if(Step<180)
		{
			GUI_RLine(150,0,Step-150,15);
			GUI_RLine(152+150,0,Step-150,15);
		}
		else
		{
			GUI_RLine(150,0,30,15);
			GUI_RLine(152+150,0,30,15);
			if(Step<330)
			{
				GUI_HLine(150,30,150-(Step-180),15);
				GUI_HLine(152+150,30,152+150-(Step-180),15);
			}
			else
			{
				GUI_HLine(150,30,0,15);
				GUI_HLine(152+150,30,152,15);
				if(Step<360)
				{
					GUI_RLine(0,30,30-(Step-330),15);
					GUI_RLine(152+0,30,30-(Step-330),15);
				}
				else
				{
					GUI_RLine(0,30,0,15);
					GUI_RLine(152+0,30,0,15);
				}
			}
		}
	}

	if(Step/2<71)
	{
		GUI_HLine(0,33,Step/2,15);
		GUI_HLine(152+0,33,152+Step/2,15);
	}
	else
	{
		GUI_HLine(0,33,71,15);
		GUI_HLine(152+0,33,152+71,15);
		if(Step/2<(71+23))
		{
			GUI_RLine(71,33,33+Step/2-71,15);
			GUI_RLine(152+71,33,33+Step/2-71,15);
		}
		else
		{
			GUI_RLine(71,33,33+23,15);
			GUI_RLine(152+71,33,33+23,15);
			if(Step/2<(71+23+71))
			{
				GUI_HLine(71,33+23,71-(Step/2-71-23),15);
				GUI_HLine(152+71,33+23,152+72-(Step/2-71-23),15);
			}
			else
			{
				GUI_HLine(71,33+23,0,15);
				GUI_HLine(152+71,33+23,152+0,15);
				if(Step/2<(71+23+71+23))
				{
					GUI_RLine(0,33+23,33+23-(Step/2-71-23-71),15);
					GUI_RLine(152+0,33+23,33+23-(Step/2-71-23-71),15);
				}
				else
				{
					GUI_RLine(0,33+23,33,15);
					GUI_RLine(152+0,33+23,33,15);
				}
			}
		}
	}
	
	if(Step/2<48)
	{
		GUI_HLine(0,60,Step/2,15);
		GUI_HLine(50,60,50+Step/2,15);
		GUI_HLine(100,60,100+Step/2,15);
	}
	else
	{
		GUI_HLine(0,60,48,15);
		GUI_HLine(50,60,50+48,15);
		GUI_HLine(100,60,100+48,15);
		if(Step/2<(48+35))
		{
			GUI_RLine(48,60,60+Step/2-48,15);
			GUI_RLine(48+50,60,60+Step/2-48,15);
			GUI_RLine(48+100,60,60+Step/2-48,15);
		}
		else
		{
			GUI_RLine(48,60,60+35,15);
			GUI_RLine(48+50,60,60+35,15);
			GUI_RLine(48+100,60,60+35,15);
			if(Step/2<(48+35+48))
			{
				GUI_HLine(48+0,95,48-(Step/2-48-35),15);
				GUI_HLine(48+50,95,50+48-(Step/2-48-35),15);
				GUI_HLine(48+100,95,100+48-(Step/2-48-35),15);
			}
			else
			{
				GUI_HLine(48+0,95,0,15);
				GUI_HLine(48+50,95,50,15);
				GUI_HLine(48+100,95,100,15);
				if(Step/2<(48+35+48+35))
				{
					GUI_RLine(0,95,95-(Step/2-48-35-48),15);
					GUI_RLine(50,95,95-(Step/2-48-35-48),15);
					GUI_RLine(100,95,95-(Step/2-48-35-48),15);
				}
				else
				{
					GUI_RLine(0,95,60,15);
					GUI_RLine(50,95,60,15);
					GUI_RLine(100,95,60,15);
				}
			}
		}
	}
		
	if(Step/4<14)
			for(i=0;i<11;i++)
			{
				GUI_RLine(5+i*14,12,12+Step/4,0x1);
				GUI_RLine(5+i*14+152,12,12+Step/4,0x1);
			}
	else
			for(i=0;i<11;i++)
			{
				GUI_RLine(5+i*14,12,12+14,0x1);
				GUI_RLine(5+i*14+152,12,12+14,0x1);
			}
		
	if(Step/4<20)
	{
			for(i=0;i<11;i++)
			{
				GUI_RLine(5+i*14+152,73,73+Step/4,0x1);
			}
	}
	else
			for(i=0;i<11;i++)
			{
				GUI_RLine(5+i*14+152,73,73+20,0x1);
			}

	if(Step<146)
	{
		GUI_HLine(2+0,2,Step,15);
		GUI_HLine(2+152,2,Step+152,15);

		GUI_HLine(148+0,28,148-Step,15);
		GUI_HLine(148+152,28,148-Step+152,15);
	}
	else
	{
		GUI_HLine(2+0,2,148,15);
		GUI_HLine(2+152,2,148+152,15);

		GUI_HLine(148+0,28,2,15);
		GUI_HLine(148+152,28,2+152,15);
	}
}

void SportUI2Mt(int Step)
{
	if(Step/2>10)
	{
		OLED_XF4x5(37+4*0,41,0);
		OLED_XF4x5(76+37+4*0,41,0);
		OLED_XF4x5(1,6,0);
		OLED_XF4x5(76+1,6,0);
		OLED_XF4x5(76+1,67,0);
	}
	if(Step/2>20)
	{
		OLED_XF4x5(37+4*1,41,1);
		OLED_XF4x5(76+37+4*1,41,1);
		OLED_XF4x5(1+7*1,6,1);
		OLED_XF4x5(76+1+7*1,6,1);
		OLED_XF4x5(76+1+7*1,67,1);
	}
	if(Step/2>30)
	{
		OLED_XF4x5(37+4*2,41,2);
		OLED_XF4x5(76+37+4*2,41,2);
		OLED_XF4x5(1+7*2,6,2);
		OLED_XF4x5(76+1+7*2,6,2);
		OLED_XF4x5(76+1+7*2,67,2);
	}
	if(Step/2>40)
	{
		OLED_XF4x5(37+4*3,41,3);
		OLED_XF4x5(76+37+4*3,41,3);
		OLED_XF4x5(1+7*3,6,3);
		OLED_XF4x5(76+1+7*3,6,3);
		OLED_XF4x5(76+1+7*3,67,3);
	}
	if(Step/2>50)
	{
		OLED_XF4x5(37+4*4,41,4);
		OLED_XF4x5(76+37+4*4,41,4);
		OLED_XF4x5(1+7*4,6,4);
		OLED_XF4x5(76+1+7*4,6,4);
		OLED_XF4x5(76+1+7*4,67,4);
	}
	if(Step/2>60)
	{
		OLED_XF4x5(37+4*5,41,5);
		OLED_XF4x5(76+37+4*5,41,5);
		OLED_XF4x5(1+7*5,6,5);
		OLED_XF4x5(76+1+7*5,6,5);
		OLED_XF4x5(76+1+7*5,67,5);
	}
	if(Step/2>70)
	{
		OLED_XF4x5(37+4*6,41,6);
		OLED_XF4x5(76+37+4*6,41,6);
		OLED_XF4x5(1+7*6,6,6);
		OLED_XF4x5(76+1+7*6,6,6);
		OLED_XF4x5(76+1+7*6,67,6);
	}
	if(Step/2>80)
	{
		OLED_XF4x5(37+4*7,41,7);
		OLED_XF4x5(76+37+4*7,41,7);
		OLED_XF4x5(1+7*7,6,7);
		OLED_XF4x5(76+1+7*7,6,7);
		OLED_XF4x5(76+1+7*7,67,7);
	}
	if(Step/2>90)
	{
		OLED_XF4x5(37+4*8,41,8);
		OLED_XF4x5(76+37+4*8,41,8);
		OLED_XF4x5(1+7*8,6,8);
		OLED_XF4x5(76+1+7*8,6,8);
		OLED_XF4x5(76+1+7*8,67,8);
	}
	if(Step/2>100)
	{
		OLED_XF4x5(37+4*9,41,9);
		OLED_XF4x5(76+37+4*9,41,9);
		OLED_XF4x5(1+7*9,6,9);
		OLED_XF4x5(76+1+7*9,6,9);
		OLED_XF4x5(76+1+7*9,67,9);
	}
	if(Step/2>110)
	{
		OLED_XF4x5(7*10,6,1);
		OLED_XF4x5(2+7*10,6,0);
		OLED_XF4x5(76+7*10,6,1);
		OLED_XF4x5(76+2+7*10,6,0);
		OLED_XF4x5(76+7*10,67,1);
		OLED_XF4x5(76+2+7*10,67,0);
	}
	
	
	if(Step>75)
	{
		OLED_Clar(31,0,13,5,Corn_SCPU);
		OLED_Clar(31+76,0,13,5,Corn_SGPU);
		
		OLED_Clar(9,33,18,5,Corn_SFANRAM);
		OLED_Clar(9+76,33,18,5,Corn_SFANRAM);
		
		OLED_Clar(50,33,12,5,Corn_SFAN);
		OLED_Clar(50+76,33,12,5,Corn_SFAN);
		
		OLED_Clar(7,60,11,5,Corn_SFENQ);
		OLED_Clar(33,60,11,5,Corn_STEMP);
		OLED_Clar(56,60,13,5,Corn_SSTATE);
		OLED_Clar(29+76,60,14,5,Corn_SRAM);
	}
}


