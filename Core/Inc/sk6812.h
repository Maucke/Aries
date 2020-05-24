
#ifndef _WS2812_H_
#define _WS2812_H_


/*******************************通信接口配置***********************************/
#include "sys.h"
#include "main.h"

extern u8 RGBTAG[64][3];
extern float RGBRAM[64][3];
extern u16 RGBBright;
#define MaxRGBLium 255

#if MARKC == 1
#define RGBIndex 64
#else
#define RGBIndex 13
#endif

#define WSDATA PBout(9)
void SK6812_Write(u8 data1,u8 data2,u8 data3);
void SK6812_Wheel(u8 Type,u16 WheelPos);
void SK6812_WheelS(u8 Type,u16 WheelPos);
void SK6812_send_L(void);
void SK6812_send_H(void);
void SK6812_Refrash(void);
void SK6812_Flow(void);
void SK6812_Same(u8 data1,u8 data2,u8 data3);
void SK6812_Init(void);
void SK6812_WheelAll(u16 WheelPos);

#endif

//-------------------------THE END------------------------*/   

