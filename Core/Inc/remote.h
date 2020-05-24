#ifndef __REMOTE_H
#define __REMOTE_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//红外遥控解码驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/5/31
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define RDATA   PAin(2)		//红外数据输入脚

//红外遥控识别码(ID),每款遥控器的该值基本都不一样,但也有一样的.
//我们选用的遥控器识别码为0
#define REMOTE_ID 0      		   

extern u8 RemoteFlag;	        //按键按下的次数
extern u8 RmtCnt;	        //按键按下的次数
extern u8 RmtSta;
void Remote_Init(void);     //红外传感器接收头引脚初始化
u8 Remote_Scan(void);

#define IRF_PW 0x80

#define IRF_1 0xA8
#define IRF_2 0x68
#define IRF_3 0xE8
#define IRF_4 0x98
#define IRF_5 0x58
#define IRF_6 0xD8
#define IRF_7 0xB8
#define IRF_8 0x78
#define IRF_9 0xF8
#define IRF_0 0x82
#define IRF_NL 0xD2
#define IRF_CH 0x22

#define IRF_PC 0xA2
#define IRF_TV 0xA0

#define IRF_LF 0x08
#define IRF_RT 0x30
#define IRF_UP 0x18
#define IRF_DN 0x38
#define IRF_OK 0x50

#define IRF_MT 0x20
#define IRF_IF 0x02

#define IRF_PIP 0x48
#define IRF_SLP 0xC8
#define IRF_WID 0x28

#endif
