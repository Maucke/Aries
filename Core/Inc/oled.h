/* !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^-'abcdefghijklmnopqrstuvwxyz*/

#ifndef __OLED_H
#define __OLED_H		

#include "stm32f1xx_hal.h"
#include "sys.h"
#include "stdlib.h"	    
		    						  
//-----------------OLED端口定义----------------  					   
#define OLED_CS PBout(3)
#define OLED_RST PBout(4)
#define OLED_A0 PBout(5)
#define OLED_WR PBout(6)
#define OLED_RD PBout(7)
//#define OLED_CS PCout(8)
//#define OLED_RST PCout(9)
//#define OLED_A0 PCout(10)
//#define OLED_WR PCout(11)
//#define OLED_RD PCout(12)  

#define DATAOUT(x) GPIOC->ODR = x; //数据输出
#define DATAIN     GPIOC->IDR;   //数据输入

#define OLED_CMD  0		    //写命令
#define OLED_DATA 1				//写数据

#define MIN 0
#define START_PAGE 0xB0
#define ROW_TOTAL 0x60
#define START_HIGH_BIT 0x11
#define START_LOW_BIT 0x04
#define STATE_MAX 0xFF
#define STATE_MIN 0x00	
#define COLUMN_MAX 152  

extern const unsigned char Corn_RYZEN[];
extern const unsigned char Corn_CPU[];
extern const unsigned char Corn_GPU[];
extern const unsigned char Corn_Temp[];
extern const unsigned char Corn_Freq[];
extern const unsigned char Corn_Load[];
extern const unsigned char Corn_ASUS[];
extern const unsigned char Corn_USB[];
extern const unsigned char Corn_USBOffline[];
extern const unsigned char Corn_Coordinate[];
extern const unsigned char UI_Sport[];
extern const unsigned char Corn_SCPU[];
extern const unsigned char Corn_Test[];
extern const unsigned char Corn_SGPU[];
extern const unsigned char Corn_SFANRAM[];
extern const unsigned char Corn_SRAM[];
extern const unsigned char Corn_SFAN[];
extern const unsigned char Corn_SFENQ[];
extern const unsigned char Corn_STEMP[];
extern const unsigned char Corn_SSTATE[];
extern const unsigned char Full_Test[];
extern const unsigned char Show_Load[];
extern const unsigned char Agency_F22x40[];
extern const unsigned char Agency_F12x24[];
extern const unsigned char MotionBiTV[][1764];
extern const unsigned char MotionShab[][1568];

extern const unsigned char RobotB_F32x46[];
extern const unsigned char Robot_F32x46[];
extern const unsigned char RobotT_F32x46[];
extern uint8_t OLED_GRAM[152][96];	
extern uint8_t OLED_DRAM[152][96];	
extern u8 NotificationCount ;

typedef enum
{
  OLED_OK       = 0x00U,
  OLED_ERROR    = 0x01U,
  OLED_BUSY     = 0x02U,
  OLED_IDLE     = 0x03U,
} OLED_STATUS;

typedef enum
{
	HIDE,
	APPEAR
} CORN_STATUS;
	

typedef struct
{
	u8 ASUS;
	u8 CPUGPU;
	u8 LINE;
	u8 USBD;
	u8 GAMEBK;
	u8 BTV;
} DISPLAY_CORN;

extern DISPLAY_CORN SHOW_CORN;
extern u8 OLEDBk;
extern u8 Notbarrefsflag ;
extern u8 Notbarautoflag ;
extern u16 OLEDBright;
void Showbar(char *ch);
void OLED_WR_CMD(u8 cmd);
void OLED_WR_DATA(u8 dat);
void OLED_Force_Dram(void);
void OLED_Force_Dram_part(u16 x,u16 y,u16 l,u16 s);
void OLED_Damp_Dram(void);
void OLED_Damp_Dram_Part(u16 x,u16 y,u16 l,u16 s);
void OLED_Refresh_Gram(void);
void OLED_Refresh_Gram_Part(u16 x,u16 y,u16 l,u16 s);
void OLED_Clear(void);
void OLED_Fill(uint8_t dot);
void OLED_Init(void);
void OLED_Full(const u8 *ch);
void OLED_OlRefresh_Gram(void);

void GUI_Point(uint16_t x,uint8_t y,uint8_t dot);
void OLED_Part(int x,int y,u16 l,u16 s,const u8 *ch);
void OLED_Clar(int x,int y,u16 l,u16 s,const u8 *ch);
void OLED_Dim(int x,int y,u16 l,u16 s,const u8 *ch);

void OLED_XF4x5(u8 x,u8 y,u8 Num);
void OLED_SF8x16(int x,int y,char *ch);
void OLED_SF10x16(int x,int y,char *ch);
void OLED_SF22x40(int x,int y,char *ch);
void OLED_SF12x24(int x,int y,char *ch);
void OLED_SF32x46(int x,int y,char *ch,char Type);

void OLED_ShowCoordinate(u16 Value);
void OLED_RefrashCoordinate(u16 Value);
void OLED_DisplayCoordinate(void);
void Clear_FRAM(void);

void Motion_BTV(u8 x,u8 y);
void Motion_VOICE(u8 x,u8 y);
void Motion_Shab(u8 x,u8 y);
OLED_STATUS GRAM_Notification(char *ch);

OLED_STATUS Flow_ASUS(void);
void Out_ASUS(void);

OLED_STATUS Flow_BTV(void);
OLED_STATUS Flow_Corn(void);
OLED_STATUS Flow_Line(void);
OLED_STATUS Flow_USB(void);
OLED_STATUS Flow_Ryzen(void);
OLED_STATUS Flow_Coordinate(u8 Func);
//void Flow_Top(void);

OLED_STATUS OLED_BAR64x10(u8 x,u8 y,u16 Load,u8 Channel);

OLED_STATUS OLED_BAR(u8 x,u8 y,u16 w,u16 h,u16 Load,float Step,u8 Channel);
#define MINDMAX 20
#define CIRCLEMAX 8
#define STARMAX 30
#define METEORMAX 20
#define DEFLINEMAX 30
void OLED_Convert(int x,int y,u8 w,u8 *ch);
OLED_STATUS OLED_MovMind(u8 Index);
void Motion_MindInit(void);
void Motion_CircleInit(void);
void Motion_SnowflakeInit(void);
void Motion_MovmeteorInit(void);
void Motion_Mind(void);
void Motion_Circle(void);
void Motion_Snowflake(void);
void Motion_Movmeteor(void);

void SportUIMt(int Step);
void SportUI2Mt(int Step);
void DampValueClear(void);
u8 DampValueBusy(void);


void OLED_Tranf_Gram(const u8 *ch);

void OLED_AllMotion_Init(void);
void OLED_AllMotion(u8 Type,u16 Speed);

extern u8 MindNum;
extern u8 CircleNum;
extern u8 SnowflakeNum;
extern u8 MeteoNum;

#endif  
	 



