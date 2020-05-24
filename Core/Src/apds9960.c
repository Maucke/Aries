/*******************************************************************************
 * ----------------------------------------------------------------------------*
 *  elektronikaembedded@gamil.com ,https://elektronikaembedded.wordpress.com   *
 * ----------------------------------------------------------------------------*
 *                                                                             *
 * File Name  : apds9960.c                                                     *
 *                                                                             *
 * Description : APDS9960 IR Gesture Driver(Library for the SparkFun APDS-9960 breakout board)*
 *               SparkFun_APDS-9960.cpp Modified apds9960.c                    *
 * Version     : PrototypeV1.0                                                 *
 *                                                                             *
 * --------------------------------------------------------------------------- *
 * Authors: Sarath S (Modified Shawn Hymel (SparkFun Electronics))             *
 * Date: May 16, 2017                                                          *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* MCU Files */
/*Std Library Files */
#include "stdlib.h"
/* User Files */
#include "apds9960.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
gesture_data_type gesture_data_;
int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define APDS9960_ADD_BASS					0x39	//器件基地址
#define APDS9960_Write_ADD				(APDS9960_ADD_BASS | 0x00)	//写APDS9960
#define APDS9960_Read_ADD					(APDS9960_ADD_BASS | 0x01)	//读APDS9960
void APDS9960_IIC_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = APDS9960_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void APDS9960_IIC_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = APDS9960_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void APDS9960_delay_us(u16 us)
{
	u8 i,j;
	for(i=0;i<us;i++)
		for(j=0;j<30;j++)
	;
}

/*I2C起始*/
void APDS9960_IIC_Start(void)
{
	APDS9960_IIC_SDA_OUT();	

	APDS9960_SDA_H();
	APDS9960_delay_us(2);	
	APDS9960_SCL_H();
	APDS9960_delay_us(2);		
	APDS9960_SDA_L();
	APDS9960_delay_us(2);	
	APDS9960_SCL_L();
	APDS9960_delay_us(2);
}
/*I2C停止*/
void APDS9960_IIC_Stop(void)
{
	APDS9960_IIC_SDA_OUT();	

	APDS9960_SDA_L();	
	APDS9960_delay_us(2);
	APDS9960_SCL_L();	
	APDS9960_delay_us(2);
	APDS9960_SDA_H();
	APDS9960_delay_us(2);
}
/*I2C发送应答*/
void APDS9960_IIC_Ack(u8 a)
{
	APDS9960_IIC_SDA_OUT();	

	if(a)	
		APDS9960_SDA_H();
	else	
		APDS9960_SDA_L();

	APDS9960_delay_us(2);
	APDS9960_SCL_H();	
	APDS9960_delay_us(2);
	APDS9960_SCL_L();
	APDS9960_delay_us(2);

}
/*I2C写入一个字节*/
u8 APDS9960_IIC_Write_Byte(u8 dat)
{
	u8 i;
	u8 APDS9960_IIC_Ack=0;	

	APDS9960_IIC_SDA_OUT();	

	for(i = 0;i < 8;i++)
	{
		if(dat & 0x80)	
			APDS9960_SDA_H();
		else
			APDS9960_SDA_L();
			
		APDS9960_delay_us(2);
		APDS9960_SCL_H();
	    APDS9960_delay_us(2);
		APDS9960_SCL_L();
		dat<<=1;
	}

	APDS9960_SDA_H();	//释放数据线

	APDS9960_IIC_SDA_IN();	//设置成输入

	APDS9960_delay_us(2);
	APDS9960_SCL_H();
	APDS9960_delay_us(2);
	
	APDS9960_IIC_Ack |= APDS9960_IN_SDA();	//读入应答位
	APDS9960_SCL_L();
	return APDS9960_IIC_Ack;	//返回应答信号
}
/*I2C读取一个字节*/
u8 APDS9960_IIC_Read_Byte(void)
{
	u8 i;
	u8 x=0;

	APDS9960_SDA_H();	//首先置数据线为高电平

	APDS9960_IIC_SDA_IN();	//设置成输入

	for(i = 0;i < 8;i++)
	{
		x <<= 1;	//读入数据，高位在前

		APDS9960_delay_us(2);
		APDS9960_SCL_H();	//突变
		APDS9960_delay_us(2);
		
		if(APDS9960_IN_SDA())	x |= 0x01;	//收到高电平

		APDS9960_SCL_L();
		APDS9960_delay_us(2);
	}	//数据接收完成

	APDS9960_SCL_L();

	return x;	//返回读取到的数据
}

/******************************************************************************
* Function Name --> APDS9960某寄存器写入一个字节数据
* Description   --> none
* Input         --> REG_ADD：要操作寄存器地址
*                   dat：要写入的数据
* Output        --> none
* Reaturn       --> none 
******************************************************************************/
u8 APDS9960_Write_Byte(u8 REG_ADD,u8 dat)
{
	APDS9960_IIC_Start();
	if(!(APDS9960_IIC_Write_Byte(APDS9960_Write_ADD)))	//发送写命令并检查应答位
	{
		APDS9960_IIC_Write_Byte(REG_ADD);
		APDS9960_IIC_Write_Byte(dat);	//发送数据
	}
	APDS9960_IIC_Stop();
	return True;	
}
/******************************************************************************
* Function Name --> APDS9960某寄存器读取一个字节数据
* Description   --> none
* Input         --> REG_ADD：要操作寄存器地址
* Output        --> none
* Reaturn       --> 读取到的寄存器的数值 
******************************************************************************/
u8 APDS9960_Read_Byte(u8 REG_ADD,u8 *ReData)
{
	APDS9960_IIC_Start();
	if(!(APDS9960_IIC_Write_Byte(APDS9960_Write_ADD)))	//发送写命令并检查应答位
	{
		APDS9960_IIC_Write_Byte(REG_ADD);	//确定要操作的寄存器
		APDS9960_IIC_Start();	//重启总线
		APDS9960_IIC_Write_Byte(APDS9960_Read_ADD);	//发送读取命令
		*ReData = APDS9960_IIC_Read_Byte();	//读取数据
		APDS9960_IIC_Ack(1);	//发送非应答信号结束数据传送
	}
	APDS9960_IIC_Stop();
	return True;	
}

u8 wireReadDataBlock(uint8_t reg,uint8_t *val,unsigned int len)
{
	unsigned char i = 0;
    APDS9960_IIC_Start(); 
    APDS9960_IIC_Write_Byte(APDS9960_Write_ADD);//发送器件地址+写命令	
    APDS9960_IIC_Write_Byte(reg);	//写寄存器地址
    APDS9960_IIC_Start();
	  APDS9960_IIC_Write_Byte(APDS9960_Read_ADD);//发送器件地址+读命令	
	  while(len)
	  {		 

//		  if(i >= len)    //不能加，否则收不到数据
//		  {
//        return -1;
//      }
		  if(len==1) {val[i]=APDS9960_IIC_Read_Byte();
					APDS9960_IIC_Ack(0);}//读数据,发送nACK 
		  else{ val[i]=APDS9960_IIC_Read_Byte();		
					APDS9960_IIC_Ack(1);} //读数据,发送ACK 
		i++; 
		len--;
	  }  
    APDS9960_IIC_Stop();	//产生一个停止条件 
	  return i;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/
u8 ADSP_9960_Init()
{
    uint8_t id;

    /* Initialize I2C */
    //Wire.begin();
//		//I2C_Inilize();
//     IIC_Init();
    /* Read ID register and check against known values for APDS-9960 */
    if( !APDS9960_Read_Byte(APDS9960_ID, &id) ) {
        return False;
    }
    if( !(id == APDS9960_ID_1 || id == APDS9960_ID_2) ) {
        return False;
    }
     
    /* Set ENABLE register to 0 (disable all features) */
    if( !setMode(ALL, OFF) ) {
        return False;
    }
    
    /* Set default values for ambient light and proximity registers */
    if( !APDS9960_Write_Byte(APDS9960_ATIME, DEFAULT_ATIME) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_WTIME, DEFAULT_WTIME) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_CONFIG1, DEFAULT_CONFIG1) ) {
        return False;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return False;
    }
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return False;
    }
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return False;
    }
    if( !setProxIntLowThresh(DEFAULT_PILT) ) {
        return False;
    }
    if( !setProxIntHighThresh(DEFAULT_PIHT) ) {
        return False;
    }
    if( !setLightIntLowThreshold(DEFAULT_AILT) ) {
        return False;
    }
    if( !setLightIntHighThreshold(DEFAULT_AIHT) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_PERS, DEFAULT_PERS) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_CONFIG2, DEFAULT_CONFIG2) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_CONFIG3, DEFAULT_CONFIG3) ) {
        return False;
    }
    
    /* Set default values for gesture sense registers */
    if( !setGestureEnterThresh(DEFAULT_GPENTH) ) {
        return False;
    }
if( !setGestureExitThresh(DEFAULT_GEXTH) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GCONF1, DEFAULT_GCONF1) ) {
        return False;
    }
    if( !setGestureGain(DEFAULT_GGAIN) ) {
        return False;
    }
    if( !setGestureLEDDrive(DEFAULT_GLDRIVE) ) {
        return False;
    }
    if( !setGestureWaitTime(DEFAULT_GWTIME) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GPULSE, DEFAULT_GPULSE) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_GCONF3, DEFAULT_GCONF3) ) {
        return False;
    }
 if( !setGestureIntEnable(DEFAULT_GIEN) ) {
        return False;
    }
    
#if 0
    /* Gesture config register dump */
    uint8_t reg;
    uint8_t val;
  
    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \
            (reg != 0x8A) && \
            (reg != 0x91) && \
            (reg != 0xA8) && \
            (reg != 0xAC) && \
            (reg != 0xAD) )
        {
            APDS9960_Read_Byte(reg, val);
            Serial.print(reg, HEX);
            Serial.print(": 0x");
            Serial.println(val, HEX);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
        APDS9960_Read_Byte(reg, val);
        Serial.print(reg, HEX);
        Serial.print(": 0x");
        Serial.println(val, HEX);
    }
#endif

    return True;
}
/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Reads and returns the contents of the ENABLE register
 *
 * @return Contents of the ENABLE register. 0xFF if error.
 */
uint8_t getMode()
{
    uint8_t enable_value;
    
    /* Read current ENABLE register */
    if( !APDS9960_Read_Byte(APDS9960_ENABLE, &enable_value) ) {
        return ERROR;
    }
    
    return enable_value;
}
/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return True if operation success. False otherwise.
 */
 u8 setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;

    /* Read current ENABLE register */
    reg_val = getMode();
    if( reg_val == ERROR ) {
        return False;
    }
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if((mode >= 0)&&(mode <= 6)) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
        
    /* Write value back to ENABLE register */
    if( !APDS9960_Write_Byte(APDS9960_ENABLE, reg_val) ) {
        return False;
    }
        
    return True;
}

/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts True to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
u8 enableLightSensor(u8 interrupts)
{
    
    /* Set default gain, interrupts, enable power, and enable sensor */
    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
        return False;
    }
    if( interrupts ) {
        if( !setAmbientLightIntEnable(1) ) {
            return False;
        }
    } else {
        if( !setAmbientLightIntEnable(0) ) {
            return False;
        }
    }
    if( !enablePower() ){
        return False;
    }
    if( !setMode(AMBIENT_LIGHT, 1) ) {
        return False;
    }
    
    return True;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return True if sensor disabled correctly. False on error.
 */
u8 disableLightSensor()
{
    if( !setAmbientLightIntEnable(0) ) {
        return False;
    }
    if( !setMode(AMBIENT_LIGHT, 0) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts True to enable hardware external interrupt on proximity
 * @return True if sensor enabled correctly. False on error.
 */
u8 enableProximitySensor(u8 interrupts)
{
    /* Set default gain, LED, interrupts, enable power, and enable sensor */
    if( !setProximityGain(DEFAULT_PGAIN) ) {
        return False;
    }
    if( !setLEDDrive(DEFAULT_LDRIVE) ) {
        return False;
    }
    if( interrupts ) {
        if( !setProximityIntEnable(1) ) {
            return False;
        }
    } else {
        if( !setProximityIntEnable(0) ) {
            return False;
        }
    }
    if( !enablePower() ){
        return False;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return False;
    }
    
    return True;
}

u8 disableProximitySensor(void)
{
	if( !setProximityIntEnable(0) ) {
		return False;
	}
	if( !setMode(PROXIMITY, 0) ) {
		return False;
	}

	return True;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts True to enable hardware external interrupt on gesture
 * @return True if engine enabled correctly. False on error.
 */
u8 enableGestureSensor(u8 interrupts)
{
    
    /* Enable gesture mode
       Set ENABLE to 0 (power off)
       Set WTIME to 0xFF
       Set AUX to LED_BOOST_300
       Enable PON, WEN, PEN, GEN in ENABLE 
    */
    resetGestureParameters();
    if( !APDS9960_Write_Byte(APDS9960_WTIME, 0xFF) ) {
        return False;
    }
    if( !APDS9960_Write_Byte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) ) {
        return False;
    }
    if( !setLEDBoost(LED_BOOST_300) ) {
        return False;
    }
    if( interrupts ) {
        if( !setGestureIntEnable(1) ) {
            return False;
        }
    } else {
        if( !setGestureIntEnable(0) ) {
            return False;
        }
    }
    if( !setGestureMode(1) ) {
        return False;
    }
    if( !enablePower() ){
        return False;
    }
    if( !setMode(WAIT, 1) ) {
        return False;
    }
    if( !setMode(PROXIMITY, 1) ) {
        return False;
    }
    if( !setMode(GESTURE, 1) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return True if engine disabled correctly. False on error.
 */
u8 disableGestureSensor()
{
    resetGestureParameters();
    if( !setGestureIntEnable(0) ) {
        return False;
    }
    if( !setGestureMode(0) ) {
        return False;
    }
    if( !setMode(GESTURE, 0) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return True if gesture available. False otherwise.
 */
u8 isGestureAvailable()
{
    uint8_t val;
    
    /* Read value from GSTATUS register */
    if( !APDS9960_Read_Byte(APDS9960_GSTATUS, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GVALID bit */
    val &= APDS9960_GVALID;
    
    /* Return True/False based on GVALID bit */
    if( val == 1) {
        return True;
    } else {
        return False;
    }
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */

int readGesture()
{
    uint8_t fifo_level = 0;
    int bytes_read = 0;
    uint8_t  fifo_data[128];
    uint8_t gstatus;
    int motion;
    int i;
    
    /* Make sure that power and gesture is on and data is valid */
    if( !isGestureAvailable() || !(getMode() & 0x41) ) {
        return DIR_NONE;
    }
    
    /* Keep looping as long as gesture data is valid */
    while(1) {
    
        /* Wait some time to collect next batch of FIFO data */
        APDS9960_delay_us(FIFO_PAUSE_TIME);
        
        /* Get the contents of the STATUS register. Is data still valid? */
        if( !APDS9960_Read_Byte(APDS9960_GSTATUS, &gstatus) ) {
            return ERROR;
        }
/* If we have valid data, read in FIFO */
        if( (gstatus & APDS9960_GVALID) == APDS9960_GVALID ) {
        
            /* Read the current FIFO level */
            if( !APDS9960_Read_Byte(APDS9960_GFLVL, &fifo_level) ) {
                return ERROR;
            }

#if DEBUG
            Serial.print("FIFO Level: ");
            Serial.println(fifo_level);
#endif

            /* If there's stuff in the FIFO, read it into our data block */
            if( fifo_level > 0) {
                bytes_read = wireReadDataBlock(  APDS9960_GFIFO_U, 
                                                (uint8_t*)fifo_data, 
                                                (fifo_level * 4) );
                if( bytes_read == -1 ) {
                    return ERROR;
                }
#if DEBUG
                Serial.print("FIFO Dump: ");
                for ( i = 0; i < bytes_read; i++ ) {
                    Serial.print(fifo_data[i]);
                    Serial.print(" ");
                }
                Serial.println();
#endif
/* If at least 1 set of data, sort the data into U/D/L/R */
                if( bytes_read >= 4 ) {
                    for( i = 0; i < bytes_read; i += 4 ) {
                        gesture_data_.u_data[gesture_data_.index] = \
                                                            fifo_data[i + 0];
                        gesture_data_.d_data[gesture_data_.index] = \
                                                            fifo_data[i + 1];
                        gesture_data_.l_data[gesture_data_.index] = \
                                                            fifo_data[i + 2];
                        gesture_data_.r_data[gesture_data_.index] = \
                                                            fifo_data[i + 3];
                        gesture_data_.index++;
                        gesture_data_.total_gestures++;
                    }
                    
#if DEBUG
                Serial.print("Up Data: ");
                for ( i = 0; i < gesture_data_.total_gestures; i++ ) { 
                    Serial.print(gesture_data_.u_data[i]);
                    Serial.print(" ");
                }
                Serial.println();
#endif

                    /* Filter and process gesture data. Decode near/far state */
                    if( processGestureData() ) {
                        if( decodeGesture() ) {
                            //***TODO: U-Turn Gestures
#if DEBUG
                            //Serial.println(gesture_motion_);
#endif
 }
                    }
                    
                    /* Reset data */
                    gesture_data_.index = 0;
                    gesture_data_.total_gestures = 0;
                }
            }
        } else {
    
            /* Determine best guessed gesture and clean up */
            APDS9960_delay_us(FIFO_PAUSE_TIME);
            decodeGesture();
            motion = gesture_motion_;
#if DEBUG
            Serial.print("END: ");
            Serial.println(gesture_motion_);
#endif
            resetGestureParameters();
            return motion;
        }
    }
}
/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
u8 enablePower()
{
    if( !setMode(POWER, 1) ) {
        return False;
    }
    
    return True;
}

/**
 * Turn the APDS-9960 off
 *
 * @return True if operation successful. False otherwise.
 */
u8 disablePower()
{
    if( !setMode(POWER, 0) ) {
        return False;
    }
    
    return True;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
u8 readAmbientLight(uint16_t *val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_CDATAL, &val_byte) ) {
        return False;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_CDATAH, &val_byte) ) {
        return False;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return True;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
u8 readRedLight(uint16_t *val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_RDATAL, &val_byte) ) {
        return False;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_RDATAH, &val_byte) ) {
        return False;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return True;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
u8 readGreenLight(uint16_t *val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_GDATAL, &val_byte) ) {
        return False;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_GDATAH, &val_byte) ) {
        return False;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return True;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return True if operation successful. False otherwise.
 */
u8 readBlueLight(uint16_t *val)
{
    uint8_t val_byte;
    *val = 0;
    
    /* Read value from clear channel, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_BDATAL, &val_byte) ) {
        return False;
    }
    *val = val_byte;
    
    /* Read value from clear channel, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_BDATAH, &val_byte) ) {
        return False;
    }
    *val = *val + ((uint16_t)val_byte << 8);
    
    return True;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return True if operation successful. False otherwise.
 */
u8 readProximity(uint8_t *val)
{
    *val = 0;
    
    /* Read value from proximity data register */
    if( !APDS9960_Read_Byte(APDS9960_PDATA, val) ) {
        return False;
    }
    
    return True;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
//#if 0
void resetGestureParameters()
{
    gesture_data_.index = 0;
    gesture_data_.total_gestures = 0;
    
    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;
    
    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;
    
    gesture_near_count_ = 0;
    gesture_far_count_ = 0;
    
    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}
 //#endif
/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return True if near or far state seen. False otherwise.
 */
// #if 0
u8 processGestureData()
{
    uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
    int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;

    /* If we have less than 4 total gestures, that's not enough */
    if( gesture_data_.total_gestures <= 4 ) {
        return False;
    }
    
    /* Check to make sure our data isn't out of bounds */
    if( (gesture_data_.total_gestures <= 32) && \
        (gesture_data_.total_gestures > 0) ) {
        
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < gesture_data_.total_gestures; i++ ) {
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_first = gesture_data_.u_data[i];
                d_first = gesture_data_.d_data[i];
                l_first = gesture_data_.l_data[i];
                r_first = gesture_data_.r_data[i];
                break;
            }
        }
        /* If one of the _first values is 0, then there is no good data */
        if( (u_first == 0) || (d_first == 0) || \
            (l_first == 0) || (r_first == 0) ) {
            
            return False;
        }
        /* Find the last value in U/D/L/R above the threshold */
        for( i = gesture_data_.total_gestures - 1; i >= 0; i-- ) {
#if DEBUG
            Serial.print(F("Finding last: "));
            Serial.print(F("U:"));
            Serial.print(gesture_data_.u_data[i]);
            Serial.print(F(" D:"));
            Serial.print(gesture_data_.d_data[i]);
            Serial.print(F(" L:"));
            Serial.print(gesture_data_.l_data[i]);
            Serial.print(F(" R:"));
            Serial.println(gesture_data_.r_data[i]);
#endif
            if( (gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT) &&
                (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_last = gesture_data_.u_data[i];
                d_last = gesture_data_.d_data[i];
                l_last = gesture_data_.l_data[i];
                r_last = gesture_data_.r_data[i];
                break;
            }
        }
    }
 /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
       
#if DEBUG
    Serial.print(F("Last Values: "));
    Serial.print(F("U:"));
    Serial.print(u_last);
    Serial.print(F(" D:"));
    Serial.print(d_last);
    Serial.print(F(" L:"));
    Serial.print(l_last);
    Serial.print(F(" R:"));
    Serial.println(r_last);

    Serial.print(F("Ratios: "));
    Serial.print(F("UD Fi: "));
    Serial.print(ud_ratio_first);
    Serial.print(F(" UD La: "));
    Serial.print(ud_ratio_last);
    Serial.print(F(" LR Fi: "));
    Serial.print(lr_ratio_first);
    Serial.print(F(" LR La: "));
    Serial.println(lr_ratio_last);
#endif
 /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;
    
#if DEBUG
    Serial.print("Deltas: ");
    Serial.print("UD: ");
    Serial.print(ud_delta);
    Serial.print(" LR: ");
    Serial.println(lr_delta);
#endif

    /* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;
    
#if DEBUG
    Serial.print("Accumulations: ");
    Serial.print("UD: ");
    Serial.print(gesture_ud_delta_);
    Serial.print(" LR: ");
    Serial.println(gesture_lr_delta_);
#endif
/* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }
    
    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }
            
            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
 gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return True;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }
            
            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }
#if DEBUG
    Serial.print("UD_CT: ");
    Serial.print(gesture_ud_count_);
    Serial.print(" LR_CT: ");
    Serial.print(gesture_lr_count_);
    Serial.print(" NEAR_CT: ");
    Serial.print(gesture_near_count_);
    Serial.print(" FAR_CT: ");
    Serial.println(gesture_far_count_);
    Serial.println("----------");
#endif
    
    return False;
}  
//#endif
/**
 * @brief Determines swipe direction or near/far state
 *
 * @return True if near/far event. False otherwise.
 */
u8 decodeGesture()
{
    /* Return if near or far event is detected */
    if( gesture_state_ == NEAR_STATE ) {
        gesture_motion_ = DIR_NEAR;
        return True;
    } else if ( gesture_state_ == FAR_STATE ) {
        gesture_motion_ = DIR_FAR;
        return True;
    }
    
    /* Determine swipe direction */
    if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ = DIR_RIGHT;//DIR_UP;
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 0) ) {
        gesture_motion_ =DIR_LEFT; //DIR_DOWN;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 1) ) {
        gesture_motion_ = DIR_DOWN;//DIR_RIGHT;
    } else if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == -1) ) {
        gesture_motion_ = DIR_UP;//DIR_LEFT;
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ =DIR_RIGHT; //DIR_UP;
        } else {
            gesture_motion_ = DIR_DOWN;//DIR_RIGHT;
}
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_LEFT;//DIR_DOWN;
        } else {
            gesture_motion_ = DIR_UP;//DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == -1) && (gesture_lr_count_ == -1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ =DIR_RIGHT; //DIR_UP;
        } else {
            gesture_motion_ =DIR_UP;// DIR_LEFT;
        }
    } else if( (gesture_ud_count_ == 1) && (gesture_lr_count_ == 1) ) {
        if( abs(gesture_ud_delta_) > abs(gesture_lr_delta_) ) {
            gesture_motion_ = DIR_LEFT;//DIR_DOWN;
        } else {
            gesture_motion_ = DIR_DOWN;//DIR_RIGHT;
        }
    } else {
        return False;
    }
    
    return True;
}
      
/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t getProxIntLowThresh()
{
    uint8_t val;
    
    /* Read value from PILT register */
    if( !APDS9960_Read_Byte(APDS9960_PILT, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return True if operation successful. False otherwise.
 */
u8 setProxIntLowThresh(uint8_t threshold)
{
    if( !APDS9960_Write_Byte(APDS9960_PILT, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t getProxIntHighThresh()
{
    uint8_t val;
    
    /* Read value from PIHT register */
    if( !APDS9960_Read_Byte(APDS9960_PIHT, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return True if operation successful. False otherwise.
 */
u8 setProxIntHighThresh(uint8_t threshold)
{
    if( !APDS9960_Write_Byte(APDS9960_PIHT, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t getLEDDrive()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out LED drive bits */
    val = (val >> 6) & 0x03;
    
    return val;
}
/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return True if operation successful. False otherwise.
 */
u8 setLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    drive &= 0X03;
    drive = drive << 6;
    val &= 0X3F;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !APDS9960_Write_Byte(APDS9960_CONTROL, val) ) {
        return False;
    }
    
    return True;
}
/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t getProximityGain()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PDRIVE bits */
    val = (val >> 2) & 0x03;
    
    return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
 u8 setProximityGain(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 2;
    val &= 0xf3;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !APDS9960_Write_Byte(APDS9960_CONTROL, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t getAmbientLightGain()
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out ADRIVE bits */
    val &= 0x03;
    
    return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return True if operation successful. False otherwise.
 */
u8 setAmbientLightGain(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from CONTROL register */
    if( !APDS9960_Read_Byte(APDS9960_CONTROL, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    val &= 0xfc;
    val |= drive;
    
    /* Write register value back into CONTROL register */
    if( !APDS9960_Write_Byte(APDS9960_CONTROL, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Get the current LED boost value
 * 
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t getLEDBoost()
{
    uint8_t val;
    
    /* Read value from CONFIG2 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out LED_BOOST bits */
    val = (val >> 4) & 0x03;
    
    return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return True if operation successful. False otherwise.
 */
u8 setLEDBoost(uint8_t boost)
{
    uint8_t val;
    
    /* Read value from CONFIG2 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG2, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    boost &= 0x03;
    boost = boost << 4;
    val &= 0xcf;
    val |= boost;
    
    /* Write register value back into CONFIG2 register */
    if( !APDS9960_Write_Byte(APDS9960_CONFIG2, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t getProxGainCompEnable()
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG3, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PCMP bits */
    val = (val >> 5) & 0x01;
    
    return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return True if operation successful. False otherwise.
 */
 u8 setProxGainCompEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG3, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xdf;
    val |= enable;
    
    /* Write register value back into CONFIG3 register */
    if( !APDS9960_Write_Byte(APDS9960_CONFIG3, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t getProxPhotoMask()
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG3, &val) ) {
        return ERROR;
    }
    
    /* Mask out photodiode enable mask bits */
    val &= 0X0F;
    
    return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return True if operation successful. False otherwise.
 */
 u8 setProxPhotoMask(uint8_t mask)
{
    uint8_t val;
    
    /* Read value from CONFIG3 register */
    if( !APDS9960_Read_Byte(APDS9960_CONFIG3, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    mask &= 0x0f;
    val &= 0xf0;
    val |= mask;
    
    /* Write register value back into CONFIG3 register */
    if( !APDS9960_Write_Byte(APDS9960_CONFIG3, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t getGestureEnterThresh()
{
    uint8_t val;
    
    /* Read value from GPENTH register */
    if( !APDS9960_Read_Byte(APDS9960_GPENTH, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return True if operation successful. False otherwise.
 */
u8 setGestureEnterThresh(uint8_t threshold)
{
    if( !APDS9960_Write_Byte(APDS9960_GPENTH, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t getGestureExitThresh()
{
    uint8_t val;
    
    /* Read value from GEXTH register */
    if( !APDS9960_Read_Byte(APDS9960_GEXTH, &val) ) {
        val = 0;
    }
    
    return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return True if operation successful. False otherwise.
 */
u8 setGestureExitThresh(uint8_t threshold)
{
    if( !APDS9960_Write_Byte(APDS9960_GEXTH, threshold) ) {
        return False;
    }
    return True;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t getGestureGain()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GGAIN bits */
    val = (val >> 5) & 0x03;
    
    return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return True if operation successful. False otherwise.
 */
 u8 setGestureGain(uint8_t gain)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    gain &= 0x03;
    gain = gain << 5;
    val &= 0x9f;
    val |= gain;
    
    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write_Byte(APDS9960_GCONF2, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t getGestureLEDDrive()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GLDRIVE bits */
    val = (val >> 3) & 0x03;
    
    return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
u8 setGestureLEDDrive(uint8_t drive)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    drive &= 0x03;
    drive = drive << 3;
    val &= 0xe7;
    val |= drive;
    
    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write_Byte(APDS9960_GCONF2, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t getGestureWaitTime()
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return ERROR;
    }
    
    /* Mask out GWTIME bits */
    val &= 0x07;
    
    return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return True if operation successful. False otherwise.
 */
 u8 setGestureWaitTime(uint8_t time)
{
    uint8_t val;
    
    /* Read value from GCONF2 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF2, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    time &= 0x07;
    val &= 0xf8;
    val |= time;
    
    /* Write register value back into GCONF2 register */
    if( !APDS9960_Write_Byte(APDS9960_GCONF2, val) ) {
        return False;
    }
    
    return True;
}


/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
u8 getLightIntLowThreshold(uint16_t *threshold)
{
    uint8_t val_byte;
    *threshold = 0;
    
    /* Read value from ambient light low threshold, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_AILTL, &val_byte) ) {
        return False;
    }
    *threshold = val_byte;
    
    /* Read value from ambient light low threshold, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_AILTH, &val_byte) ) {
        return False;
    }
    *threshold = *threshold + ((uint16_t)val_byte << 8);
    
    return True;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
u8 setLightIntLowThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !APDS9960_Write_Byte(APDS9960_AILTL, val_low) ) {
        return False;
    }
    
    /* Write high byte */
    if( !APDS9960_Write_Byte(APDS9960_AILTH, val_high) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
u8 getLightIntHighThreshold(uint16_t *threshold)		  //？
{
    uint8_t val_byte;
    *threshold = 0;
    
    /* Read value from ambient light high threshold, low byte register */
    if( !APDS9960_Read_Byte(APDS9960_AIHTL, &val_byte) ) {
        return False;
    }
    *threshold = val_byte;
    
    /* Read value from ambient light high threshold, high byte register */
    if( !APDS9960_Read_Byte(APDS9960_AIHTH, &val_byte) ) {
        return False;
    }
    *threshold = *threshold + ((uint16_t)val_byte << 8);
    
    return True;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
u8 setLightIntHighThreshold(uint16_t threshold)
{
    uint8_t val_low;
    uint8_t val_high;
    
    /* Break 16-bit threshold into 2 8-bit values */
    val_low = threshold & 0x00FF;
    val_high = (threshold & 0xFF00) >> 8;
    
    /* Write low byte */
    if( !APDS9960_Write_Byte(APDS9960_AIHTL, val_low) ) {
        return False;
    }
    
    /* Write high byte */
    if( !APDS9960_Write_Byte(APDS9960_AIHTH, val_high) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
u8 getProximityIntLowThreshold(uint8_t *threshold)	   //?
{
    *threshold = 0;
    
    /* Read value from proximity low threshold register */
    if( !APDS9960_Read_Byte(APDS9960_PILT, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
u8 setProximityIntLowThreshold(uint8_t threshold)
{
    
    /* Write threshold value to register */
    if( !APDS9960_Write_Byte(APDS9960_PILT, threshold) ) {
        return False;
    }   
    return True;
}
    

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return True if operation successful. False otherwise.
 */
u8 getProximityIntHighThreshold(uint8_t *threshold)			//?
{
    *threshold = 0;
    
    /* Read value from proximity low threshold register */
    if( !APDS9960_Read_Byte(APDS9960_PIHT, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return True if operation successful. False otherwise.
 */
u8 setProximityIntHighThreshold(uint8_t threshold)
{
    
    /* Write threshold value to register */
    if( !APDS9960_Write_Byte(APDS9960_PIHT, threshold) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getAmbientLightIntEnable()
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !APDS9960_Read_Byte(APDS9960_ENABLE, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out AIEN bit */
    val = (val >> 4) & 0x01;
    
    return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
u8 setAmbientLightIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !APDS9960_Read_Byte(APDS9960_ENABLE, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 4;
    val &= 0xef;
    val |= enable;
    
    /* Write register value back into ENABLE register */
    if( !APDS9960_Write_Byte(APDS9960_ENABLE, val) ) {
        return False;
    }
    
    return True;
}


/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getProximityIntEnable()
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !APDS9960_Read_Byte(APDS9960_ENABLE, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out PIEN bit */
    val = (val >> 5) & 0x01;
    
    return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
u8 setProximityIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from ENABLE register */
    if( !APDS9960_Read_Byte(APDS9960_ENABLE, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 5;
    val &= 0xdf;
    val |= enable;
    
    /* Write register value back into ENABLE register */
    if( !APDS9960_Write_Byte(APDS9960_ENABLE, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getGestureIntEnable()
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }
    
    /* Shift and mask out GIEN bit */
    val = (val >> 1) & 0x01;
    
    return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
u8 setGestureIntEnable(uint8_t enable)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF4, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 1;
    val &= 0xfd;
    val |= enable;
    
    /* Write register value back into GCONF4 register */
    if( !APDS9960_Write_Byte(APDS9960_GCONF4, val) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
u8 clearAmbientLightInt()
{
    uint8_t throwaway;
    if( !APDS9960_Read_Byte(APDS9960_AICLEAR, &throwaway) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return True if operation completed successfully. False otherwise.
 */
u8 clearProximityInt()
{
    uint8_t throwaway;
    if( !APDS9960_Read_Byte(APDS9960_PICLEAR, &throwaway) ) {
        return False;
    }
    
    return True;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t getGestureMode()
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF4, &val) ) {
        return ERROR;
    }
    
    /* Mask out GMODE bit */
    val &= 0x01;
    
    return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return True if operation successful. False otherwise.
 */
u8 setGestureMode(uint8_t mode)
{
    uint8_t val;
    
    /* Read value from GCONF4 register */
    if( !APDS9960_Read_Byte(APDS9960_GCONF4, &val) ) {
        return False;
    }
    
    /* Set bits in register to given value */
    mode &= 0x01;
    val &= 0xfe;
    val |= mode;
    
    /* Write register value back into GCONF4 register */
    if( !APDS9960_Write_Byte(APDS9960_GCONF4, val) ) {
        return False;
    }
   
    return True;
}
