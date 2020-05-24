#include "mpu6050.h"

void MPU6050_IIC_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = XYZ_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MPU6050_IIC_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = XYZ_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void MPU6050_delay_us(u16 us)
{
	u8 i,j;
	for(i=0;i<us;i++)
		for(j=0;j<50;j++)
	;
}

/*I2C起始*/
void MPU6050_IIC_Start(void)
{
	MPU6050_IIC_SDA_OUT();	

	MPU6050_SDA_H();
	MPU6050_delay_us(2);	
	MPU6050_SCL_H();
	MPU6050_delay_us(2);		
	MPU6050_SDA_L();
	MPU6050_delay_us(2);	
	MPU6050_SCL_L();
	MPU6050_delay_us(2);
}
/*I2C停止*/
void MPU6050_IIC_Stop(void)
{
	MPU6050_IIC_SDA_OUT();	

	MPU6050_SDA_L();	
	MPU6050_delay_us(2);
	MPU6050_SCL_L();	
	MPU6050_delay_us(2);
	MPU6050_SDA_H();
	MPU6050_delay_us(2);
}
/*I2C发送应答*/
void MPU6050_IIC_Ack(u8 a)
{
	MPU6050_IIC_SDA_OUT();	

	if(a)	
		MPU6050_SDA_H();
	else	
		MPU6050_SDA_L();

	MPU6050_delay_us(2);
	MPU6050_SCL_H();	
	MPU6050_delay_us(2);
	MPU6050_SCL_L();
	MPU6050_delay_us(2);

}
/*I2C写入一个字节*/
u8 MPU6050_IIC_Write_Byte(u8 dat)
{
	u8 i;
	u8 MPU6050_IIC_Ack=0;	

	MPU6050_IIC_SDA_OUT();	

	for(i = 0;i < 8;i++)
	{
		if(dat & 0x80)	
			MPU6050_SDA_H();
		else
			MPU6050_SDA_L();
			
		MPU6050_delay_us(2);
		MPU6050_SCL_H();
	    MPU6050_delay_us(2);
		MPU6050_SCL_L();
		dat<<=1;
	}

	MPU6050_SDA_H();	//释放数据线

	MPU6050_IIC_SDA_IN();	//设置成输入

	MPU6050_delay_us(2);
	MPU6050_SCL_H();
	MPU6050_delay_us(2);
	
	MPU6050_IIC_Ack |= MPU6050_IN_SDA();	//读入应答位
	MPU6050_SCL_L();
	MPU6050_delay_us(4);
	return MPU6050_IIC_Ack;	//返回应答信号
}
/*I2C读取一个字节*/
u8 MPU6050_IIC_Read_Byte(void)
{
	u8 i;
	u8 x=0;

	MPU6050_SDA_H();	//首先置数据线为高电平

	MPU6050_IIC_SDA_IN();	//设置成输入

	for(i = 0;i < 8;i++)
	{
		x <<= 1;	//读入数据，高位在前

		MPU6050_delay_us(2);
		MPU6050_SCL_H();	//突变
		MPU6050_delay_us(2);
		
		if(MPU6050_IN_SDA())	x |= 0x01;	//收到高电平

		MPU6050_SCL_L();
		MPU6050_delay_us(2);
	}	//数据接收完成

	MPU6050_SCL_L();

	return x;	//返回读取到的数据
}

//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(uint8_t REG_Address,uint8_t REG_data)
{
    MPU6050_IIC_Start();                  //起始信号
    MPU6050_IIC_Write_Byte(SlaveAddress);   //发送设备地址+写信号
    MPU6050_IIC_Write_Byte(REG_Address);    //内部寄存器地址，
    MPU6050_IIC_Write_Byte(REG_data);       //内部寄存器数据，
    MPU6050_IIC_Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uint8_t Single_ReadI2C(uint8_t REG_Address)
{
	uint8_t REG_data;
	MPU6050_IIC_Start();                   //起始信号
	MPU6050_IIC_Write_Byte(SlaveAddress);    //发送设备地址+写信号
	MPU6050_IIC_Write_Byte(REG_Address);     //发送存储单元地址，从0开始	
	MPU6050_IIC_Start();                   //起始信号
	MPU6050_IIC_Write_Byte(SlaveAddress+1);  //发送设备地址+读信号
	REG_data=MPU6050_IIC_Read_Byte();       //读出寄存器数据
	MPU6050_IIC_Ack(1);                //接收应答信号
	MPU6050_IIC_Stop();                    //停止信号
	return REG_data;
}
//**************************************
//初始化MPU6050
//**************************************
void MPU6050_Init(void)
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, 0x18);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//**************************************
//合成数据
//**************************************
int MPU6050_GetData(uint8_t REG_Address)
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return (H<<8)+L;   //合成数据
}

