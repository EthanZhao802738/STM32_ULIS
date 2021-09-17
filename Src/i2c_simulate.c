#include "i2c_simulate.h"
#include "main.h"





void i2c_start(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	I2C_SDA_SetOutput();
	I2C_SDA_HighSpeed();
	I2C_SDA_0();
	delay_us(3);
}

void i2c_stop(void)
{
	delay_us(30);
}

void i2c_SendAddress(void)
{
	uint8_t bit;

	I2C_SDA_SetInput();

	while(I2C_SCL_READ()){}


	for(bit = 0; bit <8;bit++)
	{
		while(!I2C_SCL_READ()){}
		while(I2C_SCL_READ()){}

	}
	I2C_SDA_SetOutput();
	I2C_SDA_0();
	while(!I2C_SCL_READ()){}
	while(I2C_SCL_READ()){}
}


void i2c_SendByte(uint8_t *u8_Dataptr)
{
	uint8_t bitnum,bit;

	for(bitnum = 8; bitnum > 0; bitnum--)
	{
		bit = *u8_Dataptr >> (bitnum-1);
		bit &= 0x01;
		if(bit)
			I2C_SDA_1();
		else
			I2C_SDA_0();
		while(!I2C_SCL_READ()){}
		while(I2C_SCL_READ()){}
		I2C_SDA_0();
	}
}


void i2c_Ack(void)
{
	I2C_SDA_SetInput();
	while(!I2C_SCL_READ()){}
	while(I2C_SCL_READ()){}
	I2C_SDA_SetOutput();
}

void i2c_NoAck(void)
{
	I2C_SDA_SetInput();
	while(!I2C_SCL_READ()){}
	while(I2C_SCL_READ()){}
}