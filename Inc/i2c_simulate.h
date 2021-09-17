#ifndef __MYIIC_H
#define __MYIIC_H

#include "main.h"


#define GPIO_PORT_I2C GPIOB
#define I2C_SCL_PIN SCL_Pin
#define I2C_SDA_PIN SDA_Pin


#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN
#define I2C_SCL_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SCL_PIN << 16U
#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN
#define I2C_SDA_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SDA_PIN << 16U


#define I2C_SDA_READ() (GPIO_PORT_I2C->IDR & I2C_SDA_PIN)
#define I2C_SCL_READ() (GPIO_PORT_I2C->IDR & I2C_SCL_PIN)


#define I2C_SDA_SetOutput() GPIOB->MODER |= (1 << 18)
#define I2C_SDA_SetInput() GPIOB->MODER &= ~(1 << 18)
#define I2C_SDA_HighSpeed() GPIOB->OSPEEDR |= (3 << 18)
#define I2C_SDA_ExFallTriger GPIOB->MODER &= (~ (1 << 18)

void i2c_start(void);

void i2c_stop(void);

void i2c_SendAddress(void);

void i2c_SendByte(uint8_t *u8_Dataptr);

void i2c_Ack(void);
void i2c_NoAck(void);

#endif
