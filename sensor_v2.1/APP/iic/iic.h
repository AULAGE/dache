#ifndef _iic_H
#define _iic_H

#include "system.h"

#define IIC_SCL_PORT 			GPIOB  
#define IIC_SCL_PIN 			GPIO_Pin_6
#define IIC_SCL_PORT_RCC		RCC_APB2Periph_GPIOB


#define IIC_SDA_PORT 			GPIOB  
#define IIC_SDA_PIN 			GPIO_Pin_7
#define IIC_SDA_PORT_RCC		RCC_APB2Periph_GPIOB


#define IIC_SCL 	PBout(6)
#define IIC_SDA		PBout(7)
#define READ_SDA	PBin(7)


void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(u8 ack);

#endif
