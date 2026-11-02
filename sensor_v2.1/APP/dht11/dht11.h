#ifndef __DHT11_H
#define __DHT11_H

#include "system.h"

#define DHT11_PIN    GPIO_Pin_11
#define DHT11_PORT   GPIOB
#define DHT11_CLK    RCC_APB2Periph_GPIOB

#define DHT11_OUT_H  GPIO_SetBits(DHT11_PORT, DHT11_PIN)
#define DHT11_OUT_L  GPIO_ResetBits(DHT11_PORT, DHT11_PIN)
#define DHT11_READ   GPIO_ReadInputDataBit(DHT11_PORT, DHT11_PIN)

u8 DHT11_Init(void);
u8 DHT11_Read_Data(u8 *temp, u8 *humi);

#endif
