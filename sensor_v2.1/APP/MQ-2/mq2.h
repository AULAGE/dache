#ifndef __MQ2_H
#define	__MQ2_H
#include "stm32f10x.h"
#include "adcx.h"
#include "math.h"

/*****************辰哥单片机设计******************
											STM32
 * 文件			:	MQ-2烟雾传感器h文件                   
 * 版本			: V1.0
 * 日期			: 2024.8.21
 * MCU			:	STM32F103C	
 * IP账号		:	辰哥单片机设计（同BILIBILI|抖音|快手|小红书|CSDN|公众号|视频号等）
 * 作者			:	辰哥
 * 工作室		: 异方辰电子工作室
 * 讲解视频	:	https://www.bilibili.com/video/BV1J9WMeGE3C/?share_source=copy_web
 * 官方网站	:	www.yfcdz.cn

**********************BEGIN***********************/		

#define MQ2_READ_TIMES	10  //MQ-2传感器ADC循环读取次数

//模式选择	
//模拟AO:	1
//数字DO:	0
#define	MODE 	1

/***************根据自己需求更改****************/
// MQ-2 GPIO宏定义
#if MODE
#define		MQ2_AO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	MQ2_AO_GPIO_PORT							GPIOA
#define 	MQ2_AO_GPIO_PIN								GPIO_Pin_2
#define   ADC_CHANNEL               		ADC_Channel_2	// ADC 通道宏定义

#else
#define		MQ2_DO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	MQ2_DO_GPIO_PORT							GPIOA
#define 	MQ2_DO_GPIO_PIN								GPIO_Pin_1			

#endif
/*********************END**********************/


void MQ2_Init(void);
uint16_t MQ2_GetData(void);
float MQ2_GetData_PPM(void);

#endif /* __ADC_H */

