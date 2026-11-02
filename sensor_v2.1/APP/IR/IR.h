#ifndef __IR_H
#define	__IR_H
#include "stm32f10x.h"
#include "adcx.h"
#include "math.h"

/*****************辰哥单片机设计******************
											STM32
 * 文件			:	火焰传感器h文件                   
 * 版本			: V1.0
 * 日期			: 2024.8.10
 * MCU			:	STM32F103C8T6
 * 接口			:	见代码							
 * IP账号		:	辰哥单片机设计（同BILIBILI|抖音|快手|小红书|CSDN|公众号|视频号等）
 * 作者			:	辰哥
 * 工作室		: 异方辰电子工作室
 * 讲解视频	:	https://www.bilibili.com/video/BV1UZ421N7ib/?share_source=copy_web
 * 官方网站	:	www.yfcdz.cn

**********************BEGIN***********************/		

#define IR_READ_TIMES	10  //火焰传感器ADC循环读取次数

//模式选择	
//模拟AO:	1
//数字DO:	0
#define	MODE 	0

/***************根据自己需求更改****************/
// LDR GPIO宏定义
#if MODE
#define		IR1_AO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	IR1_AO_GPIO_PORT								GPIOA
#define 	IR1_AO_GPIO_PIN								GPIO_Pin_7
#define   IR1_ADC_CHANNEL               ADC_Channel_7// ADC 通道宏定义

#define		IR2_AO_GPIO_CLK								RCC_APB2Periph_GPIOB
#define 	IR2_AO_GPIO_PORT							GPIOB
#define 	IR2_AO_GPIO_PIN								GPIO_Pin_1
#define   IR2_ADC_CHANNEL     					ADC_Channel_9
#else
#define		IR1_DO_GPIO_CLK								RCC_APB2Periph_GPIOA
#define 	IR1_DO_GPIO_PORT								GPIOA
#define 	IR1_DO_GPIO_PIN								GPIO_Pin_7

#define		IR2_DO_GPIO_CLK		RCC_APB2Periph_GPIOB
#define 	IR2_DO_GPIO_PORT	GPIOB
#define 	IR2_DO_GPIO_PIN		GPIO_Pin_1

#endif
/*********************END**********************/


void IR_Init(void);
uint16_t IR1_FireData(void);
uint16_t IR2_FireData(void);

#endif /* __ADC_H */

