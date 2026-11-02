#include "leddz.h"
#include "SysTick.h"

void LEDDZ_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCLK_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(SRCLK_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(SER_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(LEDDZ_COL_PORT_RCC,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=RCLK_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(RCLK_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SRCLK_PIN;
	GPIO_Init(SRCLK_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SER_PIN;
	GPIO_Init(SER_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=LEDDZ_COL_PIN;
	GPIO_Init(LEDDZ_COL_PORT,&GPIO_InitStructure);	
}

void LEDDZ_Row_Write_data(u8 dat)
{
	u8 i=0;
	
	for(i=0;i<8;i++)
	{
		SER=dat>>7;
		dat<<=1;
		SRCLK=0;
		delay_us(1);
		SRCLK=1;
		delay_us(1);
	}
	
	RCLK=0;
	delay_us(1);
	RCLK=1;
}

void LEDDZ_COL_Write_Data(u8 data)
{
	u8 i=0;
	u8 j=GPIO_Pin_0;    
	
	for(i=0;i<8;i++)
	{
		if(data&0x01)
			GPIO_WriteBit(LEDDZ_COL_PORT, j<<i ,Bit_SET); 
		else
			GPIO_WriteBit(LEDDZ_COL_PORT, j<<i ,Bit_RESET); 
		data = data >> 1 ; 
	}
}
