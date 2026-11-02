#include "bsp_tcrt5000.h"

void TRC5000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(HWL_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(HWM_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(HWR_PORT_RCC,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin=HWL_PIN;
	GPIO_Init(HWL_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=HWM_PIN;
	GPIO_Init(HWM_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=HWR_PIN;
	GPIO_Init(HWR_PORT,&GPIO_InitStructure);
	
}
