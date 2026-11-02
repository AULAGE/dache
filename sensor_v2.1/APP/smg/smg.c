#include "smg.h"
#include "SysTick.h"


u8 gsmg_code[17]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71};


void SMG_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(LSA_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(LSB_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(LSC_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(SMG_A_DP_PORT_RCC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=LSA_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(LSA_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=LSB_PIN;
	GPIO_Init(LSB_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=LSC_PIN;
	GPIO_Init(LSC_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=SMG_A_DP_PIN;
	GPIO_Init(SMG_A_DP_PORT,&GPIO_InitStructure);
}

void SMG_Write_Data(u16 GPIO_Pin,u8 data)
{
	u8 i=0;
	u8 j=GPIO_Pin;
	
	for(i=0;i<8;i++)
	{
		if(data&0x01)
			GPIO_WriteBit(SMG_A_DP_PORT,j<<i,Bit_SET);
		else
			GPIO_WriteBit(SMG_A_DP_PORT,j<<i,Bit_RESET);
		data=data>>1;
	}
}

void SMG_Display(u8 dat[],u8 pos)
{
	u8 i=0;
	u8 pos_temp=pos-1;
	
	for(i=0;i<8;i++)
	{
		switch(i)
		{
			case 0:LSC=1;LSB=1;LSA=1;break;
			case 1:LSC=1;LSB=1;LSA=0;break;
			case 2:LSC=1;LSB=0;LSA=1;break;
			case 3:LSC=1;LSB=0;LSA=0;break;
			case 4:LSC=0;LSB=1;LSA=1;break;
			case 5:LSC=0;LSB=1;LSA=0;break;
			case 6:LSC=0;LSB=0;LSA=1;break;
			case 7:LSC=0;LSB=0;LSA=0;break;			
		}
		SMG_Write_Data(GPIO_Pin_0,dat[i-pos_temp]);
		delay_ms(1);
		SMG_Write_Data(GPIO_Pin_0,0x00);
	}
}
