#include "SysTick.h"

static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������
static u16 fac_s=0;								//s��ʱ������


//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪAHBʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��Ƶ��
void SysTick_Init(u8 SYSCLK)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); 
	fac_us=SYSCLK/8;					
	fac_ms=(u16)fac_us*1000;			
	fac_s=fac_ms*1000;
}								    


/*******************************************************************************
* �� �� ��         : delay_us
* ��������		   : us��ʱ��
* ��    ��         : nus��Ҫ��ʱ��us��
					ע��:nus��ֵ,��Ҫ����798915us(���ֵ��2^24/fac_us@fac_us=21)
* ��    ��         : ��
*******************************************************************************/		    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
}

/*******************************************************************************
* �� �� ��         : delay_ms
* ��������		   : ms��ʱ��
* ��    ��         : nms��Ҫ��ʱ��ms��
					ע��:nms��ֵ,SysTick->LOADΪ24λ�Ĵ�����
					��Ҫ����0xffffff*8*1000/SYSCLK
					��72M������,nms<=1864ms 
* ��    ��         : ��
*******************************************************************************/
void delay_ms(u32 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	  	    
} 


/*******************************************************************************
* �� �� ��         : delay_s
* ��������		   : s��ʱ��
* ��    ��         : ns��Ҫ��ʱ��s��
					ע��:nms��ֵ,SysTick->LOADΪ24λ�Ĵ�����
					��Ҫ����0xffffff*8*1000/SYSCLK
					
* ��    ��         : ��
*******************************************************************************/
void delay_s(u32 ns)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)ns*fac_s;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	  	    
} 


