#include "IR.h"
#include "SysTick.h"


void IR_Init(void)
{
	#if MODE
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_APB2PeriphClockCmd (IR1_AO_GPIO_CLK, ENABLE );	// �� ADC IO�˿�ʱ��
		GPIO_InitStructure.GPIO_Pin = IR1_AO_GPIO_PIN;					// ���� ADC IO ����ģʽ
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		// ����Ϊģ������
		GPIO_Init(IR1_AO_GPIO_PORT, &GPIO_InitStructure);	
		
		RCC_APB2PeriphClockCmd (IR2_AO_GPIO_CLK, ENABLE );
		GPIO_InitStructure.GPIO_Pin = IR2_AO_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	// 模拟输入
		GPIO_Init(IR2_AO_GPIO_PORT, &GPIO_InitStructure);
					// ��ʼ�� ADC IO

		ADCx_Init();
	}
	#else
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		// 初始化IR1 DO引脚
		RCC_APB2PeriphClockCmd (IR1_DO_GPIO_CLK, ENABLE );
		GPIO_InitStructure.GPIO_Pin = IR1_DO_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// 上拉输入
		GPIO_Init(IR1_DO_GPIO_PORT, &GPIO_InitStructure);
		
		// 初始化IR2 DO引脚
		RCC_APB2PeriphClockCmd (IR2_DO_GPIO_CLK, ENABLE );
		GPIO_InitStructure.GPIO_Pin = IR2_DO_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	// 上拉输入
		GPIO_Init(IR2_DO_GPIO_PORT, &GPIO_InitStructure);
		
	}
	#endif
	
}

#if MODE
static uint16_t IR1_ADC_Read(void)
{
	return ADC_GetValue(IR1_ADC_CHANNEL, ADC_SampleTime_55Cycles5);
}

// 读取IR2（PA4）的ADC值
static uint16_t IR2_ADC_Read(void)
{
	return ADC_GetValue(IR2_ADC_CHANNEL, ADC_SampleTime_55Cycles5);
}
#endif

uint16_t IR1_FireData(void)
{
	#if MODE  // 模拟AO模式：多次采样求平均，提高稳定性
	uint32_t  tempData = 0;
	for (uint8_t i = 0; i < IR_READ_TIMES; i++)
	{
		tempData += IR1_ADC_Read();
		delay_ms(5);
	}
	tempData /= IR_READ_TIMES;
	return (uint16_t)tempData;
	
	#else  // 数字DO模式：返回高低电平（0=无火焰，1=有火焰）
	uint16_t tempData;
	tempData = !GPIO_ReadInputDataBit(IR1_DO_GPIO_PORT, IR1_DO_GPIO_PIN);
	return tempData;
	#endif
}

// 读取第二个火焰传感器（IR2：PA4）数据
uint16_t IR2_FireData(void)
{
	#if MODE  // 模拟AO模式：多次采样求平均
	uint32_t  tempData = 0;
	for (uint8_t i = 0; i < IR_READ_TIMES; i++)
	{
		tempData += IR2_ADC_Read();
		delay_ms(5);
	}
	tempData /= IR_READ_TIMES;
	return (uint16_t)tempData;
	
	#else  // 数字DO模式：返回高低电平
	uint16_t tempData;
	tempData = !GPIO_ReadInputDataBit(IR2_DO_GPIO_PORT, IR2_DO_GPIO_PIN);
	return tempData;
	#endif
}



