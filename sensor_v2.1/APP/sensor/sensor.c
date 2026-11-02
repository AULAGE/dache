#include "sensor.h"
#include "adcx.h"
#include "SysTick.h"
#include "math.h"

static u16 ADC_ReadAvg(uint8_t ch)
{
    u32 sum = 0;
    u8  i;
    for (i = 0; i < SENSOR_READ_TIMES; i++)
    {
        sum += ADC_GetValue(ch, ADC_SampleTime_55Cycles5);
        delay_ms(5);
    }
    return (u16)(sum / SENSOR_READ_TIMES);
}

static u16 mq2_adc_cache = 0;

void Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(FLAME_AO_CLK | MQ2_AO_CLK | LIGHT_AO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    GPIO_InitStructure.GPIO_Pin = FLAME_AO_PIN;
    GPIO_Init(FLAME_AO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MQ2_AO_PIN;
    GPIO_Init(MQ2_AO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LIGHT_AO_PIN;
    GPIO_Init(LIGHT_AO_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(FLAME_DO_CLK | MQ2_DO_CLK | LIGHT_DO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = FLAME_DO_PIN;
    GPIO_Init(FLAME_DO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MQ2_DO_PIN;
    GPIO_Init(MQ2_DO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LIGHT_DO_PIN;
    GPIO_Init(LIGHT_DO_PORT, &GPIO_InitStructure);

    ADCx_Init();
}

u16 Flame_ReadAO(void)  { return ADC_ReadAvg(FLAME_ADC_CH); }

u8 Flame_ReadDO(void)
{
    return (GPIO_ReadInputDataBit(FLAME_DO_PORT, FLAME_DO_PIN) == Bit_RESET) ? 1 : 0;
}

u16 MQ2_ReadAO(void)
{
    mq2_adc_cache = ADC_ReadAvg(MQ2_ADC_CH);
    return mq2_adc_cache;
}

u8 MQ2_ReadDO(void)
{
    return (GPIO_ReadInputDataBit(MQ2_DO_PORT, MQ2_DO_PIN) == Bit_RESET) ? 1 : 0;
}

float MQ2_GetPPM(void)
{
    float adc_val = (float)mq2_adc_cache;
    if (adc_val == 0) adc_val = (float)ADC_ReadAvg(MQ2_ADC_CH);

    float Vol = adc_val * 3.3f / 4096.0f;
    if (Vol < 0.01f) Vol = 0.01f;
    float RS  = (3.3f - Vol) / (Vol * 0.5f);
    float R0  = 6.64f;
    float ppm = pow(11.5428f * R0 / RS, 0.6549f);
    return ppm;
}

float MQ2_Calibrate(void)
{
    u8 i;
    float sum = 0;
    for (i = 0; i < 10; i++)
    {
        float adc_val = (float)ADC_ReadAvg(MQ2_ADC_CH);
        float Vol = adc_val * 3.3f / 4096.0f;
        if (Vol < 0.01f) Vol = 0.01f;
        float RS = (3.3f - Vol) / (Vol * 0.5f);
        sum += RS / 9.83f;
        delay_ms(500);
    }
    return sum / 10.0f;
}

u16 Light_ReadAO(void)  { return ADC_ReadAvg(LIGHT_ADC_CH); }

u8 Light_ReadDO(void)
{
    return (GPIO_ReadInputDataBit(LIGHT_DO_PORT, LIGHT_DO_PIN) == Bit_RESET) ? 1 : 0;
}
