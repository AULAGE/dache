#ifndef __ADCX_H
#define __ADCX_H

#include "system.h"

#define ADC_CLK     RCC_APB2Periph_ADC1
#define ADCx        ADC1

void ADCx_Init(void);
u16  ADC_GetValue(uint8_t ADC_Channel, uint8_t ADC_SampleTime);

#endif
