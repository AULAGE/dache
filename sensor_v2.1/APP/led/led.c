#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(MODE_LED_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = MODE_LED_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MODE_LED_PORT, &GPIO_InitStructure);

    GPIO_SetBits(MODE_LED_PORT, MODE_LED_PIN);  /* PC13低电平亮，先关 */
}

void LED_SetMode(u8 ao_mode)
{
    if (ao_mode)
        GPIO_ResetBits(MODE_LED_PORT, MODE_LED_PIN);  /* 亮 = AO模式 */
    else
        GPIO_SetBits(MODE_LED_PORT, MODE_LED_PIN);    /* 灭 = DO模式 */
}
