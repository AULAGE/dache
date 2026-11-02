#ifndef __LED_H
#define __LED_H

#include "system.h"

#define MODE_LED_PIN    GPIO_Pin_13
#define MODE_LED_PORT   GPIOC
#define MODE_LED_CLK    RCC_APB2Periph_GPIOC

void LED_Init(void);
void LED_SetMode(u8 ao_mode);   /* 1=AOÁÁµÆ  0=DOÃðµÆ */

#endif
