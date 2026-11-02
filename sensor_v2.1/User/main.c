#include "system.h"
#include "SysTick.h"
#include "usart.h"
#include "adcx.h"
#include "sensor.h"
#include "dht11.h"
#include "beep.h"
#include "led.h"
#include "Key.h"
#include <stdio.h>
#include <math.h>

int main(void)
{
    u8 mode     = 0;
    u8 temp = 0, humi = 0;
    u16 dht_timer = 0;
    u8 dht_ok = 0;

    SysTick_Init(72);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(115200);
    LED_Init();
    BEEP_Init();
    Key_Init();
    Sensor_Init();

    while (DHT11_Init())
    {
        printf("DHT11 Init Failed, retrying...\r\n");
        delay_ms(1000);
    }

    LED_SetMode(mode);
    printf("System Ready [DO Mode]\r\n");

    while (1)
    {
        if (Key_GetPressed())
        {
            mode = !mode;
            LED_SetMode(mode);
            printf(">> Switched to %s Mode\r\n", mode ? "AO" : "DO");
        }

        if (dht_timer >= 2000)
        {
            if (DHT11_Read_Data(&temp, &humi) == 0)
                dht_ok = 1;
            else
                printf("DHT11 Read Error\r\n");
            dht_timer = 0;
        }

        if (mode)
        {
            u16   flame_ao = Flame_ReadAO();
            u16   mq2_ao   = MQ2_ReadAO();
            float ppm      = MQ2_GetPPM();
            u16   light_ao = Light_ReadAO();

            if (dht_ok)
                printf("[AO] Temp:%d  Humi:%d  Flame:%d  MQ2:%d(%.1fppm)  Light:%d\r\n",
                       temp, humi, flame_ao, mq2_ao, ppm, light_ao);

            if (flame_ao < 500 || ppm > 200.0f)
                BEEP_ON();
            else
                BEEP_OFF();
        }
        else
        {
            u8 flame_do = Flame_ReadDO();
            u8 mq2_do   = MQ2_ReadDO();
            u8 light_do = Light_ReadDO();

            if (dht_ok)
                printf("[DO] Temp:%d  Humi:%d  Flame:%d  MQ2:%d  Light:%d\r\n",
                       temp, humi, flame_do, mq2_do, light_do);

            if (flame_do == 1 || mq2_do == 1)
                BEEP_ON();
            else
                BEEP_OFF();
        }

        delay_ms(100);
        dht_timer += 100;
    }
}
