#include "dht11.h"
#include "SysTick.h"

static void DHT11_Mode_Out(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = DHT11_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DHT11_PORT, &GPIO_InitStructure);
}

static void DHT11_Mode_In(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin  = DHT11_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DHT11_PORT, &GPIO_InitStructure);
}

static void DHT11_Start(void)
{
    DHT11_Mode_Out();
    DHT11_OUT_L;
    delay_ms(20);
    DHT11_OUT_H;
    delay_us(30);
    DHT11_Mode_In();
}

static u8 DHT11_WaitResponse(void)
{
    u8 retry = 0;
    while (DHT11_READ && retry < 100) { retry++; delay_us(1); }
    if (retry >= 100) return 1;
    retry = 0;
    while (!DHT11_READ && retry < 100) { retry++; delay_us(1); }
    if (retry >= 100) return 1;
    retry = 0;
    while (DHT11_READ && retry < 100) { retry++; delay_us(1); }
    if (retry >= 100) return 1;
    return 0;
}

static u8 DHT11_ReadByte(void)
{
    u8 i, byte = 0;
    for (i = 0; i < 8; i++)
    {
        u8 retry = 0;
        while (DHT11_READ == Bit_RESET && retry < 100) { retry++; delay_us(1); }
        delay_us(40);
        if (DHT11_READ == Bit_SET)
            byte |= (0x80 >> i);
        retry = 0;
        while (DHT11_READ == Bit_SET && retry < 100) { retry++; delay_us(1); }
    }
    return byte;
}

u8 DHT11_Init(void)
{
    RCC_APB2PeriphClockCmd(DHT11_CLK, ENABLE);
    DHT11_Mode_Out();
    DHT11_OUT_H;
    delay_ms(100);

    DHT11_Start();
    return DHT11_WaitResponse();
}

u8 DHT11_Read_Data(u8 *temp, u8 *humi)
{
    u8 buf[5];
    u8 i;

    DHT11_Start();
    if (DHT11_WaitResponse() != 0) return 1;

    for (i = 0; i < 5; i++)
        buf[i] = DHT11_ReadByte();

    if (buf[0] + buf[1] + buf[2] + buf[3] == buf[4])
    {
        *humi = buf[0];
        *temp = buf[2];
        return 0;
    }
    return 1;
}
