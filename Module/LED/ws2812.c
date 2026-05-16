#include "ws2812.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "dwt/bsp_dwt.h"

static WS2812_instance ws2812_instance;

/*开启DMA和PWM传输*/
void Data_Transmit()
{
    while(HAL_TIM_PWM_Start_DMA(&ws2812_instance.htim, ws2812_instance.channel, (uint32_t*)&ws2812_instance.buffer, ws2812_instance.num) != HAL_OK);
}

/*将颜色参数写入数组*/
void Set_SINGLE_LED_RGB(const uint16_t n , const uint8_t red, const uint8_t green, const uint8_t blue)
{
    if(n < WS2812_MAX_NUM)
    {
        for(uint8_t i = 0; i < RGB; i++)
        {
            //输入顺序为G,R,B,且高位先写入
            ws2812_instance.buffer.LEDS_Buffer[n][i] = WS2812_RGB_BIT(i,red,green,blue) ? WS2812_HIGH : WS2812_LOW;
        }
    }
}

/*打开灯*/
void WS2812_LEDS_Set(const uint32_t aRGB)
{
    if (!ws2812_instance.init)
        return;
    for(uint8_t i=0;i<WS2812_MAX_NUM;i++)
        Set_SINGLE_LED_RGB(i,(aRGB & 0x00FF0000) >> 16, (aRGB & 0x0000FF00) >> 8, (aRGB & 0x000000FF) >> 0);
    for(uint8_t i=0;i<RESET_TIME;i++)
        ws2812_instance.buffer.RESET_Buffer[i] = 0;
    Data_Transmit();
}

/*关闭灯*/
void WS2812_LEDS_Reset(void)
{
    if (!ws2812_instance.init)
        return;
    for(uint8_t i=0;i<WS2812_MAX_NUM;i++)
        Set_SINGLE_LED_RGB(i,0,0,0);
    for(uint8_t i=0;i<RESET_TIME;i++)
        ws2812_instance.buffer.RESET_Buffer[i] = 0;
    Data_Transmit();
}

void WS2812_LEDS_Shine(const uint32_t aRGB, const uint16_t Hz)
{
    for (int i = 0; i < Hz; i++)
    {
        WS2812_LEDS_Set(aRGB);
        osDelay(1000 / Hz / 2);
        WS2812_LEDS_Reset();
        osDelay(1000 / Hz / 2);
    }
}

void WS2812_Init(uint8_t num, const TIM_HandleTypeDef *htim, const uint32_t channel)
{
    if(num > WS2812_MAX_NUM)
        num = WS2812_MAX_NUM;
    ws2812_instance.num = num;
    ws2812_instance.htim = *htim;
    ws2812_instance.channel = channel;
    ws2812_instance.init = 1;
    WS2812_LEDS_Reset();
    DWT_Delay_ms(1000);
}

WS2812_instance* Get_WS2812_Ptr(void)
{
    return &ws2812_instance;
}