#ifndef DAMIAO_WS2812_H
#define DAMIAO_WS2812_H
#include "typedef.h"
#include "tim.h"

#define WS2812_LOW 23	// 23*(1/800kHz)=2.875us
#define WS2812_HIGH 45
#define WS2812_MAX_NUM 9
#define RGB 24
#define RESET_TIME 90	// 80us/(1/800kHz)=64
#define WS2812_BUFFER_SIZE (RGB*WS2812_MAX_NUM+RESET_TIME)
#define WS2812_RGB(r,g,b) ((g << 16) | (r << 8) | b)
#define WS2812_RGB_BIT(i,r,g,b) ((WS2812_RGB(r,g,b) << i) & 0x800000)

typedef struct
{
    uint16_t RESET_Buffer[RESET_TIME];
    uint16_t LEDS_Buffer[WS2812_MAX_NUM][RGB];
}WS2812_Buffer_s;

typedef struct
{
    WS2812_Buffer_s buffer;
    uint8_t num;
    uint8_t init;
    TIM_HandleTypeDef htim;
    uint32_t channel;
} WS2812_instance;

void WS2812_Init(uint8_t num, const TIM_HandleTypeDef *htim, uint32_t channel);
void WS2812_LEDS_Shine(uint32_t aRGB, uint16_t Hz);
WS2812_instance* Get_WS2812_Ptr(void);

#endif //DAMIAO_WS2812_H