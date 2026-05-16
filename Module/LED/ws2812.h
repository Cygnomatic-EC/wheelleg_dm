#ifndef __WS2812_SPI_H
#define __WS2812_SPI_H

#include "typedef.h"
#include "spi.h"
#include "cmsis_os2.h"

#define WS2812_MAX_NUM      64
#define WS2812_RESET_PULSES 50  // 复位信号需要的0字节数
#define WS2812_BITS_PER_LED 24   // 每个LED 24位(GRB)

typedef struct {
    SPI_HandleTypeDef *hspi;
    uint8_t buffer[WS2812_MAX_NUM * WS2812_BITS_PER_LED + WS2812_RESET_PULSES];
    uint16_t num;
    uint8_t init;
    uint8_t dma_busy;
} WS2812_instance;

void WS2812_SPI_Init(uint16_t num);
void WS2812_SPI_SetColor(uint32_t aRGB);
void WS2812_SPI_Send(void);
void WS2812_SPI_Clear(void);
WS2812_instance *Get_WS2812_Ptr(void);

#endif