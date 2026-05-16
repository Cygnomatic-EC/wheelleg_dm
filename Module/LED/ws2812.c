#include "ws2812.h"

__attribute__((section(".bdma_buffer"), aligned(32))) static WS2812_instance ws2812 = {0}; // BDMA只能访问D3

static void RGB_to_SPI_Buffer(const uint16_t led_index, const uint8_t r, const uint8_t g, const uint8_t b)
{
    const uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
    uint8_t *buffer_ptr = &ws2812.buffer[led_index * WS2812_BITS_PER_LED];

    for (int i = 23; i >= 0; i--) {
        if (grb & (1 << i)) {
            *buffer_ptr++ = 0xF8;  // 11111000
        } else {
            *buffer_ptr++ = 0xC0;  // 11000000
        }
    }
}

void WS2812_SPI_Init(const uint16_t num)
{
    ws2812.hspi = &hspi6;
    ws2812.init = 1;
    ws2812.num = (num > WS2812_MAX_NUM) ? WS2812_MAX_NUM : num;
    ws2812.dma_busy = 0;

    memset(ws2812.buffer, 0, sizeof(ws2812.buffer));
    WS2812_SPI_Clear();
}

void WS2812_SPI_SetColor(const uint32_t aRGB)
{
    const uint8_t r = (aRGB >> 16) & 0xFF;
    const uint8_t g = (aRGB >> 8) & 0xFF;
    const uint8_t b = aRGB & 0xFF;
    for (uint16_t i = 0; i < ws2812.num; i++) {
        RGB_to_SPI_Buffer(i, r, g, b);
    }
    WS2812_SPI_Send();
}

void WS2812_SPI_Send(void)
{
    if (!ws2812.init || ws2812.dma_busy) return;

    ws2812.dma_busy = 1;

    const uint16_t total_bytes = WS2812_MAX_NUM * WS2812_BITS_PER_LED;
    memset(&ws2812.buffer[total_bytes], 0, WS2812_RESET_PULSES);
    HAL_SPI_Transmit_DMA(ws2812.hspi, ws2812.buffer, total_bytes + WS2812_RESET_PULSES);
}

/* 清空所有LED */
void WS2812_SPI_Clear(void)
{
    for (int i = 0; i < WS2812_MAX_NUM; i++) {
        WS2812_SPI_SetColor(0x00000000);
    }
    WS2812_SPI_Send();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == ws2812.hspi->Instance) {
        ws2812.dma_busy = 0;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == ws2812.hspi->Instance) {
        ws2812.dma_busy = 0;
    }
}

WS2812_instance *Get_WS2812_Ptr(void) {
    return &ws2812;
}