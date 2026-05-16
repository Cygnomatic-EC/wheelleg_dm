#include "vofa.h"
#include "user_lib.h"

void vofacallback(uint8_t* data, uint16_t len);

__attribute__((section(".dma_buffer"), aligned(32))) static vofa_t vofa_instance; // 将vofa_instance放在DMA可访问的内存区域

void vofa_init(UART_HandleTypeDef* huart, const uint16_t txnum, const uint16_t rxnum)
{
    memset(&vofa_instance, 0, sizeof(vofa_t)); // 结构体放在了未被清零的区域，可能会导致初始化失败

    vofa_instance.tx_num = txnum;
    if (rxnum > VOFA_RX_NUM_MAX)
        return ;
    vofa_instance.rx_num = rxnum;
    vofa_instance.rx_cnt = 0;
    BSP_UART_Init(&vofa_instance.vofa_uart, huart, 115200, vofacallback, NULL, TX_BUFFER_SIZE, RX_BUFFER_SIZE);
    vofa_instance.init = 1;
}

void vofacallback(uint8_t* data, uint16_t len)
{
    const uint16_t rxlen = vofa_instance.rx_num * sizeof(fp32);
    if(data[0] != PARAM_HEADER || data[3] != 0 || data[4] != 0x5A)
        return ;
    if(data[1] != rxlen)
        return ;
    if (data[5] > 0 && data[5] <= vofa_instance.rx_num)
    {
        unpack_4bytes_to_floats(&data[7], &vofa_instance.rxdata[data[5]]);
        vofa_instance.rx_cnt |= (1 << (data[5] - 1));
    }
    else if (data[5] == 0xFF)
    {
        __HAL_RCC_CLEAR_RESET_FLAGS();
        osDelay(100);
        HAL_NVIC_SystemReset();
    }
    if (vofa_instance.rx_cnt == (1 << vofa_instance.rx_num) - 1)
    {
        vofa_instance.ready = 1;
        vofa_instance.rx_cnt = 0;
    }
}

void vofa_print(const fp32* data)
{
    if (!vofa_instance.init)
        return ;
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t txdata[vofa_instance.tx_num * sizeof(fp32) + 4];
    memcpy(txdata, data, vofa_instance.tx_num * sizeof(fp32));
    memcpy(txdata + vofa_instance.tx_num * sizeof(fp32), tail, 4);
    BSP_UART_Transmit(&vofa_instance.vofa_uart, (uint8_t*)txdata, vofa_instance.tx_num * sizeof(fp32) + 4, 100);
}

vofa_t *Get_VOFA_Ptr(void)
{
    return &vofa_instance;
}