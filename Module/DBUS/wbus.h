#ifndef DAMIAO_WBUS_H
#define DAMIAO_WBUS_H
#include "typedef.h"
#include "usart/bsp_uart.h"
#include "can/bsp_can.h"

/* ----------------------- WBUS Channel Definition---------------------------- */
#define WBUS_CH_VALUE_MIN ((uint16_t)364)
#define WBUS_CH_VALUE_OFFSET ((uint16_t)1024)
#define WBUS_CH_VALUE_MAX ((uint16_t)1684)
#define WBUS_VAL_MAX 672.0f

#define WBUS_MAX_LEN     (50)
#define WBUS_BUFLEN      (25)
#define WBUS_HUART       huart5
#define WBUS_1_CANID        0x600
#define WBUS_2_CANID        (WBUS_1_CANID + 1)
#define WBUS_3_CANID        (WBUS_1_CANID + 2)
#define WBUS_4_CANID        (WBUS_1_CANID + 3)

typedef enum
{
    WBUS_DIRECT = 0,
    WBUS_CAN = 1
} WBUS_MODE;
typedef struct
{
    int16_t ch[16];
    WBUS_MODE mode;
} wbus_data_t;

typedef struct
{
    wbus_data_t wbus_data;
    UART_Instance_t wbus_usart;
    CAN_Instance_t* wbus_can;
    uint8_t init;
} wbus_instance;

void wbus_init(WBUS_MODE mode, FDCAN_HandleTypeDef* hcan);
void WBUSData_UART2CAN();
wbus_instance *Get_WBUS_Ptr(void);

#endif //DAMIAO_WBUS_H