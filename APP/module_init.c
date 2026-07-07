#include "module_init.h"

#include "fdcan.h"
#include "usart.h"
#include "dwt/bsp_dwt.h"
#include "DBUS/dbus.h"
#include "DBUS/wbus.h"
#include "IMU/INS/ins.h"
#include "motor/DAMIAO/DM8009P/dm8009p.h"
#include "motor/DJI/M3508/m3508.h"
#include "LED/ws2812.h"
#include "motor/LK/lk.h"
#include "VOFA/vofa.h"

void Modules_Init(void) {
    DWT_Init(DWT_CLOCK_FREQ);

    //dbus_init(RC_DIRECT, &hfdcan3);
    wbus_init(WBUS_DIRECT, &hfdcan3);
    BMI088_Init();
    INS_Init();
    dm8009p_init(&hfdcan1, 0x02, 0x12); // 左前
    dm8009p_init(&hfdcan1, 0x04, 0x14); // 左后
    dm8009p_init(&hfdcan2, 0x01, 0x11); // 右前
    dm8009p_init(&hfdcan2, 0x03, 0x13); // 右后
#ifdef INFANTRY
    m3508_init(&hfdcan1, M3508_TX_1, 1); // 左轮
    m3508_init(&hfdcan2, M3508_TX_2, 1); // 右轮
#elif HERO
    lk_init(&hfdcan1, 0x142);
    lk_init(&hfdcan2, 0x141);
#endif
    WS2812_SPI_Init(1);
    vofa_init(&huart1, 6, 0);

#ifdef CHASSIS
#endif
}
