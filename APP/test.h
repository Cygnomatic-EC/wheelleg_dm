#ifndef DAMIAO_TEST_H
#define DAMIAO_TEST_H
#include "DBUS/dbus.h"
#include "IMU/INS/ins.h"
#include "motor/DAMIAO/DM8009P/dm8009p.h"
#include "motor/DJI/M3508/m3508.h"
#include "LED/ws2812.h"
#include "VOFA/vofa.h"

void TestTask(void *argument);

typedef struct
{
    rc_instance *rc;
    INS_t *ins;
    dm8009p_instance *dm8009p;
    m3508_instance *m3508;
    WS2812_instance *ws2812;
    vofa_t *vofa;
} Test_t;

#endif //DAMIAO_TEST_H