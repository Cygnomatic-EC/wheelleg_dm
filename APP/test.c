#include "test.h"

static Test_t test;
void test_init(Test_t *test_ins)
{
    test_ins->rc = Get_DBUS_Ptr();
    test_ins->ins = Get_INS_Ptr();
    test_ins->dm8009p = Get_DM8009P_Ptr(2);
    test_ins->m3508 = Get_M3508_Ptr(M3508_TX_1);
    //test_ins->ws2812 = Get_WS2812_Ptr();
    test_ins->vofa = Get_VOFA_Ptr();
    for (uint8_t i = 0; i < 10; i++)
    {
        dm8009p_enable(test_ins->dm8009p);
        osDelay(10);
    }
}

void TestTask(void *argument)
{
    test_init(&test);
    //WS2812_LEDS_Shine(0x00FFFF00, 3);
    while (1)
    {
        const fp32 speed_temp = (fp32)test.rc->rc_data.ch3 * 1000.0f / 660.0f;
        const fp32 tor_temp = (fp32)test.rc->rc_data.ch1 * 1.0f / 660.0f;
        int16_t m3508_speed[4] = {(int16_t)speed_temp, 0, 0, 0};
        fp32 q[4];
        for (int i = 0; i < 4; i++)
        {
            q[i] = test.ins->ins.q[i];
        }
        m3508_ctrl(test.m3508, m3508_speed);
        dm8009p_ctrl_mit(test.dm8009p, 0.0f, 0.0f, 0.0f, 0.0f, tor_temp);
        //vofa_print(q);
        osDelay(1);
    }
}