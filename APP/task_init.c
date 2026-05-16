#include "task_init.h"
#include "cmsis_os.h"
#include "robot_config.h"
#include "test.h"
osThreadId_t chassis_taskHandle;
osThreadId_t daemon_taskHandle;

const osThreadAttr_t chassisTask_attr = {
    .name = "chassisTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
void task_init()
{
#ifdef CHASSIS
    chassis_taskHandle = osThreadNew(TestTask, NULL, &chassisTask_attr);
#else
#endif

}