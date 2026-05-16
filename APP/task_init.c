#include "task_init.h"
#include "cmsis_os2.h"
#include "robot_config.h"
#include "chassis_task.h"
osThreadId_t chassisR_taskHandle;
osThreadId_t chassisL_taskHandle;
osThreadId_t daemon_taskHandle;

const osThreadAttr_t chassisRTask_attr = {
    .name = "chassisRTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
const osThreadAttr_t chassisLTask_attr = {
    .name = "chassisLTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
void task_init()
{
#ifdef CHASSIS
    chassisR_taskHandle = osThreadNew(chassisL_task, NULL, &chassisRTask_attr);
    chassisL_taskHandle = osThreadNew(chassisR_task, NULL, &chassisLTask_attr);
#else
#endif

}