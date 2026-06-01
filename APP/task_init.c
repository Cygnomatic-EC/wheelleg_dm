#include "task_init.h"
#include "cmsis_os2.h"
#include "robot_config.h"
#include "chassis_task.h"
#include "observe_task.h"
osThreadId_t chassisR_taskHandle;
osThreadId_t chassisL_taskHandle;
osThreadId_t observe_taskHandle;

const osThreadAttr_t chassisRTask_attr = {
    .name = "chassisRTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t chassisLTask_attr = {
    .name = "chassisLTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t observeTask_attr = {
    .name = "observeTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityHigh,
  };
void task_init()
{
#ifdef CHASSIS
    chassisR_taskHandle = osThreadNew(chassisR_task, NULL, &chassisRTask_attr);
    chassisL_taskHandle = osThreadNew(chassisL_task, NULL, &chassisLTask_attr);
    observe_taskHandle = osThreadNew(Observe_Task, NULL, &observeTask_attr);
#else
#endif

}