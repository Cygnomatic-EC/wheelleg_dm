#include "super_cap.h"
#include "user_lib.h"

static super_cap_instance supercap_instance;

void Update_Power(const uint8_t *rx_data, uint32_t id, void *arg);

void super_cap_init(FDCAN_HandleTypeDef *hcan)
{
    super_cap_instance* supercap_ins = &supercap_instance;
    supercap_ins->can_ins = BSP_CAN_Init(hcan);
    BSP_CAN_RegisterStdCallback(supercap_ins->can_ins, 0x22, Update_Power, supercap_ins);
    supercap_instance.init = 1;
}

void super_cap_use(const uint8_t use)
{
    if (!supercap_instance.init)
        return;
    uint8_t send_data[8];
    send_data[0] = use ? 1 : 0;
    BSP_CAN_Transmit(supercap_instance.can_ins, 0x21, FDCAN_STANDARD_ID, send_data, 8);
}

void Update_Power(const uint8_t* rx_data, uint32_t id, void* arg)
{
    if(rx_data == NULL || arg == NULL)
        return;

    super_cap_instance *supercap_ins = (super_cap_instance*)arg;
    unpack_4bytes_to_floats(&rx_data[0], &supercap_ins->power_data.total_power);
    unpack_4bytes_to_floats(&rx_data[4], &supercap_ins->power_data.referee_power);
    supercap_ins->power_data.supercap_power = supercap_ins->power_data.total_power - supercap_ins->power_data.referee_power;
}

super_cap_instance *Get_SuperCap_Instance(void)
{
    return &supercap_instance;
}