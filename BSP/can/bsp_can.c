// TODO: 增加对扩展id的适配，但考虑到目前只有sb灵足用到，暂时不增加对扩展id的适配
#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"

/* 全局CAN实例映射 */
static CAN_Instance_t can_ins1 = {.init = 0}, can_ins2 = {.init = 0}, can_ins3 = {.init = 0};
static CAN_Instance_t *g_can_instances[MAX_CAN_INSTANCES] = {&can_ins1, &can_ins2, &can_ins3};

/* 静态函数原型 */
static CAN_Status_t CAN_ConfigureFilters(const CAN_Instance_t *can_ins);
static uint8_t CAN_FindEmptyIndex(const CAN_Instance_t *can_ins);
static uint8_t CAN_FindCallbackIndex(const CAN_Instance_t *can_ins, uint32_t can_id);
CAN_Status_t BSP_CAN_Start(const CAN_Instance_t *can_ins);

/**
 * @brief  初始化CAN外设
 * @param  hcan: 指向HAL CAN句柄
 * @retval 指向CAN实例句柄的指针
 */
CAN_Instance_t* BSP_CAN_Init(FDCAN_HandleTypeDef *hcan)
{
    CAN_Instance_t *can_ins = NULL;
    if (hcan == NULL) {
        return NULL;
    }

    /* 确定CAN实例号 */
    uint8_t can_index;
    if (hcan->Instance == FDCAN1)
        can_index = 0;
    else if (hcan->Instance == FDCAN2)
        can_index = 1;
    else if (hcan->Instance == FDCAN3)
        can_index = 2;
    else
        return NULL;

    can_ins = g_can_instances[can_index];

    /* 若已完成初始化则直接返回地址 */
    if (can_ins->init)
        return can_ins;

    /* 若未完成初始化则进行以下步骤 */
    /* 初始化CAN实例结构 */
    can_ins->handle = hcan;
    can_ins->can_instance = can_index;

    /* 清空回调函数数组 */
    memset(can_ins->rx_callbacks, 0, sizeof(can_ins->rx_callbacks));
    memset(can_ins->callback_ids, 0, sizeof(can_ins->callback_ids));
    memset(can_ins->callback_idmasks, 0xFF, sizeof(can_ins->callback_idmasks));
    memset(can_ins->callback_ide, 0, sizeof(can_ins->callback_ide));

    /* 配置CAN过滤器 */
    if (CAN_ConfigureFilters(can_ins) != CAN_OK) {
        return NULL;
    }

    can_ins->init = 1;
    return can_ins;
}

/**
 * @brief  注册CAN 标准ID接收回调函数
 * @param  can_ins: 指向CAN实例句柄的指针
 * @param  can_id: CAN ID
 * @param  rxCallback: 接收回调函数指针
 * @param  arg: 回调函数存储地址
 * @retval 注册结果 (CAN_Status_t)
 */
CAN_Status_t BSP_CAN_RegisterStdCallback(CAN_Instance_t *can_ins,
                                     const uint32_t can_id,
                                     const CAN_RxCallback_t rxCallback,
                                     void* arg)
{
    if (can_ins == NULL || rxCallback == NULL) {
        return CAN_ERROR_INVALID_PARAM;
    }

    if (can_id > 0x7FF) {
        return CAN_ERROR_INVALID_PARAM;
    }

    /* 检查是否已存在相同ID的回调 */
    const uint8_t existing_index = CAN_FindCallbackIndex(can_ins, can_id);
    if (existing_index < MAX_CAN_FILTERS) {
        /* 更新现有回调 */
        can_ins->rx_callbacks[existing_index] = rxCallback;
        return CAN_OK;
    }

    /* 查找可用位置 */
    const uint8_t filter_bank = CAN_FindEmptyIndex(can_ins);
    if (filter_bank >= MAX_CAN_FILTERS) {
        return CAN_ERROR_NO_MEMORY;
    }

    /* 注册回调函数 */
    can_ins->rx_callbacks[filter_bank] = rxCallback;
    can_ins->callback_ids[filter_bank] = can_id;
    can_ins->callback_ide[filter_bank] = FDCAN_STANDARD_ID;
    can_ins->callback_args[filter_bank] = arg;
    can_ins->callback_idmasks[filter_bank] = 0x7FF; // 标准ID掩码

    return CAN_OK;
}

/**
 * @brief  注册CAN 扩展ID接收回调函数
 * @param  can_ins: 指向CAN实例句柄的指针
 * @param  can_id: CAN ID
 * @param  id_mask: CAN ID掩码，接收时会将接收到的ID与can_id进行掩码匹配，匹配成功则调用回调函数
 * @param  rxCallback: 接收回调函数指针
 * @param  arg: 回调函数存储地址
 * @retval 注册结果 (CAN_Status_t)
 */
CAN_Status_t BSP_CAN_RegisterExtCallback(CAN_Instance_t *can_ins,
                                     const uint32_t can_id,
                                     const uint32_t id_mask,
                                     const CAN_RxCallback_t rxCallback,
                                     void* arg)
{
    if (can_ins == NULL || rxCallback == NULL) {
        return CAN_ERROR_INVALID_PARAM;
    }

    if (can_id > 0x1FFFFFFF) {
        return CAN_ERROR_INVALID_PARAM;
    }

    /* 检查是否已存在相同ID的回调 */
    const uint8_t existing_index = CAN_FindCallbackIndex(can_ins, can_id);
    if (existing_index < MAX_CAN_FILTERS) {
        /* 更新现有回调 */
        can_ins->rx_callbacks[existing_index] = rxCallback;
        return CAN_OK;
    }

    /* 查找可用位置 */
    const uint8_t filter_bank = CAN_FindEmptyIndex(can_ins);
    if (filter_bank >= MAX_CAN_FILTERS) {
        return CAN_ERROR_NO_MEMORY;
    }

    /* 注册回调函数 */
    can_ins->rx_callbacks[filter_bank] = rxCallback;
    can_ins->callback_ids[filter_bank] = can_id;
    can_ins->callback_ide[filter_bank] = FDCAN_EXTENDED_ID;
    can_ins->callback_args[filter_bank] = arg;
    can_ins->callback_idmasks[filter_bank] = id_mask;

    return CAN_OK;
}


/**
 * @brief  发送CAN消息
 * @param  can_ins: 指向CAN实例句柄的指针
 * @param  can_id: CAN ID
 * @param  id_type: ID类型
 * @param  data: 数据缓冲区指针
 * @param  len: 数据长度
 * @retval 发送结果 (CAN_Status_t)
 */
CAN_Status_t BSP_CAN_Transmit(const CAN_Instance_t *can_ins,
                                const uint32_t can_id,
                                const uint32_t id_type,
                                const uint8_t *data,
                                const uint8_t len)
{
    if (can_ins == NULL || data == NULL) {
        return CAN_ERROR_INVALID_PARAM;
    }

    if (can_id > (id_type == FDCAN_STANDARD_ID ? 0x7FF : 0x1FFFFFFF)) {
        return CAN_ERROR_INVALID_PARAM;
    }

    FDCAN_TxHeaderTypeDef tx_header;

    /* 配置发送头 */
    tx_header.Identifier = can_id;
    tx_header.IdType = id_type;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    if(len<=8)
        tx_header.DataLength = len;
    else
    {
        switch (len)
        {
            case 12:
                tx_header.DataLength = FDCAN_DLC_BYTES_12;
                break;
            case 16:
                tx_header.DataLength = FDCAN_DLC_BYTES_16;
                break;
            case 20:
                tx_header.DataLength = FDCAN_DLC_BYTES_20;
                break;
            case 24:
                tx_header.DataLength = FDCAN_DLC_BYTES_24;
                break;
            case 32:
                tx_header.DataLength = FDCAN_DLC_BYTES_32;
                break;
            case 48:
                tx_header.DataLength = FDCAN_DLC_BYTES_48;
                break;
            case 64:
                tx_header.DataLength = FDCAN_DLC_BYTES_64;
                break;
            default:
                return CAN_ERROR_INVALID_PARAM;
        }
    }
    tx_header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat =  FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(can_ins->handle, &tx_header, data) != HAL_OK) {
        return CAN_ERROR;
    }

    return CAN_OK;
}

/**
 * @brief  CAN接收中断处理程序
 * @param  can_ins: 指向CAN实例句柄的指针
 * @retval 无
 */
void BSP_CAN_Rx_IRQHandler(CAN_Instance_t *can_ins)
{
    if (can_ins == NULL || can_ins->handle == NULL) {
        return;
    }

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t* rx_data = can_ins->rxbuf;

    /* 从FIFO0读取消息 */
    if (HAL_FDCAN_GetRxMessage(can_ins->handle, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        /* 填充CAN消息结构 */
        const uint32_t rxid = rx_header.Identifier;
        const uint8_t idpos = CAN_FindCallbackIndex(can_ins, rxid);
        void* arg = can_ins->callback_args[idpos];
        if (idpos < MAX_CAN_FILTERS && can_ins->rx_callbacks[idpos] != NULL && can_ins->callback_ide[idpos] == rx_header.IdType)
            can_ins->rx_callbacks[idpos](rx_data, rxid, arg);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
    {
        if(hfdcan -> Instance == FDCAN1 && g_can_instances[0] != NULL)
        {
            BSP_CAN_Rx_IRQHandler(g_can_instances[0]);
        }
        else if (hfdcan -> Instance == FDCAN2 && g_can_instances[1] != NULL)
        {
            BSP_CAN_Rx_IRQHandler(g_can_instances[1]);
        }
        else if (hfdcan -> Instance == FDCAN3 && g_can_instances[2] != NULL)
        {
            BSP_CAN_Rx_IRQHandler(g_can_instances[2]);
        }
    }
    else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)
    {
        /* 处理FIFO满中断 */
        // 可以在这里添加相应的处理代码，例如记录日志或清空FIFO
    }
    else if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST)
    {
        /* 处理消息丢失中断 */
        // 可以在这里添加相应的处理代码，例如记录日志或清空FIFO
    }
}

/**
 * @brief  配置CAN过滤器
 * @param  can_ins: 指向CAN实例句柄的指针
 * @retval 配置结果 (CAN_Status_t)
 */
static CAN_Status_t CAN_ConfigureFilters(const CAN_Instance_t *can_ins)
{
    if (can_ins == NULL || can_ins->handle == NULL) {
        return CAN_ERROR_INVALID_PARAM;
    }

    FDCAN_FilterTypeDef filter_config;

    /* 配置基本过滤器 - 接受所有标准ID消息 */
    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = can_ins->can_instance;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = 0x000; // 接受所有标准ID
    filter_config.FilterID2 = 0x000; // 标准ID掩码

    if (HAL_FDCAN_ConfigFilter(can_ins->handle, &filter_config) != HAL_OK) {
        return CAN_ERROR;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(can_ins->handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(can_ins->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(can_ins->handle) != HAL_OK)
    {
        Error_Handler();
    }
    return CAN_OK;
}

/**
 * @brief  查找可用存放
 * @param  can_ins: 指向CAN实例句柄的指针
 * @retval 可用过滤器组号，若无可用则返回MAX_CAN_FILTERS
 */
static uint8_t CAN_FindEmptyIndex(const CAN_Instance_t *can_ins)
{
    for (uint8_t i = 0; i < MAX_CAN_FILTERS; i++) {
        if (can_ins->callback_ids[i] == 0) {
            return i;
        }
    }
    return MAX_CAN_FILTERS;
}

/**
 * @brief  根据CAN ID查找回调函数索引
 * @param  can_ins: 指向CAN实例句柄的指针
 * @param  can_id: CAN ID
 * @retval 回调函数索引，若未找到则返回MAX_CAN_FILTERS
 */
static uint8_t CAN_FindCallbackIndex(const CAN_Instance_t *can_ins, const uint32_t can_id)
{
    for (uint8_t i = 0; i < MAX_CAN_FILTERS; i++) {
        if ((can_ins->callback_ids[i] & can_ins->callback_idmasks[i]) == (can_id & can_ins->callback_idmasks[i])) {
            return i;
        }
    }
    return MAX_CAN_FILTERS;
}
