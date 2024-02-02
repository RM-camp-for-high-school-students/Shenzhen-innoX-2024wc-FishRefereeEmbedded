/*
 * @Description: Task of DM-Bot control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-02-02 12:28:16
 * @LastEditors: qianwan
 */
#include "TaskMotor.h"
#include "RefMessage.h"
#include "ZDT.hpp"
#include "can.h"
#include "om.h"
#include "tx_api.h"


using namespace TASK_MOTOR;

static cDMFace dm_motor[4];
static ZDT::cZDT zdt[4];

#define DM_SPEED_FACE 1.0f


static uint32_t *can_mail_box = nullptr;
static CAN_TxHeaderTypeDef tx_header2_0 = {
        .ExtId = 0x00, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 8};
static CAN_TxHeaderTypeDef tx_header2_1 = {
        .ExtId = 0x00, .IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA, .DLC = 8};

static uint8_t can_data2_0[8];
static uint8_t can_data2_1[8];

static bool can_filter_config();

static void MotorReInit();

TX_THREAD MotorThread;
uint8_t MotorThreadStack[3072] = {0};

/*Close-loop control Motors*/
[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    /*Creat Wheel Topic*/
    om_suber_t *DM_suber = om_subscribe(om_find_topic("DM", UINT32_MAX));
    om_suber_t *ZDT_suber = om_subscribe(om_find_topic("ZDT", UINT32_MAX));

    Msg_ZDT_t msg_zdt{};
    Msg_DM_t msg_dm{};


    for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
        dm_motor[id].SetDM(0x01 + id, &hcan1);
    }
    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
        zdt[id].SetID(id + 0x01);
    }

    uint8_t dm_enable_last[4] = {0, 0, 0, 0};
    uint8_t zdt_enable_last[4] = {0, 0, 0, 0};

//    for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
//        msg_dm.enable[id] = true;
//    }
//    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
//        msg_zdt.enable[id] = true;
//    }

    LL_GPIO_SetOutputPin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin);
    can_filter_config();

    MotorReInit();

    /*Config: Motor PST Set*/
    if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
        LL_TIM_SetAutoReload(TIM2, 399);
        LL_TIM_OC_SetCompareCH4(TIM2, 249);
        tx_thread_sleep(100);
        LL_TIM_OC_SetCompareCH4(TIM2, 0);
        tx_thread_sleep(100);
        LL_TIM_OC_SetCompareCH4(TIM2, 249);
        tx_thread_sleep(100);
        LL_TIM_OC_SetCompareCH4(TIM2, 0);

        tx_thread_sleep(3000);
        /*Set zero pst*/
        // DM
        for (uint8_t id = 0; id < 4; id++) {
            dm_motor[id].SetZero();
            tx_thread_sleep(100);
        }

        LL_TIM_OC_SetCompareCH4(TIM2, 249);
        __disable_interrupts();
        HAL_NVIC_SystemReset();
    }

    // Update ZDT
    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
        zdt[id].TimeUpdate(tx_time_get());
    }
    for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
        dm_motor[id].UpdateTim(tx_time_get());
    }

//    float dm_pst_test[4] = {0.8f, 0.8f, 0.8f, 0.8f};
//    float zdt_pst_test[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//    uint8_t key_last = 1;
//    uint8_t key_step = 0;

    for (;;) {
        om_suber_export(DM_suber, &msg_dm, false);
        om_suber_export(ZDT_suber, &msg_zdt, false);
//        if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
//            if (key_last) {
//                zdt_pst_test[0] += 60.0f;
//                zdt_pst_test[1] += 60.0f;
//                zdt_pst_test[2] += 60.0f;
//                zdt_pst_test[3] += 60.0f;
//                if (key_step == 0) {
//                    dm_pst_test[0] = 0.01f;
//                    dm_pst_test[2] = 0.01f;
//                    key_step++;
//                } else if (key_step == 1) {
//                    dm_pst_test[1] = 0.01f;
//                    dm_pst_test[3] = 0.01f;
//                    key_step++;
//                } else if (key_step == 2) {
//                    dm_pst_test[1] = 0.8f;
//                    dm_pst_test[3] = 0.8f;
//                    key_step++;
//                } else if (key_step == 3) {
//                    dm_pst_test[0] = 0.8f;
//                    dm_pst_test[2] = 0.8f;
//                    key_step = 0;
//                }
//                //0.8 pond
//                //1.1 table
//            }
//            key_last = 0;
//        } else {
//            key_last = 1;
//        }

        /*Motor Close-loop*/
        // DM
        for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
            if (msg_dm.enable[id] == false) {
                dm_motor[id].DisableMotor();
                dm_enable_last[id] = false;
                tx_thread_sleep(1);
            } else if (dm_enable_last[id] == false) {
                dm_motor[id].EnableMotor();
                dm_enable_last[id] = true;
                tx_thread_sleep(1);
            } else {
//                dm_motor[id].PstVelTransmit(dm_pst_test[id], DM_SPEED_FACE);
                dm_motor[id].PstVelTransmit(msg_dm.pst[id], DM_SPEED_FACE);
                tx_thread_sleep(1);
            }
        }
        tx_thread_sleep(5 - sys_state.motor_dm_num);

        // ZDT
        for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
            if (msg_zdt.enable[id] == false) {
                zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0,
                               false);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                zdt_enable_last[id] = false;
                tx_thread_sleep(1);
            } else if (zdt_enable_last[id] == false) {
                zdt[id].ClearPstZero(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, true);
                zdt[id].Enable(tx_header2_1.ExtId, tx_header2_1.DLC, can_data2_1, true);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_1, can_data2_1, can_mail_box);
                zdt_enable_last[id] = true;
                tx_thread_sleep(1);
            } else {
//                zdt[id].TpzPst(tx_header2_0.ExtId, tx_header2_1.ExtId, tx_header2_0.DLC,
//                               tx_header2_1.DLC, can_data2_0, can_data2_1,
//                               ZDT::ANTICLOCKWISE, 120, 120, 60, zdt_pst_test[0],
//                               ZDT::ABSOLUTE);
                zdt[id].TpzPst(tx_header2_0.ExtId, tx_header2_1.ExtId, tx_header2_0.DLC,
                               tx_header2_1.DLC, can_data2_0, can_data2_1,
                               ZDT::ANTICLOCKWISE, 120, 120, 60, msg_zdt.pst[id],
                               ZDT::ABSOLUTE);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_1, can_data2_1, can_mail_box);
                tx_thread_sleep(1);
            }
        }
        tx_thread_sleep(5 - sys_state.motor_zdt_num);

        /*Over Time Check*/
        /*Check motor status*/
        for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
            if (tx_time_get() - dm_motor[id].GetTim() > 1000) {
                dm_motor[id].SetState(ZDT::STATE_ERROR);
            }
            if (dm_motor[id].GetState() == ZDT::STATE_ERROR) {
                sys_state.motor_error = 1;
            }
        }

        /*Check ZDT status*/
        for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
            if (tx_time_get() - zdt[id].GetTime() > 1000) {
                zdt[id].SetState(ZDT::STATE_ERROR);
            }
            if (zdt[id].GetState() == ZDT::STATE_ERROR) {
                sys_state.motor_error = 1;
            }
        }

        while (sys_state.motor_error) {
            /*Disable All motor*/
            // DM
            for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                dm_motor[id].DisableMotor();
                tx_thread_sleep(1);
            }
            // ZDT
            for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
                zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0,
                               false);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                tx_thread_sleep(1);
            }
//            LL_TIM_OC_SetCompareCH4(TIM2, 249);
//            tx_thread_sleep(1000);
//            LL_TIM_OC_SetCompareCH4(TIM2, 0);
            tx_thread_sleep(1000);
        }
    }
}

static void MotorReInit() {
    /*Wait*/
    LL_GPIO_ResetOutputPin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin);
    LL_GPIO_ResetOutputPin(POWER_OUT2_EN_GPIO_Port, POWER_OUT2_EN_Pin);
    tx_thread_sleep(400);
    LL_GPIO_SetOutputPin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin);
    LL_GPIO_SetOutputPin(POWER_OUT2_EN_GPIO_Port, POWER_OUT2_EN_Pin);
    tx_thread_sleep(5000);

    // Disable All Motor
    // DM
    for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
        dm_motor[id].DisableMotor();
        dm_motor[id].SetState(ZDT::FDB_NORMAL);
        tx_thread_sleep(1);
    }

    //Disable ZDT
    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
        zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
        zdt[id].SetState(ZDT::FDB_NORMAL);
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
        tx_thread_sleep(1);
    }
    //Clear OverCurrent
    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
        zdt[id].ClearOverBlock(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
        tx_thread_sleep(1);
    }
    //Clear Zero
    for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
        zdt[id].ClearPstZero(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
        tx_thread_sleep(1);
    }
    sys_state.motor_error = 0;
    // BB
    LL_TIM_OC_SetCompareCH4(TIM2, 249);
    tx_thread_sleep(1000);
    LL_TIM_OC_SetCompareCH4(TIM2, 0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_data[8];
    uint8_t rx_id;
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (rx_header.StdId == 0x00) {
        rx_id = (rx_data[0] & 0x0F) - 1;
        dm_motor[rx_id].MessageDecode(rx_data);
        dm_motor[rx_id].UpdateTim(tx_time_get());
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_data[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    switch (rx_header.ExtId) {
        case 0x100:
        case 0x200:
        case 0x300:
        case 0x400:
            uint8_t id = (uint8_t)(rx_header.ExtId >> 8) - 1;

            zdt[id].TimeUpdate(tx_time_get());

            if (zdt[id].ReadEncoderFDB(rx_data) == ZDT::FDB_NORMAL) {
                break;
            } else if (zdt[id].ReadEncoderFDB(rx_data) == ZDT::FDB_NORMAL) {
                break;
            } else if (zdt[id].TpzPstFDB(rx_data) == ZDT::FDB_NORMAL) {
                break;
            } else if (zdt[id].ClearPstZeroFDB(rx_data) == ZDT::FDB_NORMAL) {
                break;
            } else if (zdt[id].ClearOverBlockFDB(rx_data) == ZDT::FDB_NORMAL) {
                break;
            }
            break;
    }
}

static bool can_filter_config() {
    bool ret = false;
    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterIdHigh = 0;
    filter.FilterIdHigh = 0;
    ret |= HAL_CAN_ConfigFilter(&hcan1, &filter);

    // Allow ID range: All
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    filter.FilterBank = 14;
    filter.SlaveStartFilterBank = 14;
    ret |= HAL_CAN_ConfigFilter(&hcan2, &filter);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    return ret;
}