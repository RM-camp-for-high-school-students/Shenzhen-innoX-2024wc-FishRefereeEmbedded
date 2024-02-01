/*
 * @Description: Task of DM-Bot control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-02-01 11:27:09
 * @LastEditors: qianwan
 */
#include "TaskMotor.h"
#include "ZDT.hpp"
#include "DWT.hpp"
#include "libpid-i-1.0.hpp"
#include "RefMessage.h"
#include "can.h"
#include "tx_api.h"
#include "om.h"
#include "Filter.hpp"

#define ENCODER_TO_RADIAN 0.00076699039394f

using namespace PID;

DM_Motor_t dm_motor[4];
static ZDT::cZDT zdt[4];
static cDWT dm_vel_dwt[4];
static cFilterBTW2_100Hz vel_filter[4];

uint8_t motor_error = 0;
extern uint8_t motor_dm_num;
extern uint8_t motor_zdt_num;

PID_Inc_f dm_vel_pid[4];
PID_Pst_f dm_pst_pid[4];

static uint32_t *can_mail_box = nullptr;
static CAN_TxHeaderTypeDef tx_header1_0 = {.StdId=0x3FE, .IDE=CAN_ID_STD, .RTR=CAN_RTR_DATA, .DLC=8};

static CAN_TxHeaderTypeDef tx_header2_0 = {.ExtId=0x00, .IDE=CAN_ID_EXT, .RTR=CAN_RTR_DATA, .DLC=8};
static CAN_TxHeaderTypeDef tx_header2_1 = {.ExtId=0x00, .IDE=CAN_ID_EXT, .RTR=CAN_RTR_DATA, .DLC=8};


static uint8_t can_data1_0[8];
static uint8_t can_data2_0[8];
static uint8_t can_data2_1[8];


static bool can_filter_config();

static void MotorReInit();

TX_THREAD MotorThread;
uint8_t MotorThreadStack[2048] = {0};

/*Close-loop control Motors*/
[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    /*Creat Wheel Topic*/
    om_suber_t *DM_suber = om_subscribe(om_config_topic(nullptr, "CA", "DM_Motor", sizeof(Msg_DM_t)));
    om_suber_t *ZDT_suber = om_subscribe(om_config_topic(nullptr, "CA", "ZDT_Motor", sizeof(Msg_ZDT_t)));
    Msg_ZDT_t msg_zdt;
    Msg_DM_t msg_dm;


    cDWT dwt[2][4];
//    PID_Inc_f dm_vel_pid[4];
    for (uint8_t id = 0; id < motor_dm_num; id++) {
        dm_vel_pid[id].SetParam(0.014f, 0.001f, 0.0015f, 0, 0.004f, 8000.0f, -8000.0f, false, 0, false, 0);
        dm_pst_pid[id].SetParam(0.018f, 0.5f, 0.08f, 0, 0.004, 5, -5, 2, -2, true, 0.26, true, 0.03);
    }

    uint8_t dm_enable_last[4];
    uint8_t zdt_enable_last[4];

    msg_dm.enable[0] = true;


    zdt[0].SetID(0x01);
    zdt[1].SetID(0x02);
    zdt[2].SetID(0x03);
    zdt[3].SetID(0x04);

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
        //DM
        tx_header1_0.StdId = 0x7FF;
        tx_header1_0.DLC = 0x04;
        can_data1_0[0] = 0x33;
        can_data1_0[2] = 0x00;
        can_data1_0[3] = 0xCC;
        for (uint8_t id = 1; id < 5; id++) {
            can_data1_0[1] = id;
            HAL_CAN_AddTxMessage(&hcan1, &tx_header1_0, can_data1_0, can_mail_box);
            tx_thread_sleep(1);
        }

        LL_TIM_OC_SetCompareCH4(TIM2, 249);

        __disable_interrupts();
        NVIC_SystemReset();
        tx_thread_sleep(100);
    }


    //Update ZDT
    for (uint8_t id = 0; id < motor_zdt_num; id++) {
        zdt[id].TimeUpdate(tx_time_get());
    }

    ULONG timer = tx_time_get();
    float dm_pst_test[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t key_last = 1;
    uint8_t dir = 0;
    for (;;) {
//        om_suber_export(DM_suber, &msg_dm, false);
//        om_suber_export(ZDT_suber, &msg_zdt, false);
        if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
            if (key_last) {
                if (dm_pst_test[0] < 0.3) {
                    dm_pst_test[0] = 1.1f;
                } else {
                    dm_pst_test[0] = 0.0f;
                }
            }
            key_last = 0;
        } else {
            key_last = 1;
        }
        /*Motor Close-loop*/
        //PST loop
        for (uint8_t id = 0; id < motor_dm_num; id++) {
            if (msg_dm.enable[id]) {
                //Calculate PID
                dm_pst_pid[id].SetRef(dm_pst_test[id]);
                dm_pst_pid[id].Calculate(dm_motor[id].pst_radian, dwt[0][id].dt_sec());
            } else {
                dm_pst_pid[id].Rst();
            }
        }
        //Velocity loop
        for (uint8_t id = 0; id < motor_dm_num; id++) {
            if (msg_dm.enable[id]) {
                //Calculate PID
                dm_vel_pid[id].SetRef(dm_pst_pid[id].Out());
//                dm_vel_pid[id].SetRef(0.0f);
                dm_vel_pid[id].Calculate(dm_motor[id].vel, dwt[1][id].dt_sec());
                can_data1_0[2 * id] = ((int16_t) dm_vel_pid[id].Out()) >> 8;
                can_data1_0[2 * id + 1] = ((int16_t) dm_vel_pid[id].Out()) & 0xFF;
            } else {
                dm_vel_pid[id].Rst();
                can_data1_0[2 * id] = 0;
                can_data1_0[2 * id + 1] = 0;
            }
        }
        HAL_CAN_AddTxMessage(&hcan1, &tx_header1_0, can_data1_0, can_mail_box);

        //ZDT
        for (uint8_t id = 0; id < motor_zdt_num; id++) {
            if (msg_zdt.enable[id] == false) {
                zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                zdt_enable_last[id] = false;
                tx_thread_sleep(1);
            } else if (zdt_enable_last[id] == false) {
                zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, true);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                zdt_enable_last[id] = true;
                tx_thread_sleep(1);
            } else {
                zdt[id].TpzPst(tx_header2_0.ExtId, tx_header2_1.ExtId, tx_header2_0.DLC, tx_header2_1.DLC, can_data2_0,
                               can_data2_1, ZDT::ANTICLOCKWISE, 120, 120, 60.0, msg_zdt.pst[id], ZDT::ABSOLUTE);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
                HAL_CAN_AddTxMessage(&hcan2, &tx_header2_1, can_data2_1, can_mail_box);
                tx_thread_sleep(1);
            }
        }
        tx_thread_sleep(4 - motor_zdt_num);

        /*Over Time Check*/
        /*Check motor status*/
        for (uint8_t id = 0; id < motor_dm_num; id++) {
            if (tx_time_get() - dm_motor[id].last_update_time > 1000) {
                dm_motor[id].state = ZDT::STATE_ERROR;
            }
            if (dm_motor[id].state == ZDT::STATE_ERROR) {
                motor_error = 1;
            }
        }

        /*Check ZDT status*/
        for (uint8_t id = 0; id < motor_zdt_num; id++) {
            if (tx_time_get() - zdt[id].GetTime() > 1000) {
                zdt[id].SetState(ZDT::STATE_ERROR);
            }
            if (zdt[id].GetState() == ZDT::STATE_ERROR) {
                motor_error = 1;
            }
        }

        while (motor_error) {
            /*Disable All motor*/
            //DM
            memset(can_data1_0, 0x00, 8);
            HAL_CAN_AddTxMessage(&hcan1, &tx_header1_0, can_data1_0, can_mail_box);
            //ZDT
            for (uint8_t id = 0; id < motor_zdt_num; id++) {
                zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
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
    //Disable All Motor
    //DM
    memset(can_data1_0, 0x00, 8);

    tx_header1_0.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header1_0, can_data1_0, can_mail_box);

    //ZDT
    for (uint8_t id = 0; id < motor_zdt_num; id++) {
        zdt[id].SetState(ZDT::STATE_NORMAL);
        zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
        tx_thread_sleep(1);
    }

    //DM
    for (uint8_t id = 0; id < motor_dm_num; id++) {
        dm_motor[id].state = ZDT::STATE_NORMAL;
        dm_motor[id].vel = 0;
        dm_motor[id].last_pst = dm_motor[id].pst;
        //Assume that rotation is less than PI
        if (dm_motor[id].pst > 4096) {
            dm_motor[id].pst_radian = (float) (dm_motor[id].pst - 8192) * ENCODER_TO_RADIAN;
        } else {
            dm_motor[id].pst_radian = (float) (dm_motor[id].pst) * ENCODER_TO_RADIAN;
        }

        dm_vel_pid[id].Rst();
        vel_filter[id].CleanBuf();
    }

    LL_GPIO_ResetOutputPin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin);
    LL_GPIO_ResetOutputPin(POWER_OUT2_EN_GPIO_Port, POWER_OUT2_EN_Pin);
    tx_thread_sleep(300);
    LL_GPIO_SetOutputPin(POWER_OUT1_EN_GPIO_Port, POWER_OUT1_EN_Pin);
    LL_GPIO_SetOutputPin(POWER_OUT2_EN_GPIO_Port, POWER_OUT2_EN_Pin);
    tx_thread_sleep(3000);

    //DM
    memset(can_data1_0, 0x00, 8);
    tx_header1_0.DLC = 8;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header1_0, can_data1_0, can_mail_box);
    //ZDT
    for (uint8_t id = 0; id < motor_zdt_num; id++) {
        zdt[id].Enable(tx_header2_0.ExtId, tx_header2_0.DLC, can_data2_0, false);
        HAL_CAN_AddTxMessage(&hcan2, &tx_header2_0, can_data2_0, can_mail_box);
        tx_thread_sleep(1);
    }

    motor_error = 0;
    //BB
    LL_TIM_OC_SetCompareCH4(TIM2, 249);
    tx_thread_sleep(1000);
    LL_TIM_OC_SetCompareCH4(TIM2, 0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_data[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    uint8_t id = rx_header.StdId - 0x301;
    dm_motor[id].pst = rx_data[0] << 8 | rx_data[1];
    dm_motor[id].rpm = rx_data[2] << 8 | rx_data[3];
    dm_motor[id].current = rx_data[4] << 8 | rx_data[5];
    dm_motor[id].tmp_coil = rx_data[6];
    dm_motor[id].tmp_pcb = rx_data[7];
    dm_motor[id].last_update_time = tx_time_get();

    int16_t d_pst = dm_motor[id].pst - dm_motor[id].last_pst;
    if (d_pst > 5000) {
        d_pst -= 8192;
    } else if (d_pst < -5000) {
        d_pst += 8192;
    }

    dm_motor[id].pst_radian += (float) d_pst * ENCODER_TO_RADIAN;
    dm_motor[id].vel = vel_filter[id].Update(ENCODER_TO_RADIAN * (float) d_pst / dm_vel_dwt[id].dt_sec());
    dm_motor[id].last_pst = dm_motor[id].pst;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t rx_data[8];
    CAN_RxHeaderTypeDef rx_header;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);

    switch (rx_header.ExtId) {
        case 0x100:
            zdt[0].ReadEncoderFDB(rx_data);
            zdt[0].TpzPstFDB(rx_data);
            zdt[0].TimeUpdate(tx_time_get());
            break;
        case 0x200:
            zdt[1].ReadEncoderFDB(rx_data);
            zdt[1].TpzPstFDB(rx_data);
            zdt[1].TimeUpdate(tx_time_get());
            break;
        case 0x300:
            zdt[2].ReadEncoderFDB(rx_data);
            zdt[2].TpzPstFDB(rx_data);
            zdt[2].TimeUpdate(tx_time_get());
            break;
        case 0x400:
            zdt[3].ReadEncoderFDB(rx_data);
            zdt[3].TpzPstFDB(rx_data);
            zdt[3].TimeUpdate(tx_time_get());
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