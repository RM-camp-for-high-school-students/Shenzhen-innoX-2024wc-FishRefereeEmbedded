#include "app_threadx.h"
#include "TaskBooster.h"
#include "Flash.hpp"
#include "RefMessage.h"
#include "om.h"

uint8_t ref_system_id = 0;
uint8_t motor_dm_num = 0;
uint8_t motor_zdt_num = 0;


/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

extern TX_THREAD HeartBeatThread;
extern uint8_t HeartBeatThreadStack[256];

extern void HeartBeatThreadFun(ULONG initial_input);

extern TX_THREAD ScreenThread;
extern uint8_t ScreenThreadStack[512];

extern void ScreenThreadFun(ULONG initial_input);


extern TX_THREAD MotorThread;
extern uint8_t MotorThreadStack[2048];

extern void MotorThreadFun(ULONG initial_input);

extern TX_THREAD RefereeThread;
extern uint8_t RefereeThreadStack[1024];

extern void RefereeThreadFun(ULONG initial_input);

void Task_Booster() {
    LL_TIM_EnableAllOutputs(TIM2);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM2);
    tx_byte_pool_create(
            &MsgPool,
            (CHAR *) "Msg_Pool",
            Msg_PoolBuf,
            sizeof(Msg_PoolBuf));

    /*Enable File Service*/
    flashCore.init();
    /*Enable OneMessage Service*/
    om_init();

    flashCore.flash_memcpy(Flash::Element_ID_REFID, (uint8_t *) &ref_system_id);
    flashCore.config_data(Flash::Element_ID_REFID, (uint8_t *) &ref_system_id, sizeof(ref_system_id));
    if (ref_system_id != 0) {
        if((ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE) || (ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE)){
            motor_dm_num = 1;
            motor_zdt_num = 0;
        } else {
            motor_dm_num = 4;
            motor_zdt_num = 4;
        }
        motor_dm_num = 1;
        motor_zdt_num = 1;
/**********信号量***********/
//	tx_semaphore_create(
//		&MotorHS100Sem,
//		(CHAR*)"MotorHS100Sem",
//		0
//		);


/***********互斥量************/

//	tx_mutex_create(
//		&msgtubeLock,
//		(CHAR*)"TubeMutex",
//		TX_NO_INHERIT);

/**********消息队列***********/
//	/*DBUS*/
//	tx_queue_create(
//		&RemoterRXQue,
//		(CHAR*)"REMOTERQUE",
//		4,
//		RemoterQueueStack,
//		sizeof(RemoterQueueStack));

        tx_thread_create(
                &MotorThread,
                (CHAR *) "Motor",
                MotorThreadFun,
                0x0000,
                MotorThreadStack,
                sizeof(MotorThreadStack),
                3,
                3,
                TX_NO_TIME_SLICE,
                TX_AUTO_START);

        tx_thread_create(
                &RefereeThread,
                (CHAR *) "Referee",
                RefereeThreadFun,
                0x0000,
                RefereeThreadStack,
                sizeof(RefereeThreadStack),
                5,
                5,
                TX_NO_TIME_SLICE,
                TX_AUTO_START);

//        tx_thread_create(
//                &ScreenThread,
//                (CHAR *) "Screen",
//                ScreenThreadFun,
//                0x0000,
//                ScreenThreadStack,
//                sizeof(ScreenThreadStack),
//                5,
//                5,
//                TX_NO_TIME_SLICE,
//                TX_AUTO_START);
/**********进程***********/
    }
    tx_thread_create(
            &HeartBeatThread,
            (CHAR *) "HeatBeat",
            HeartBeatThreadFun,
            0x0000,
            HeartBeatThreadStack,
            sizeof(HeartBeatThreadStack),
            15,
            15,
            TX_NO_TIME_SLICE,
            TX_AUTO_START);
}
