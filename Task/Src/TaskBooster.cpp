#include "app_threadx.h"
#include "TaskBooster.h"
#include "Flash.hpp"
#include "om.h"

extern TX_THREAD HeartBeatThread;
extern uint8_t HeartBeatThreadStack[256];

uint8_t ref_system_id = 3;

[[noreturn]] static void SetSysID();

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

extern void HeartBeatThreadFun(ULONG initial_input);

void Task_Booster() {

    tx_byte_pool_create(
            &MsgPool,
            (CHAR *) "Msg_Pool",
            Msg_PoolBuf,
            sizeof(Msg_PoolBuf));

    /*Enable File Service*/
    flashCore.init();
    /*Enable OneMessage Service*/
    om_init();
    if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
        SetSysID();
    }
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

/**********进程***********/
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

static void SetSysID() {
    ref_system_id = 2;
    for (;;) {
        if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {

        }
    }
}