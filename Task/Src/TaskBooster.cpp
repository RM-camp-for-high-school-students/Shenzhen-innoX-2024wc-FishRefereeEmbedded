#include "app_threadx.h"
#include "TaskBooster.h"
#include "Flash.hpp"
#include "RefMessage.h"
#include "om.h"

Sys_state_t sys_state{};

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

extern TX_THREAD HeartBeatThread;
extern uint8_t HeartBeatThreadStack[512];
extern void HeartBeatThreadFun(ULONG initial_input);

extern TX_THREAD ScreenThread;
extern uint8_t ScreenThreadStack[512];
extern void ScreenThreadFun(ULONG initial_input);


extern TX_THREAD MotorThread;
extern uint8_t MotorThreadStack[3072];
extern void MotorThreadFun(ULONG initial_input);

extern TX_SEMAPHORE RefereeRXSem;

extern TX_THREAD RefereeThread;
extern uint8_t RefereeThreadStack[2048];
extern void RefereeThreadFun(ULONG initial_input);

extern TX_THREAD ServoThread;
extern uint8_t ServoThreadStack[1024];
/*Close-loop control Motors*/
extern void ServoThreadFun(ULONG initial_input);

extern TX_THREAD RemoterThread;
extern uint8_t RemoterThreadStack[1024];
extern TX_SEMAPHORE RemoterThreadSem;
extern void RemoterThreadFun(ULONG initial_input);


extern TX_THREAD StateMachineThread;
extern uint8_t StateMachineThreadStack[1024];
extern void StateMachineThreadFun(ULONG initial_input);



uint32_t debug_flag;

void Task_Booster() {
    uint8_t sys_id;
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

    flashCore.flash_memcpy(Flash::Element_ID_REFID, (uint8_t *) &sys_id);
    flashCore.config_data(Flash::Element_ID_REFID, (uint8_t *) &sys_id, sizeof(sys_id));

    sys_state.ref_system_id = sys_id;

    om_config_topic(nullptr, "CA", "Servo", sizeof(Msg_Servo_t));
    om_config_topic(nullptr, "CA", "Remoter", sizeof(Msg_Remoter_Judge_t));
    om_config_topic(nullptr, "CA", "DM", sizeof(Msg_DM_t));
    om_config_topic(nullptr, "CA", "ZDT", sizeof(Msg_ZDT_t));

    if (sys_state.ref_system_id != 0) {

        //Work as feeding table
        if ((sys_state.ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE) || (sys_state.ref_system_id == REF_COMPONENT_ID_Y_FEEDINGTBALE)) {
            sys_state.state_now = REF_FEEDINGTABLE_STATE_FIXING;
            sys_state.state_new = sys_state.state_now;
            sys_state.motor_dm_num = 1;
            sys_state.motor_zdt_num = 0;
            sys_state.angle_set_clean = 1.1f;
            sys_state.angle_set_normal = 0.05f;

        	tx_semaphore_create(
        		&RemoterThreadSem,
        		(CHAR*)"RemoterSem",
        		0
        		);

            tx_thread_create(
                    &ServoThread,
                    (CHAR *) "Servo",
                    ServoThreadFun,
                    0x0000,
                    ServoThreadStack,
                    sizeof(ServoThreadStack),
                    4,
                    4,
                    TX_NO_TIME_SLICE,
                    TX_AUTO_START);

            tx_thread_create(
                    &RemoterThread,
                    (CHAR *) "Remoter",
                    RemoterThreadFun,
                    0x0000,
                    RemoterThreadStack,
                    sizeof(RemoterThreadStack),
                    4,
                    4,
                    TX_NO_TIME_SLICE,
                    TX_AUTO_START);

        } else {
            sys_state.state_now = REF_FISHPOND_STATE_FIXING;
            sys_state.state_new = sys_state.state_now;
            sys_state.motor_dm_num = 4;
            sys_state.motor_zdt_num = 4;
            sys_state.angle_set_clean = 0.8f;
            sys_state.angle_set_normal = 0.05f;
        }
/**********信号量***********/
	tx_semaphore_create(
		&RefereeRXSem,
		(CHAR*)"ReferRXSem",
		0
		);


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
                &StateMachineThread,
                (CHAR *) "StateMachine",
                StateMachineThreadFun,
                0x0000,
                StateMachineThreadStack,
                sizeof(StateMachineThreadStack),
                2,
                2,
                TX_NO_TIME_SLICE,
                TX_AUTO_START);

        tx_thread_create(
                &RefereeThread,
                (CHAR *) "Com",
                RefereeThreadFun,
                0x0000,
                RefereeThreadStack,
                sizeof(RefereeThreadStack),
                4,
                4,
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

uint32_t fishPrintf(uint8_t *buf, const char *str, ...) {
    /*计算字符串长度,并将字符串输出到数据区*/
    va_list ap;
    va_start(ap, str);
    uint32_t len = vsnprintf((char *) buf, 512, str, ap);
    va_end(ap);
    return len;
}