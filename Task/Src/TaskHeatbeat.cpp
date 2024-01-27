/*
 * @Description: 
 * @Author: qianwan
 * @Date: 2023-12-17 23:50:03
 * @LastEditTime: 2024-01-18 03:16:08
 * @LastEditors: qianwan
 */
#include "main.h"
#include "app_threadx.h"
#define SHAKE_TIME 100
/*TraceX utilities*/
#define TRC_BUF_SIZE (2048 * 32) /* Buffer size */
#define TRC_MAX_OBJ_COUNT (40)   /* Max number of ThreadX objects */
UCHAR TraceXBuf[TRC_BUF_SIZE];

extern uint8_t ref_system_id;
TX_THREAD HeartBeatThread;
uint8_t HeartBeatThreadStack[256] = {0};

void HeartBeatThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    tx_thread_sleep(100);

    uint32_t peroid = 2000;


    for (;;) {

        for(uint8_t _id=0;_id<ref_system_id;_id++){
            LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
            tx_thread_sleep( SHAKE_TIME);
            LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
            tx_thread_sleep( SHAKE_TIME);
        }
        tx_thread_sleep(peroid-SHAKE_TIME*ref_system_id);
//        tx_trace_enable(&TraceXBuf, TRC_BUF_SIZE, TRC_MAX_OBJ_COUNT);
    }
}
