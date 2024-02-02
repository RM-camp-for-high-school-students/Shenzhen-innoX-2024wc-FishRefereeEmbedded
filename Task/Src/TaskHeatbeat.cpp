/*
 * @Description: 
 * @Author: qianwan
 * @Date: 2023-12-17 23:50:03
 * @LastEditTime: 2024-01-18 03:16:08
 * @LastEditors: qianwan
 */
#include "main.h"
#include "app_threadx.h"
#include "Flash.hpp"
#include "ReferCom/mavlink.h"
#include "RefMessage.h"

#define SHAKE_TIME 150
/*TraceX utilities*/
#define TRC_BUF_SIZE (2048 * 32) /* Buffer size */
#define TRC_MAX_OBJ_COUNT (40)   /* Max number of ThreadX objects */
UCHAR TraceXBuf[TRC_BUF_SIZE];

static void SetSysID();
TX_THREAD HeartBeatThread;
uint8_t HeartBeatThreadStack[512] = {0};

/*Heart beat led show and id config*/
void HeartBeatThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    UINT old_pir;
    LL_TIM_OC_SetCompareCH4(TIM2,249);
    if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
        tx_thread_priority_change(&HeartBeatThread,0,&old_pir);
        SetSysID();
    }

    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH4(TIM2,0);
    uint32_t period = 2000;

    while(sys_state.ref_system_id==0){
        LL_TIM_OC_SetCompareCH4(TIM2,249);
        LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
        tx_thread_sleep(1500);
        LL_TIM_OC_SetCompareCH4(TIM2,0);
        LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
        tx_thread_sleep(1500);
    }

    for (;;) {
        for(uint8_t _id=0;_id<sys_state.ref_system_id;_id++){
            LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
            tx_thread_sleep( SHAKE_TIME);
            LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
            tx_thread_sleep( SHAKE_TIME);
        }
        tx_thread_sleep(period - SHAKE_TIME * sys_state.ref_system_id);
//        tx_trace_enable(&TraceXBuf, TRC_BUF_SIZE, TRC_MAX_OBJ_COUNT);
    }
}

//Allowd id: REF_COMPONENT_ID_X_FISHMONGER_A-REF_COMPONENT_ID_Y_FEEDINGTBALE
static void SetSysID() {
    ULONG time = tx_time_get();
    bool last_status;
    uint8_t ref_system_id = 0;
    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH4(TIM2,0);
    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH4(TIM2,249);
    tx_thread_sleep(100);
    LL_TIM_OC_SetCompareCH4(TIM2,0);
    for (;;) {
        if ((!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) && last_status) {
            LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
            tx_thread_sleep(50);
            LL_GPIO_SetOutputPin(LED_GPIO_Port,LED_Pin);
            ref_system_id += 1;
            time = tx_time_get();
            tx_thread_sleep(100);
            last_status = false;
        }
        else{
            last_status = LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin);
            if(tx_time_get()-time > 2000){
                LL_GPIO_ResetOutputPin(LED_GPIO_Port,LED_Pin);
                break;
            }
        }
        tx_thread_sleep(20);
    }
    if(ref_system_id<REF_COMPONENT_ID_X_FEEDINGTBALE){
        ref_system_id = 0;
    }
    else if(ref_system_id>REF_COMPONENT_ID_FISHPOND){
        ref_system_id = REF_COMPONENT_ID_FISHPOND;
    }
    flashCore.config_data(Flash::Element_ID_REFID,&ref_system_id,sizeof(ref_system_id));
    __disable_interrupts();
    flashCore.rebuild();
    NVIC_SystemReset();
}