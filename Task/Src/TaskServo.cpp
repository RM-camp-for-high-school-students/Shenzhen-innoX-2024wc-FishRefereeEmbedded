/*
 * @Description: Task of DM-Bot control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-02-01 11:27:09
 * @LastEditors: qianwan
 */
#include "TaskServo.h"
#include "DWT.hpp"
#include "RefMessage.h"
#include "can.h"
#include "tx_api.h"
#include "om.h"


TX_THREAD ServoThread;
uint8_t ServoThreadStack[1024] = {0};

/*Close-loop control Motors*/
[[noreturn]] void ServoThreadFun(ULONG initial_input) {
    om_suber_t *Servo_suber = om_subscribe(om_config_topic(nullptr, "CA", "Servo", sizeof(Msg_Servo_t)));
    Msg_Servo_t msg_servo{.enable=true};
    LL_TIM_EnableAllOutputs(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableCounter(TIM3);
    uint8_t key_last;
    for (;;) {
        om_suber_export(Servo_suber, &msg_servo, false);

        if (!LL_GPIO_IsInputPinSet(KEY_GPIO_Port, KEY_Pin)) {
            if (key_last) {
                msg_servo.release[0] = !msg_servo.release[0];
                msg_servo.release[1] = !msg_servo.release[1];
                msg_servo.release[2] = !msg_servo.release[2];
            }
            key_last = 0;
        } else {
            key_last = 1;
        }

        msg_servo.enable ? LL_GPIO_SetOutputPin(PWR_5V_EN_GPIO_Port,PWR_5V_EN_Pin) : LL_GPIO_ResetOutputPin(PWR_5V_EN_GPIO_Port,PWR_5V_EN_Pin);

        msg_servo.release[0] ? LL_TIM_OC_SetCompareCH1(TIM3, 899) : LL_TIM_OC_SetCompareCH1(TIM3, 1499);
        msg_servo.release[1] ? LL_TIM_OC_SetCompareCH2(TIM3, 899) : LL_TIM_OC_SetCompareCH2(TIM3, 1499);
        msg_servo.release[2] ? LL_TIM_OC_SetCompareCH3(TIM3, 899) : LL_TIM_OC_SetCompareCH3(TIM3, 1499);

        tx_thread_sleep(10);
    }
}
