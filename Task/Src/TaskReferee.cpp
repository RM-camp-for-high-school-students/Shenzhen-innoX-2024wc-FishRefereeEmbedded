#include "TaskReferee.h"
#include "tx_api.h"
#include "main.h"
#include "ReferCom/mavlink.h"
#include "usart.h"
#include "RefMessage.h"

static mavlink_message_t msg_tx;
static mavlink_message_t msg_rx;

mavlink_component_heartbeat_t heart_beat{};
mavlink_state_rc_t rc_state{};

uint8_t uart_tx_buf[1024];
uint8_t uart_rx0_buf[1024];
uint8_t uart_rx1_buf[1024];
uint16_t tx_len;

TX_SEMAPHORE RefereeRXSem;

TX_THREAD StateMachineThread;
uint8_t StateMachineThreadStack[1024] = {0};

[[noreturn]] void StateMachineThreadFun(ULONG initial_input) {
    Msg_DM_t msg_dm{};
    Msg_ZDT_t msg_zdt{};
    Msg_Servo_t msg_servo{};

    om_topic_t *dm_pub = om_find_topic("DM", UINT32_MAX);
    om_topic_t *zdt_pub = om_find_topic("ZDT", UINT32_MAX);
    om_topic_t *servo_pub = om_find_topic("Servo", UINT32_MAX);

    if ((sys_state.ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE) ||
        (sys_state.ref_system_id == REF_COMPONENT_ID_Y_FEEDINGTBALE)) {

        for (;;) {
            if (sys_state.motor_error) {
                sys_state.state_now = REF_FEEDINGTABLE_STATE_STOP;
                if (sys_state.state_last != REF_FEEDINGTABLE_STATE_STOP) {
                    sys_state.state_new = sys_state.state_now;
                }
            }

            if (sys_state.state_now == REF_FEEDINGTABLE_STATE_STOP) {
                sys_state.motor_error = true;

                sys_state.state_switch_flag = false;
                heart_beat.error_code = REF_ERROR_COMMUNICATION;
                if (sys_state.state_new == REF_FEEDINGTABLE_STATE_FIXING) {
                    __disable_interrupts();
                    HAL_NVIC_SystemReset();
                }
            } else if (sys_state.state_now == REF_FEEDINGTABLE_STATE_FIXING) {
                for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                    msg_dm.enable[id] = false;
                    msg_dm.pst[id] = sys_state.angle_set_clean;
                }
                msg_servo.enable = false;
                msg_servo.release[0] = false;
                msg_servo.release[1] = false;
                msg_servo.release[2] = false;

                om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                om_publish(servo_pub, &msg_servo, sizeof(msg_servo), true, false);

                if (sys_state.state_new != sys_state.state_now) {
                    if (sys_state.state_new == REF_FEEDINGTABLE_STATE_NORMAL) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                            msg_dm.enable[id] = true;
                            msg_dm.pst[id] = sys_state.angle_set_normal;
                        }
                        msg_servo.enable = true;

                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        om_publish(servo_pub, &msg_servo, sizeof(msg_servo), true, false);

                        tx_thread_sleep(100);
                    }
                    sys_state.state_switch_flag = false;
                }
            } else if (sys_state.state_now == REF_FEEDINGTABLE_STATE_NORMAL) {
                for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                    msg_dm.enable[id] = true;
                    msg_dm.pst[id] = sys_state.angle_set_normal;
                }

                msg_servo.enable = true;

                om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                om_publish(servo_pub, &msg_servo, sizeof(msg_servo), true, false);

                if (sys_state.state_new != sys_state.state_now) {
                    if (sys_state.state_new == REF_FEEDINGTABLE_STATE_CLEAN) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        msg_dm.pst[0] = sys_state.angle_set_clean;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(1000);

                        msg_dm.pst[0] = sys_state.angle_set_normal;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(500);

                        sys_state.state_switch_flag = false;
                    } else if (sys_state.state_new == REF_FEEDINGTABLE_STATE_RELEASE) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        if (sys_state.state_parameter <= 3) {
                            msg_servo.release[sys_state.state_parameter - 1] = true;
                        }
                        om_publish(servo_pub, &msg_servo, sizeof(msg_servo), true, false);

                        sys_state.state_switch_flag = false;
                    }
                }
            } else if (sys_state.state_now == REF_FEEDINGTABLE_STATE_CLEAN) {
                tx_thread_sleep(100);
                sys_state.state_new = REF_FEEDINGTABLE_STATE_NORMAL;
                sys_state.state_now = REF_FEEDINGTABLE_STATE_NORMAL;
            } else if (sys_state.state_now == REF_FEEDINGTABLE_STATE_RELEASE) {
                tx_thread_sleep(100);
                sys_state.state_new = REF_FEEDINGTABLE_STATE_NORMAL;
                sys_state.state_now = REF_FEEDINGTABLE_STATE_NORMAL;
            }

            sys_state.state_last = sys_state.state_now;
            tx_thread_sleep(5);
        }
    } else {
        for (;;) {
            if (sys_state.motor_error) {
                sys_state.state_now = REF_FISHPOND_STATE_STOP;
                if (sys_state.state_last != REF_FISHPOND_STATE_STOP) {
                    sys_state.state_new = sys_state.state_now;
                }
            }

            if (sys_state.state_now == REF_FISHPOND_STATE_STOP) {
                sys_state.motor_error = true;
                sys_state.state_switch_flag = false;
                heart_beat.error_code = REF_ERROR_COMMUNICATION;
                if (sys_state.state_new == REF_FISHPOND_STATE_FIXING) {
                    __disable_interrupts();
                    HAL_NVIC_SystemReset();
                }
            } else if (sys_state.state_now == REF_FISHPOND_STATE_FIXING) {
                for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                    msg_dm.enable[id] = false;
                    msg_dm.pst[id] = sys_state.angle_set_clean;
                }
                for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
                    msg_zdt.enable[id] = false;
                }

                om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                om_publish(zdt_pub, &msg_zdt, sizeof(msg_zdt), true, false);


                if (sys_state.state_new != sys_state.state_now) {
                    if (sys_state.state_new == REF_FISHPOND_STATE_NORMAL) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                            msg_dm.enable[id] = true;
                            msg_dm.pst[id] = sys_state.angle_set_normal;
                        }
                        for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
                            msg_zdt.enable[id] = true;
                        }
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        om_publish(zdt_pub, &msg_zdt, sizeof(msg_zdt), true, false);
                        tx_thread_sleep(300);
                    }
                    sys_state.state_switch_flag = false;
                }

            } else if (sys_state.state_now == REF_FISHPOND_STATE_NORMAL) {
                for (uint8_t id = 0; id < sys_state.motor_dm_num; id++) {
                    msg_dm.enable[id] = true;
                    msg_dm.pst[id] = sys_state.angle_set_normal;
                }
                for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
                    msg_zdt.enable[id] = true;
                }

                om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                om_publish(zdt_pub, &msg_zdt, sizeof(msg_zdt), true, false);

                if (sys_state.state_new != sys_state.state_now) {
                    if (sys_state.state_new == REF_FISHPOND_STATE_CLEAN) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        msg_dm.pst[0] = sys_state.angle_set_clean;
                        msg_dm.pst[2] = sys_state.angle_set_clean;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(100);
                        msg_dm.pst[1] = sys_state.angle_set_clean;
                        msg_dm.pst[3] = sys_state.angle_set_clean;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(1000);
                        msg_dm.pst[1] = sys_state.angle_set_normal;
                        msg_dm.pst[3] = sys_state.angle_set_normal;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(100);
                        msg_dm.pst[0] = sys_state.angle_set_normal;
                        msg_dm.pst[2] = sys_state.angle_set_normal;
                        om_publish(dm_pub, &msg_dm, sizeof(msg_dm), true, false);
                        tx_thread_sleep(200);

                        sys_state.state_switch_flag = false;
                    } else if (sys_state.state_new == REF_FISHPOND_STATE_RELEASE) {
                        sys_state.state_switch_flag = true;
                        sys_state.state_now = sys_state.state_new;

                        if (sys_state.state_parameter <= 6) {
                            for (uint8_t id = 0; id < sys_state.motor_zdt_num; id++) {
                                msg_zdt.pst[id] = 60.0f * (float) sys_state.state_parameter;
                            }
                        }
                        om_publish(zdt_pub, &msg_zdt, sizeof(msg_zdt), true, false);
                        tx_thread_sleep(500);

                        sys_state.state_switch_flag = false;
                    }
                }
            } else if (sys_state.state_now == REF_FISHPOND_STATE_CLEAN) {
                tx_thread_sleep(100);
                sys_state.state_new = REF_FEEDINGTABLE_STATE_NORMAL;
                sys_state.state_now = REF_FEEDINGTABLE_STATE_NORMAL;
            } else if (sys_state.state_now == REF_FISHPOND_STATE_RELEASE) {
                tx_thread_sleep(100);
                sys_state.state_new = REF_FEEDINGTABLE_STATE_NORMAL;
                sys_state.state_now = REF_FEEDINGTABLE_STATE_NORMAL;
            }

            sys_state.state_last = sys_state.state_now;
            tx_thread_sleep(5);
        }
    }
}

TX_THREAD RefereeThread;
uint8_t RefereeThreadStack[2048] = {0};

[[noreturn]] void RefereeThreadFun(ULONG initial_input) {
    om_suber_t *suber_remoter = om_subscribe(om_find_topic("Remoter", UINT32_MAX));
    sys_state.uart_rx_buf = uart_rx0_buf;
    sys_state.buf_len = sizeof(uart_rx0_buf);
    tx_thread_sleep(5000);

    Msg_Remoter_Judge_t rmt_msg{};
    uint32_t server_lose_flag = 0;
    int chan = MAVLINK_COMM_0;
    mavlink_status_t mavlink_status;
    mavlink_set_conponent_state_t set_state_msg{};

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, sys_state.uart_rx_buf, sys_state.buf_len);

    for (;;) {
        heart_beat.pack_count++;
        heart_beat.battery_voltage = 24000;

        tx_len = 0;
        if ((sys_state.ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE) ||
            (sys_state.ref_system_id == REF_COMPONENT_ID_Y_FEEDINGTBALE)) {
            om_suber_export(suber_remoter, &rmt_msg, false);
            rc_state.state = rmt_msg.connected;

            mavlink_msg_component_heartbeat_encode(sys_state.ref_system_id, sys_state.ref_system_id, &msg_tx,
                                                   &heart_beat);
            tx_len += mavlink_msg_to_send_buffer(uart_tx_buf + tx_len, &msg_tx);
            mavlink_msg_state_rc_encode(sys_state.ref_system_id, sys_state.ref_system_id, &msg_tx,
                                        &rc_state);
            tx_len += mavlink_msg_to_send_buffer(uart_tx_buf + tx_len, &msg_tx);

        } else {
            mavlink_msg_component_heartbeat_encode(sys_state.ref_system_id, sys_state.ref_system_id, &msg_tx,
                                                   &heart_beat);
            tx_len += mavlink_msg_to_send_buffer(uart_tx_buf + tx_len, &msg_tx);
        }

        HAL_UART_Transmit_DMA(&huart4, uart_tx_buf, tx_len);

        if (tx_semaphore_get(&RefereeRXSem, TX_NO_WAIT) == TX_SUCCESS) {
            server_lose_flag = 0;
            uint8_t *rx_buf_now = sys_state.uart_rx_buf;
            uint16_t rx_len = sys_state.rx_len;
            uint16_t cnt = 0;
            sys_state.uart_rx_buf = sys_state.uart_rx_buf == uart_rx0_buf ? uart_rx1_buf : uart_rx0_buf;

            while (rx_len--) {
                if (mavlink_parse_char(chan, rx_buf_now[cnt++], &msg_rx, &mavlink_status)) {
                    switch (msg_rx.msgid) {
                        case MAVLINK_MSG_ID_SERVER_HEARTBEAT:
                            break;
                        case MAVLINK_MSG_ID_SET_CONPONENT_STATE:
                            mavlink_msg_set_conponent_state_decode(&msg_rx, &set_state_msg);
                            if (set_state_msg.component == sys_state.ref_system_id) {
                                if ((set_state_msg.component == REF_FISHPOND_STATE_STOP) ||
                                    (set_state_msg.component == REF_FEEDINGTABLE_STATE_STOP)) {
                                    sys_state.state_now = set_state_msg.new_state;
                                    sys_state.state_new = set_state_msg.new_state;
                                    sys_state.motor_error = true;
                                } else if (sys_state.state_switch_flag == 0) {
                                    sys_state.state_now = set_state_msg.new_state;
                                    sys_state.state_new = set_state_msg.new_state;
                                    sys_state.state_parameter = set_state_msg.parameter;
                                }
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        else {
            server_lose_flag++;
            if (server_lose_flag > 100) {
                if((sys_state.state_now != REF_FISHPOND_STATE_STOP) && (sys_state.state_now != REF_FEEDINGTABLE_STATE_STOP)) {
                    if ((sys_state.ref_system_id == REF_COMPONENT_ID_X_FEEDINGTBALE) ||
                        (sys_state.ref_system_id == REF_COMPONENT_ID_Y_FEEDINGTBALE)) {
                        sys_state.state_new = REF_FEEDINGTABLE_STATE_FIXING;
                    } else {
                        sys_state.state_new = REF_FISHPOND_STATE_FIXING;
                    }
                }
            }
        }
        tx_thread_sleep(10);
    }
}