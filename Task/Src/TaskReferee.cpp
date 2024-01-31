#include "TaskReferee.h"
#include "tx_api.h"
#include "main.h"
#include "ReferCom/mavlink.h"
#include "usart.h"

static mavlink_message_t msg_tx;
mavlink_component_heartbeat_t heart_beat_pond = {.pack_count = 0 ,.battery_voltage=24000, .state=REF_FISHPOND_STATE_NORMAL, .error_code=REF_ERROR_NONE,};
mavlink_component_heartbeat_t heart_beat_table_x = {.pack_count = 0 ,.battery_voltage=24000, .state=REF_FEEDINGTABLE_STATE_NORMAL, .error_code=REF_ERROR_NONE,};
mavlink_component_heartbeat_t heart_beat_table_y = {.pack_count = 0 ,.battery_voltage=24000, .state=REF_FEEDINGTABLE_STATE_NORMAL, .error_code=REF_ERROR_NONE,};
mavlink_state_rc_t rc_state_x = {.state=1};
mavlink_state_rc_t rc_state_y = {.state=1};

uint8_t tx_buf[1024];
uint16_t tx_len;


TX_THREAD RefereeThread;
uint8_t RefereeThreadStack[1024] = {0};

[[noreturn]] void RefereeThreadFun(ULONG initial_input) {

    tx_thread_sleep(5000);
    for(;;){
        tx_len = 0;
        mavlink_msg_component_heartbeat_encode(REF_COMPONENT_ID_X_FEEDINGTBALE,REF_COMPONENT_ID_X_FEEDINGTBALE,&msg_tx,&heart_beat_table_x);
        tx_len += mavlink_msg_to_send_buffer(tx_buf+tx_len, &msg_tx);
        mavlink_msg_state_rc_encode(REF_COMPONENT_ID_X_FEEDINGTBALE,REF_COMPONENT_ID_X_FEEDINGTBALE,&msg_tx,&rc_state_x);
        tx_len += mavlink_msg_to_send_buffer(tx_buf+tx_len, &msg_tx);
        mavlink_msg_component_heartbeat_encode(REF_COMPONENT_ID_Y_FEEDINGTBALE,REF_COMPONENT_ID_Y_FEEDINGTBALE,&msg_tx,&heart_beat_table_x);
        tx_len += mavlink_msg_to_send_buffer(tx_buf+tx_len, &msg_tx);
        mavlink_msg_state_rc_encode(REF_COMPONENT_ID_Y_FEEDINGTBALE,REF_COMPONENT_ID_Y_FEEDINGTBALE,&msg_tx,&rc_state_y);
        tx_len += mavlink_msg_to_send_buffer(tx_buf+tx_len, &msg_tx);
        mavlink_msg_component_heartbeat_encode(REF_COMPONENT_ID_FISHPOND,REF_COMPONENT_ID_FISHPOND,&msg_tx,&heart_beat_pond);
        tx_len += mavlink_msg_to_send_buffer(tx_buf+tx_len, &msg_tx);

        heart_beat_pond.pack_count++;
        heart_beat_table_x.pack_count++;
        heart_beat_table_y.pack_count++;
        rc_state_x.state = !rc_state_x.state;
        rc_state_y.state = !rc_state_y.state;

        HAL_UART_Transmit_DMA(&huart4, tx_buf, tx_len);
        tx_thread_sleep(10);
    }
}