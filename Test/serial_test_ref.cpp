/*
 * @Description:
 * @Author: qianwan
 * @Date: 2023-12-25 16:13:17
 * @LastEditTime: 2024-02-03 07:21:39
 * @LastEditors: qianwan
 */
#include "../Module/Mavlink/C/ReferCom/mavlink.h"
#include "iostream"
#include <string>

using namespace std;

int main() {
  uint32_t len;
  uint8_t tx_buf[256];
  mavlink_message_t msg_tx = {};
  mavlink_server_heartbeat_t server_heartbeat = {};

  mavlink_msg_server_heartbeat_encode(REF_COMPONENT_ID_SERVER, REF_COMPONENT_ID_SERVER, &msg_tx, &server_heartbeat);
  len = mavlink_msg_to_send_buffer(tx_buf, &msg_tx);
  for (int i = 0; i < len; i++) {
    printf("%02x ", tx_buf[i]);
  }

  printf(" ");
  return 0;
}