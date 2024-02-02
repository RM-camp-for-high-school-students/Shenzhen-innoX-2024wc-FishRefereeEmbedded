#pragma once
#ifndef FISHMESSAGE_H
#define FISHMESSAGE_H
#include <cstdint>
#include "ReferCom/mavlink.h"
#include "om.h"

extern uint32_t fishPrintf(uint8_t *buf, const char *str, ...);

#pragma pack(push) //保存对齐状态
#pragma pack(1)

struct Msg_DM_t {
    float pst[4];      //Position
    uint8_t enable[4]; //Enable Number
};

struct Msg_ZDT_t {
    float pst[4];      //Position
    uint8_t enable[4]; //Enable Number
};

struct Msg_Servo_t{
    uint8_t enable;
    uint8_t release[3];
};

struct Msg_Remoter_Judge_t{
    uint8_t connected;
};

struct Sys_state_t{
    uint8_t ref_system_id;
    uint8_t motor_dm_num;
    uint8_t motor_zdt_num;
    uint8_t motor_error;

    uint8_t state_last;
    uint8_t state_now;
    uint8_t state_new;

    uint16_t state_parameter;

    uint8_t state_switch_flag;

    float angle_set_clean;
    float angle_set_normal;

    uint8_t error_code;

    uint8_t* uart_rx_buf;
    uint16_t buf_len;
    uint16_t rx_len;
};

#pragma pack(pop) //恢复对齐状态

extern Sys_state_t sys_state;
#endif