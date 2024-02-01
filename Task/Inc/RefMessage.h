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


#pragma pack(pop) //恢复对齐状态
#endif