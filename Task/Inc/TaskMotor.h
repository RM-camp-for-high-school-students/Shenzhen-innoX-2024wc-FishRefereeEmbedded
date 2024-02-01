/*
 * @Description: Task of Servos control
 * @Author: qianwan
 * @Date: 2023-12-25 11:44:40
 * @LastEditTime: 2024-02-01 11:26:13
 * @LastEditors: qianwan
 */
#pragma once
#ifndef TASK_MOTOR_H
#define TASK_MOTOR_H

#include "stm32f4xx.h"
#include "cstdint"

__PACKED_STRUCT DM_Motor_t {
    int16_t pst;
    int16_t last_pst;
    float pst_radian;
    float vel;
    int16_t rpm;
    int16_t current;
    uint8_t tmp_coil;
    uint8_t tmp_pcb;
    uint64_t last_update_time;
    uint8_t state;
};

#endif