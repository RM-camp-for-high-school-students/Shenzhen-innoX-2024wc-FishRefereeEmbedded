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

#include "DMDriver.h"
#include "stm32f4xx.h"
#include "cstdint"
#include "can.h"

namespace TASK_MOTOR {
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

    class cDMFace : public cDMMotor {
    protected:
        uint64_t _update_time;
        uint8_t _state;
        CAN_TxHeaderTypeDef _TxHeader{};
        CAN_HandleTypeDef *_pcan = nullptr;
        uint32_t *_can_mail_box = nullptr;

        uint8_t CAN_Transmit(uint16_t head, uint8_t *pdata, uint8_t len) override {
            _TxHeader.StdId = head;
            _TxHeader.DLC = len;
            return HAL_CAN_AddTxMessage(_pcan, &_TxHeader, pdata, _can_mail_box);
        }

    public:
        cDMFace(uint16_t id, CAN_HandleTypeDef *pcan) :
                cDMMotor(id), _pcan(pcan) {
            _TxHeader.RTR = CAN_RTR_DATA;
            _TxHeader.IDE = CAN_ID_STD;
        }

        cDMFace() : cDMMotor(0x00) {
            _TxHeader.RTR = CAN_RTR_DATA;
            _TxHeader.IDE = CAN_ID_STD;
        }

        void SetDM(uint16_t id, CAN_HandleTypeDef *pcan) {
            cDMMotor::_id = id;
            _pcan = pcan;
        }

        void UpdateTim(uint64_t time) {
            _update_time = time;
        }

        uint64_t GetTim() {
            return _update_time;
        }

        void SetState(uint8_t state) {
            _state = state;
        }

        uint8_t GetState() {
            return _state;
        }
    };
}
#endif