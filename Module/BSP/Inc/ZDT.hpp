#pragma once
#ifndef ZDT_H
#define ZDT_H

#include "cstdint"

namespace ZDT {
    const uint8_t ANTICLOCKWISE = 0;
    const uint8_t CLOCKWISE = 1;
    const uint8_t OPPOSITE = 0;
    const uint8_t ABSOLUTE = 1;
    const uint8_t CLOSEST = 0;
    const uint8_t DIRECTION = 0;

    const uint8_t STATE_IDLE = 0;
    const uint8_t STATE_NORMAL = 1;
    const uint8_t STATE_ERROR = 2;

    const uint8_t FDB_NORMAL = 0;
    const uint8_t FDB_NOT_THIS = 1;
    const uint8_t FDB_ERROR_POINTER = 2;
    const uint8_t FDB_ERROR_CHECK = 3;
    const uint8_t FDB_ERROR_COM = 4;
    const uint8_t FDB_ERROR_UNSATISFIED = 0xE2;
    const uint8_t FDB_ERROR_COMMAND = 0xEE;

    class cZDT {
    protected:
        uint16_t _id;
        uint8_t _check = 0x6B;
        uint8_t _state = FDB_NORMAL;
        uint16_t _encoder;
        uint64_t _last_update_time = 0;
    public:
        cZDT(uint8_t ID = 0x01, uint8_t check_sum = 0x6B) :
                _id(ID), _check(check_sum) {
        }

        void SetID(uint16_t id) { _id = id; }

        uint8_t Enable(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t enable_flag, bool syn_flag = false,
                       uint8_t check_sum = 0x6B);

//        uint8_t SetPID(uint32_t &exID0, uint32_t &exID1, uint32_t &exID2, uint32_t &datalen0, uint32_t &datalen1,
//                       uint32_t &datalen2, uint8_t *pbuf0, uint8_t *pbuf1, uint8_t *pbuf2,
//                       bool save_flag, uint32_t kp_trape, uint32_t kp_dirct, uint32_t kp_v, uint32_t ki_v,
//                       uint8_t check_sum = 0x6B);
//        uint8_t SetPIDFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        uint8_t
        SetZeroPst(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool storage = true, uint8_t check_sum = 0x6B);

        uint8_t ZeroTrigger(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool move_mode, bool syn_flag = false,
                            uint8_t check_sum = 0x6B);

        uint8_t
        TpzPst(uint32_t &exID0, uint32_t &exID1, uint32_t &datalen0, uint32_t &datalen1, uint8_t *pbuf0, uint8_t *pbuf1,
               bool dir, uint16_t accel,
               uint16_t deaccel, float maxspeed, float pst, bool opposite_mode, bool syn_flag = false,
               uint8_t check_sum = 0x6B);

        uint8_t TpzPstFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        uint8_t ReadEncoder(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t check_sum = 0x6B);

        uint8_t ReadEncoderFDB(uint8_t *pdata, uint8_t check_sum = 0x6B);

        uint8_t TimeUpdate(uint64_t new_time);

        uint64_t GetTime();

        void SetState(uint8_t state) {
            _state = state;
        };

        uint8_t GetState();
    };
}


#endif